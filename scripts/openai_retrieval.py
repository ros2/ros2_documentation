"""
OpenAI Responses API helpers for short-description generation.

Example RSTs are indexed into a vector store once per run and attached via ``file_search``.
Target articles are uploaded once per file in ``analyze_files`` and passed by ``file_id``.
"""

from __future__ import annotations

import logging
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Iterable

from openai import OpenAI, RateLimitError, APIConnectionError
from tenacity import retry, stop_after_attempt, wait_random_exponential, retry_if_exception_type

from config import (
    MAX_RETRIES,
    MAX_WAIT,
    MIN_WAIT,
    RESPONSE_TIMEOUT,
)

# Define the logger for the module
logger = logging.getLogger(__name__)

_SCRIPTS_DIR = Path(__file__).resolve().parent
REPO_ROOT = _SCRIPTS_DIR.parent

# Define the user preamble for the short description
SHORT_DESCRIPTION_USER_PREAMBLE = (
    "Article RST (generate only the short description prose per your instructions; "
    "use file_search on the indexed examples for tone and structure). "
    "The article to enhance is attached as a file."
)

# Define the class for the retrieval resources
# Used to store the vector store id for the retrieval resources
class RetrievalResources:
    """
    Immutable data structure to store IDs created for one enhancement run.
    
    Used for cleaning up resources after the run completes.
    """

    __slots__ = ("vector_store_id",)

    def __init__(self, vector_store_id: str) -> None:
        """
        Initialise the retrieval resources with a vector store ID.

        Args:
            vector_store_id: The ID of the OpenAI vector store.
        """
        self.vector_store_id = vector_store_id


def _resolve_example_paths(example_paths: Iterable[str]) -> list[Path]:
    """
    Resolve a sequence of example RST file paths (relative to the repository root)
    into absolute, validated Path objects. Raises FileNotFoundError if any file does not exist.

    Args:
        example_paths (Iterable[str]): Iterable of example file paths (relative to repo root).

    Returns:
        list[Path]: List of absolute Path objects for the example RST files.

    Raises:
        FileNotFoundError: If any example file cannot be found at the expected path.
    """
    paths: list[Path] = []
    for rel in example_paths:
        p = (REPO_ROOT / rel).resolve()
        if not p.is_file():
            raise FileNotFoundError(f"Example RST not found: {p}")
        paths.append(p)
    return paths


@retry(
    retry=retry_if_exception_type((RateLimitError, APIConnectionError)),
    stop=stop_after_attempt(MAX_RETRIES),
    wait=wait_random_exponential(multiplier=MIN_WAIT, max=MAX_WAIT),
    reraise=True,
)
def ensure_example_vector_store(client: OpenAI, example_paths: Iterable[str]) -> str:
    """
    Create a vector store, upload example RST files, and wait for indexing to finish.

    Returns:
        vector_store_id
    """
    paths = _resolve_example_paths(example_paths)
    logger.info(
        "Indexing %s example RST file(s) into OpenAI vector store",
        len(paths),
    )
    vs = client.vector_stores.create(name="ros2-doc-short-description-examples")

    from contextlib import ExitStack

    with ExitStack() as stack:
        streams = [stack.enter_context(open(p, "rb")) for p in paths]
        batch = client.vector_stores.file_batches.upload_and_poll(
            vector_store_id=vs.id,
            files=streams,
        )

    # If upload_and_poll returns before completion (e.g. due to internal timeout),
    # we continue polling manually.
    poll_start = time.time()
    while batch.status in ("in_progress", "queued"):
        if time.time() - poll_start > RESPONSE_TIMEOUT:
            logger.error(
                "Vector store indexing timed out after %s seconds (status: %s)",
                RESPONSE_TIMEOUT,
                batch.status,
            )
            break
        time.sleep(5)
        batch = client.vector_stores.file_batches.retrieve(
            vector_store_id=vs.id,
            batch_id=batch.id,
        )
        logger.debug("Polled vector store batch %s: status=%s", batch.id, batch.status)

    if batch.status != "completed":
        logger.error("Vector store file batch ended with status %r", batch.status)
        if batch.status == "failed":
            logger.error("Vector store batch failed with error details: %r", getattr(batch, "last_error", None))
        raise RuntimeError(f"Vector store indexing did not complete: {batch.status}")

    logger.debug("Vector store %s ready (batch status=%s)", vs.id, batch.status)
    return vs.id


def extract_response_output_text(response: object) -> str:
    """
    Return concatenated assistant output text from a Responses API result.

    Args:
        response: The response object from the OpenAI API.

    Returns:
        The extracted and concatenated output text.
    """
    output_text = getattr(response, "output_text", None)
    if isinstance(output_text, str) and output_text.strip():
        return output_text.strip()

    parts: list[str] = []
    output = getattr(response, "output", None) or []
    for item in output:
        item_type = getattr(item, "type", None)
        if item_type != "message":
            continue
        role = getattr(item, "role", None)
        if role is not None and role != "assistant":
            continue
        for block in getattr(item, "content", []) or []:
            btype = getattr(block, "type", None)
            if btype == "output_text":
                text = getattr(block, "text", None)
                if text:
                    parts.append(text)
    return "".join(parts).strip()


@retry(
    retry=retry_if_exception_type((RateLimitError, APIConnectionError)),
    stop=stop_after_attempt(MAX_RETRIES),
    wait=wait_random_exponential(multiplier=MIN_WAIT, max=MAX_WAIT),
    reraise=True,
)
def _create_short_description_response(
    client: OpenAI,
    file_id: str,
    vector_store_id: str,
    instructions: str,
    model: str,
) -> object:
    """
    Create a request for short-description generation via the OpenAI Responses API.

    Args:
        client: OpenAI client instance.
        file_id: ID of the uploaded RST file.
        vector_store_id: ID of the vector store containing examples.
        instructions: System instructions for the model.
        model: The GPT model to use.

    Returns:
        The response object from the OpenAI API.
    """
    logger.info("Generating short description via Responses API using vector store and uploaded file")
    return client.responses.create(
        model=model,
        instructions=instructions,
        tools=[{"type": "file_search", "vector_store_ids": [vector_store_id]}],
        input=[
            {
                "role": "user",
                "content": [
                    {"type": "input_text", "text": SHORT_DESCRIPTION_USER_PREAMBLE},
                    {"type": "input_file", "file_id": file_id},
                ],
            },
        ],
    )


def _run_short_description_response(
    client: OpenAI,
    file_id: str,
    vector_store_id: str,
    instructions: str,
    model: str,
) -> str:
    """
    Execute a short-description generation request and extract the text.

    Args:
        client: OpenAI client instance.
        file_id: ID of the uploaded RST file.
        vector_store_id: ID of the vector store containing examples.
        instructions: System instructions for the model.
        model: The GPT model to use.

    Returns:
        The generated short description text, or an empty string on failure.
    """
    response = _create_short_description_response(
        client,
        file_id,
        vector_store_id,
        instructions,
        model,
    )
    status = getattr(response, "status", None)
    if status and status != "completed":
        logger.error("Responses API ended with status %r", status)
        return ""

    text = extract_response_output_text(response)
    logger.debug("Responses API completed; output length %s", len(text))
    return text


def analyze_with_responses(
    client: OpenAI,
    vector_store_id: str,
    file_id: str,
    instructions: str,
    model: str,
    timeout: int,
) -> str:
    """
    Run short-description generation for one article via the Responses API.

    Expects ``file_id`` from a prior Files API upload (see ``analyze_files``).
    Uses ThreadPoolExecutor so ``timeout`` bounds wall-clock time for the response.

    Args:
        client: OpenAI client instance.
        vector_store_id: ID of the vector store containing examples.
        file_id: ID of the uploaded RST file.
        instructions: System instructions for the model.
        model: The GPT model to use.
        timeout: Maximum time to wait for the response in seconds.

    Returns:
        The generated short description text.
    """

    def _bounded_attempt() -> str:
        """Execute the response generation within the thread pool."""
        return _run_short_description_response(
            client,
            file_id,
            vector_store_id,
            instructions,
            model,
        )

    with ThreadPoolExecutor(max_workers=1) as executor:
        future = executor.submit(_bounded_attempt)
        try:
            return future.result(timeout=timeout)
        except TimeoutError:
            logger.error("analyze_with_responses timed out after %s seconds", timeout)
            raise


def cleanup_short_description_resources(client: OpenAI, resources: RetrievalResources | None) -> None:
    """
    Best-effort deletion of vector store and hosted example files.

    Args:
        client: OpenAI client instance.
        resources: The retrieval resources to clean up.
    """
    if resources is None:
        return

    logger.info(
        "Cleaning up OpenAI vector store and hosted files",
    )
    try:
        listed = client.vector_stores.files.list(vector_store_id=resources.vector_store_id)
        file_entries = getattr(listed, "data", None)
        if file_entries is None:
            file_entries = list(listed)
        for vs_file in file_entries:
            fid = getattr(vs_file, "id", None)
            if not fid:
                continue
            try:
                client.files.delete(fid)
                logger.debug("Deleted file %s from vector store", fid)
            except Exception as exc:  # noqa: BLE001
                logger.warning("Could not delete file %s: %s", fid, exc)
    except Exception as exc:  # noqa: BLE001
        logger.warning("Could not list vector store files for %s: %s", resources.vector_store_id, exc)

    try:
        client.vector_stores.delete(resources.vector_store_id)
        logger.debug("Deleted vector store %s", resources.vector_store_id)
    except Exception as exc:  # noqa: BLE001
        logger.warning("Could not delete vector store %s: %s", resources.vector_store_id, exc)
