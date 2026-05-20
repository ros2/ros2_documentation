"""
OpenAI Responses API helpers for short-description generation.

Example RSTs are indexed into a vector store once per run and attached via ``file_search``.
Each target article is uploaded with the Files API (``purpose=user_data``) and referenced as
``input_file`` so the full extracted text is available in the request context.
"""

from __future__ import annotations

import logging
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Iterable

from openai import OpenAI, RateLimitError, APIConnectionError
from tenacity import retry, stop_after_attempt, wait_random_exponential, retry_if_exception_type

from config import (
    MAX_RETRIES,
    MAX_WAIT,
    MIN_WAIT,
)

logger = logging.getLogger(__name__)

# File-input guidance (see OpenAI file inputs documentation).
_MAX_INPUT_FILE_BYTES = 50 * 1024 * 1024

_SCRIPTS_DIR = Path(__file__).resolve().parent
REPO_ROOT = _SCRIPTS_DIR.parent

SHORT_DESCRIPTION_USER_PREAMBLE = (
    "Article RST (generate only the short description prose per your instructions; "
    "use file_search on the indexed examples for tone and structure). "
    "The article to enhance is attached as a file."
)


class RetrievalResources:
    """IDs created for one enhance_short_descriptions run (for cleanup)."""

    __slots__ = ("vector_store_id",)

    def __init__(self, vector_store_id: str) -> None:
        self.vector_store_id = vector_store_id


def _resolve_example_paths(example_paths: Iterable[str]) -> list[Path]:
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

    if batch.status != "completed":
        logger.error("Vector store file batch ended with status %r", batch.status)
        raise RuntimeError(f"Vector store indexing did not complete: {batch.status}")

    logger.debug("Vector store %s ready (batch status=%s)", vs.id, batch.status)
    return vs.id


def _extract_response_output_text(response: object) -> str:
    """Return concatenated assistant output text from a Responses API result."""
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
def _upload_article_and_create_response(
    client: OpenAI,
    article_path: str,
    vector_store_id: str,
    instructions: str,
    model: str,
) -> tuple[object, str]:
    path = Path(article_path)
    size = path.stat().st_size
    if size > _MAX_INPUT_FILE_BYTES:
        logger.warning(
            "Article file %s is %s bytes (exceeds %s byte file-input guidance); request may fail.",
            article_path,
            size,
            _MAX_INPUT_FILE_BYTES,
        )

    logger.info(
        "Generating short description via Responses API for %s",
        article_path,
    )
    with open(path, "rb") as f:
        uploaded = client.files.create(file=f, purpose="user_data")

    try:
        return client.responses.create(
            model=model,
            instructions=instructions,
            tools=[{"type": "file_search", "vector_store_ids": [vector_store_id]}],
            input=[
                {
                    "role": "user",
                    "content": [
                        {"type": "input_text", "text": SHORT_DESCRIPTION_USER_PREAMBLE},
                        {"type": "input_file", "file_id": uploaded.id},
                    ],
                },
            ],
        ), uploaded.id
    except Exception:
        try:
            client.files.delete(uploaded.id)
        except Exception as exc:  # noqa: BLE001
            logger.warning("Could not delete uploaded article file %s: %s", uploaded.id, exc)
        raise


def _run_responses_once(
    client: OpenAI,
    vector_store_id: str,
    article_path: str,
    instructions: str,
    model: str,
) -> str:
    response, file_id = _upload_article_and_create_response(
        client,
        article_path,
        vector_store_id,
        instructions,
        model,
    )
    try:
        status = getattr(response, "status", None)
        if status and status != "completed":
            logger.error("Responses API ended with status %r", status)
            return ""

        text = _extract_response_output_text(response)
        logger.debug("Responses API completed; output length %s", len(text))
        return text
    finally:
        try:
            client.files.delete(file_id)
            logger.debug("Deleted uploaded article file %s", file_id)
        except Exception as exc:  # noqa: BLE001
            logger.warning("Could not delete uploaded article file %s: %s", file_id, exc)


def analyze_with_responses(
    client: OpenAI,
    vector_store_id: str,
    article_path: str,
    instructions: str,
    model: str,
    timeout: int,
) -> str:
    """
    Run short-description generation for one article via the Responses API.

    Uploads the article with the Files API, calls ``responses.create`` with ``input_file`` and
    ``file_search`` over the example vector store, then deletes the uploaded file.

    Uses ThreadPoolExecutor so ``timeout`` bounds wall-clock time for upload plus the response.
    """

    def _bounded_attempt() -> str:
        return _run_responses_once(
            client,
            vector_store_id,
            article_path,
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
    """Best-effort deletion of vector store and hosted example files."""
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
