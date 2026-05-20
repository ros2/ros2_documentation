"""
OpenAI Assistants API helpers for short-description generation with file_search.

Uses a vector store of example RST files so each target article is sent once per run,
without inlining full examples in every request.
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
    ASSISTANT_POLL_INTERVAL,
    ASSISTANT_RUN_TIMEOUT,
    MAX_CONTENT_LENGTH,
    MAX_RETRIES,
    MAX_WAIT,
    MIN_WAIT,
)

logger = logging.getLogger(__name__)

_SCRIPTS_DIR = Path(__file__).resolve().parent
REPO_ROOT = _SCRIPTS_DIR.parent


class RetrievalResources:
    """IDs created for one enhance_short_descriptions run (for cleanup)."""

    __slots__ = ("assistant_id", "vector_store_id")

    def __init__(self, assistant_id: str, vector_store_id: str) -> None:
        self.assistant_id = assistant_id
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
    logger.debug("Creating vector store for %s example file(s)", len(paths))
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


@retry(
    retry=retry_if_exception_type((RateLimitError, APIConnectionError)),
    stop=stop_after_attempt(MAX_RETRIES),
    wait=wait_random_exponential(multiplier=MIN_WAIT, max=MAX_WAIT),
    reraise=True,
)
def create_short_description_assistant(
    client: OpenAI,
    vector_store_id: str,
    instructions: str,
    model: str,
) -> str:
    """Create an assistant with file_search over the given vector store. Returns assistant_id."""
    assistant = client.beta.assistants.create(
        name="ROS documentation short description",
        instructions=instructions,
        model=model,
        tools=[{"type": "file_search"}],
        tool_resources={"file_search": {"vector_store_ids": [vector_store_id]}},
    )
    logger.debug("Created assistant %s", assistant.id)
    return assistant.id


def _extract_assistant_message_text(client: OpenAI, thread_id: str) -> str:
    messages = client.beta.threads.messages.list(thread_id=thread_id, order="desc", limit=10)
    for msg in messages.data:
        if msg.role != "assistant":
            continue
        parts: list[str] = []
        for block in msg.content:
            if block.type == "text":
                parts.append(block.text.value)
        return "".join(parts).strip()
    return ""


def _retrieve_run_with_backoff(
    client: OpenAI,
    thread_id: str,
    run_id: str,
    deadline: float,
) -> object:
    """Retrieve run status; on rate limit / connection errors sleep and retry until deadline."""
    attempt = 0
    while True:
        if time.monotonic() >= deadline:
            raise TimeoutError(f"Assistant run {run_id}: deadline before status retrieve")
        try:
            return client.beta.threads.runs.retrieve(thread_id=thread_id, run_id=run_id)
        except (RateLimitError, APIConnectionError) as exc:
            attempt += 1
            wait = min(ASSISTANT_POLL_INTERVAL * (2**attempt), 30.0)
            logger.warning("Run retrieve retry %s after %s: sleeping %.1fs", attempt, exc, wait)
            time.sleep(wait)


def _poll_run_until_terminal(
    client: OpenAI,
    thread_id: str,
    run_id: str,
    deadline: float,
) -> object:
    """Poll run status until terminal state or deadline. Returns final run object."""
    run = _retrieve_run_with_backoff(client, thread_id, run_id, deadline)
    while run.status in ("queued", "in_progress", "cancelling"):
        if time.monotonic() >= deadline:
            raise TimeoutError(
                f"Assistant run {run_id} still {run.status!r} after polling deadline",
            )
        time.sleep(ASSISTANT_POLL_INTERVAL)
        run = _retrieve_run_with_backoff(client, thread_id, run_id, deadline)
    return run


def _run_assistant_once(client: OpenAI, assistant_id: str, content: str, run_timeout: int) -> str:
    """Single attempt: new thread, user message, run, poll until terminal, read assistant text."""
    if len(content) > MAX_CONTENT_LENGTH:
        logger.warning(
            "Article RST truncated to %s characters for assistant message.",
            MAX_CONTENT_LENGTH,
        )
        content = content[:MAX_CONTENT_LENGTH]

    user_text = (
        "Article RST (generate only the short description prose per your instructions; "
        "use file_search on the indexed examples for tone and structure):\n\n"
        f"{content}"
    )

    thread = client.beta.threads.create()
    client.beta.threads.messages.create(
        thread_id=thread.id,
        role="user",
        content=user_text,
    )
    run = client.beta.threads.runs.create(
        thread_id=thread.id,
        assistant_id=assistant_id,
    )
    deadline = time.monotonic() + float(run_timeout)
    run = _poll_run_until_terminal(client, thread.id, run.id, deadline)

    if run.status == "completed":
        text = _extract_assistant_message_text(client, thread.id)
        logger.debug("Assistant run completed; response length %s", len(text))
        return text

    if run.status == "failed":
        err = getattr(run, "last_error", None)
        logger.error("Assistant run failed: %s", err)
        return ""

    if run.status == "expired":
        logger.error("Assistant run expired")
        return ""

    if run.status == "cancelled":
        logger.warning("Assistant run cancelled")
        return ""

    if run.status == "requires_action":
        logger.error("Assistant run requires_action (unexpected for file_search-only flow)")
        return ""

    logger.error("Assistant run ended with unexpected status %r", run.status)
    return ""


def analyze_with_file_search(
    client: OpenAI,
    assistant_id: str,
    content: str,
    timeout: int = ASSISTANT_RUN_TIMEOUT,
) -> str:
    """
    Run the short-description assistant on one article's RST.

    Uses ThreadPoolExecutor so ``timeout`` bounds wall-clock time including polling
    (same value as the internal poll deadline for the run).
    """

    def _bounded_attempt() -> str:
        return _run_assistant_once(client, assistant_id, content, timeout)

    with ThreadPoolExecutor(max_workers=1) as executor:
        future = executor.submit(_bounded_attempt)
        try:
            return future.result(timeout=timeout)
        except TimeoutError:
            logger.error("analyze_with_file_search timed out after %s seconds", timeout)
            raise


def cleanup_short_description_resources(client: OpenAI, resources: RetrievalResources | None) -> None:
    """Best-effort deletion of assistant and vector store (and hosted files in the store)."""
    if resources is None:
        return

    try:
        client.beta.assistants.delete(resources.assistant_id)
        logger.debug("Deleted assistant %s", resources.assistant_id)
    except Exception as exc:  # noqa: BLE001 — cleanup must not raise
        logger.warning("Could not delete assistant %s: %s", resources.assistant_id, exc)

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
