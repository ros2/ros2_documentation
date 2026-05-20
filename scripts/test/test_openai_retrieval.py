"""Unit tests for ``openai_retrieval`` (Responses API short-description path)."""

from __future__ import annotations

from unittest.mock import MagicMock

from openai_retrieval import (
    RetrievalResources,
    analyze_with_responses,
    cleanup_short_description_resources,
    extract_response_output_text,
)


def test_extract_response_output_text_prefers_output_text_attribute() -> None:
    response = MagicMock()
    response.output_text = "  hello  "
    assert extract_response_output_text(response) == "hello"


def test_extract_response_output_text_walks_output_blocks() -> None:
    block = MagicMock()
    block.type = "output_text"
    block.text = "from blocks"

    msg = MagicMock()
    msg.type = "message"
    msg.role = "assistant"
    msg.content = [block]

    class FakeResp:
        output = [msg]

    assert extract_response_output_text(FakeResp()) == "from blocks"


def test_analyze_with_responses_success() -> None:
    client = MagicMock()
    resp = MagicMock()
    resp.status = "completed"
    resp.output_text = "Generated short description."
    client.responses.create.return_value = resp

    out = analyze_with_responses(
        client,
        "vs_store_1",
        "file_uploaded_1",
        "system instructions here",
        "gpt-test-model",
        timeout=60,
    )

    assert out == "Generated short description."
    client.files.create.assert_not_called()
    client.files.delete.assert_not_called()

    rc = client.responses.create.call_args.kwargs
    assert rc["model"] == "gpt-test-model"
    assert rc["instructions"] == "system instructions here"
    assert rc["tools"] == [{"type": "file_search", "vector_store_ids": ["vs_store_1"]}]
    user_block = rc["input"][0]["content"]
    types = [c["type"] for c in user_block]
    assert "input_text" in types
    assert "input_file" in types
    file_part = next(c for c in user_block if c["type"] == "input_file")
    assert file_part["file_id"] == "file_uploaded_1"


def test_analyze_with_responses_non_completed_status_returns_empty() -> None:
    client = MagicMock()
    resp = MagicMock()
    resp.status = "failed"
    client.responses.create.return_value = resp

    assert (
        analyze_with_responses(
            client,
            "vs_1",
            "file_2",
            "instr",
            "m",
            timeout=60,
        )
        == ""
    )


def test_cleanup_short_description_resources_no_assistants_delete() -> None:
    client = MagicMock()
    delete_asst = MagicMock()
    client.beta.assistants.delete = delete_asst
    listed = MagicMock()
    listed.data = []
    client.vector_stores.files.list.return_value = listed

    cleanup_short_description_resources(client, RetrievalResources("vs_x"))

    delete_asst.assert_not_called()
    client.vector_stores.delete.assert_called_once_with("vs_x")
