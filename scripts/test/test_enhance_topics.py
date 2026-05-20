import pytest
from unittest.mock import MagicMock, patch, mock_open
import sys
import os
from openai import OpenAIError

# Add the scripts directory to sys.path to allow importing enhance_topics
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import MAX_CONTENT_LENGTH
from enhance_topics import (
    analyze_content,
    get_openai_client,
    analyze_files,
    update_meta_files,
    enhance_metadata,
    enhance_short_descriptions,
    main,
    _metadata_enhancement_task,
)
from enhance_data import EnhanceData, create_enhance_data, calculate_metrics

@pytest.fixture
def mock_client():
    """Provides a mocked OpenAI client."""
    return MagicMock()

# --- Tests for analyze_content ---

def test_analyze_content_success(mock_client):
    """Test successful content analysis."""
    mock_completion = MagicMock()
    mock_completion.choices = [MagicMock(message=MagicMock(content='Analysis result'))]
    mock_client.chat.completions.create.return_value = mock_completion

    result = analyze_content(mock_client, "Some content", "Some prompt")
    assert result == 'Analysis result'
    mock_client.chat.completions.create.assert_called_once()

def test_analyze_content_truncation(mock_client):
    """Test that content is truncated if it exceeds MAX_CONTENT_LENGTH."""
    long_content = "a" * (MAX_CONTENT_LENGTH + 100)
    mock_completion = MagicMock()
    mock_completion.choices = [MagicMock(message=MagicMock(content='Result'))]
    mock_client.chat.completions.create.return_value = mock_completion

    analyze_content(mock_client, long_content, "Prompt")
    
    # Check the call arguments to ensure content was truncated
    args, kwargs = mock_client.chat.completions.create.call_args
    sent_content = kwargs['messages'][1]['content']
    assert len(sent_content) <= MAX_CONTENT_LENGTH + len("Content:\n\n")

def test_analyze_content_empty_response(mock_client):
    """Test handling of empty response from API."""
    mock_completion = MagicMock()
    mock_completion.choices = [MagicMock(message=MagicMock(content=None))]
    mock_client.chat.completions.create.return_value = mock_completion

    result = analyze_content(mock_client, "Content", "Prompt")
    assert result == ""

# --- Tests for get_openai_client ---

@patch('enhance_topics.load_dotenv')
@patch.dict(os.environ, {"OPENAI_API_KEY": "test-key"})
def test_get_openai_client_success(mock_load_dotenv):
    """Test successful client initialisation."""
    client = get_openai_client()
    assert client.api_key == "test-key"

@patch('enhance_topics.load_dotenv')
@patch.dict(os.environ, {}, clear=True)
def test_get_openai_client_missing_key(mock_load_dotenv):
    """Test error when API key is missing."""
    with pytest.raises(OpenAIError, match="OpenAI API key not found"):
        get_openai_client()

# --- Tests for analyze_files ---

@patch('enhance_topics.get_meta_names_from_content')
@patch('enhance_topics.analyze_content')
@patch('enhance_topics.validate_content')
@patch('enhance_topics.add_analysis_result')
@patch('enhance_topics.create_enhance_data')
def test_analyze_files_basic_flow(
    mock_create_data, 
    mock_add_result, 
    mock_validate, 
    mock_analyze, 
    mock_get_meta, 
    mock_client
):
    """Test the basic flow of analyze_files."""
    mock_create_data.return_value = EnhanceData(results={}, updated_files=set())
    mock_get_meta.return_value = [] # No existing metadata
    mock_analyze.return_value = "Generated result"
    mock_validate.return_value = True
    mock_add_result.return_value = EnhanceData(
        results={"file1.rst": {"description": "res"}}, 
        updated_files=set()
    )

    files = ["file1.rst"]
    tasks = [_metadata_enhancement_task("description", "desc prompt")]

    with patch("builtins.open", mock_open(read_data="File content")):
        analyze_files(files, mock_client, tasks)

    mock_analyze.assert_called_once()
    mock_validate.assert_called_once()
    mock_add_result.assert_called_once()

@patch('enhance_topics.get_meta_names_from_content')
def test_analyze_files_skips_existing_meta(mock_get_meta, mock_client):
    """Test that files with existing metadata are skipped."""
    mock_get_meta.return_value = {"description"}

    files = ["file1.rst"]
    tasks = [_metadata_enhancement_task("description", "desc prompt")]

    with patch("builtins.open", mock_open(read_data="File content")):
        with patch('enhance_topics.analyze_content') as mock_analyze:
            analyze_files(files, mock_client, tasks)
            mock_analyze.assert_not_called()

# --- Tests for update_meta_files ---

@patch('enhance_topics.get_results_for_file')
@patch('enhance_topics.inject_metadata_to_content')
@patch('enhance_topics.mark_file_updated')
def test_update_meta_files_writes_on_change(
    mock_mark_updated, 
    mock_inject, 
    mock_get_results,
    mock_client
):
    """Test that files are written only when metadata changes."""
    mock_get_results.return_value = {"description": "new desc"}
    mock_inject.return_value = ("New content", True) # Changed is True
    mock_mark_updated.return_value = EnhanceData(
        results={}, 
        updated_files={"file1.rst"}
    )

    data = EnhanceData(
        results={"file1.rst": {"description": "new desc"}}, 
        updated_files=set()
    )
    
    m_open = mock_open(read_data="Old content")
    with patch("builtins.open", m_open):
        update_meta_files(["file1.rst"], data)

    # Verify write was called
    m_open().write.assert_called_once_with("New content")
    mock_mark_updated.assert_called_once()

@patch('enhance_topics.get_results_for_file')
@patch('enhance_topics.inject_metadata_to_content')
def test_update_meta_files_skips_no_change(mock_inject, mock_get_results):
    """Test that files are NOT written when no metadata changes."""
    mock_get_results.return_value = {"description": "same desc"}
    mock_inject.return_value = ("Old content", False) # Changed is False

    data = EnhanceData(
        results={"file1.rst": {"description": "same desc"}}, 
        updated_files=set()
    )
    
    m_open = mock_open(read_data="Old content")
    with patch("builtins.open", m_open):
        update_meta_files(["file1.rst"], data)

    # Verify write was NOT called
    m_open().write.assert_not_called()

@patch("enhance_topics.get_meta_names_from_content")
@patch("enhance_topics.analyze_content")
@patch("enhance_topics.validate_content")
def test_analyze_files_accumulates_onto_initial_data(
    mock_validate,
    mock_analyze,
    mock_get_meta,
    mock_client,
):
    """Passing an accumulator extends per-file results via add_analysis_result."""
    mock_get_meta.return_value = []
    mock_analyze.return_value = "Generated description"
    mock_validate.return_value = True
    initial = EnhanceData(
        results={"file1.rst": {"keywords": "existing"}},
        updated_files=set(),
    )
    tasks = [_metadata_enhancement_task("description", "desc prompt")]

    with patch("builtins.open", mock_open(read_data="File content")):
        result = analyze_files(["file1.rst"], mock_client, tasks, initial)

    assert result.results["file1.rst"]["keywords"] == "existing"
    assert result.results["file1.rst"]["description"] == "Generated description"


# --- Tests for enhance_metadata ---

@patch('enhance_topics.get_openai_client')
@patch('enhance_topics.analyze_files')
@patch('enhance_topics.update_meta_files')
def test_enhance_metadata_orchestration(mock_update, mock_analyze, mock_get_client):
    """Test the orchestration in enhance_metadata."""
    mock_get_client.return_value = MagicMock()
    mock_analyze.return_value = EnhanceData(results={"f": {"d": "r"}}, updated_files=set())
    mock_update.return_value = EnhanceData(results={"f": {"d": "r"}}, updated_files={"f"})

    result = enhance_metadata(["file1.rst"])
    
    assert result.updated_files == {"f"}
    mock_get_client.assert_called_once()
    mock_analyze.assert_called_once()
    mock_update.assert_called_once()


@patch("enhance_topics.cleanup_short_description_resources")
@patch("enhance_topics.update_enhanced_files")
@patch("enhance_topics.analyze_files")
@patch("enhance_topics.create_short_description_assistant")
@patch("enhance_topics.ensure_example_vector_store")
@patch("enhance_topics.get_openai_client")
def test_enhance_short_descriptions_orchestration(
    mock_get_client,
    mock_ensure_vs,
    mock_create_asst,
    mock_analyze,
    mock_update,
    mock_cleanup,
):
    """Short-description path creates VS + assistant, analyses, updates, and cleans up."""
    mock_client = MagicMock()
    mock_get_client.return_value = mock_client
    mock_ensure_vs.return_value = "vs_1"
    mock_create_asst.return_value = "asst_1"
    empty = EnhanceData(results={}, updated_files=set())
    mock_analyze.return_value = empty
    mock_update.return_value = empty

    enhance_short_descriptions(["article.rst"])

    mock_ensure_vs.assert_called_once()
    mock_create_asst.assert_called_once()
    mock_analyze.assert_called_once()
    mock_update.assert_called_once()
    mock_cleanup.assert_called_once()
    res = mock_cleanup.call_args[0][1]
    assert res is not None
    assert res.assistant_id == "asst_1"
    assert res.vector_store_id == "vs_1"


@patch("enhance_topics.enhance_short_descriptions")
@patch("enhance_topics.enhance_metadata")
@patch("enhance_topics.get_openai_client")
def test_main_threads_accumulator_through_both_enhancements(
    mock_get_client,
    mock_metadata,
    mock_short_descriptions,
):
    """CLI entry point folds one EnhanceData through metadata then short description."""
    mock_get_client.return_value = MagicMock()
    empty = create_enhance_data()
    after_meta = EnhanceData(
        results={"topic.rst": {"description": "d"}},
        updated_files={"topic.rst"},
    )
    after_short = EnhanceData(
        results={"topic.rst": {"description": "d", "short-description": "s"}},
        updated_files={"topic.rst"},
    )

    def metadata_side_effect(files, client, data):
        assert data == empty
        return after_meta

    def short_side_effect(files, client, data):
        assert data == after_meta
        return after_short

    mock_metadata.side_effect = metadata_side_effect
    mock_short_descriptions.side_effect = short_side_effect

    with patch.object(sys, "argv", ["enhance_topics.py", "topic.rst"]):
        main()

    mock_get_client.assert_called_once()
    metrics = calculate_metrics(after_short)
    assert metrics.files_with_results_count == 1
    assert metrics.updated_files_count == 1
