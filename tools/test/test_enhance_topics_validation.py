import pytest
from unittest.mock import MagicMock
import sys
import os

# Add the tools directory to sys.path to allow importing enhance_topics
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from enhance_topics import validate_content

@pytest.fixture
def mock_client():
    """Provides a mocked OpenAI client."""
    return MagicMock()

def test_validate_content_success(mock_client):
    """Test that valid English content passes both moderation and language checks."""
    # Mock Moderation: Not flagged
    mock_result = MagicMock()
    mock_result.flagged = False
    mock_client.moderations.create.return_value.results = [mock_result]
    
    # Mock Responses API: Returns 'yes'
    mock_response = MagicMock()
    mock_response.status = "completed"
    mock_response.output_text = "yes"
    mock_client.responses.create.return_value = mock_response

    assert validate_content(mock_client, "This is a valid English sentence.") is True
    mock_client.responses.create.assert_called_once()

def test_validate_content_moderation_fail(mock_client):
    """Test that content flagged by moderation returns False."""
    # Mock Moderation: Flagged
    mock_result = MagicMock()
    mock_result.flagged = True
    # Mock categories.model_dump() for the logger
    mock_result.categories.model_dump.return_value = {"hate": True, "violence": False}
    mock_client.moderations.create.return_value.results = [mock_result]

    assert validate_content(mock_client, "Some offensive content.") is False
    mock_client.responses.create.assert_not_called()

def test_validate_content_language_fail(mock_client):
    """Test that non-English content (as determined by the LLM) returns False."""
    # Mock Moderation: Not flagged
    mock_result = MagicMock()
    mock_result.flagged = False
    mock_client.moderations.create.return_value.results = [mock_result]
    
    # Mock Responses API: Returns 'no'
    mock_response = MagicMock()
    mock_response.status = "completed"
    mock_response.output_text = "no"
    mock_client.responses.create.return_value = mock_response

    assert validate_content(mock_client, "Ceci n'est pas anglais.") is False

def test_validate_content_empty_input(mock_client):
    """Test that empty or whitespace-only input returns False immediately."""
    assert validate_content(mock_client, "") is False
    assert validate_content(mock_client, "   ") is False
    mock_client.moderations.create.assert_not_called()
