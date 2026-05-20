"""
Central configuration for the enhancement scripts.

Holds tuning constants and prompt strings used by ``enhance_topics`` and
``openai_retrieval``. Kept as a leaf module (no imports from sibling scripts)
so it can be imported freely without risk of circular dependencies.
"""

# Define constants
GPT_MODEL = "gpt-5.4-nano" # GPT model to use for the API calls
# Maximum content length in characters, approximately 300k tokens (leaving 100k for instructions/output)
MAX_CONTENT_LENGTH = 1_200_000
RST_EXTENSION = '.rst' # File extension for RST files

# Define timeout and retry parameters for API calls
# - Individual API calls timeout after DEFAULT_TIMEOUT seconds
# - On rate limits/connection errors, retry up to MAX_RETRIES times
# - Wait between retries, increasing exponentially: MIN_WAIT → MAX_WAIT (capped)
DEFAULT_TIMEOUT = 30  # Default timeout in seconds for an individual API call
MAX_RETRIES = 10     # Maximum number of retry attempts for exponential backoff
MIN_WAIT = 10        # Minimum wait time between retries in seconds
MAX_WAIT = 120        # Maximum wait time between retries in seconds

# Assistant-run tuning (used by openai_retrieval)
# Maximum time for one assistant run (thread message + run + polling)
ASSISTANT_RUN_TIMEOUT = 120
# Interval between run status polls
ASSISTANT_POLL_INTERVAL = 1.5

# Example RST paths (relative to repository root) indexed into the vector store for file_search
SHORT_DESCRIPTION_EXAMPLE_PATHS = [
    "source/About-ROS.rst",
]

# Define prompts for the AI model

SHORT_DESCRIPTION_PROMPT = """You are a technical author, and your role is to analyze RST content within supplied documents, and then create new, supplementary content for a new draft article based on this analysis.

## Examples
Use file_search to read through the following RST files in their entirety as examples of completed articles:

- About-ROS.rst
- First-Steps.rst
- Interfaces-Topics-Services-Actions.rst

## Short Description
For each article in this set of examples, analyse the content associated with the "short-description" directive, and what it constitutes in relation to the article it describes. 
For example, in the First-Steps article, the 3 sentences which begin as follows comprise the specified short description:

* "Interfaces in ROS..."
* "This article explains the..."
* "With this information..."

This short description content does not include the single line of text commencing with "**Area...", or the "contents" (Table of Contents) directive.

When you have identified the short description in all example articles, remember the formatting and how the paragraph is constructed, including tone/style and length. We call this the article Short Description.

Finally, generate the short description for the new article given in the user message, with no additional styling, characters, or formatting.
"""

KEYWORDS_PROMPT = """You are a content analyst, and your role is to analyze text content within supplied documents.

Your role is to extract 3 to 5 keywords from the content for use in metadata. The keywords should be single words that are the most important and relevant words to the content topic.

Finally, generate a comma-separated list of these keywords, in lowercase, with no additional styling, characters, or formatting."""

DESCRIPTION_PROMPT = """You are a content analyst, and your role is to analyze text content within supplied documents.

Your role is to create a concise description of the content for use in metadata. The description should be a single sentence (of a maximum of 130 characters) that captures the main idea of the content.

Finally, generate this description, with no additional styling, characters, or formatting."""

ENGLISH_LANGUAGE_CHECK_PROMPT = """You are a validation assistant, and your role is to determine whether the following text is written entirely in English. Common technical terms, acronyms, and internationally recognised proper nouns are acceptable if they are normally used in English technical documentation.

Answer ONLY with the single word yes or no in lowercase, with no punctuation, explanation, or additional text."""
