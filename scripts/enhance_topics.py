import logging
import re
import sys
import os
from dataclasses import dataclass
from typing import Callable, Optional

from dotenv import load_dotenv
from openai import OpenAI, RateLimitError, APIConnectionError, OpenAIError
from tenacity import retry, stop_after_attempt, wait_random_exponential, retry_if_exception_type
from concurrent.futures import ThreadPoolExecutor

from enhance_data import (
    EnhanceData,
    add_analysis_result,
    calculate_metrics,
    create_enhance_data,
    get_results_for_file,
    mark_file_updated,
)
from openai_retrieval import (
    ASSISTANT_RUN_TIMEOUT,
    RetrievalResources,
    analyze_with_file_search,
    cleanup_short_description_resources,
    create_short_description_assistant,
    ensure_example_vector_store,
)
from rst_utils import (
    get_meta_names_from_content,
    has_short_description_content,
    inject_metadata_to_content,
    inject_short_description_to_content,
)

logger = logging.getLogger(__name__)

# Define constants
GPT_MODEL = "gpt-5.4-nano" # GPT model to use for the API calls
# Maximum content length in characters, approximately 300k tokens (leaving 100k for instructions/output)
MAX_CONTENT_LENGTH = 1200000
RST_EXTENSION = '.rst' # File extension for RST files

# Define timeout and retry parameters for API calls
# - Individual API calls timeout after DEFAULT_TIMEOUT seconds
# - On rate limits/connection errors, retry up to MAX_RETRIES times
# - Wait between retries, increasing exponentially: MIN_WAIT → MAX_WAIT (capped)
DEFAULT_TIMEOUT = 30  # Default timeout in seconds for an individual API call
MAX_RETRIES = 10     # Maximum number of retry attempts for exponential backoff
MIN_WAIT = 10        # Minimum wait time between retries in seconds
MAX_WAIT = 120        # Maximum wait time between retries in seconds

# Example RST paths (relative to repository root) indexed into the vector store for file_search
SHORT_DESCRIPTION_EXAMPLE_PATHS = [
    "source/About-ROS.rst",
]

# Define prompts for the AI model

SHORT_DESCRIPTION_PROMPT = """You are a Technical Author in the technology industry working on documenting a robotics product, and your role is to analyze RST content within supplied documents. 
You'll then create new content based on this analysis for a new draft article, which I can use to supplement that article.

## Examples
Use file_search to read through the following RST files in their entirety as examples of completed articles:

- About-ROS.rst
- First-Steps.rst
- Interfaces-Topics-Services-Actions.rst

## Short Description
For each article in this set of examples, analyse the content associated with the "short-description" directive, and what it constitutes in relation to the article it describes. For example, in the First-Steps article, the 3 sentences which begin as follows comprise the specified short description:

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

@dataclass(frozen=True)
class EnhancementTask:
    """One analysable enhancement (metadata field or short description) applied per file."""

    key: str
    should_skip: Callable[[str], bool]
    analyze: Callable[[OpenAI, str, int], str]
    timeout: int = DEFAULT_TIMEOUT


def _metadata_enhancement_task(key: str, prompt: str) -> EnhancementTask:
    """Build a task that writes to ``.. meta::`` under the given field name."""

    def should_skip(content: str) -> bool:
        return key in get_meta_names_from_content(content)

    def analyze(cl: OpenAI, content: str, to: int) -> str:
        return analyze_content(cl, content, prompt, timeout=to)

    return EnhancementTask(key=key, should_skip=should_skip, analyze=analyze, timeout=DEFAULT_TIMEOUT)


def _short_description_enhancement_task(assistant_id: str) -> EnhancementTask:
    """Build a task that writes to the ``.. short-description::`` directive body."""

    def analyze(cl: OpenAI, content: str, to: int) -> str:
        return analyze_with_file_search(cl, assistant_id, content, timeout=to)

    def should_skip(content: str) -> bool:
        return has_short_description_content(content)

    return EnhancementTask("short-description", should_skip, analyze, ASSISTANT_RUN_TIMEOUT)


@retry(
    retry=retry_if_exception_type((RateLimitError, APIConnectionError)),
    stop=stop_after_attempt(MAX_RETRIES),
    wait=wait_random_exponential(multiplier=MIN_WAIT, max=MAX_WAIT),
    reraise=True
)
def analyze_content(client: OpenAI, content: str, prompt: str, timeout: int = DEFAULT_TIMEOUT) -> str:
    """
    Analyse content using OpenAI's API with retry and timeout logic.
    Uses ThreadPoolExecutor for cross-platform timeout handling and retries for transient API errors.

    Args:
        client (OpenAI): OpenAI client instance.
        content (str): Preprocessed content.
        prompt (str): Prompt for the AI model.
        timeout (int): Maximum time to wait for response in seconds.

    Returns:
        str: Analysis result from the AI model, or empty string if analysis fails.

    Raises:
        TimeoutError: If the API call exceeds the specified timeout.
        RateLimitError: If API rate limits are exceeded (will trigger retry).
        APIConnectionError: If connection fails (will trigger retry).
    """
    # Log the content length before potential truncation
    logger.debug(f"Processing content of length: {len(content)} characters")
    
    # Truncate content if its too long
    if len(content) > MAX_CONTENT_LENGTH:
        logger.warning(f"Content truncated to {MAX_CONTENT_LENGTH} characters for analysis.")
        content = content[:MAX_CONTENT_LENGTH]
    
    def _make_api_call() -> str:
        """
        Inner function to handle the OpenAI API call.
        Separated to allow for clean timeout handling via ThreadPoolExecutor.
        
        Returns:
            str: The model's response content
            
        Raises:
            RateLimitError, APIConnectionError: Propagated for retry handling
        """
        try:
            logger.debug("Sending request to OpenAI API...")
            completion = client.chat.completions.create(
                model=GPT_MODEL,
                messages=[
                    {"role": "system", "content": prompt},
                    {"role": "user", "content": f"Content:\n\n{content}"}
                ]
            )
            result = completion.choices[0].message.content
            logger.debug("Successfully received response from OpenAI API")
            return result if result is not None else ""
        except (RateLimitError, APIConnectionError) as e:
            logger.warning(f"Retryable error occurred: {str(e)}")
            raise  # Re-raise for retry decorator to handle

    # Use ThreadPoolExecutor for cross-platform timeout handling
    with ThreadPoolExecutor() as executor:
        try:
            future = executor.submit(_make_api_call)
            return future.result(timeout=timeout)
        except TimeoutError:
            logger.error(f"API call timed out after {timeout} seconds")
            raise  # Re-raise the original timeout error

@retry(
    retry=retry_if_exception_type((RateLimitError, APIConnectionError)),
    stop=stop_after_attempt(MAX_RETRIES),
    wait=wait_random_exponential(multiplier=MIN_WAIT, max=MAX_WAIT),
    reraise=True
)
def validate_content(client: OpenAI, generated: str, timeout: int = DEFAULT_TIMEOUT) -> bool:
    """
    Validate generated content using the moderation API and a separate English-language check.

    Intended for any model-generated text before it is persisted.
    Uses ThreadPoolExecutor for cross-platform timeout handling and retries for transient API errors.

    Args:
        client (OpenAI): OpenAI client instance.
        generated (str): Model-generated text to validate.
        timeout (int): Maximum time to wait for the combined validation calls in seconds.

    Returns:
        bool: True if content passes moderation and the language check; False otherwise.

    Raises:
        TimeoutError: If the validation calls exceed the specified timeout.
        RateLimitError: If API rate limits are exceeded (will trigger retry).
        APIConnectionError: If connection fails (will trigger retry).
    """
    if not generated.strip():
        logger.debug("Validation skipped: empty generated content")
        return False

    text = generated
    if len(text) > MAX_CONTENT_LENGTH:
        logger.warning(
            "Generated text truncated to %s characters for validation.",
            MAX_CONTENT_LENGTH,
        )
        text = text[:MAX_CONTENT_LENGTH]

    def _run_validation() -> bool:
        """
        Run moderation and English checks sequentially.

        Returns:
            bool: True if both checks pass.

        Raises:
            RateLimitError, APIConnectionError: Propagated for retry handling.
        """
        try:
            logger.debug("Sending generated text to moderation API...")
            moderation = client.moderations.create(input=text)
        except (RateLimitError, APIConnectionError) as e:
            logger.warning("Retryable error during moderation: %s", e)
            raise

        if not moderation.results:
            logger.warning("Moderation API returned no results; treating as validation failure")
            return False

        result0 = moderation.results[0]
        if result0.flagged:
            categories = [
                name
                for name, flagged in result0.categories.model_dump().items()
                if flagged
            ]
            logger.warning(
                "Content failed moderation (flagged). Categories: %s",
                ", ".join(categories) if categories else "unknown",
            )
            return False

        try:
            logger.debug("Sending generated text for English-language validation...")
            completion = client.chat.completions.create(
                model=GPT_MODEL,
                messages=[
                    {"role": "system", "content": ENGLISH_LANGUAGE_CHECK_PROMPT},
                    {"role": "user", "content": f"Text:\n\n{text}"},
                ],
            )
        except (RateLimitError, APIConnectionError) as e:
            logger.warning("Retryable error during language validation: %s", e)
            raise

        answer = completion.choices[0].message.content
        raw = (answer or "").strip().lower()
        # Accept a single leading yes/no token even if the model adds stray whitespace
        match = re.match(r"^(yes|no)\b", raw)
        if not match or match.group(1) != "yes":
            logger.warning(
                "Content failed English-language validation (model answer: %r)",
                answer,
            )
            return False

        logger.debug("Generated content passed moderation and English-language validation")
        return True

    with ThreadPoolExecutor() as executor:
        try:
            future = executor.submit(_run_validation)
            return future.result(timeout=timeout)
        except TimeoutError:
            logger.error("Validation timed out after %s seconds", timeout)
            raise

def analyze_files(files: list[str], client: OpenAI, tasks: list[EnhancementTask]) -> EnhanceData:
    """
    Process a list of files and analyse their content using each enhancement task.

    Args:
        files (list[str]): List of paths to files.
        client (OpenAI): OpenAI client instance.
        tasks (list[EnhancementTask]): Enhancement tasks to run per file.

    Returns:
        EnhanceData: Enhancement data structure containing analysis results and update tracking.
    """
    data = create_enhance_data()

    logger.debug("============================")
    logger.debug("Performing content analysis:")
    logger.debug("============================")

    for file_path in files:  # Iterate through each file in the list
        logger.debug("Analysing file: %s", file_path)

        # Read the content of the file
        try:
            with open(file_path, encoding="utf-8") as f:
                content = f.read()
        except (OSError, PermissionError) as e:
            logger.error("Error reading file %s: %s", file_path, e)
            continue
        except UnicodeDecodeError as e:
            logger.error("Unicode decode error reading file %s: %s", file_path, e)
            continue

        # Check if the content is not empty
        if not content.strip():
            logger.info("No analysable content found for %s", file_path)
            continue

        # Iterate through each task and run the analysis
        for task in tasks:
            if task.should_skip(content):
                logger.warning(
                    "Skipping analysis for %s: task %r (content already satisfies skip rule)",
                    file_path,
                    task.key,
                )
                continue
            logger.debug("Running analysis: %s", task.key)
            try:
                result = task.analyze(client, content, task.timeout)
                if result:
                    if validate_content(client, result, timeout=DEFAULT_TIMEOUT):
                        data = add_analysis_result(data, file_path, task.key, result)
                    else:
                        logger.warning(
                            "Validation failed for generated %s in %s; result not stored",
                            task.key,
                            file_path,
                        )
                else:
                    logger.warning("No result for %s with task %r", file_path, task.key)

            except (RateLimitError, APIConnectionError) as e:
                logger.error("Failed to analyse %s with task %r after %s retries: %s", file_path, task.key, MAX_RETRIES, e)
                continue
            except TimeoutError as e:
                logger.error("Analysis timed out for %s with task %r: %s", file_path, task.key, e)
                continue
            except (OpenAIError, ValueError) as e:
                logger.error("Failed to analyse %s with task %r: %s", file_path, task.key, e)
                continue

    return data


def get_openai_client() -> OpenAI:
    """
    Create an OpenAI client with proper authentication.
    
    The API key is sourced in the following order:
    1. Environment variable OPENAI_API_KEY
    2. .env file in the project root
    
    Returns:
        OpenAI: Authenticated OpenAI client instance
        
    Raises:
        AuthenticationError: If no valid API key is found
    """
    # Load environment variables from .env file if present
    load_dotenv()
    
    api_key = os.environ.get("OPENAI_API_KEY")
    if not api_key:
        raise OpenAIError("OpenAI API key not found. Set OPENAI_API_KEY environment variable.")
        
    return OpenAI(api_key=api_key)


def _apply_metadata_results(content: str, results: dict[str, str]) -> tuple[str, bool]:
    """Merge ``description`` / ``keywords`` results into ``.. meta::``."""
    # Create a subset of the results dictionary containing only the description and keywords
    subset = {k: v for k, v in results.items() if k in ("description", "keywords")}
    if not subset:
        return content, False
    return inject_metadata_to_content(content, subset)


def _apply_short_description_results(content: str, results: dict[str, str]) -> tuple[str, bool]:
    """Insert or fill ``.. short-description::`` from analysis results."""
    # Get the short description result from the results dictionary
    val = results.get("short-description")
    # If the short description is not found or is empty, return the content and False
    if not val or not val.strip():
        return content, False
    return inject_short_description_to_content(content, val)


def update_enhanced_files(
    files: list[str],
    data: EnhanceData,
    apply_hooks: list[Callable[[str, dict[str, str]], tuple[str, bool]]],
    log_label: str,
) -> EnhanceData:
    """
    Process a list of files and apply enhancement hooks that may rewrite RST.

    Each hook receives the current file content and the per-file results dictionary,
    and returns ``(new_content, changed)``. Hooks run in order; the file is written
    once if any hook reported a change.
    """
    logger.debug("===========================")
    logger.debug("Updating %s in files:", log_label)
    logger.debug("===========================")

    current_data = data

    for file_path in files:
        logger.debug("Updating %s in file: %s", log_label, file_path)
        file_results = get_results_for_file(current_data, file_path)

        if not file_results:
            logger.info("Skipping %s as it has no results for enhancement", file_path)
            continue

        logger.debug("Results found for %s, proceeding with updates.", file_path)

        try:
            with open(file_path, encoding="utf-8") as file:
                content = file.read()
        except (OSError, PermissionError) as exc:
            logger.error("Error reading file %s: %s", file_path, exc)
            continue
        except UnicodeDecodeError as exc:
            logger.error("Unicode decode error reading file %s: %s", file_path, exc)
            continue

        # Apply the enhancement hooks to the content
        working = content
        changed_any = False
        # Iterate through each hook and apply it to the content
        for hook in apply_hooks:
            working, changed = hook(working, file_results)
            changed_any = changed_any or changed

        # If no changes were made, log a message and continue
        if not changed_any:
            logger.debug("No %s changes applied for %s", log_label, file_path)
            continue

        # Write the updated content to the file
        try:
            with open(file_path, "w", encoding="utf-8") as file:
                file.write(working)
        except (OSError, PermissionError) as exc:
            logger.error("Error writing file %s: %s", file_path, exc)
            continue
        except UnicodeEncodeError as exc:
            logger.error("Unicode encode error while writing file %s: %s", file_path, exc)
            continue

        # Mark the file as updated in the enhancement data
        current_data = mark_file_updated(current_data, file_path)
        logger.debug("Updated file with %s: %s", log_label, file_path)
        logger.debug("-" * 50)

    metrics = calculate_metrics(current_data)
    logger.info(
        "Updated %s in %s files out of %s files processed.",
        log_label,
        metrics.updated_files_count,
        len(files),
    )
    return current_data


def enhance_metadata(files: list[str], client: Optional[OpenAI] = None) -> EnhanceData:
    """
    Enhance files with metadata based on content analysis.

    Args:
        files (list[str]): Paths to files to enhance.
        client (OpenAI, optional): OpenAI client instance. If None, creates new instance.

    Returns:
        EnhanceData: Enhancement data structure containing analysis results and update tracking.
        
    Raises:
        OpenAIError: If no valid API key is found when creating a new client.
    """
    try:
        client = client or get_openai_client()
    except OpenAIError as e:
        logger.error(f"Failed to initialise OpenAI client: {e}")
        return create_enhance_data()
    
    # TODO: Make this config-driven, so that we can easily add more prompts and analysis types
    tasks = [
        _metadata_enhancement_task("description", DESCRIPTION_PROMPT),
        _metadata_enhancement_task("keywords", KEYWORDS_PROMPT),
    ]

    data = analyze_files(files, client, tasks)
    data = update_meta_files(files, data)

    return data


def enhance_short_descriptions(files: list[str], client: Optional[OpenAI] = None) -> EnhanceData:
    """
    Enhance RST files with a ``.. short-description::`` body using an assistant with file_search.

    Example articles are taken from ``SHORT_DESCRIPTION_EXAMPLE_PATHS`` (indexed once per run).
    Each target file is sent in its own thread; the vector store and assistant are deleted
    afterwards. Not wired to ``main()``; import and call from a REPL or another script.

    Args:
        files: Paths to RST files to enhance.
        client: Optional pre-built OpenAI client.

    Returns:
        ``EnhanceData`` with results under the key ``short-description`` and ``updated_files`` set
        after successful writes.
    """
    try:
        client = client or get_openai_client()
    except OpenAIError as e:
        logger.error("Failed to initialise OpenAI client: %s", e)
        return create_enhance_data()

    resources: RetrievalResources | None = None
    try:
        vector_store_id = ensure_example_vector_store(client, SHORT_DESCRIPTION_EXAMPLE_PATHS)
        assistant_id = create_short_description_assistant(
            client,
            vector_store_id,
            SHORT_DESCRIPTION_PROMPT,
            GPT_MODEL,
        )
        resources = RetrievalResources(assistant_id, vector_store_id)

        tasks = [_short_description_enhancement_task(assistant_id)]
        data = analyze_files(files, client, tasks)
        data = update_enhanced_files(
            files,
            data,
            [_apply_short_description_results],
            "short description",
        )
        return data
    finally:
        cleanup_short_description_resources(client, resources)


def update_meta_files(files: list[str], data: EnhanceData) -> EnhanceData:
    """
    Process a list of files and update them with passed metadata (``.. meta::`` fields).

    Args:
        files (list[str]): List of paths to files.
        data (EnhanceData): Enhancement data structure containing metadata for files.

    Returns:
        EnhanceData: Updated enhancement data with files marked as updated.
    """
    return update_enhanced_files(files, data, [_apply_metadata_results], "metadata")

def main() -> None:
    """
    Main entry point for the script.

    - Parses command-line arguments to collect input file paths.
    - Filters the provided files to include only reStructuredText (.rst) files.
    - Enhances the metadata of each RST file using AI-based analysis (keywords and description).
    - Writes updated metadata back to files and logs processing metrics.

    Usage:
        python enhance_topics.py <rst_file1> <rst_file2> ...

    Only files with the .rst extension will be processed. 
    Logs the number of files successfully enhanced.
    """
    
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s %(name)s: %(message)s",
    )

    # Collect filenames from command line arguments and filter for RST files
    input_files = sys.argv[1:]
    rst_files = [f for f in input_files if f.lower().endswith(RST_EXTENSION)]

    if not rst_files:
        if input_files:
            logger.info("No RST files found among provided arguments. Skipping enhancement.")
        else:
            logger.error("No input files provided. Pass a list of RST files as arguments.")
        sys.exit(0)
    
    # Enhance the metadata in the RST files and return the enhancement data with updated files
    data = enhance_metadata(rst_files)
    # Log the metrics for the enhancement data
    metrics = calculate_metrics(data)
    logger.info(f"Enhanced files: {metrics.files_with_results_count} with at least one valid analysis result, and {metrics.updated_files_count} files updated, out of {len(rst_files)} RST files.")

if __name__ == "__main__":
    main()
