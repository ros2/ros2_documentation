import logging
import re
import sys
import os
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable, Optional

from dotenv import load_dotenv
from openai import OpenAI, RateLimitError, APIConnectionError, OpenAIError
from tenacity import retry, stop_after_attempt, wait_random_exponential, retry_if_exception_type
from concurrent.futures import ThreadPoolExecutor

from config import (
    DEFAULT_TIMEOUT,
    DESCRIPTION_PROMPT,
    ENGLISH_LANGUAGE_CHECK_PROMPT,
    GPT_MODEL,
    KEYWORDS_PROMPT,
    MAX_CONTENT_LENGTH,
    MAX_RETRIES,
    MAX_WAIT,
    MIN_WAIT,
    RESPONSE_TIMEOUT,
    RST_EXTENSION,
    SHORT_DESCRIPTION_EXAMPLE_PATHS,
    SHORT_DESCRIPTION_PROMPT,
)
from enhance_data import (
    EnhanceData,
    add_analysis_result,
    calculate_metrics,
    create_enhance_data,
    get_results_for_file,
    mark_file_updated,
)
from openai_retrieval import (
    RetrievalResources,
    analyze_with_responses,
    cleanup_short_description_resources,
    ensure_example_vector_store,
)
from rst_utils import (
    get_meta_names_from_content,
    has_short_description_content,
    inject_metadata_to_content,
    inject_short_description_to_content,
)

logger = logging.getLogger(__name__)

# OpenAI SDK uses httpx; httpcore may also emit request-level INFO lines.
_QUIET_HTTP_LOGGERS = ("httpx", "httpcore")


def configure_logging() -> None:
    """Configure application logging and quiet noisy HTTP client libraries."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s %(asctime)s: %(message)s",
    )
    for name in _QUIET_HTTP_LOGGERS:
        logging.getLogger(name).setLevel(logging.WARNING)


@dataclass(frozen=True)
class AppliedContent:
    """RST body after applying analysis results, and whether it differs from the input."""

    content: str
    changed: bool


class ApplyHook(ABC):
    """Apply stored analysis results to an RST file body."""

    @abstractmethod
    def apply(self, content: str, results: dict[str, str]) -> AppliedContent:
        """Return updated content and whether the source was modified."""


@dataclass(frozen=True)
class MetadataApplyHook(ApplyHook):
    """Merge ``description`` / ``keywords`` results into ``.. meta::``."""

    def apply(self, content: str, results: dict[str, str]) -> AppliedContent:
        subset = {k: v for k, v in results.items() if k in ("description", "keywords")}
        if not subset:
            return AppliedContent(content=content, changed=False)
        new_content, changed = inject_metadata_to_content(content, subset)
        return AppliedContent(content=new_content, changed=changed)


@dataclass(frozen=True)
class ShortDescriptionApplyHook(ApplyHook):
    """Insert or fill ``.. short-description::`` from analysis results."""

    def apply(self, content: str, results: dict[str, str]) -> AppliedContent:
        val = results.get("short-description")
        if not val or not val.strip():
            return AppliedContent(content=content, changed=False)
        new_content, changed = inject_short_description_to_content(content, val)
        return AppliedContent(content=new_content, changed=changed)


@dataclass(frozen=True)
class EnhancementTask:
    """One analysable enhancement (metadata field or short description) applied per file."""

    key: str
    should_skip: Callable[[str], bool]
    analyze: Callable[[OpenAI, str, str, int], str]
    timeout: int = DEFAULT_TIMEOUT


def _metadata_enhancement_task(key: str, prompt: str) -> EnhancementTask:
    """Build a task that writes to ``.. meta::`` under the given field name."""

    def should_skip(content: str) -> bool:
        return key in get_meta_names_from_content(content)

    def analyze(cl: OpenAI, _file_path: str, content: str, to: int) -> str:
        return analyze_content(cl, content, prompt, timeout=to)

    return EnhancementTask(key=key, should_skip=should_skip, analyze=analyze, timeout=DEFAULT_TIMEOUT)


def _short_description_enhancement_task(vector_store_id: str) -> EnhancementTask:
    """Build a task that writes to the ``.. short-description::`` directive body."""

    def should_skip(content: str) -> bool:
        return has_short_description_content(content)

    def analyze(cl: OpenAI, file_path: str, _content: str, to: int) -> str:
        return analyze_with_responses(
            cl,
            vector_store_id,
            file_path,
            SHORT_DESCRIPTION_PROMPT,
            GPT_MODEL,
            timeout=to,
        )

    return EnhancementTask(
        key="short-description",
        should_skip=should_skip,
        analyze=analyze,
        timeout=RESPONSE_TIMEOUT,
    )


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
            logger.info("Calling OpenAI Chat Completions API for content analysis")
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
            logger.info(
                "Validating generated content via moderation and language APIs"
            )
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

        # Get the answer from the completion (should be yes or no)
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

def analyze_files(
    files: list[str],
    client: OpenAI,
    tasks: list[EnhancementTask],
    data: Optional[EnhanceData] = None,
) -> EnhanceData:
    """
    Process a list of files and analyse their content using each enhancement task.

    Args:
        files (list[str]): List of paths to files.
        client (OpenAI): OpenAI client instance.
        tasks (list[EnhancementTask]): Enhancement tasks to run per file.
        data (EnhanceData, optional): Accumulator for results; empty if omitted.

    Returns:
        EnhanceData: Enhancement data structure containing analysis results and update tracking.
    """
    acc = data if data is not None else create_enhance_data()

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
                logger.info(f"Skipping analysis for {file_path}: task {task.key} (already satisfies skip rule)")
           
                continue
            logger.debug("Running analysis: %s", task.key)
            try:
                logger.info("Analyzing content for %s: task %r", file_path, task.key)
                # Analyse the content using the task's analyze function
                result = task.analyze(client, file_path, content, task.timeout)
                if result:
                    # Validate the generated content
                    if validate_content(client, result, timeout=DEFAULT_TIMEOUT):
                        # Add the analysis result to the enhancement data
                        acc = add_analysis_result(acc, file_path, task.key, result)
                    else:
                        logger.warning(
                            "Validation failed for generated %s in %s; result not stored",
                            task.key,
                            file_path,
                        )
                else:
                    # Log a warning if no result was generated
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

    return acc


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


def update_enhanced_files(
    files: list[str],
    data: EnhanceData,
    apply_hook: ApplyHook,
    log_label: str,
) -> EnhanceData:
    """
    Process a list of files and apply an enhancement hook that may rewrite RST.

    The hook receives the file content and the per-file results dictionary, and
    returns ``AppliedContent``. The file is written when ``changed`` is true.
    """
    current_data = data

    for file_path in files:
        file_results = get_results_for_file(current_data, file_path)

        if not file_results:
            logger.info("Skipping %s as it has no results for enhancement", file_path)
            continue

        logger.info("Enhancing %s in file: %s", log_label, file_path)

        try:
            with open(file_path, encoding="utf-8") as file:
                content = file.read()
        except (OSError, PermissionError) as exc:
            logger.error("Error reading file %s: %s", file_path, exc)
            continue
        except UnicodeDecodeError as exc:
            logger.error("Unicode decode error reading file %s: %s", file_path, exc)
            continue

        applied = apply_hook.apply(content, file_results)

        if not applied.changed:
            logger.debug("No %s changes applied for %s", log_label, file_path)
            continue

        # Write the updated content to the file
        try:
            with open(file_path, "w", encoding="utf-8") as file:
                file.write(applied.content)
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


def enhance_metadata(
    files: list[str],
    client: Optional[OpenAI] = None,
    data: Optional[EnhanceData] = None,
) -> EnhanceData:
    """
    Enhance files with metadata based on content analysis.

    Args:
        files (list[str]): Paths to files to enhance.
        client (OpenAI, optional): OpenAI client instance. If None, creates new instance.
        data (EnhanceData, optional): Accumulator to extend (for multi-phase CLI runs).

    Returns:
        EnhanceData: Enhancement data structure containing analysis results and update tracking.
        
    Raises:
        OpenAIError: If no valid API key is found when creating a new client.
    """
    acc = data if data is not None else create_enhance_data()
    try:
        client = client or get_openai_client()
    except OpenAIError as e:
        logger.error(f"Failed to initialise OpenAI client: {e}")
        return acc
    
    # Create the list of enhancement tasks for the metadata analysis
    tasks = [
        _metadata_enhancement_task("description", DESCRIPTION_PROMPT),
        _metadata_enhancement_task("keywords", KEYWORDS_PROMPT),
    ]

    acc = analyze_files(files, client, tasks, acc)
    return update_meta_files(files, acc)


def enhance_short_descriptions(
    files: list[str],
    client: Optional[OpenAI] = None,
    data: Optional[EnhanceData] = None,
) -> EnhanceData:
    """
    Enhance RST files with a ``.. short-description::`` body using the Responses API.

    Example articles are taken from ``SHORT_DESCRIPTION_EXAMPLE_PATHS`` (indexed once per run
    into a vector store for ``file_search``). Each target file is uploaded with the Files API
    and referenced as ``input_file``; the vector store is deleted afterwards.

    Args:
        files: Paths to RST files to enhance.
        client: Optional pre-built OpenAI client.
        data: Optional accumulator to extend (for multi-phase CLI runs).

    Returns:
        ``EnhanceData`` with results under the key ``short-description`` and ``updated_files`` set
        after successful writes.
    """
    acc = data if data is not None else create_enhance_data()
    try:
        client = client or get_openai_client()
    except OpenAIError as e:
        logger.error("Failed to initialise OpenAI client: %s", e)
        return acc

    resources: RetrievalResources | None = None
    try:
        vector_store_id = ensure_example_vector_store(client, SHORT_DESCRIPTION_EXAMPLE_PATHS)
        resources = RetrievalResources(vector_store_id)

        tasks = [_short_description_enhancement_task(vector_store_id)]
        acc = analyze_files(files, client, tasks, acc)
        return update_enhanced_files(
            files,
            acc,
            ShortDescriptionApplyHook(),
            "short description",
        )
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
    return update_enhanced_files(files, data, MetadataApplyHook(), "metadata")

def main() -> None:
    """
    Main entry point for the script.

    - Parses command-line arguments to collect input file paths.
    - Filters the provided files to include only reStructuredText (.rst) files.
    - Enhances ``.. meta::`` fields and ``.. short-description::`` bodies.
    - Writes updates back to files and logs a single combined metrics summary.

    Usage:
        python enhance_topics.py <rst_file1> <rst_file2> ...

    Only files with the .rst extension will be processed.
    """
    
    configure_logging()

    # Collect filenames from command line arguments and filter for RST files
    input_files = sys.argv[1:]
    rst_files = [f for f in input_files if f.lower().endswith(RST_EXTENSION)]

    if not rst_files:
        if input_files:
            logger.info("No RST files found among provided arguments. Skipping enhancement.")
        else:
            logger.error("No input files provided. Pass a list of RST files as arguments.")
        sys.exit(0)
    
    # Get the OpenAI client and create the enhancement data
    try:
        client = get_openai_client()
    except OpenAIError as e:
        logger.error("Failed to initialise OpenAI client: %s", e)
        data = create_enhance_data()
    else:
        data = create_enhance_data()
        data = enhance_metadata(rst_files, client, data)
        data = enhance_short_descriptions(rst_files, client, data)

    metrics = calculate_metrics(data)
    logger.info(
        "Enhanced files: %s with at least one valid analysis result, and %s files updated, "
        "out of %s RST files.",
        metrics.files_with_results_count,
        metrics.updated_files_count,
        len(rst_files),
    )

if __name__ == "__main__":
    main()
