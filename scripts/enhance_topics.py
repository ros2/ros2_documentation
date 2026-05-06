import logging
import re
import sys
import os
from typing import Optional

from dotenv import load_dotenv
from openai import OpenAI, RateLimitError, APIConnectionError, OpenAIError
from tenacity import retry, stop_after_attempt, wait_random_exponential, retry_if_exception_type
from concurrent.futures import ThreadPoolExecutor

from enhance_data import EnhanceData, add_analysis_result, calculate_metrics, create_enhance_data, get_results_for_file, mark_file_updated
from rst_utils import get_meta_names_from_content, inject_metadata_to_content

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

KEYWORDS_PROMPT = """You are a content analyst, and your role is to analyze text content within supplied documents.

Your role is to extract 3 to 5 keywords from the content for use in metadata. The keywords should be single words that are the most important and relevant words to the content topic.

Finally, generate a comma-separated list of these keywords, in lowercase, with no additional styling, characters, or formatting."""

DESCRIPTION_PROMPT = """You are a content analyst, and your role is to analyze text content within supplied documents.

Your role is to create a concise description of the content for use in metadata. The description should be a single sentence (of a maximum of 130 characters) that captures the main idea of the content.

Finally, generate this description, with no additional styling, characters, or formatting."""

ENGLISH_LANGUAGE_CHECK_PROMPT = """You are a validation assistant, and your role is to determine whether the following text is written entirely in English. Common technical terms, acronyms, and internationally recognised proper nouns are acceptable if they are normally used in English technical documentation.

Answer ONLY with the single word yes or no in lowercase, with no punctuation, explanation, or additional text."""

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

    Intended for any model-generated text before it is persisted (metadata today; other content later).
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

def analyze_files(files: list[str], client: OpenAI, prompts: dict[str, str], timeout: int = DEFAULT_TIMEOUT) -> EnhanceData:
    """
    Process a list of files and analyse their content using each of the passed prompts.

    Args:
        files (list[str]): List of paths to files.
        client (OpenAI): OpenAI client instance.
        prompts (dict[str, str]): Dictionary of prompts for the AI model.
        timeout (int): Maximum time to wait for each API call in seconds.

    Returns:
        EnhanceData: Enhancement data structure containing analysis results and update tracking.
    """
    data = create_enhance_data()
    
    logger.debug("============================")
    logger.debug("Performing content analysis:")
    logger.debug("============================")

    for file_path in files: # Iterate through each file in the list
        logger.debug(f"Analysing file: {file_path}")

        # Read the content of the file
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
        except (OSError, PermissionError) as e:
            logger.error("Error reading file %s: %s", file_path, e)
            continue
        except UnicodeDecodeError as e:
            logger.error("Unicode decode error reading file %s: %s", file_path, e)
            continue

        # Check if the content is not empty
        if content.strip():
            # Check if the content has any meta fields already
            existing_meta_names = get_meta_names_from_content(content)
            for prompt_name, prompt in prompts.items():  # Iterate through each prompt in the dictionary
                if prompt_name in existing_meta_names:
                    logger.warning(
                        "Skipping analysis for %s: meta field %r already present in .. meta::",
                        file_path,
                        prompt_name,
                    )
                    continue
                logger.debug(f"Running analysis: {prompt_name}")
                try:
                    # Analyse the content using API with timeout and retry logic
                    result = analyze_content(
                        client, 
                        content, 
                        prompt,
                        timeout=timeout
                    )
                    if result:
                        if validate_content(client, result, timeout=timeout):
                            # Add the analysis result to the data structure
                            data = add_analysis_result(data, file_path, prompt_name, result)
                        else:
                            logger.warning(
                                "Validation failed for generated %s in %s; result not stored",
                                prompt_name,
                                file_path,
                            )
                    else:
                        logger.warning(f"No result for {file_path} with prompt name: {prompt_name}")

                except (RateLimitError, APIConnectionError) as e:
                    # Exhausted all retries due to rate limits or connection errors
                    logger.error(f"Failed to analyse {file_path} with prompt {prompt_name} after {MAX_RETRIES} retries: {e}")
                    continue
                except TimeoutError as e:
                    # Timeout error due to an individual API call timing out
                    logger.error(f"Analysis timed out for {file_path} with prompt {prompt_name}: {e}")
                    continue
                except (OpenAIError, ValueError) as e:
                    # Other API errors and value errors
                    logger.error(f"Failed to analyse {file_path} with prompt {prompt_name}: {e}")
                    continue
        else:
            logger.info(f"No analysable content found for {file_path}")

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
    prompts: dict[str, str] = {"description": DESCRIPTION_PROMPT, "keywords": KEYWORDS_PROMPT}

    data = analyze_files(files, client, prompts)  # Populate and validate ``EnhanceData.results`` from the model
    data = update_meta_files(files, data)  # Persist results as metadata fields and set ``updated_files``

    return data

def update_meta_files(files: list[str], data: EnhanceData) -> EnhanceData:
    """
    Process a list of files and update them with passed metadata.

    Args:
        files (list[str]): List of paths to files.
        data (EnhanceData): Enhancement data structure containing metadata for files.

    Returns:
        EnhanceData: Updated enhancement data with files marked as updated.
    """

    logger.debug("===========================")
    logger.debug("Updating metadata in files:")
    logger.debug("===========================")

    current_data = data  # Thread results through ``mark_file_updated`` immutably

    for file_path in files:
        logger.debug("Updating metadata in file: %s", file_path)
        metadata = get_results_for_file(current_data, file_path)

        # Confirm the metadata is not empty for the file, else skip
        if not metadata:
            logger.info("Skipping %s as it has no results for enhancement", file_path)
            continue

        logger.debug("Metadata found for %s, proceeding with updates.", file_path)

        try:
            with open(file_path, encoding="utf-8") as file:
                content = file.read()  # Full document; helpers locate or synthesise ``.. meta::``
        except (OSError, PermissionError) as exc:
            logger.error("Error reading file %s: %s", file_path, exc)
            continue
        except UnicodeDecodeError as exc:
            logger.error("Unicode decode error reading file %s: %s", file_path, exc)
            continue

        new_content, changed = inject_metadata_to_content(content, metadata)

        # Confirm that at least one metadata has been changed for the file, else skip
        if not changed:
            logger.debug("No metadata changes applied for %s", file_path)
            continue  # All keys already present or no additions—do not touch the file

        try:
            with open(file_path, "w", encoding="utf-8") as file:
                file.write(new_content)  # Full-document rewrite (same path as read)
        except (OSError, PermissionError) as exc:
            logger.error("Error writing file %s: %s", file_path, exc)
            continue
        except UnicodeEncodeError as exc:
            logger.error("Unicode encode error while writing file %s: %s", file_path, exc)
            continue

        current_data = mark_file_updated(current_data, file_path)  # Record success for metrics only after a clean write
        logger.debug("Updated file with supplied metadata: %s", file_path)
        logger.debug("-" * 50)

    # ``files_with_results_count`` reflects files with at least one valid analysis result, and ``updated_files_count`` reflects files we rewrote
    metrics = calculate_metrics(current_data)
    logger.info("Updated metadata in %s files out of %s files processed.", metrics.updated_files_count, len(files))
    return current_data

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
