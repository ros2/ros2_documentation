# Enhancement Tools

This directory contains a command-line tool designed to automatically enhance the ROS documentation (`.rst` files) with metadata and descriptive content, using Sphinx directives.

## Overview

The primary tool (`enhance_topics.py`) uses OpenAI models and APIs to analyse articles and inject:
1.  **SEO Metadata**: `description`, `keywords`, `distribution`, and `product` fields within a `.. meta::` directive. The first two are model-generated; `distribution` and `product` are fixed placeholders (`{DISTRO}` and `{PRODUCT}`) for substitution by the Sphinx build.
2.  **Short Descriptions**: A concise descriptive paragraph injected into the custom `.. short-description::` directive, using Retrieval-Augmented Generation (RAG) to match the style required by the information model.

Both model-generated metadata (`description`, `keywords`) and short descriptions are produced via OpenAI's **Responses API**, with the article uploaded through the **Files API**. Short descriptions additionally attach a vector store for `file_search` over example articles.

## Code layout

- **`enhance_topics.py`**
  - CLI entry point and orchestration
  - `EnhancementTask` / `ApplyHook` definitions
  - Metadata analysis (`analyze_content`)
  - Shared `analyze_files` and validation
- **`openai_retrieval.py`**
  - Vector-store setup
  - Short-description Responses calls (`analyze_with_responses`)
  - Cleanup of hosted retrieval resources
- **`enhance_data.py`**
  - `EnhanceData` accumulator, per-file results, and metrics
- **`rst_utils.py`**
  - Regex-based read/write of `.. meta::` and `.. short-description::` in RST source
- **`config.py`**
  - Model name, timeouts, retries, prompts, and example paths for RAG
  - Leaf module (no imports from sibling scripts)

## Configuration

Tuning constants and prompt strings live in `tools/config.py`. Authentication is **not** configured here: `OPENAI_API_KEY` is read from the environment or a `.env` file in the repository root by `get_openai_client()` in `enhance_topics.py`.

### Model and input limits

- **`GPT_MODEL`** (default: `gpt-5.4-nano`)
  - Used by: `enhance_topics` (metadata analysis, validation, short-description task)
  - Purpose: model for all Responses API calls
- **`MAX_CONTENT_LENGTH`** (default: `1_200_000`)
  - Used by: `validate_content`
  - Purpose: truncate generated text before moderation and language checks (~300k tokens)
- **`RST_EXTENSION`** (default: `.rst`)
  - Used by: `main()`
  - Purpose: filter CLI arguments to reStructuredText files only

### Timeouts and retries

- **`DEFAULT_TIMEOUT`** (default: `30` seconds)
  - Used by: metadata `analyze_content`, `validate_content`, metadata tasks
  - Purpose: wall-clock limit per individual API call
- **`RESPONSE_TIMEOUT`** (default: `120` seconds)
  - Used by: short-description task, `openai_retrieval`
  - Purpose: wall-clock limit for one short-description Responses call (includes file reference)
- **`MAX_RETRIES`** (default: `10`)
  - Used by: `analyze_content`, `validate_content`, `openai_retrieval`
  - Purpose: retry attempts on rate limits and connection errors (`tenacity`)
- **`MIN_WAIT`** (default: `10` seconds)
  - Used by: same as retry constants above
  - Purpose: minimum wait between retries (exponential backoff base)
- **`MAX_WAIT`** (default: `120` seconds)
  - Used by: same as retry constants above
  - Purpose: maximum wait between retries (backoff cap)

### RAG example articles

- **`SHORT_DESCRIPTION_EXAMPLE_PATHS`**
  - Used by: `enhance_short_descriptions`, `SHORT_DESCRIPTION_PROMPT`
  - Purpose: paths relative to the repository root; uploaded once per run into an OpenAI vector store for `file_search` when generating short descriptions

To change style or tone guidance for short descriptions, update this list and/or `SHORT_DESCRIPTION_PROMPT` together so the prompt’s example list stays in sync.

### Prompts

- **`DESCRIPTION_PROMPT`**
  - Used by: Phase 1 (`description` task)
  - Purpose: instructions for the meta `description` field (single sentence, max 130 characters)
- **`KEYWORDS_PROMPT`**
  - Used by: Phase 1 (`keywords` task)
  - Purpose: instructions for comma-separated lowercase keywords (3–5 words)
- **`SHORT_DESCRIPTION_PROMPT`**
  - Used by: Phase 2 (`analyze_with_responses`)
  - Purpose: instructions for short-description prose; references example paths via `file_search`
- **`ENGLISH_LANGUAGE_CHECK_PROMPT`**
  - Used by: `validate_content`
  - Purpose: yes/no check that model output is English before results are stored

Fixed metadata placeholders (`{DISTRO}`, `{PRODUCT}`) are defined in `enhance_topics.py`, not in `config.py`.

## Orchestration Logic

The execution follows a top-to-bottom flow through several key layers:

### 1. Entry Point: `main()`
The execution starts in the `main()` function, which handles the high-level setup:
- **Logging**: Configures standard logging and silences noisy HTTP libraries (`httpx`, `httpcore`).
- **Argument Parsing**: Collects file paths from the command line and filters for `.rst` files.
- **Client Setup**: Initialises the `OpenAI` client by loading the API key from environment variables or a `.env` file in the repository root.
- **Orchestration**: Executes the two main enhancement phases: `enhance_metadata()` and `enhance_short_descriptions()`.
- **Metrics**: Calculates and logs a summary of how many files were processed, how many had valid results, and how many were actually updated.

### 2. Phase 1: Metadata Enhancement (`enhance_metadata`)
This phase focuses on the `.. meta::` block:
- **Task Definition**: Creates four `EnhancementTask` objects—`description` and `keywords` (model-generated via prompts), and `distribution` and `product` (fixed values `{DISTRO}` and `{PRODUCT}` with no API call).
- **Skip Logic**: Each task includes a `should_skip` check that reads the file content to see if that metadata field name already exists in the `.. meta::` block.
- **Analysis**: Calls `analyze_files()`, which uploads the file to OpenAI and runs analysis. Model-generated fields use the Responses API (`analyze_content`); fixed fields return their placeholder values and skip content validation.
- **Application**: Calls `update_meta_files()`, which uses the `MetadataApplyHook` to merge the new metadata into the existing (or new) `.. meta::` block in the RST file.

### 3. Phase 2: Short Description Enhancement (`enhance_short_descriptions`)
This phase uses Retrieval-Augmented Generation (RAG), implemented in `openai_retrieval.py`:
- **Vector Store Setup**: Uploads a set of "gold standard" example RST files to an OpenAI Vector Store. This allows the model to "search" for examples of good short descriptions to match the project's style.
- **Task Definition**: Creates a task for `short-description`.
- **Skip Logic**: Skips if the file already has a non-empty `.. short-description::` body (an empty directive is still enhanced).
- **Analysis**: Calls `analyze_files()`, where the analysis function (`analyze_with_responses`) includes the `vector_store_id` to enable the file search capability.
- **Application**: Uses the `ShortDescriptionApplyHook` to inject the generated text into the file.
- **Cleanup**: A `finally` block ensures the temporary vector store and hosted files are deleted from OpenAI.

### 4. Core Processing Engines

#### `analyze_files`
This is the central engine for interacting with the AI:
1. **File Upload**: Uploads the target RST file to OpenAI's Files API.
2. **Task Execution**: For every task that isn't skipped:
   - Calls the task's `analyze` function (metadata and short-description analysers use `tenacity` retries on transient API errors).
   - **Validation**: If a result is returned, runs `validate_content()` (moderation and English-language checks) unless the task sets `skip_validation` (fixed metadata placeholders).
3. **Storage**: Valid results are stored in an `EnhanceData` object (`enhance_data.py`).
4. **Cleanup**: Deletes the uploaded file from OpenAI.

#### `update_enhanced_files`
This handles the file I/O and content modification:
1. **Hook Application**: Passes the file content and the AI results to an `ApplyHook` (either `MetadataApplyHook` or `ShortDescriptionApplyHook`).
2. **Regex Injection**: The hooks use utility functions (from `rst_utils.py`) to perform precise regex-based injection of the new content.
3. **Persistence**: If the content has changed, it overwrites the file and marks it as "updated" in the metrics.

### 5. Key Abstractions
- **`EnhancementTask`**: Bundles the "what" (key), "when to skip" (logic), and "how to analyse" (API call or fixed value). Defined in `enhance_topics.py`.
- **`ApplyHook`**: A strategy pattern for defining how different types of AI results should be written back to the RST format (`MetadataApplyHook`, `ShortDescriptionApplyHook`).
- **`EnhanceData`**: Immutable state object (in `enhance_data.py`) that carries analysis results, updated-file tracking, and metrics through both enhancement phases.

## Workflow Diagrams

Each enhancement phase is a **two-pass** pipeline: `analyze_files` (upload, generate, validate, store in `EnhanceData`) then `update_enhanced_files` / `update_meta_files` (read local RST, apply hook, write only if changed). Disk writes never occur inside `analyze_files`.

### High-Level Orchestration
```mermaid
flowchart TD
    A([main]) --> B[configure_logging]
    B --> C[Filter CLI args to .rst files]
    C --> D{Any .rst files?}
    D -->|No| Z([Exit 0])
    D -->|Yes| E[get_openai_client]
    E --> F["Phase 1: enhance_metadata<br/>analyze_files → update_meta_files"]
    F --> G["Phase 2: enhance_short_descriptions<br/>vector store → analyze_files → update_enhanced_files → cleanup"]
    G --> H[calculate_metrics + log summary]
    H --> Z
```

### Per File: Analysis Pass (`analyze_files`)
Runs inside both phases (once per file per phase). Phase 2 also creates the example vector store once before this loop and deletes it in `finally` after the apply pass.

```mermaid
flowchart TD
    A[Read local RST] --> B{Empty or every task skipped?}
    B -->|Yes| N[Next file]
    B -->|No| C[files.create — upload article]
    C --> D[For each pending EnhancementTask]
    D --> E{Model-generated task?}
    E -->|Yes| F[responses.create<br/>+ file_search in Phase 2]
    F --> G{skip_validation?}
    G -->|No| H[moderations.create + English check via responses.create]
    G -->|Yes| I[Store in EnhanceData]
    H -->|Pass| I
    H -->|Fail| W[Log warning; discard result]
    E -->|No — distribution / product| I
    I --> J[files.delete — hosted upload]
    J --> N
```

### Per Phase: Apply Pass (`update_enhanced_files`)
Runs after all files in that phase have been analysed. Regex injection is via `rst_utils` inside the `ApplyHook`.

```mermaid
flowchart TD
    A[For each input file with results in EnhanceData] --> B[Read local RST]
    B --> C[ApplyHook.apply]
    C --> D{Content changed?}
    D -->|Yes| E[Write RST + mark_file_updated]
    D -->|No| F[Next file]
    E --> F
```

### Analysis Pass (sequence)
```mermaid
sequenceDiagram
    participant S as analyze_files
    participant L as Local RST
    participant O as OpenAI API

    S->>L: Read content
    Note over S: Skip file if empty or all tasks' should_skip is true
    S->>O: files.create (purpose=user_data)
    loop Each pending EnhancementTask
        alt description, keywords, or short-description
            S->>O: responses.create (file_search in Phase 2)
            O-->>S: Generated text
            opt validate_content (not for fixed meta)
                S->>O: moderations.create
                S->>O: responses.create (English yes/no check)
            end
            alt Passes validation or skip_validation
                S->>S: add_analysis_result → EnhanceData
            else Failed or empty result
                S->>S: Log warning; do not store
            end
        else distribution or product
            S->>S: Fixed placeholder (no API call)
            S->>S: add_analysis_result → EnhanceData
        end
    end
    S->>O: files.delete (uploaded article)
    Note over S,L: RST on disk unchanged until apply pass
```
