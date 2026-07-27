# Documentation tools

Helpers for ensuring reStructuredText (`.rst`) metadata on documentation pull requests.

## Layout

| File | Purpose |
|------|---------|
| [`rst_utils.py`](rst_utils.py) | Regex-based read/write of `.. meta::` and `.. short-description::` directives |
| [`meta_tags.yaml`](meta_tags.yaml) | Metadata rules (severity and optional default values) |
| [`ensure_meta_tags.py`](ensure_meta_tags.py) | CLI that checks and fixes metadata from the config |
| [`tests/`](tests/) | Unit tests for the tools in this directory |

## Configuration

[`meta_tags.yaml`](meta_tags.yaml) defines every `.. meta::` field the script checks. Each entry has:

- **`severity`**: `warning` (advisory; soft-fails the ensure step in CI) or `error` (fails the workflow after reviews are posted).
- **`value`**: default text to inject when the field is missing or blank. Leave empty when the contributor must supply a non-empty value.

```yaml
meta:
  product:
    severity: warning
    value: "{PRODUCT}"
  distribution:
    severity: warning
    value: "{DISTRO}"
  area:
    severity: error
    value:
  experience:
    severity: warning
    value:
  content-type:
    severity: warning
    value:
```

Add a new key under `meta` to extend coverage without changing Python code. The script applies every rule in the file.

`{PRODUCT}` and `{DISTRO}` are Sphinx substitution macros expanded at build time from [`conf.py`](../conf.py).

### Severity behaviour

| Severity | Missing or blank field | CI ensure step | Workflow job |
|----------|------------------------|----------------|--------------|
| `warning` | Annotation + review | Soft warning (`continue-on-error`) | Succeeds |
| `error` | Error annotation + review | Soft warning (same step) | **Fails** on final enforce step |

## `rst_utils.py`

Low-level utilities for locating and editing Sphinx directives in RST source:

- **`get_meta_fields_from_content`** — field names and values in the first `.. meta::` block
- **`get_meta_names_from_content`** — field names only
- **`inject_metadata_to_content`** — add missing fields or fill blank values; never overwrites non-empty contributor values

The module also contains helpers for `.. short-description::` directives for future use.

## `ensure_meta_tags.py`

Checks each given `.rst` file against `meta_tags.yaml`. Fields with a configured `value` are added or filled automatically when the edit can be suggested or applied locally. Fields with an empty `value` must be completed manually in the `.. meta::` block.

### Usage

From the repository root:

```bash
python3 tools/ensure_meta_tags.py path/to/article.rst
```

Multiple files:

```bash
python3 tools/ensure_meta_tags.py source/Topic/A.rst source/Topic/B.rst
```

Pull request scope (discover changed ``.rst`` files with a three-dot diff against a base commit):

```bash
make ensure-meta-tags DIFF_BASE=origin/rolling
```

The repository [`Makefile`](../Makefile) target runs `ensure_meta_tags.py` with `--diff-base` and no explicit paths; changed ACMR ``*.rst`` files are discovered via ``git diff``. For CI-style output locally, pass ``STATUS_FILE=/path/to/file``.

Options:

- `paths` — optional; when omitted, `--diff-base` is required and changed ``.rst`` files are discovered automatically
- `--config PATH` — YAML config file (default: `tools/meta_tags.yaml`)
- `--diff-base SHA` — PR base commit; limits on-disk writes to lines in the PR diff (inline suggestions); files that need a copy-paste or manual field list use a review comment instead
- `--status-file PATH` — write `meta_checked`, `inline_suggestions`, `review_comment`, `has_results`, `has_errors`, and the review comment body for CI; when issues remain, emits annotations and exits `1` (the ensure step uses `continue-on-error`)
- `-v` / `--verbose` — enable debug logging

**Exit codes:** With `--status-file` (CI), exit `1` when any issues remain. Locally, exit `1` only when **error**-severity fields are still unresolved; warning-only issues exit `0` after applying automatic fixes.

### Example (configured values)

Before:

```rst
My Article
==========

Some content.
```

After a local run (new `.. meta::` at the top of the file):

```rst
.. meta::
   :product: {PRODUCT}
   :distribution: {DISTRO}

My Article
==========

Some content.
```

Fields such as `area` with an empty `value` in the config are listed in the review for manual completion; they are not given placeholder text.

## Continuous integration

The workflow [`.github/workflows/enhance.yml`](../.github/workflows/enhance.yml) runs on every pull request (including from forks):

1. Checks out the PR’s `.rst` files as untrusted data
2. Checks out the base branch into `.trusted-base/` for the trusted Makefile, script, and config
3. Runs `make -f .trusted-base/Makefile ensure-meta-tags` (with `TOOLS_DIR=.trusted-base/tools`, `DIFF_BASE`, and `STATUS_FILE=$GITHUB_OUTPUT`) so discovery and metadata checks use trusted code only
4. Emits per-file warning/error annotations and soft-fails the ensure step when metadata is still missing
5. Posts inline suggestions (`suggest-changes`) and/or a review comment (`Post meta tag review comment`)
6. **Fails the job** if any **error**-severity metadata remains (`Enforce required metadata`)

Priority: **inline suggestions wherever GitHub allows them**. Copy-paste blocks and manual field lists are delivered via a pull request review comment when inline suggestions are not possible.

### Per-file modes and CI outputs

Each changed `.rst` file is classified with an internal **mode**:

| Mode | Meaning |
|------|---------|
| `suggestable` | Configured values were written to the working tree for inline “Commit suggestion” |
| `snippet` | Configured values could not be written inline; the review includes a copy-paste `.. meta::` block |
| `manual_fields` | Only fields with empty `value` in the config are missing; the review lists field names |

The script writes **CI outputs** (for example `$GITHUB_OUTPUT`) that describe which workflow steps to run:

| Output | Meaning |
|--------|---------|
| `meta_checked` | At least one changed `.rst` was in scope (`false` when discovery finds no changed RST; supersede and review steps are skipped) |
| `inline_suggestions` | Run [`suggest-changes`](https://github.com/marketplace/actions/suggest-changes-action) |
| `review_comment` | Post the generated review body with `gh pr review` when inline suggestions are not used (covers `snippet` and `manual_fields` files) |
| `has_results` | Metadata issues remain (used to decide whether to post a new review after superseding stale ones) |
| `has_errors` | Unresolved **error**-severity fields (triggers the final enforce step) |
| `comment` | Full stamped review body (multiline) for suggest-changes or `gh pr review` |

When `inline_suggestions` is `true`, the workflow runs suggest-changes even if `review_comment` is also `true` (mixed per-file modes). A separate review comment is posted only when `review_comment` is `true` and `inline_suggestions` is not.

The ensure step uses `continue-on-error: true`, so warning-only gaps do not fail the job. Error-severity gaps (for example `area`) still fail the workflow after contributors receive review feedback.

### Annotations

When metadata is missing or blank:

- **Warning** severity → `::warning file=...,line=N::Missing meta fields: ...`
- **Error** severity → `::error file=...,line=N::Missing meta fields: ...`

`N` is the start line of an existing `.. meta::` block, or `1` when a new block would be inserted at the top of the file.

### Contributor experience

| Situation | Ensure step | Annotations | Pull request review | Job result |
|-----------|-------------|-------------|---------------------|------------|
| All fields resolved | Green | None | None (stale bot reviews cleared) | Success |
| Warning-only gaps | Soft warning | Warnings | Suggestions and/or manual list | Success |
| Error gaps (e.g. missing `area`) | Soft warning | Errors (and warnings) | Suggestions and/or manual list | **Failure** after enforce step |
| Auto-fix in diff | Soft warning | As above | Inline “Commit suggestion” | As per severity |
| Auto-fix outside diff (`snippet`) | Soft warning | As above | Copy-paste `.. meta::` for configured values | As per severity |
| Manual fields only (`manual_fields`) | Soft warning | As above | Field list with required/warning labels | As per severity |

Reviews, annotations, and the soft-failed ensure step appear in different parts of the GitHub UI (Conversation, Files changed, Checks); only error-severity issues fail the overall workflow.

### When inline suggestions appear

GitHub only allows review suggestions on [lines already in the pull request diff](https://github.com/marketplace/actions/suggest-changes-action). The script compares each automatic edit to that diff:

| Situation | What happens |
|-----------|----------------|
| Missing configured values; existing `.. meta::` overlaps the PR diff | Write append/fill to the working tree → inline suggestion via [`suggest-changes`](https://github.com/marketplace/actions/suggest-changes-action) |
| No `.. meta::`; top of file overlaps the PR diff | Insert at top → inline suggestion |
| Automatic edit does **not** overlap the PR diff (`snippet` mode) | No inline write; review includes a copy-paste block for configured values |
| Only manual fields (empty `value` in config, `manual_fields` mode) | Review lists fields; no placeholder injection |
| All configured fields present and non-empty | No action |
| No changed `.rst` files in the PR | No check; `meta_checked=false`; supersede/review steps skipped |

### Superseding outdated reviews

Each bot review body includes a hidden marker (`<!-- ros2-meta-tags-ensure -->`). On every run that checks changed RST files:

1. All prior stamped reviews on the pull request are minimized as **Outdated** via the GitHub API.
2. A fresh review is posted only when work still remains.
3. When all issues are fixed, stale reviews are minimized and no new “all clear” comment is posted.

The workflow uses [`pull_request_target`](https://docs.github.com/en/actions/using-workflows/events-that-trigger-workflows#pull_request_target) so the default `GITHUB_TOKEN` can post review suggestions on fork PRs. The workflow definition itself is taken from the repository **default branch**; the trusted Makefile, script, and config come from the PR **base** branch checkout at `.trusted-base/`. See [Mitigating the risks of untrusted code checkout](https://docs.github.com/en/actions/reference/security/secure-use#mitigating-the-risks-of-untrusted-code-checkout).

## Tests

Unit tests live in [`tests/`](tests/). From the repository root (with PyYAML installed):

```bash
python3 -m unittest discover -s tools/tests -p 'test_*.py'
```
