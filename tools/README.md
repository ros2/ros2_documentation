# Documentation tools

Helpers for ensuring reStructuredText (`.rst`) metadata on documentation pull requests.

---

## User guide

Information for documentation contributors creating or updating `.rst` files.

### Prerequisites

| Component | Used by |
|-----------|---------|
| [PyYAML](https://pyyaml.org/) (`pip install pyyaml`) | [`ensure_meta_tags.py`](ensure_meta_tags.py) and unit tests in [`tests/`](tests/) |
| Git (with a usable `HEAD` and refs for your base commit) | PR-scope discovery and diff overlap checks (`--diff-base`) |
| [GitHub CLI](https://cli.github.com/) (`gh`) and `jq` | [`supersede_meta_tag_reviews.sh`](supersede_meta_tag_reviews.sh) only (Enhance workflow on `ubuntu-24.04`; optional for local supersede testing) |

### Metadata configuration

[`meta_tags.yaml`](meta_tags.yaml) defines every `.. meta::` field checked by the tooling. Each entry has:

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

`{PRODUCT}` and `{DISTRO}` are Sphinx substitution macros expanded at build time from [`conf.py`](../conf.py).

#### Severity behaviour

| Severity | Missing or blank field | CI ensure step | Workflow job |
|----------|------------------------|----------------|--------------|
| `warning` | Annotation + review | Soft warning (`continue-on-error`) | Succeeds |
| `error` | Error annotation + review | Soft warning (same step) | **Fails** on final enforce step |

### Checking metadata locally

`ensure_meta_tags.py` checks `.rst` files against `meta_tags.yaml`. Fields with a configured `value` are added or filled automatically when the edit can be suggested or applied locally. Fields with an empty `value` must be completed manually in the `.. meta::` block.

#### Usage

From the repository root:

```bash
python3 tools/ensure_meta_tags.py path/to/article.rst
```

Multiple files:

```bash
python3 tools/ensure_meta_tags.py source/Topic/A.rst source/Topic/B.rst
```

Pull request scope (discovers changed `ACMR` `*.rst` files via `git diff`; requires Makefile variables `DIFF_BASE` and `STATUS_FILE`):

```bash
make ensure-meta-tags DIFF_BASE=origin/rolling STATUS_FILE=/tmp/meta-tags-out.txt
```

PR scope without Make (optional `--status-file`; exit codes follow local rules when omitted):

```bash
python3 tools/ensure_meta_tags.py --diff-base origin/rolling
python3 tools/ensure_meta_tags.py --diff-base "$(git merge-base HEAD origin/rolling)" --status-file /tmp/out.txt
```

For day-to-day editing of known files, pass paths explicitly and omit `--status-file` so the tool exits `1` only when **error**-severity fields remain.

#### Exit codes

With `--status-file` (CI), exit `1` when any issues remain. Locally, exit `1` only when **error**-severity fields are still unresolved; warning-only issues exit `0` after applying automatic fixes.

#### Example (configured values)

Before:

```rst
My Article
==========

Some content.
```

After a local run (new `.. meta::` added at the top of the file):

```rst
.. meta::
   :product: {PRODUCT}
   :distribution: {DISTRO}

My Article
==========

Some content.
```

Fields such as `area` with an empty `value` in the config are listed in the review for manual completion; they are not given placeholder text.

### Contributor CI experience

When you open or update a pull request, CI automatically checks metadata on all modified `.rst` files.

#### Contributor experience overview

| Situation | Ensure step | Annotations | Pull request review | Job result |
|-----------|-------------|-------------|---------------------|------------|
| All fields resolved | Green | None | None (stale bot reviews cleared) | Success |
| Warning-only gaps | Soft warning | Warnings | Suggestions and/or manual list | Success |
| Error gaps (e.g. missing `area`) | Soft warning | Errors (and warnings) | Suggestions and/or manual list | **Failure** after enforce step |
| Auto-fix in diff | Soft warning | As above | Inline “Commit suggestion” | As per severity |
| Auto-fix outside diff (`snippet`) | Soft warning | As above | Copy-paste `.. meta::` for configured values | As per severity |
| Manual fields only (`manual_fields`) | Soft warning | As above | Field list with required/warning labels | As per severity |

Reviews, annotations, and the soft-failed ensure step appear in different parts of the GitHub UI (Conversation, Files changed, Checks); only error-severity issues fail the overall workflow.

#### When inline suggestions appear

GitHub only allows review suggestions on [lines already in the pull request diff](https://github.com/marketplace/actions/suggest-changes-action). The script compares each automatic edit to that diff:

| Situation | What happens |
|-----------|----------------|
| Missing configured values; existing `.. meta::` overlaps the PR diff | Write append/fill to the working tree → inline suggestion via [`suggest-changes`](https://github.com/marketplace/actions/suggest-changes-action) |
| No `.. meta::`; top of file overlaps the PR diff | Insert at top → inline suggestion |
| Automatic edit does **not** overlap the PR diff (`snippet` mode) | No inline write; review includes a copy-paste block for configured values |
| Only manual fields (empty `value` in config, `manual_fields` mode) | Review lists fields; no placeholder injection |
| All configured fields present and non-empty | No action |
| No changed `.rst` files in the PR | No check; `meta_checked=false`; supersede/review steps skipped |

---

## Developer guidance

Information for maintainers and developers working on or extending the metadata tooling and CI workflows.

### Repository layout

| File | Purpose |
|------|---------|
| [`rst_utils.py`](rst_utils.py) | Regex-based read/write of `.. meta::` and `.. short-description::` directives |
| [`meta_tags.yaml`](meta_tags.yaml) | Metadata rules (severity and optional default values) |
| [`ensure_meta_tags.py`](ensure_meta_tags.py) | CLI that checks and fixes metadata from the config |
| [`supersede_meta_tag_reviews.sh`](supersede_meta_tag_reviews.sh) | Minimise outdated bot PR reviews and set `should_post` for CI |
| [`tests/`](tests/) | Unit tests for the tools in this directory |

### Code modules

#### `rst_utils.py`

Low-level utilities for locating and editing Sphinx directives in RST source:

- **`get_meta_fields_from_content`** — field names and values in the first `.. meta::` block
- **`get_meta_names_from_content`** — field names only
- **`inject_metadata_to_content`** — add missing fields or fill blank values; never overwrites non-empty contributor values

The module also contains helpers for `.. short-description::` directives for future use.

#### Extending configuration

Add a new key under `meta` in [`meta_tags.yaml`](meta_tags.yaml) to extend coverage without changing Python code. The script applies every rule in the file.

### CLI options & Makefile targets

#### Options (`ensure_meta_tags.py`)

- `paths` — optional; when omitted, `--diff-base` is required and changed `.rst` files are discovered automatically
- `--config PATH` — YAML config file (default: `tools/meta_tags.yaml`)
- `--diff-base SHA` — PR base commit; limits on-disk writes to lines in the PR diff (inline suggestions); files that need a copy-paste or manual field list use a review comment instead
- `--status-file PATH` — write `meta_checked`, `inline_suggestions`, `review_comment`, `has_results`, `has_errors`, and the review comment body for CI; when issues remain, emits annotations and exits `1` (the ensure step uses `continue-on-error`)
- `-v` / `--verbose` — enable debug logging

#### Makefile targets (metadata CI)

Both targets live in the repository root [`Makefile`](../Makefile). CI invokes them with `make -f .trusted-base/Makefile …` so recipes run from the PR **base** branch, not the PR head.

| Target | Required variables | Purpose |
|--------|-------------------|---------|
| `ensure-meta-tags` | `DIFF_BASE`, `STATUS_FILE`; optional `TOOLS_DIR` (default `tools`) | Discover changed RST, run metadata check, append CI outputs |
| `supersede-meta-tag-reviews` | `PR_NUMBER`, `REPOSITORY`, `HAS_RESULTS`, `STATUS_FILE` (or `GITHUB_OUTPUT`); optional `TOOLS_DIR` | Minimise stamped bot reviews; append `should_post` |

Environment for `supersede-meta-tag-reviews` (set by the workflow or locally): `GH_TOKEN`, plus `PR_NUMBER`, `REPOSITORY`, and `HAS_RESULTS` from the ensure step’s `has_results` output.

### Continuous integration architecture

The workflow [`.github/workflows/enhance.yml`](../.github/workflows/enhance.yml) runs on **`pull_request_target`** when a pull request is **opened**, **synchronised**, or **reopened** (including from forks). That event type allows the default `GITHUB_TOKEN` to post review suggestions on fork PRs; the workflow file on the repository **default branch** defines the job, while trusted tooling comes from the PR **base** branch (see [Security](#security) below).

The Enhance workflow installs PyYAML in the job; it does not install the full documentation `requirements.txt` for metadata checks.

#### Job flow (`ensure-meta-tags`)

1. Check out the PR **head** (`.rst` content to inspect and, where allowed, modify for suggestions).
2. Check out the PR **base** into `.trusted-base/` (Makefile, `ensure_meta_tags.py`, `meta_tags.yaml`, `supersede_meta_tag_reviews.sh`).
3. Install Python 3.12 and PyYAML.
4. **Ensure documentation metadata** — `git fetch` the base SHA, then `make -f .trusted-base/Makefile ensure-meta-tags` with `TOOLS_DIR=.trusted-base/tools`, `DIFF_BASE`, and `STATUS_FILE=$GITHUB_OUTPUT`. Emits per-file annotations; the step uses `continue-on-error: true` so warning-only gaps do not fail the job immediately.
5. **Supersede stale meta-tag reviews** (only if `meta_checked=true`) — `make -f .trusted-base/Makefile supersede-meta-tag-reviews` with `HAS_RESULTS` from the ensure step; writes `should_post`.
6. **Suggest meta tag changes** — if `should_post` and `inline_suggestions`, run [`suggest-changes`](https://github.com/marketplace/actions/suggest-changes-action) with the multiline `comment` output.
7. **Post meta tag review comment** — if `should_post`, `review_comment`, and not `inline_suggestions`, post the same `comment` body via `gh pr review`.
8. **Enforce required metadata** — if `has_errors`, fail the job (runs `always()` so error gaps fail even when the ensure step soft-failed).

Priority: **inline suggestions wherever GitHub allows them**. Copy-paste blocks and manual field lists are delivered via a pull request review comment when inline suggestions are not used for that run.

#### Per-file modes and CI outputs

Each changed `.rst` file is classified with an internal **mode**:

| Mode | Meaning |
|------|---------|
| `suggestable` | Configured values were written to the working tree for inline “Commit suggestion” |
| `snippet` | Configured values could not be written inline; the review includes a copy-paste `.. meta::` block |
| `manual_fields` | Only fields with empty `value` in the config are missing; the review lists field names |

A single file can still list **manual** fields in the review when its mode is `suggestable` or `snippet` (auto-filled fields were handled; empty-config fields remain for the contributor).

The script writes **CI outputs** (for example `$GITHUB_OUTPUT`) that describe which workflow steps to run:

| Output | Meaning |
|--------|---------|
| `meta_checked` | At least one changed `.rst` was in scope (`false` when discovery finds no changed RST; supersede and review steps are skipped) |
| `inline_suggestions` | Run [`suggest-changes`](https://github.com/marketplace/actions/suggest-changes-action) |
| `review_comment` | Post the generated review body with `gh pr review` when inline suggestions are not used (covers `snippet` and `manual_fields` files) |
| `has_results` | Metadata issues remain (used to decide whether to post a new review after superseding stale ones) |
| `has_errors` | Unresolved **error**-severity fields (triggers the final enforce step) |
| `comment` | Full stamped review body (multiline heredoc) for suggest-changes or `gh pr review` |

**Supersede step output:** `should_post` — `true` when `has_results` is `true` (post a new review after minimising old ones); `false` when all metadata issues are resolved (minimise only, no new review).

When `inline_suggestions` is `true`, the workflow runs suggest-changes even if `review_comment` is also `true` (mixed per-file modes in one PR). A separate `gh pr review` runs only when `review_comment` is `true` and `inline_suggestions` is not.

The ensure step uses `continue-on-error: true`, so warning-only gaps do not fail the job. Error-severity gaps (for example `area`) still fail the workflow on the enforce step after contributors receive review feedback.

#### Annotations

When metadata is missing or blank:

- **Warning** severity → `::warning file=...,line=N::Missing meta fields: ...`
- **Error** severity → `::error file=...,line=N::Missing meta fields: ...`

`N` is the start line of an existing `.. meta::` block, or `1` when a new block would be inserted at the top of the file.

#### Superseding outdated reviews

Each bot review body includes a hidden HTML marker (`<!-- ros2-meta-tags-ensure -->`). The marker id `ros2-meta-tags-ensure` (constant `REVIEW_MARKER_ID` in Python; override in the shell script with `META_TAG_REVIEW_MARKER_ID`) is what the supersede script searches for in review bodies.

When `meta_checked=true`, the workflow runs `make -f .trusted-base/Makefile supersede-meta-tag-reviews` ([`supersede_meta_tag_reviews.sh`](supersede_meta_tag_reviews.sh)):

1. Lists pull request reviews whose body contains the marker id.
2. Minimises each as **Outdated** via the GitHub GraphQL API (`gh api graphql`; individual failures are ignored).
3. Appends `should_post=true` or `should_post=false` to `$GITHUB_OUTPUT` from `HAS_RESULTS` (`has_results` from the ensure step).

When all issues are fixed (`has_results=false`), stale reviews are minimised and no new “all clear” comment is posted.

#### Security

The workflow uses [`pull_request_target`](https://docs.github.com/en/actions/using-workflows/events-that-trigger-workflows#pull_request_target) so the default `GITHUB_TOKEN` can post review suggestions on fork PRs. **Do not** run `make` or scripts from the PR head checkout in that job; always use `-f .trusted-base/Makefile` and `TOOLS_DIR=.trusted-base/tools`. See [Mitigating the risks of untrusted code checkout](https://docs.github.com/en/actions/reference/security/secure-use#mitigating-the-risks-of-untrusted-code-checkout).

### Tests

Unit tests for this directory live in [`tests/`](tests/). From the repository root (with PyYAML installed):

```bash
python3 -m unittest discover -s tools/tests -p 'test_*.py'
```

The main documentation CI job [`test-tools`](../Makefile) runs `pytest` on the top-level [`test/`](../test/) tree; that is separate from `tools/tests`.
