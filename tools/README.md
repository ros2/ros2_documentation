# Documentation tools

Helpers for ensuring reStructuredText (`.rst`) documentation enhancements on pull requests.

---

## User guide

Information for documentation contributors creating or updating `.rst` files.

When you open or update a pull request, the **Enhance** GitHub Actions workflow runs automatically. If your changes include `.rst` files, those files are checked for standard metadata and after-title directives defined in [`enhance.yaml`](enhance.yaml). The tooling is read-only: it never edits your files or posts inline commit suggestions. If anything is missing, you receive a single **Documentation enhancements** review on the pull request and fix the RST yourself.

### What gets checked

| Scope | Detail |
|-------|--------|
| **When** | Each time a pull request is opened, updated (`synchronize`), or reopened |
| **Which files** | Only `.rst` files **added, copied, modified, or renamed** in that pull request (not the whole repository) |
| **How deeply** | The entire content of each in-scope file is checked against [`enhance.yaml`](enhance.yaml) |
| **When skipped** | Pull requests that do not touch any `.rst` files — no check runs and no review is posted |

Locally, you can check specific files before pushing, or use `--diff-base` to mimic the same PR-scoped discovery (see [Checking enhancements locally](#checking-enhancements-locally)).

### Which pull requests

The Enhance workflow runs on **every** pull request — any target branch, including from forks — once [`.github/workflows/enhance.yml`](../.github/workflows/enhance.yml) has been merged to the repository default branch (`rolling`).

GitHub Actions treats `pull_request_target` workflows specially:

| Source | What it provides |
|--------|------------------|
| **Default branch** (`rolling`) | The workflow definition (when the job runs, permissions, step order) |
| **Pull request base branch** | The check scripts and rules (`ensure_enhancements.py`, `enhance.yaml`, Makefile targets) |
| **Pull request head branch** | The `.rst` file content to inspect |

So the job is defined centrally on `rolling`, but the rules applied are those on whichever branch your pull request targets. Until the workflow is on `rolling`, the Enhance check will not run on any pull request.

See [Continuous integration architecture](#continuous-integration-architecture) below for the full job flow and security rationale.

### Prerequisites

| Component | Used by |
|-----------|---------|
| [PyYAML](https://pyyaml.org/) (`pip install pyyaml`) | [`ensure_enhancements.py`](ensure_enhancements.py) and unit tests in [`tests/`](tests/) |
| Git (with a usable `HEAD` and refs for your base commit) | PR-scope discovery (`--diff-base`) |
| [GitHub CLI](https://cli.github.com/) (`gh`) and `jq` | [`supersede_enhancement_reviews.sh`](supersede_enhancement_reviews.sh) only (Enhance workflow on `ubuntu-24.04`; optional for local supersede testing) |

### Enhancement configuration

[`enhance.yaml`](enhance.yaml) defines documentation enhancement rules. The `meta` section lists every `.. meta::` field checked by the tooling. Each entry has:

- **`severity`**: `warning` (advisory; soft-fails the ensure step in CI) or `error` (fails the workflow after the review is posted).
- **`value`**: suggested default text when the field is missing or blank. Leave empty when the contributor must supply a non-empty value.

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

The `after_title` section maps directive names to rules, in the order they should appear after the first document title:

```yaml
after_title:
  short-description:
    severity: warning
    content: first_paragraph
  showmeta:
    severity: warning
    options:
      order: area, content-type, experience
    required_options:
      - order
```

The `:order:` value lists `.. meta::` field names and must match the `meta` section (e.g. `content-type`, not `contentType`).

For `short-description`, the contributor should wrap the first prose paragraph after the title into the directive. For `showmeta`, the contributor should add the directive with the configured `:order:` option when missing.

#### Severity behaviour

| Severity | Missing or blank field | CI ensure step | Workflow job |
|----------|------------------------|----------------|--------------|
| `warning` | Listed in review | Soft warning (`continue-on-error`) | Succeeds |
| `error` | Listed in review | Soft warning (same step) | **Fails** on final enforce step |

### Checking enhancements locally

[`ensure_enhancements.py`](ensure_enhancements.py) checks `.rst` files against [`enhance.yaml`](enhance.yaml). It reports missing meta fields and after-title directives; it does not modify files.

#### Usage

From the repository root:

```bash
python3 tools/ensure_enhancements.py path/to/article.rst
```

Multiple files:

```bash
python3 tools/ensure_enhancements.py source/Topic/A.rst source/Topic/B.rst
```

Pull request scope (discovers changed `ACMR` `*.rst` files via `git diff`):

```bash
python3 tools/ensure_enhancements.py --diff-base origin/rolling
```

For day-to-day editing, pass paths explicitly so the tool exits `1` only when **error**-severity issues remain.

To simulate the CI ensure step locally (writes status outputs and uses CI exit codes):

```bash
make ensure-enhancements DIFF_BASE=origin/rolling STATUS_FILE=/tmp/enhance-out.txt
```

#### Exit codes

With `--status-file` (CI), exit `1` when any issues remain. Locally, exit `1` only when **error**-severity issues are still unresolved; warning-only issues exit `0`.

### When you open or update a pull request

The [Enhance workflow](../.github/workflows/enhance.yml) runs automatically on every pull request (any target branch, including from forks). From a contributor’s perspective:

1. **No `.rst` changes** — The workflow succeeds immediately. Nothing is posted and no enhancement check runs.
2. **One or more `.rst` files changed** — CI discovers those files and checks each against [`enhance.yaml`](enhance.yaml).
3. **Everything present** — The workflow succeeds. Any earlier **Documentation enhancements** review from a previous push is marked outdated; no new review is posted.
4. **Something missing** — CI posts a **Documentation enhancements** review on the pull request **Conversation** tab. The review lists each affected file and what is still missing:
   - `.. meta::` fields labelled **required** (error severity) or **optional** (warning severity), with suggested default values where configured
   - After-title directives (`.. short-description::`, `.. showmeta::`) with brief guidance
5. **Warning-only gaps** — The workflow job still **succeeds**; treat the review as advisory and fix when you can.
6. **Required gaps** (for example a missing `area` meta field) — The review is posted first, then the workflow job **fails** so the pull request cannot merge until you fix the error-severity items.
7. **Further pushes** — Each update re-runs the check. Previous bot reviews are minimised as outdated and, if issues remain, a fresh summary review replaces them.

#### Outcomes at a glance

| Situation | Pull request review | Can merge? |
|-----------|---------------------|------------|
| No changed `.rst` files | None | Yes |
| All enhancements present | None (stale reviews cleared) | Yes |
| Warning-only gaps | Summary review comment | Yes |
| Required gaps (e.g. missing `area`) | Summary review comment | **No** until fixed |

#### How to fix issues

1. Open the **Documentation enhancements** review on your pull request.
2. Edit the listed `.rst` files: add or complete the missing `.. meta::` fields and after-title directives (see [Enhancement configuration](#enhancement-configuration) above).
3. Optionally verify locally before pushing:

   ```bash
   python3 tools/ensure_enhancements.py path/to/your/file.rst
   ```

4. Push new commits. CI re-runs the workflow; when all required items are resolved, the check passes and stale reviews are cleared.

The tool does not apply fixes for you — all changes are manual edits in your branch.

---

## Developer guidance

Information for maintainers and developers working on or extending the enhancement tooling and CI workflows.

### Repository layout

| File | Purpose |
|------|---------|
| [`rst_utils.py`](rst_utils.py) | Read-only detection of `.. meta::`, `.. short-description::`, and `.. showmeta::` directives |
| [`enhance_config.py`](enhance_config.py) | Load and validate rules from [`enhance.yaml`](enhance.yaml) |
| [`enhance.yaml`](enhance.yaml) | Enhancement rules (`meta` fields and `after_title` directives) |
| [`ensure_enhancements.py`](ensure_enhancements.py) | CLI, per-file checks, review comments, and CI outputs |
| [`supersede_enhancement_reviews.sh`](supersede_enhancement_reviews.sh) | Minimise outdated bot PR reviews |
| [`tests/`](tests/) | Unit tests for the tools in this directory |

### Code modules

#### `enhance_config.py`

Loads and validates [`enhance.yaml`](enhance.yaml):

- **`MetaRule`**, **`AfterTitleRule`**, **`EnhanceConfig`** — configuration schema
- **`load_enhance_config`** — parse and validate a config file

#### `rst_utils.py`

Read-only utilities for locating Sphinx directives in RST source:

- **`get_meta_fields_from_content`** — field names and values in the first `.. meta::` block
- **`has_short_description_content`** — whether a non-empty `.. short-description::` body exists
- **`has_showmeta_with_order`** — whether `.. showmeta::` exists with a non-empty `:order:` option

#### Extending configuration

Add a new key under `meta` in [`enhance.yaml`](enhance.yaml) to extend metadata coverage without changing Python code. `_parse_meta_rules` in [`enhance_config.py`](enhance_config.py) accepts any key with `severity` and `value`, and the rest of the pipeline (unresolved-field detection and review hints) is driven entirely by that mapping.

`after_title` is only partly config-driven. Its mapping shape lets you reorder or retune the two existing directives (`short-description`, `showmeta`) from YAML alone — but `_parse_after_title_rules` in [`enhance_config.py`](enhance_config.py) whitelists `supported_directives = {"short-description", "showmeta"}`, so adding a *new* after-title directive needs Python changes:

1. Add the name to `supported_directives` and its directive-specific validation in `_parse_after_title_rules` ([`enhance_config.py`](enhance_config.py)).
2. Extend `_after_title_rule_satisfied` in [`ensure_enhancements.py`](ensure_enhancements.py) with an "already present" check for the new directive.
3. Extend `_after_title_hint` in [`ensure_enhancements.py`](ensure_enhancements.py) with review text for the new directive.
4. Add matching read-only detection helpers to [`rst_utils.py`](rst_utils.py) — see `has_showmeta_with_order` for the `showmeta` example.

### CLI options & Makefile targets

#### Options (`ensure_enhancements.py`)

- `paths` — optional; when omitted, `--diff-base` is required and changed `.rst` files are discovered automatically
- `--config PATH` — YAML config file (default: `tools/enhance.yaml`)
- `--diff-base SHA` — PR base commit; discovers changed `.rst` files for the check
- `--status-file PATH` — write `enhancements_checked`, `has_results`, `has_errors`, and the review comment body for CI; when issues remain, exits `1` (the ensure step uses `continue-on-error`)
- `-v` / `--verbose` — enable debug logging

#### Makefile targets

Enhancement Make targets live in the repository root [`Makefile`](../Makefile). CI invokes them with `make -f .trusted-base/Makefile …` so those recipes run from the PR **base** branch, not the PR head.

| Target | Required variables | Purpose |
|--------|-------------------|---------|
| `ensure-enhancements` | `DIFF_BASE`, `STATUS_FILE`; optional `TOOLS_DIR` (default `tools`) | Discover changed RST, run enhancement check, append CI outputs |
| `supersede-enhancement-reviews` | `PR_NUMBER`, `REPOSITORY`; optional `TOOLS_DIR` | Minimise stamped bot reviews |

Environment for `supersede-enhancement-reviews` (set by the workflow or locally): `GH_TOKEN`, plus `PR_NUMBER` and `REPOSITORY`. It writes no CI outputs; the workflow decides what to post from the ensure step’s outputs.

### Continuous integration architecture

The workflow [`.github/workflows/enhance.yml`](../.github/workflows/enhance.yml) runs on **`pull_request_target`** when a pull request is **opened**, **synchronised**, or **reopened**. It triggers for pull requests targeting **any** branch (there is no `branches:` filter). That event type allows the default `GITHUB_TOKEN` to post review comments on fork PRs.

GitHub always executes the workflow **as defined on the repository default branch** (`rolling`), not from the pull request head. The check scripts and [`enhance.yaml`](enhance.yaml) come from the pull request **base** branch (checked out into `.trusted-base/`). See [Which pull requests](#which-pull-requests) in the user guide and [Security](#security) below.

The Enhance workflow installs PyYAML in the job; it does not install the full documentation `requirements.txt` for enhancement checks.

#### Job flow (`ensure-enhancements`)

1. Check out the PR **head** (`.rst` content to inspect).
2. Check out the PR **base** into `.trusted-base/` (Makefile, `ensure_enhancements.py`, `enhance.yaml`, `supersede_enhancement_reviews.sh`).
3. Install Python 3.12 and PyYAML.
4. **Ensure documentation enhancements** — `git fetch` the base SHA, then `make -f .trusted-base/Makefile ensure-enhancements` with `TOOLS_DIR=.trusted-base/tools`, `DIFF_BASE`, and `STATUS_FILE=$GITHUB_OUTPUT`. The step uses `continue-on-error: true` so warning-only gaps do not fail the job immediately.
5. **Verify enhancement check ran** — fail the job if `enhancements_checked` is empty. The ensure step soft-fails by design, so a missing output is the only way to tell a crashed check from a clean run.
6. **Supersede stale enhancement reviews** (only if `enhancements_checked=true`) — `make -f .trusted-base/Makefile supersede-enhancement-reviews`, which minimises stamped reviews and writes nothing back.
7. **Post enhancement review comment** — if `has_results`, post the stamped `comment` body via `gh pr review` (Conversation view).
8. **Enforce required enhancements** — if `has_errors`, fail the job (runs `always()` so error gaps fail even when the ensure step soft-failed).

Step 7 uses `!cancelled()` rather than depending on the supersede step, so a transient GitHub API failure while minimising old reviews cannot stop contributors receiving feedback.

#### CI outputs

The script writes **CI outputs** (for example `$GITHUB_OUTPUT`) that describe which workflow steps to run:

| Output | Meaning |
|--------|---------|
| `enhancements_checked` | At least one changed `.rst` was in scope (`false` when discovery finds no changed RST; supersede is skipped). Empty only when the check never ran |
| `has_results` | Enhancement issues remain; post the Conversation review |
| `has_errors` | Unresolved **error**-severity issues (triggers the final enforce step) |
| `comment` | Full stamped review body (multiline heredoc) posted to Conversation via `gh pr review`; written only when `has_results` |

The outputs are self-consistent by construction: `comment` exists whenever `has_results` is true, and `has_errors` implies `has_results`. The enforce step cannot fail the job without a review having been posted.

The ensure step uses `continue-on-error: true`, so warning-only gaps do not fail the job. Error-severity gaps (for example `area`) still fail the workflow on the enforce step after contributors receive review feedback.

#### Superseding outdated reviews

The summary review body includes a hidden HTML marker (`<!-- ros2-doc-enhance-ensure -->`). The marker id `ros2-doc-enhance-ensure` (constant `REVIEW_MARKER_ID` in Python; override in the shell script with `ENHANCEMENT_REVIEW_MARKER_ID`) is what the supersede script searches for in review bodies.

When `enhancements_checked=true`, the workflow runs `make -f .trusted-base/Makefile supersede-enhancement-reviews` ([`supersede_enhancement_reviews.sh`](supersede_enhancement_reviews.sh)):

1. Lists pull request reviews whose body contains the marker id.
2. Minimises each as **Outdated** via the GitHub GraphQL API (`gh api graphql`; individual failures are ignored).

When all issues are fixed (`has_results=false`), stale summary reviews are minimised and no new "all clear" comment is posted.

#### Security

The workflow uses [`pull_request_target`](https://docs.github.com/en/actions/using-workflows/events-that-trigger-workflows#pull_request_target) so the default `GITHUB_TOKEN` can post review comments on fork PRs. **Do not** run `make` or scripts from the PR head checkout in that job; always use `-f .trusted-base/Makefile` and `TOOLS_DIR=.trusted-base/tools`. See [Mitigating the risks of untrusted code checkout](https://docs.github.com/en/actions/reference/security/secure-use#mitigating-the-risks-of-untrusted-code-checkout).

### Tests

Unit tests for this directory live in [`tests/`](tests/). From the repository root (with PyYAML installed):

```bash
PYTHONPATH=tools python3 -m pytest tools/tests/
```

The [`test-tools`](../Makefile) target (run in [`.github/workflows/test.yml`](../.github/workflows/test.yml)) runs `pytest` on the top-level [`test/`](../test/) tree and, with `PYTHONPATH=tools`, on [`tests/`](tests/) in this directory.
