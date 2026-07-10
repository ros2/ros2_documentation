# Documentation tools

Helpers for ensuring reStructuredText (`.rst`) metadata on documentation pull requests.

## Layout

| File | Purpose |
|------|---------|
| [`rst_utils.py`](rst_utils.py) | Regex-based read/write of `.. meta::` and `.. short-description::` directives |
| [`meta_tags.yaml`](meta_tags.yaml) | Default values for missing meta fields |
| [`ensure_meta_tags.py`](ensure_meta_tags.py) | CLI that adds missing meta fields from the config |

## Configuration

[`meta_tags.yaml`](meta_tags.yaml) defines which `.. meta::` fields to ensure and the value to inject when each is missing:

```yaml
meta:
  product: "{PRODUCT}"
  distribution: "{DISTRO}"
```

The `meta` map lists every field the script checks. Add a new key to extend coverage without changing Python code.

`{PRODUCT}` and `{DISTRO}` are Sphinx substitution macros expanded at build time from [`conf.py`](../conf.py). Edit this file when you need different default values for suggested meta tags.

## `rst_utils.py`

Low-level utilities for locating and editing Sphinx directives in RST source:

- **`get_meta_names_from_content`** — field names already present in the first `.. meta::` block
- **`inject_metadata_to_content`** — append missing `:name: value` lines to an existing block, or prepend a new `.. meta::` block; never overwrites existing fields

The module also contains helpers for `.. short-description::` directives for future use.

## `ensure_meta_tags.py`

Checks each given `.rst` file for the fields listed in `meta_tags.yaml`. When a field is missing, it is added with the configured value. Files that already have all configured fields are left unchanged.

### Usage

From the repository root:

```bash
python3 tools/ensure_meta_tags.py path/to/article.rst
```

Multiple files:

```bash
python3 tools/ensure_meta_tags.py source/Topic/A.rst source/Topic/B.rst
```

Options:

- `--config PATH` — YAML config file (default: `tools/meta_tags.yaml`)
- `-v` / `--verbose` — enable debug logging

### Example

Before:

```rst
My Article
==========

Some content.
```

After:

```rst
.. meta::
   :product: {PRODUCT}
   :distribution: {DISTRO}

My Article
==========

Some content.
```

If a `.. meta::` block already exists, missing fields are appended to it rather than creating a new block.

## Continuous integration

The workflow [`.github/workflows/enhance.yml`](../.github/workflows/enhance.yml) runs on every pull request (including from forks):

1. Checks out the PR’s `.rst` files as untrusted data
2. Checks out the base branch into `.trusted-base/` for the script and config
3. Runs the trusted `ensure_meta_tags.py` against changed RST files
4. Posts any resulting edits as review suggestions via [`parkerbxyz/suggest-changes`](https://github.com/marketplace/actions/suggest-changes-action)

Contributors can review and accept the suggested meta tags before merging. No GitHub App or repository secrets are required.

The workflow uses [`pull_request_target`](https://docs.github.com/en/actions/using-workflows/events-that-trigger-workflows#pull_request_target) so the default `GITHUB_TOKEN` can post review suggestions on fork PRs. To avoid running untrusted code with elevated permissions, only the base-branch copy of `ensure_meta_tags.py` and `meta_tags.yaml` is executed; PR content is treated as data to read and update. See [Mitigating the risks of untrusted code checkout](https://docs.github.com/en/actions/reference/security/secure-use#mitigating-the-risks-of-untrusted-code-checkout).
