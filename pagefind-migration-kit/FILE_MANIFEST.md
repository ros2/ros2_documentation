# Pagefind migration — file manifest

Inventory of everything required for Pagefind search, isolated from unrelated
POC work (related packages, enhance-topics, lyrical-only content, live-serve
tweaks, etc.).

Base target: clean `ros2/ros2_documentation` `rolling` (or a fork thereof).

## New files (copy as-is)

| Path | Role |
|------|------|
| `pagefind.yml` | Pagefind CLI options (`exclude_selectors`) + Sphinx `search_result_meta` labels/order |
| `plugins/meta_util.py` | Shared `.. meta::` helpers (required by `pagefind_meta` / `showmeta`) |
| `plugins/pagefind_config.py` | Loads `pagefind.yml` into Sphinx config |
| `plugins/pagefind_meta.py` | Emits Pagefind meta/filter tags + template context for search UI |
| `plugins/showmeta.py` | Optional `.. showmeta::` directive (in-page meta summary; recommended) |
| `source/_static/pagefind-docsearch.css` | DocSearch-inspired styling for modal + search page |
| `source/_templates/layout.html` | Injects Pagefind/SEO meta into `<head>` |
| `source/_templates/search.html` | Full-page Pagefind results UI (replaces Sphinx `searchtools`) |
| `source/_templates/searchbox.html` | Sidebar modal + Component UI wiring |
| `test/test_pagefind_config.py` | Unit tests for `pagefind.yml` loading |
| `test/test_pagefind_meta.py` | Unit tests for Pagefind meta plugin helpers |

These live under `files/` in this kit with the same relative paths.

## Modified files (apply patches)

| Path | Patch | What changes |
|------|-------|--------------|
| `Makefile` | `patches/Makefile.patch` | `pagefind`, `html-search`, `multiversion-search` targets; pin `PAGEFIND_VERSION` |
| `conf.py` | `patches/conf.py.patch` | Register extensions; Pagefind config; add CSS |
| `requirements.txt` | `patches/requirements.txt.patch` | Node.js note + `PyYAML` (for `pagefind.yml`) |
| `.github/workflows/test.yml` | `patches/test.yml.patch` | Node 24 + `make pagefind` after html/multiversion builds |
| `README.md` | `patches/README.md.patch` | Local/CI indexing docs |

## Explicitly excluded (not Pagefind)

Do **not** pull these in when migrating search alone:

- `plugins/ros_related_packages.py`, `ros_related_articles.py`, `short_description.py`
- `source/_static/related_packages.js`, `vendor/pako.min.js`, `vendor/js-yaml.min.js`
- `tools/enhance_topics.py`, enhance Makefile/CI workflows
- Lyrical distro / Ubuntu platform macros unrelated to search
- `make serve` / `sphinx-autobuild` ignore tweaks for rosdistro cache

## Content metadata (optional follow-up)

Faceted search works only when pages declare `.. meta::` fields whose keys appear
in `pagefind.yml` → `search_result_meta`. Upstream `rolling` currently has no such
blocks. Adding them is separate editorial work; see `IMPLEMENTATION.md`.
