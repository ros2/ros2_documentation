# Pagefind search — implementation guide

Apply the Pagefind search stack from this kit onto a **clean** fork of
[`ros2/ros2_documentation`](https://github.com/ros2/ros2_documentation) (branch
`rolling` or equivalent). This isolates search from other POC features.

## What you get

- Sphinx plugins that turn `.. meta::` into Pagefind filters / result metadata
- RTD theme overrides: sidebar search modal + dedicated `search.html` results page
- DocSearch-inspired CSS
- `make pagefind` / `make html-search` / `make multiversion-search`
- CI steps that index HTML after Sphinx builds

## Prerequisites

- Python 3.12+ with the repo’s `requirements.txt` + `constraints.txt`
- **Node.js 18+** (CI uses 24) with `npx` for the Pagefind CLI
- A checkout whose shared files still match upstream closely enough for the
  patches (`Makefile`, `conf.py`, `requirements.txt`, `.github/workflows/test.yml`,
  `README.md`). If a patch rejects, merge the hunks manually using the patch as a checklist.

## Quick apply (recommended)

From the **documentation repo root** that contains this kit:

```bash
chmod +x pagefind-migration-kit/apply.sh
./pagefind-migration-kit/apply.sh
```

Or against another checkout:

```bash
./pagefind-migration-kit/apply.sh /path/to/ros2_documentation
```

Then:

```bash
pip install --user -r requirements.txt -c constraints.txt
make test-tools          # pagefind unit tests
make html-search         # Sphinx HTML + Pagefind index
python -m http.server 8000 --directory build/html
# open http://localhost:8000/
```

Pagefind assets are under `build/html/pagefind/`. Serving over HTTP matters;
`file://` often breaks the search bundle.

## Manual apply (if you prefer)

### 1. Copy new files

Copy everything under `files/` into the repo root, preserving paths:

```text
pagefind.yml
plugins/meta_util.py
plugins/pagefind_config.py
plugins/pagefind_meta.py
plugins/showmeta.py
source/_static/pagefind-docsearch.css
source/_templates/layout.html
source/_templates/search.html
source/_templates/searchbox.html
test/test_pagefind_config.py
test/test_pagefind_meta.py
```

`conf.py` already adds `plugins/` to `sys.path` on upstream; no path change needed.

### 2. Patch shared files

```bash
patch -p1 < pagefind-migration-kit/patches/Makefile.patch
patch -p1 < pagefind-migration-kit/patches/conf.py.patch
patch -p1 < pagefind-migration-kit/patches/requirements.txt.patch
patch -p1 < pagefind-migration-kit/patches/test.yml.patch
patch -p1 < pagefind-migration-kit/patches/README.md.patch
```

### 3. Confirm `conf.py` Pagefind pieces

You should have:

```python
extensions = [
    # ... existing ...
    'pagefind_config',
    'pagefind_meta',
    'showmeta',
]

from pagefind_config import load_always_show_filters
from pagefind_config import load_search_result_meta_order

pagefind_merge_enabled = False
# ... other pagefind_merge_* defaults ...

pagefind_result_meta_order = load_search_result_meta_order(os.path.dirname(__file__))
pagefind_always_show_filters = load_always_show_filters(os.path.dirname(__file__))

html_css_files = ['custom.css', 'adopters.css', 'pagefind-docsearch.css']
```

Do **not** add related-packages JS (`pako`, `js-yaml`, `related_packages.js`); that is a separate feature.

## Configure facets (`pagefind.yml`)

```yaml
exclude_selectors:
  - a.headerlink

search_result_meta:
  product: Product
  distribution: Distribution
  area: Area
  # ...
  contentType: Content type
  experience: Level

# Show these facet sections even when no page has that meta yet.
always_show_filters:
  - product
  - distribution
  - area
  - contentType
  - experience
```

- Keys must match `.. meta::` field names on pages.
- Order = display order on result cards and the facet sidebar.
- Keys in `always_show_filters` always appear in the sidebar (empty
  placeholder until indexed); other keys appear only once present in the corpus.
- The Pagefind CLI reads `exclude_selectors` from this file when you run
  `make pagefind` from the repo root; Sphinx-only keys are ignored by the CLI.

## Tag pages for faceted search (editorial)

Upstream pages usually lack `.. meta::`. Without it, full-text search still works;
filters/result chips stay empty until you tag content.

Example:

```rst
.. meta::
   :product: {PRODUCT}
   :distribution: {DISTRO}
   :area: framework
   :contentType: tutorial
   :experience: beginner

.. showmeta::
   :order: product, distribution, area, contentType, experience
```

`{PRODUCT}` / `{DISTRO}` expand via Sphinx `macros` in `conf.py`.

Comma-separated values (e.g. `:experience: beginner, intermediate`) become
multiple facet values.

## Verify

1. **Unit tests:** `make test-tools`
2. **Build + index:** `make html-search`
3. **Bundle present:** `ls build/html/pagefind/` (JS/CSS/WASM index files)
4. **Modal:** open any page over HTTP → Ctrl/Cmd+K → type a term → Enter → land on `search.html?q=...`
5. **Direct URL:** `http://localhost:8000/search.html?q=tutorial`
6. **Facets (after tagging):** filters appear for keys in `pagefind.yml` that exist on indexed pages
7. **CI:** PR workflow runs Node setup + `make pagefind` after html and multiversion jobs

## Production / Jenkins

Deployed docs must run `make pagefind` (or equivalent `npx pagefind@… --site <html_root>`)
**after** Sphinx on the published HTML tree. Without that step, UI loads but the
index 404s.

Pin version via `PAGEFIND_VERSION` in the Makefile (kit default: `1.5.2`).

## Optional: merge remote package indexes

`pagefind_merge_*` settings in `conf.py` can attach extra Pagefind indexes at
runtime (Component UI). Defaults are off (`pagefind_merge_enabled = False`).
Leave them alone unless you intentionally merge package doc indexes.

## Creating a clean branch in this repo

```bash
git fetch upstream rolling
git checkout -b feature/pagefind-search upstream/rolling
./pagefind-migration-kit/apply.sh .
git add -A
git status   # should only show Pagefind-related paths
git commit -m "$(cat <<'EOF'
Add Pagefind search for Sphinx HTML docs.

Wire Sphinx meta into Pagefind filters/results UI, add indexing Make targets,
and run Pagefind in CI after HTML builds.
EOF
)"
```

## Troubleshooting

| Symptom | Likely cause |
|---------|----------------|
| Search UI empty / network 404 for `pagefind/*` | Forgot `make pagefind`, or opened `file://` instead of HTTP |
| Patch rejects | Local file drifted from upstream; apply hunks by hand |
| `ModuleNotFoundError: yaml` | Install deps with `-c constraints.txt` (includes PyYAML) or ensure `PyYAML` in `requirements.txt` |
| No facet sidebar | No `.. meta::` keys overlapping `search_result_meta`, or index built before meta was added |
| Result titles contain `` | `exclude_selectors: [a.headerlink]` missing / not loaded (run `make pagefind` from repo root so `pagefind.yml` is found) |

## Kit layout

```text
pagefind-migration-kit/
  IMPLEMENTATION.md      ← this guide
  FILE_MANIFEST.md       ← inventory + exclusions
  apply.sh               ← copy files + apply patches
  files/                 ← new files (repo-relative tree)
  patches/               ← pagefind-only diffs vs upstream rolling
```
