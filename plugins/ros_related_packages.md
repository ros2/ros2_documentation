# Related packages

Sphinx extension that lists the ROS packages relevant to a page, matched against package metadata published in the rosdistro cache.

---

## User guide

Information for documentation authors adding a related-packages list to an `.rst` page.

Unlike related articles, these lists are **built in the browser**. The directive emits an empty placeholder at build time; when a reader opens the page the script downloads the rosdistro cache for the distribution being viewed, filters it, and fills the placeholder in. This keeps the lists current between documentation builds and avoids baking thousands of package names into the HTML.

### What the directive does

| Scope | Detail |
|-------|--------|
| **When (build)** | The directive writes a placeholder `div` and the build may download a cache snapshot into `_static/` |
| **When (page load)** | The browser fetches the cache, filters packages, and renders the lists |
| **Which pages** | Only pages that include `.. ros-related-packages::` and declare a non-empty `area` |
| **What is listed** | Released packages whose `<area>` export contains this page's **primary** (first) `area` value |
| **When skipped** | Pages without the directive. A missing `area` fails the build rather than skipping |

### Prerequisites

| Requirement | Detail |
|-------------|--------|
| `area` on this page | **Required.** Declared in a `.. meta::` block, or passed as `:area:` on the directive. The directive raises a build error when neither is present |
| `<area>` on the package | **Required.** An export in the package's `package.xml`. Packages without it are never listed |
| `<related_scope>` on the package | Optional. Decides which of the two groups the package appears under |
| Network access at build time | The build downloads the rosdistro cache into `_static/`. Without it the page falls back to whatever snapshot is already there |

### Adding the directive

```rst
.. meta::
   :area: nodes, framework

Understanding nodes
===================

Related packages:

.. ros-related-packages::
```

Write the `Related packages:` intro yourself in the source. At page load it is promoted to an `h3` and the trailing colon is removed, so it becomes a real heading in the page structure rather than a stray paragraph.

### Options

| Option | Values | Default | Purpose |
|--------|--------|---------|---------|
| `:area:` | comma-separated values | the page's `.. meta::` `area` | Override the area used for matching on this directive only |
| `:max:` | positive integer | no cap | Limit the number of packages in **each** group |

### Package metadata

Matching relies on two custom exports in the package's `package.xml`. Anything inside `<export>` that a tool does not recognise is ignored by the rest of the ROS toolchain (see [REP 149](https://ros.org/reps/rep-0149.html)), so adding these is safe:

```xml
<export>
  <build_type>ament_cmake</build_type>
  <area>nodes, framework</area>
  <related_scope>core</related_scope>
</export>
```

| Export | Values | Effect |
|--------|--------|--------|
| `<area>` | comma-separated values, most specific first | Decides which pages the package appears on. Several `<area>` elements are also accepted |
| `<related_scope>` | `core` or `federation` | Decides the group. Missing, empty or unrecognised values are treated as `federation` |

The package's `<description>` is shown after the package name, so its quality is visible to readers.

### How packages are matched

The **primary area** is the first value in the page's `area`. A package is listed when that value appears **anywhere** in its `<area>` export. Matching on the primary value rather than the whole list is deliberate: it stops a page tagged `nodes, framework` from listing every package that merely shares `framework`.

### The two groups

Matching packages are split under `h4` subheadings and each group is sorted alphabetically:

| Heading | `<related_scope>` |
|---------|-------------------|
| Core ROS packages | `core` |
| Community packages | `federation`, or no `related_scope` export at all |

### Mixing in hand-written links

A bullet list immediately before and/or after the directive is treated as the author's own:

```rst
Related packages:

* `rclcpp <https://docs.ros.org/en/rolling/p/rclcpp/>`_

.. ros-related-packages::
```

Each bullet's package name is recognised from the `/p/<name>/` segment of its link, or from the link text when it is a single word. What happens next depends on whether it also matches by area:

- **It matches.** The plain bullet is removed and the package is shown in its proper group instead, gaining the description and the correct heading.
- **It does not match.** It stays where you put it, under the author-written `Related packages` heading.

Either way a package appears exactly once. A manual list left empty after absorption is removed, and a list after the directive is merged into the one before it.

### Long lists

Each group shows its first **7** packages, with the rest behind a **Show N more packages** button that toggles to **Show fewer**. The two groups get their own button. This visible cap is separate from `:max:`, which limits how many packages are in the group at all.

### What happens when a reader opens the page

1. **No directive on the page** — The script finds no widgets and exits.
2. **Directive present, cache loads, matches found** — The intro becomes an `h3`. Non-empty groups are inserted as `h4` + list. Matching hand-written bullets are absorbed into those groups.
3. **Directive present, cache loads, no matches, no hand-written list** — *No packages matched this filter.*
4. **Directive present, cache loads, no matches, hand-written list present** — The widget is removed and the author's list is left alone.
5. **Cache cannot be loaded** — *Could not load package metadata…* and a suggestion to rebuild while online.

#### Outcomes at a glance

| Situation | What the reader sees |
|-----------|----------------------|
| Matches found | `Related packages` heading, then the non-empty Core / Community groups |
| No matches, manual list present | The author's list only |
| No matches, no manual list | *No packages matched this filter.* |
| Cache unavailable | Error status on the placeholder |
| Missing `area` at build time | **Build fails** |

### Troubleshooting

| Symptom | Cause |
|---------|-------|
| Build error: `define 'area' with '.. meta::'` | The page has no `area` metadata and no `:area:` option |
| Every page shows *Could not load package metadata* | The cache is missing from `_static/rosdistro_cache/`. Rebuild while online |
| Lists are empty everywhere | No released package carries an `<area>` export yet. See [Where the data comes from](#where-the-data-comes-from) |
| A package is in the wrong group | Its `<related_scope>` is missing or misspelled, so it defaulted to Community |
| Descriptions look poor | They come verbatim from each package's `<description>`; the fix belongs upstream in that package |

---

## Developer guidance

Information for maintainers and developers working on or extending the related-packages extension.

### Repository layout

| File | Purpose |
|------|---------|
| [`ros_related_packages.py`](ros_related_packages.py) | Sphinx extension: directive, config value, and the build-time cache download |
| [`../source/_static/related_packages.js`](../source/_static/related_packages.js) | Fetches, parses and filters the cache, then builds the lists |
| [`../source/_static/vendor/pako.min.js`](../source/_static/vendor/pako.min.js) | Gzip decompression in the browser |
| [`../source/_static/vendor/js-yaml.min.js`](../source/_static/vendor/js-yaml.min.js) | YAML parsing in the browser |
| [`../source/_static/custom.css`](../source/_static/custom.css) | Styling for the headings, lists and expand button |
| [`../tools/rosdistro_cache_proxy.py`](../tools/rosdistro_cache_proxy.py) | Standalone local proxy for the cache endpoint |
| [`../tools/serve_docs_with_proxy.py`](../tools/serve_docs_with_proxy.py) | Serves built HTML and the cache endpoint on one origin |
| [`../conf.py`](../conf.py) | Registers the extension, the scripts, and the proxy URL setting |

### Where the data comes from

Everything is derived from one file, `{distro}-cache.yaml.gz`, published by the ROS build farm at `https://repo.ros2.org/rosdistro_cache/`. Its `release_package_xmls` key maps package names to the full text of each released `package.xml`, which is what the script filters.

The browser tries three sources in order:

| Order | Source | Freshness | Caveat |
|-------|--------|-----------|--------|
| 1 | Same-origin proxy at `data-proxy-cache-href` | Live | **The production path does not exist yet** |
| 2 | Bundled snapshot in `_static/rosdistro_cache/` | Last docs build | Always same-origin, so this is what actually serves today |
| 3 | `repo.ros2.org` directly | Live | Normally blocked by CORS |

Source 1 is currently an assumption. It is provided locally by [`serve_docs_with_proxy.py`](../tools/serve_docs_with_proxy.py), but in production the hosting layer would need to map `/api/rosdistro-cache/` to `repo.ros2.org/rosdistro_cache/`. The alternative is a CORS header on `repo.ros2.org`, which would make source 3 work and let the proxy be dropped entirely. Until one of those exists the lists render from source 2, which is correct but only as fresh as the last build.

No upstream `package.xml` carries `<area>` or `<related_scope>` yet, so against the real cache these lists match nothing. Both exports have to be adopted upstream before the feature does anything in production.

### Code modules

#### `ros_related_packages.py`

- **`RosRelatedPackagesDirective`** — resolves `area`, then emits a raw HTML `div` with the data attributes the script needs. Raises a directive error when `area` is missing.
- **`download_rosdistro_cache`** (`builder-inited`) — fetches `https://repo.ros2.org/rosdistro_cache/{distro}-cache.yaml.gz` into `source/_static/rosdistro_cache/`, for HTML builders only. A failure logs a warning and does not break the build. The downloaded file is gitignored.
- **`_bundled_cache_href`** — relative URL from this page's HTML file to the gzip in `_static/`, using the docname depth so it stays correct in multiversion builds.
- **`_proxy_cache_href`** — substitutes `{distro}` into the configured proxy template.

#### `related_packages.js`

- **`loadXmls` / `fetchAndParse` / `tryUrls`** — three-source fetch, gzip inflate via `pako`, YAML parse via `js-yaml`. Widgets are grouped by distro so each cache is fetched once per page.
- **`extractAreaTokens` / `matchesArea`** — containment match on the page's primary area.
- **`extractRelatedScope`** — `core` or `federation`; anything else becomes `federation`.
- **`extractDescription`** — prefers `DOMParser`, falls back to a regex, so a malformed `package.xml` anywhere in the cache cannot take out the whole list.
- **`collectManualEntries` / `packageNameFromManualLink`** — recognise hand-written package links.
- **`promoteRelatedPackagesIntro`** — rewrite the adjacent intro paragraph to `h3` when its normalised text is exactly `related packages`.
- **`fillWidget`** — filter, split by scope, absorb matching manuals, prune emptied lists, promote the intro, insert each group's `h4` and list, remove the placeholder.

`promoteRelatedPackagesIntro` walks backwards past any adjacent `ul` to find the intro paragraph. Reword the intro in the RST and the promotion silently stops.

### Extending

| Change | Where |
|--------|-------|
| New `<related_scope>` value | `extractRelatedScope` in [`related_packages.js`](../source/_static/related_packages.js), plus the heading passed to `appendScopedPackageList` |
| Different visible-item cap | `DEFAULT_RELATED_PACKAGES_VISIBLE_MAX` in [`ros_related_packages.py`](ros_related_packages.py) (currently `7`); the script reads it from `data-visible-max` |
| New page-side metadata used for matching | The directive must put it on the placeholder as a `data-*` attribute; the script must read it in `fillWidget` |
| Different cache URL | `ROSDISTRO_CACHE_TEMPLATE` (build download) and `fetchAndParse` (browser fallback) |

Adding a new after-title heading or a third group is a JavaScript change only. The Python side does not know about Core vs Community.

### Configuration

| Setting | Where | Default |
|---------|-------|---------|
| `ros_related_packages_proxy_url` | `conf.py` config value, registered by the extension | `/api/rosdistro-cache/{distro}-cache.yaml.gz` |
| `ROS_RELATED_PACKAGES_PROXY_URL` | Environment variable read in [`conf.py`](../conf.py) | Overrides the above; set it to empty to disable the proxy attempt and use the bundled snapshot only |

[`conf.py`](../conf.py) normalises this value before use: GNU make and MSYS on Windows can rewrite a leading `/api/...` into an absolute Windows path, so `_normalize_ros_related_packages_proxy_url` recovers the intended same-origin path.

### Generated markup

The placeholder written at build time:

```html
<div class="related-packages related-packages--loading js-related-packages"
     data-area="nodes, framework"
     data-max="0"
     data-visible-max="7"
     data-distro="rolling"
     data-bundled-cache-href="../_static/rosdistro_cache/rolling-cache.yaml.gz"
     data-proxy-cache-href="/api/rosdistro-cache/rolling-cache.yaml.gz"
     role="region" aria-live="polite">
```

| Hook | Where it comes from |
|------|---------------------|
| `.js-related-packages` | The placeholder; the script's entry point |
| `.related-packages--loading` / `--error` | State classes on the placeholder |
| `.related-packages__heading` | The promoted `h3` |
| `.related-packages__scope-heading` | The two group `h4`s |
| `ul.related-packages__list` | Each group's list, and any retained manual list |
| `.related-packages__item--extra` | Items past the visible cap |
| `button.related-packages__expand` | Inserted per group |

### Registration

`plugins/` is already on `sys.path` via [`conf.py`](../conf.py), so the extension is registered by bare module name. Imports between plugin modules must be absolute.

```python
extensions = [
    ...
    'ros_related_packages',
]

html_js_files = [
    ('vendor/pako.min.js', {'defer': ''}),
    ('vendor/js-yaml.min.js', {'defer': ''}),
    ...
    'related_packages.js',
]
```

The two vendored libraries must stay ahead of `related_packages.js` in `html_js_files`. `setup()` declares `parallel_read_safe` and `parallel_write_safe`. Non-HTML builders get the raw `div` and no list, since the whole feature is HTML-only by design.

### Tests

There is no dedicated unit suite for this extension. Changes are verified by building the docs, serving them, and inspecting a page that includes the directive.

### Previewing locally

The bundled snapshot alone is enough to see the widget (lists stay empty until packages carry `<area>`):

```bash
make html
python -m http.server -d build/html 8000
```

To exercise the proxy path as well, serve the HTML and the cache endpoint from one origin:

```bash
make html
python tools/serve_docs_with_proxy.py
```

If you would rather keep an existing static server, [`rosdistro_cache_proxy.py`](../tools/rosdistro_cache_proxy.py) serves the same endpoint on its own port. It sends permissive CORS headers so the cross-port request works, but the build has to be pointed at it:

```bash
python tools/rosdistro_cache_proxy.py --port 9000
ROS_RELATED_PACKAGES_PROXY_URL='http://127.0.0.1:9000/api/rosdistro-cache/{distro}-cache.yaml.gz' make html
```

Both tools cache upstream responses for five minutes, so repeated page loads do not hammer `repo.ros2.org`.
