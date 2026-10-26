# Related articles

Sphinx extension that generates related-article lists from page metadata, so a page can point readers at its siblings without an author maintaining the links by hand.

---

## User guide

Information for documentation authors adding a related-articles list to an `.rst` page.

The `.. ros-related-articles::` directive is replaced **during the Sphinx build** with one or more bullet lists of links to other pages in this documentation set. The links are ordinary HTML in the built page — nothing is fetched when a reader opens it. JavaScript promotes the author-written intro to a heading and expands a long list.

### What the directive does

| Scope | Detail |
|-------|--------|
| **When** | At documentation build time, after every page has been read |
| **Which pages** | Only pages that include `.. ros-related-articles::` and declare a non-empty `area` |
| **What is listed** | Other pages whose `area` contains this page's **primary** (first) `area` value |
| **When nothing is emitted** | No other page shares that primary area, and there is no adjacent hand-written list to keep |

### Prerequisites

| Requirement | Detail |
|-------------|--------|
| `area` on this page | **Required.** Declared in a `.. meta::` block. The directive raises a build error when it is missing or empty |
| `area` on other pages | Pages that do not declare `area` are never listed |
| `content-type` on this page | Optional. Selects the layout and takes part in ordering |
| `content-type` on other pages | Optional. Controls where a page sorts within the list |

`experience` is read and indexed, but is **not** used for matching or ordering.

### Adding the directive

```rst
.. meta::
   :area: nodes, framework
   :contentType: about

Understanding nodes
===================

Related articles:

.. ros-related-articles::
```

Write the `Related articles:` intro yourself in the source. At page load it is promoted to an `h3` and the trailing colon is removed, so it becomes a real heading in the page structure rather than a stray paragraph.

`area` holds one or more comma-separated values ordered **most specific first**, for example `nodes, framework` or `debugging, introspection, tools, framework`.

Field names match after normalizing case and separators, so `:contentType:` and `:content-type:` are the same field.

### Options

| Option | Values | Default | Purpose |
|--------|--------|---------|---------|
| `:max:` | positive integer | no cap | Limit the number of items in **each** generated list |
| `:layout:` | `default`, `by-area` | `default` | Force one list per `area` value instead of a single list |

### How pages are matched

The **primary area** is the first value in this page's `area`. Another page is related when that primary value appears **anywhere** in its own `area` list.

Matching on the primary value rather than the whole list stops a page tagged `nodes, framework` from pulling in every page that merely shares the broad `framework` parent.

The page itself is always excluded, as is any page already linked from an adjacent hand-written list (see [Mixing in hand-written links](#mixing-in-hand-written-links)).

### How results are ordered

Results sort by `content-type` first, then alphabetically by title. Pages with no content type, or one not in the list below, sort last.

| Order | `content-type` | Accepted spellings |
|-------|----------------|--------------------|
| 1 | About | `about` |
| 2 | Process overview | `process overview`, `process-overview` |
| 3 | Learning path | `learning path`, `learning-path` |
| 4 | How-to | `how-to`, `howto`, `how to` |
| 5 | Tutorial | `tutorial` |
| 6 | Example | `example` |
| 7 | Reference | `reference` |
| 8 | Release note | `release-note`, `release note` |

### Layouts

The layout is chosen from this page's `content-type`, unless `:layout:` overrides it.

| This page's `content-type` | What renders |
|----------------------------|--------------|
| anything not listed below | A single list of related articles |
| `tutorial` | Two blocks: other tutorials first, then **More related articles:** with everything else. A tutorial is never repeated across the two |
| `navigation` | One list per `area` value on the page, each headed **Articles about \<Value\>:**, listing pages that contain that value anywhere in their own `area` |

Setting `:layout: by-area` produces the `navigation` behaviour on any page.

On a tutorial page the first block is labelled **More tutorials:** only when there is no hand-written list above the directive. When there is one, the generated tutorials are appended to it without a new label, so the author's list reads as essential reading followed by more of the same.

### Mixing in hand-written links

A bullet list placed immediately before and/or immediately after the directive is treated as the author's own list and is preserved:

```rst
Related articles:

* :doc:`A page you specifically want first <../Some-Page>`

.. ros-related-articles::
```

Generated links are appended to the list above the directive, and a list below the directive is merged into it. Any page you link by hand is removed from the generated results, so nothing appears twice.

### Long lists

When a list ends up with more than **10** items, the extras are hidden behind a **Show N more articles** button that toggles to **Show fewer**. The cap applies to each generated list separately, and is independent of `:max:` — `:max:` limits how many items exist at all; the cap of 10 limits how many are visible before expanding.

### What happens when the docs are built

From an author's perspective:

1. **No directive on the page** — Nothing related-articles runs for that page.
2. **Directive present, `area` missing** — The build fails with `ros-related-articles: define 'area' with '.. meta::'`.
3. **Directive present, matches found** — The placeholder is replaced with one or more static bullet lists of titles linking to the matching pages. The intro becomes an `h3`.
4. **Directive present, no matches, no hand-written list** — The placeholder is removed. Nothing is rendered.
5. **Directive present, no matches, hand-written list present** — The placeholder is removed and the author's list is left as written.
6. **A generated list has more than 10 items** — Extra items are marked in the HTML; the expand control is attached in the browser.

#### Outcomes at a glance

| Situation | What the reader sees |
|-----------|----------------------|
| No directive | Nothing |
| Matches found | `Related articles` heading, then one or more related-article lists |
| No matches | Nothing (or only the author's hand-written list) |
| Missing `area` | **Build fails** |

### Troubleshooting

| Symptom | Cause |
|---------|-------|
| Build error: `define 'area' with '.. meta::'` | The page has no `area` metadata, or it is empty |
| Nothing renders where the directive is | No other page shares this page's primary area, and there is no hand-written list to keep |
| A page you expected is missing | It has no `area`, or its `area` does not contain this page's **primary** value |
| Too many loosely related pages appear | The page's `area` starts with a broad value. Reorder it so the most specific value comes first |
| Items appear in an odd order | Those pages have no `content-type`, so they sort after everything that does |

---

## Developer guidance

Information for maintainers and developers working on or extending the related-articles extension.

### Repository layout

| File | Purpose |
|------|---------|
| [`ros_related_articles.py`](ros_related_articles.py) | Sphinx extension: directive, index build, and doctree resolution |
| [`../source/_static/related_articles.js`](../source/_static/related_articles.js) | Promotes the intro heading and expand/collapse for long lists |
| [`../source/_static/custom.css`](../source/_static/custom.css) | Styling for the heading, lists and the expand button |
| [`../conf.py`](../conf.py) | Registers the extension in `extensions` and the script in `html_js_files` |

### Code modules

#### `ros_related_articles.py`

- **`RosRelatedArticlesDirective`** — reads this page's metadata and emits a `RosRelatedArticlesNode` placeholder. Raises a directive error when `area` is missing.
- **`build_related_articles_index`** (`env-updated`) — walks `env.found_docs`, keeps pages that declare `area`, and stores records on `env.ros_related_articles_index`.
- **`resolve_related_articles`** (`doctree-resolved`) — replaces each placeholder with static bullet lists, resolving links with `builder.get_relative_uri`.
- **`_read_page_meta`** — `area`, `experience`, and `content-type` via three fallbacks (see below).
- **`_area_tokens`** — splits `area` on commas into ordered, lowercased, de-duplicated tokens.
- **`_filter_by_area_containment`** — the matching rule; excludes the current page and anything already linked by hand.
- **`_sort_articles`** — content-type rank then lowercased title.
- **`_resolve_related_articles_list`** — merge path: append generated items to a preceding manual list, absorb a following one, collapse, then insert any trailing sections.
- **`_collapse_article_list`** — marks items past the visible cap with `related-articles__item--extra`.
- **`_manual_docnames_from_lists`** — maps `refuri`s in adjacent lists back to docnames by comparing against `builder.get_relative_uri` for every known document.

Metadata is read through three fallbacks in order, because the same field can reach Sphinx by different routes:

1. `_meta_content_from_docutils` — `docutils.nodes.meta` emitted by `.. meta::` (the normal path)
2. `_meta_get` — `env.metadata`, which Sphinx populates for some field forms
3. `_field_value_from_doctree` — a docinfo field list at the top of the page

Building the index once on `env-updated` rather than per directive is what keeps this cheap on a full build.

#### `related_articles.js`

Read-only against the built HTML. It finds every `ul.related-articles`, promotes an adjacent `Related articles:` paragraph to `h3` (stripping the colon), counts `li.related-articles__item--extra`, and inserts a **Show N more articles** button. It sets `hidden` on the extras itself rather than relying only on the stylesheet, so a page still behaves correctly if `custom.css` fails to load. It is idempotent: it will not attach a second button to a list that already has one, and it will not rewrite an intro that is already a heading.

### Extending

| Change | Where |
|--------|-------|
| New `content-type` in the sort order | Add every accepted spelling to `CONTENT_TYPE_ORDER` in [`ros_related_articles.py`](ros_related_articles.py) |
| Different visible-item cap | `DEFAULT_RELATED_ARTICLES_VISIBLE_MAX` (currently `10`) |
| New layout | Branch in `resolve_related_articles`; add the name to `_layout_option` if authors should be able to select it |
| Start matching on `experience` | `_filter_by_area_containment` (or a new filter). `experience` is already indexed |

`experience` is indexed but unused for matching and ordering.

### Tunable constants

| Constant | Current | Effect |
|----------|---------|--------|
| `CONTENT_TYPE_ORDER` | 8 types plus spelling variants | Sort order; unknown types rank last |
| `DEFAULT_RELATED_ARTICLES_VISIBLE_MAX` | `10` | Items visible before the expand control appears |

### Generated markup

| Hook | Where it comes from |
|------|---------------------|
| `.related-articles__heading` | The promoted `h3` |
| `ul.related-articles` | Every generated or merged list |
| `li.related-articles__item--extra` | Items past the visible cap; hidden by CSS and by `hidden` in JS |
| `ul.related-articles.is-expanded` | Added while the list is expanded |
| `button.related-articles__expand` | Inserted by the script after the list |

### Registration

`plugins/` is already on `sys.path` via `sys.path.append(os.path.abspath('plugins'))` in [`conf.py`](../conf.py), so the extension is registered by bare module name. Imports between plugin modules must be absolute, not relative.

```python
extensions = [
    ...
    'ros_related_articles',
]

html_js_files = [
    ...
    'related_articles.js',
]
```

`setup()` declares `parallel_read_safe` and `parallel_write_safe`, so the extension is safe under `make html`'s parallel build. The extension only ever emits docutils nodes, so non-HTML builders get plain bullet lists with working cross-references and simply no expand control.

### Tests

There is no dedicated unit suite for this extension. Changes are verified by building the docs and inspecting a page that includes the directive.

### Previewing locally

From the repository root:

```bash
make html
python -m http.server -d build/html 8000
```

No network access or extra services are needed — everything this extension produces is resolved at build time.
