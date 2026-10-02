# Copyright 2026 Open Robotics and contributors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Sphinx directive for related article lists, generated at build time.

Matching rules
--------------
- A page includes the ``.. ros-related-articles::`` directive and must declare
  ``area`` via ``.. meta::``. ``area`` may hold several values separated by
  commas, ordered so that the most specific value comes first (for example
  ``debugging, introspection, tools, framework`` or ``nodes, framework``).
- The first ``area`` value is the primary (lowest level) area of the page.
  Related articles are those that include that primary value **anywhere** in
  their own ``area`` list. Matching on the primary value avoids flooding the
  list with every page that only shares a broader parent such as ``framework``.
- Results are ordered by ``content-type`` (About, Process overview, Learning
  path, How-to, Tutorial, Example, Reference), then by title. Pages without a
  content type sort after known types.
- When the current page's ``content-type`` is ``tutorial``, the directive emits
  two lists: other tutorials first, then remaining related articles (tutorials
  removed so they are not duplicated).
- When the current page's ``content-type`` is ``navigation``, or the directive
  option ``:layout: by-area`` is set, the directive emits one list per ``area``
  value on the page (``Articles about <Value>:``), each listing articles that
  contain that value anywhere in ``area``.
- ``experience`` is not used for matching. ``area`` is required.
- The page itself is excluded. Adjacent manual lists are merged with
  duplicates removed. ``:max:`` limits each generated list (default: no cap).
  When a list has more than 10 items, extras are collapsed behind a
  ``Show N more articles`` control (same pattern as related packages).

Build flow: all documents are indexed on ``env-updated``, then each directive
is resolved into static bullet lists on ``doctree-resolved``.
"""

from __future__ import annotations

from typing import List, Set, TypedDict

from docutils import nodes
from docutils.parsers.rst import directives
from sphinx.util.docutils import SphinxDirective

# Article order by content type.
CONTENT_TYPE_ORDER = (
    'about',
    'process overview',
    'process-overview',
    'learning path',
    'learning-path',
    'how-to',
    'howto',
    'how to',
    'tutorial',
    'example',
    'reference',
    'release-note',
    'release note',
)

_CONTENT_TYPE_RANK = {name: index for index, name in enumerate(CONTENT_TYPE_ORDER)}

# First N list items stay visible; the rest get a Show N more articles control.
DEFAULT_RELATED_ARTICLES_VISIBLE_MAX = 10


def _normalize_field_name(raw: str) -> str:
    """Normalize a metadata key for comparison.

    Case, spaces, hyphens, and underscores are ignored, so ``contentType``,
    ``content-type``, and ``Content-Type`` compare equal.
    """
    name = raw.strip().lower().rstrip(':')
    name = name.replace('_', '-').replace(' ', '-')
    return name.replace('-', '')


def _field_value_from_doctree(document: nodes.document, wanted: str) -> str | None:
    """Return the body of the first matching docinfo/RST field in the document."""
    wanted_norm = _normalize_field_name(wanted)
    for field in document.traverse(nodes.field):
        children = getattr(field, 'children', ()) or ()
        if len(children) < 2:
            continue
        label = children[0].astext()
        if _normalize_field_name(label) != wanted_norm:
            continue
        return children[1].astext().strip()
    return None


def _meta_get(metadata: dict, *names: str) -> str | None:
    """Look up metadata using several possible keys (Sphinx/docutils variants)."""
    for name in names:
        for key, val in metadata.items():
            if not val:
                continue
            if _normalize_field_name(str(key)) == _normalize_field_name(name):
                return str(val).strip()
    return None


def _meta_content_from_docutils(document: nodes.document, meta_name: str) -> str | None:
    """Read ``docutils.nodes.meta`` emitted by ``.. meta::``."""
    wanted = _normalize_field_name(meta_name)
    for node in document.traverse(nodes.meta):
        if _normalize_field_name(str(node.get('name') or '')) != wanted:
            continue
        raw = node.get('content')
        if raw:
            return str(raw).strip()
    return None


def _positive_int_option(argument: str) -> int:
    """Parse a positive integer option for the directive."""
    if argument is None:
        raise ValueError('option requires a number')
    value = int(argument)
    if value < 1:
        raise ValueError('must be positive')
    return value


def _layout_option(argument: str) -> str:
    """Parse ``:layout:`` (``default`` or ``by-area``)."""
    value = (argument or 'default').strip().lower()
    if value not in {'default', 'by-area'}:
        raise ValueError('layout must be default or by-area')
    return value


class RelatedArticle(TypedDict):
    docname: str
    title: str
    area_tokens: List[str]
    experience: str
    content_type: str


def _normalized_value(raw: str) -> str:
    """Normalize a metadata value for stable matching."""
    return ' '.join(raw.strip().lower().split())


def _area_tokens(raw: str) -> List[str]:
    """Split an ``area`` value into ordered, normalized tokens (most specific first)."""
    tokens: List[str] = []
    for part in raw.split(','):
        token = _normalized_value(part)
        if token and token not in tokens:
            tokens.append(token)
    return tokens


def _content_type_rank(content_type: str) -> int:
    """Return sort rank for a content type (unknown types sort last)."""
    return _CONTENT_TYPE_RANK.get(content_type, len(CONTENT_TYPE_ORDER))


def _sort_articles(matches: List[RelatedArticle]) -> List[RelatedArticle]:
    """Order by content type (see CONTENT_TYPE_ORDER), then title."""
    return sorted(
        matches,
        key=lambda item: (
            _content_type_rank(item['content_type']),
            item['title'].lower(),
        ),
    )


def _title_case_area(token: str) -> str:
    """Humanize an area token for a navigation heading."""
    return token.replace('-', ' ').replace('_', ' ').title()


def _previous_sibling(node: nodes.Node) -> nodes.Node | None:
    """Return the node immediately before *node* among its parent's children."""
    parent = node.parent
    if parent is None:
        return None
    children = parent.children
    idx = children.index(node)
    if idx == 0:
        return None
    return children[idx - 1]


def _next_sibling(node: nodes.Node) -> nodes.Node | None:
    """Return the node immediately after *node* among its parent's children."""
    parent = node.parent
    if parent is None:
        return None
    children = parent.children
    idx = children.index(node)
    if idx + 1 >= len(children):
        return None
    return children[idx + 1]


def _ensure_class(node: nodes.Element, class_name: str) -> None:
    """Append *class_name* to *node* if it is not already present."""
    classes = list(node.get('classes', []) or [])
    if class_name not in classes:
        classes.append(class_name)
        node['classes'] = classes


def _append_article_items(
    bullet_list: nodes.bullet_list,
    matches: List[RelatedArticle],
    app,
    fromdocname: str,
) -> None:
    """Append related-article links as list items to *bullet_list*."""
    for item in matches:
        refuri = app.builder.get_relative_uri(fromdocname, item['docname'])
        link = nodes.reference('', item['title'], refuri=refuri)
        entry = nodes.list_item()
        para = nodes.paragraph()
        para += link
        entry += para
        bullet_list += entry


def _absorb_bullet_list(
    target: nodes.bullet_list,
    source: nodes.bullet_list,
) -> None:
    """Move all list items from *source* onto the end of *target*."""
    for child in list(source.children):
        if isinstance(child, nodes.list_item):
            source.remove(child)
            target.append(child)


def _docname_from_refuri(app, fromdocname: str, refuri: str) -> str | None:
    """Map an internal ``refuri`` on *fromdocname* to a Sphinx docname."""
    if not refuri or '://' in refuri:
        return None
    refuri_base = refuri.split('#', 1)[0]
    if not refuri_base:
        return None
    builder = app.builder
    for candidate in app.env.found_docs:
        if builder.get_relative_uri(fromdocname, candidate) == refuri_base:
            return candidate
    return None


def _manual_docnames_from_lists(
    app,
    fromdocname: str,
    *bullet_lists: nodes.bullet_list | None,
) -> Set[str]:
    """Collect docnames linked from manual bullet items adjacent to the directive."""
    docnames: Set[str] = set()
    for blist in bullet_lists:
        if blist is None:
            continue
        for ref in blist.traverse(nodes.reference):
            refuri = ref.get('refuri')
            if not refuri:
                continue
            docname = _docname_from_refuri(app, fromdocname, refuri)
            if docname:
                docnames.add(docname)
    return docnames


def _read_page_meta(env, docname: str, doctree: nodes.document | None = None) -> dict:
    """Return area, experience and content-type strings for *docname*."""
    if doctree is None:
        doctree = env.get_doctree(docname)
    meta = env.metadata.get(docname, {})
    area = (
        _meta_content_from_docutils(doctree, 'area')
        or _meta_get(meta, 'area')
        or _field_value_from_doctree(doctree, 'area')
        or ''
    )
    experience = (
        _meta_content_from_docutils(doctree, 'experience')
        or _meta_get(meta, 'experience')
        or _field_value_from_doctree(doctree, 'experience')
        or ''
    )
    content_type = (
        _meta_content_from_docutils(doctree, 'content-type')
        or _meta_content_from_docutils(doctree, 'content_type')
        or _meta_get(meta, 'content-type', 'content_type')
        or _field_value_from_doctree(doctree, 'content-type')
        or _field_value_from_doctree(doctree, 'content_type')
        or ''
    )
    return {
        'area': area,
        'experience': experience,
        'content_type': _normalized_value(content_type),
    }


def _filter_by_area_containment(
    index: List[RelatedArticle],
    fromdocname: str,
    want_area: str,
    manual_docnames: Set[str],
) -> List[RelatedArticle]:
    """Articles that include *want_area* anywhere in their area tokens."""
    want = _normalized_value(want_area)
    if not want:
        return []
    matches = [
        item for item in index
        if item['docname'] != fromdocname
        and item['docname'] not in manual_docnames
        and want in item['area_tokens']
    ]
    return _sort_articles(matches)


def _apply_max(matches: List[RelatedArticle], max_items: int | None) -> List[RelatedArticle]:
    """Optionally truncate *matches*."""
    if max_items is None or max_items < 1:
        return matches
    return matches[:max_items]


def _heading_paragraph(text: str) -> nodes.paragraph:
    """Return a strong-label paragraph used above a related list."""
    para = nodes.paragraph()
    para += nodes.strong(text=text)
    return para


def _new_article_list() -> nodes.bullet_list:
    """Return an empty related-articles bullet list."""
    return nodes.bullet_list(classes=['related-articles'])


def _collapse_article_list(
    bullet_list: nodes.bullet_list,
    visible_max: int = DEFAULT_RELATED_ARTICLES_VISIBLE_MAX,
) -> None:
    """Hide list items beyond *visible_max* for the Show N more articles control.

    Marks extras with ``related-articles__item--extra``; ``related_articles.js``
    inserts the expand button and toggles visibility.
    """
    items = [child for child in bullet_list.children if isinstance(child, nodes.list_item)]
    if len(items) <= visible_max:
        return
    for item in items[visible_max:]:
        _ensure_class(item, 'related-articles__item--extra')


def _resolve_related_articles_list(
    node: RosRelatedArticlesNode,
    matches: List[RelatedArticle],
    app,
    fromdocname: str,
    *,
    lead_nodes: List[nodes.Node] | None = None,
) -> None:
    """Replace *node* with generated links, merging adjacent manual bullet lists."""
    prev = _previous_sibling(node)
    next_sib = _next_sibling(node)
    prev_list = prev if isinstance(prev, nodes.bullet_list) else None
    next_list = next_sib if isinstance(next_sib, nodes.bullet_list) else None

    replacement: List[nodes.Node] = list(lead_nodes or [])

    if prev_list is not None:
        target = prev_list
        _ensure_class(target, 'related-articles')
        # Manual list stays where it is; generated items append to it.
        _append_article_items(target, matches, app, fromdocname)
        if next_list is not None:
            _absorb_bullet_list(target, next_list)
            next_list.replace_self([])
        _collapse_article_list(target)
        if replacement:
            for extra in replacement:
                if isinstance(extra, nodes.bullet_list):
                    _collapse_article_list(extra)
            # Insert extra sections after the merged manual+auto list.
            parent = target.parent
            if parent is not None:
                idx = parent.children.index(target)
                for offset, extra in enumerate(replacement):
                    parent.insert(idx + 1 + offset, extra)
        node.replace_self([])
        return

    target = _new_article_list()
    _append_article_items(target, matches, app, fromdocname)
    if next_list is not None:
        _absorb_bullet_list(target, next_list)
        next_list.replace_self([])
    _collapse_article_list(target)

    for extra in replacement:
        if isinstance(extra, nodes.bullet_list):
            _collapse_article_list(extra)

    replacement.append(target)
    node.replace_self(replacement)


class RosRelatedArticlesNode(nodes.General, nodes.Element):
    """Placeholder node replaced during ``doctree-resolved``."""


class RosRelatedArticlesDirective(SphinxDirective):
    """Emit a placeholder replaced by related article lists.

    Matching uses the page's **primary area** (first ``area`` value). Other
    articles are related when that value appears **anywhere** in their ``area``
    list. Results are ordered by ``content-type``, then title.

    .. code-block:: rst

       .. meta::
          :area: nodes, framework
          :contentType: about
    """

    has_content = False
    required_arguments = 0
    optional_arguments = 0
    option_spec = {
        'max': _positive_int_option,
        'layout': _layout_option,
    }

    def run(self) -> List[nodes.Node]:
        page = _read_page_meta(self.env, self.env.docname, self.state.document)
        if not page['area']:
            raise self.error(
                'ros-related-articles: define `area` with `.. meta::` '
                '(recommended), or field list metadata.'
            )

        node = RosRelatedArticlesNode()
        node['area'] = page['area']
        node['experience'] = page['experience']
        node['content_type'] = page['content_type']
        node['layout'] = self.options.get('layout', 'default')
        # None = uncapped (show the full related list).
        node['max'] = self.options.get('max')
        return [node]


def _collect_article_index(env) -> List[RelatedArticle]:
    """Build an index of docs that declare ``area`` metadata (tokenized)."""
    records: List[RelatedArticle] = []
    for docname in sorted(env.found_docs):
        page = _read_page_meta(env, docname)
        if not page['area']:
            continue
        title_node = env.titles.get(docname)
        title = title_node.astext().strip() if title_node else docname
        records.append({
            'docname': docname,
            'title': title,
            'area_tokens': _area_tokens(page['area']),
            'experience': _normalized_value(page['experience']),
            'content_type': page['content_type'],
        })
    return records


def build_related_articles_index(app, env) -> None:
    """Build metadata map once after Sphinx has read all source documents."""
    env.ros_related_articles_index = _collect_article_index(env)


def resolve_related_articles(app, doctree, fromdocname) -> None:
    """Replace placeholders with static related-article bullet list(s)."""
    index: List[RelatedArticle] = getattr(app.env, 'ros_related_articles_index', [])
    for node in list(doctree.traverse(RosRelatedArticlesNode)):
        area_tokens = _area_tokens(str(node.get('area', '')))
        primary_area = area_tokens[0] if area_tokens else ''
        max_items = node.get('max')
        layout = str(node.get('layout') or 'default')
        page_content_type = _normalized_value(str(node.get('content_type') or ''))

        prev = _previous_sibling(node)
        next_sib = _next_sibling(node)
        prev_list = prev if isinstance(prev, nodes.bullet_list) else None
        next_list = next_sib if isinstance(next_sib, nodes.bullet_list) else None
        manual_docnames = _manual_docnames_from_lists(
            app, fromdocname, prev_list, next_list,
        )

        # Navigation pages: one list per area value on this page.
        if layout == 'by-area' or page_content_type == 'navigation':
            pieces: List[nodes.Node] = []
            for token in area_tokens:
                group = _apply_max(
                    _filter_by_area_containment(
                        index, fromdocname, token, manual_docnames,
                    ),
                    max_items,
                )
                if not group:
                    continue
                pieces.append(
                    _heading_paragraph(f'Articles about {_title_case_area(token)}:')
                )
                blist = _new_article_list()
                _append_article_items(blist, group, app, fromdocname)
                _collapse_article_list(blist)
                pieces.append(blist)
            if prev_list is not None and next_list is not None:
                _absorb_bullet_list(prev_list, next_list)
                next_list.replace_self([])
            if not pieces:
                node.replace_self([])
                continue
            # Keep any manual list; append generated sections after the directive.
            if prev_list is not None:
                parent = prev_list.parent
                idx = parent.children.index(node) if parent is not None else -1
                node.replace_self([])
                if parent is not None and idx >= 0:
                    for offset, extra in enumerate(pieces):
                        parent.insert(idx + offset, extra)
            else:
                node.replace_self(pieces)
            continue

        matches = _filter_by_area_containment(
            index, fromdocname, primary_area, manual_docnames,
        )

        # Tutorials: "More tutorials" then "More related articles".
        if page_content_type == 'tutorial':
            tutorials = [
                item for item in matches if item['content_type'] == 'tutorial'
            ]
            others = [
                item for item in matches if item['content_type'] != 'tutorial'
            ]
            tutorials = _apply_max(tutorials, max_items)
            others = _apply_max(others, max_items)
            lead: List[nodes.Node] = []
            if tutorials:
                # First block uses the normal merge path for manuals + tutorials.
                if others:
                    extra_heading = _heading_paragraph('More related articles:')
                    extra_list = _new_article_list()
                    _append_article_items(extra_list, others, app, fromdocname)
                    _collapse_article_list(extra_list)
                    lead = [extra_heading, extra_list]
                # Prepend a tutorials label when we are about to emit the main list.
                # Manual list (if any) remains unlabeled "essential reading".
                if prev_list is None:
                    # No manuals: label the tutorials list explicitly.
                    wrapper: List[nodes.Node] = [
                        _heading_paragraph('More tutorials:'),
                    ]
                    main = _new_article_list()
                    _append_article_items(main, tutorials, app, fromdocname)
                    wrapper.append(main)
                    if next_list is not None:
                        _absorb_bullet_list(main, next_list)
                        next_list.replace_self([])
                    _collapse_article_list(main)
                    wrapper.extend(lead)
                    node.replace_self(wrapper)
                    continue
                _resolve_related_articles_list(
                    node, tutorials, app, fromdocname, lead_nodes=lead,
                )
                continue
            matches = others

        matches = _apply_max(matches, max_items)

        if not matches:
            if prev_list is not None and next_list is not None:
                _absorb_bullet_list(prev_list, next_list)
                next_list.replace_self([])
            node.replace_self([])
            continue

        _resolve_related_articles_list(node, matches, app, fromdocname)


def setup(app) -> dict:
    """Register the directive, placeholder node and index/render hooks with Sphinx."""
    app.add_directive('ros-related-articles', RosRelatedArticlesDirective)
    app.add_node(RosRelatedArticlesNode)
    app.connect('env-updated', build_related_articles_index)
    app.connect('doctree-resolved', resolve_related_articles)
    return {
        'parallel_read_safe': True,
        'parallel_write_safe': True,
    }
