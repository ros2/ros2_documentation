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

"""Sphinx directive for build-time related article lists."""

from __future__ import annotations

from typing import List, TypedDict

from docutils import nodes
from docutils.parsers.rst import directives
from sphinx.util.docutils import SphinxDirective


def _normalize_field_name(raw: str) -> str:
    """Normalize a metadata key for comparison (e.g. ``Experience`` -> ``experience``)."""
    name = raw.strip().lower().rstrip(':')
    return name.replace(' ', '-')


def _field_value_from_doctree(document: nodes.document, wanted: str) -> str | None:
    """Return the body of the first matching docinfo/rST field in the document."""
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
    for node in document.traverse(nodes.meta):
        if node.get('name') != meta_name:
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


class RelatedArticle(TypedDict):
    docname: str
    title: str
    area: str
    experience: str


def _normalized_value(raw: str) -> str:
    """Normalize metadata value for stable matching."""
    return ' '.join(raw.strip().lower().split())


class RosRelatedArticlesNode(nodes.General, nodes.Element):
    """Placeholder node replaced during ``doctree-resolved``."""


class RosRelatedArticlesDirective(SphinxDirective):
    """Emit a placeholder for static related-article links.

    Uses page metadata values from ``.. meta::``:

    .. code-block:: rst

       .. meta::
          :area: Tutorials
          :experience: Beginner
    """

    has_content = False
    required_arguments = 0
    optional_arguments = 0
    option_spec = {'max': _positive_int_option}

    def run(self) -> List[nodes.Node]:
        meta = self.env.metadata.get(self.env.docname, {})
        area = (
            _meta_content_from_docutils(self.state.document, 'area')
            or _meta_get(meta, 'area')
            or _field_value_from_doctree(self.state.document, 'area')
            or ''
        )
        experience = (
            _meta_content_from_docutils(self.state.document, 'experience')
            or _meta_get(meta, 'experience')
            or _field_value_from_doctree(self.state.document, 'experience')
            or ''
        )

        if not area or not experience:
            raise self.error(
                'ros-related-articles: define both `area` and `experience` '
                'with `.. meta::` (recommended), or field list metadata.'
            )

        node = RosRelatedArticlesNode()
        node['area'] = area
        node['experience'] = experience
        node['max'] = self.options.get('max', 10)
        return [node]


def _collect_article_index(env) -> List[RelatedArticle]:
    """Build an index of docs that declare both ``area`` and ``experience`` metadata."""
    records: List[RelatedArticle] = []
    for docname in sorted(env.found_docs):
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
        if not area or not experience:
            continue
        title_node = env.titles.get(docname)
        title = title_node.astext().strip() if title_node else docname
        records.append({
            'docname': docname,
            'title': title,
            'area': _normalized_value(area),
            'experience': _normalized_value(experience),
        })
    return records


def build_related_articles_index(app, env) -> None:
    """Build metadata map once after Sphinx has read all source documents."""
    env.ros_related_articles_index = _collect_article_index(env)


def resolve_related_articles(app, doctree, fromdocname) -> None:
    """Replace placeholders with static paragraph + list markup."""
    index: List[RelatedArticle] = getattr(app.env, 'ros_related_articles_index', [])
    for node in list(doctree.traverse(RosRelatedArticlesNode)):
        area = _normalized_value(str(node.get('area', '')))
        experience = _normalized_value(str(node.get('experience', '')))
        max_items = int(node.get('max', 10))

        matches = [
            item for item in index
            if item['docname'] != fromdocname
            and item['area'] == area
            and item['experience'] == experience
        ]
        matches.sort(key=lambda item: item['title'].lower())
        matches = matches[:max_items]

        if not matches:
            node.replace_self([])
            continue

        container = nodes.container(classes=['related-articles'])
        intro = nodes.paragraph()
        intro += nodes.Text('Related articles:')
        container += intro

        bullets = nodes.bullet_list()
        for item in matches:
            refuri = app.builder.get_relative_uri(fromdocname, item['docname'])
            link = nodes.reference('', item['title'], refuri=refuri)
            entry = nodes.list_item()
            para = nodes.paragraph()
            para += link
            entry += para
            bullets += entry
        container += bullets

        node.replace_self(container)


def setup(app):
    app.add_directive('ros-related-articles', RosRelatedArticlesDirective)
    app.add_node(RosRelatedArticlesNode)
    app.connect('env-updated', build_related_articles_index)
    app.connect('doctree-resolved', resolve_related_articles)
    return {
        'parallel_read_safe': True,
        'parallel_write_safe': True,
        'version': '1.0.0',
    }
