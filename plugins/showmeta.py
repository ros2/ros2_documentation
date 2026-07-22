# Copyright 2026 Open Robotics — explicit in-body ``.. showmeta::`` summary
"""
Render selected ``.. meta::`` fields in the document body with author-controlled
order and labels. Place ``.. showmeta::`` where the summary should appear (HTML only).
"""

from __future__ import annotations

import html as html_module
import re
from typing import List

from docutils import nodes
from docutils.parsers.rst import directives
from sphinx.util.docutils import SphinxDirective

from meta_util import all_doctree_meta, expand_all_meta_values


def _macros_flat(app) -> dict[str, str]:
    return {str(k): str(v) for k, v in (getattr(app.config, 'macros', {}) or {}).items()}


def _default_showmeta_label(key: str) -> str:
    spaced = re.sub(r'([a-z])([A-Z])', r'\1 \2', key)
    return spaced.replace('_', ' ').replace('-', ' ').strip().title()


class showmeta_node(nodes.General, nodes.Element):
    """Placeholder replaced on ``doctree-resolved`` (HTML builds only)."""


class ShowMetaDirective(SphinxDirective):
    """Insert a visible metadata line built from ``.. meta::`` on this page."""

    has_content = False
    option_spec = {
        'order': directives.unchanged,
        'labels': directives.unchanged,
    }

    def run(self) -> List[nodes.Node]:
        node = showmeta_node()
        node['order'] = self.options.get('order', '')
        node['labels'] = self.options.get('labels', '')
        self.set_source_info(node)
        return [node]


def visit_skip_showmeta(self, node: showmeta_node) -> None:
    raise nodes.SkipNode


def depart_showmeta_noop(self, node: showmeta_node) -> None:
    pass


def _parse_labels(raw: str) -> dict[str, str]:
    out: dict[str, str] = {}
    for part in [p.strip() for p in raw.split(',') if p.strip() and '=' in p]:
        key, _, value = part.partition('=')
        key, value = key.strip(), value.strip()
        if key:
            out[key] = value
    return out


def replace_showmeta_nodes(app, doctree: nodes.document, docname: str) -> None:
    if app.builder.format != 'html':
        for node in list(doctree.findall(showmeta_node)):
            node.parent.remove(node)
        return

    macros = _macros_flat(app)
    meta = expand_all_meta_values(all_doctree_meta(doctree), macros)

    for node in list(doctree.findall(showmeta_node)):
        order = [x.strip() for x in node.get('order', '').split(',') if x.strip()]
        labels_map = _parse_labels(node.get('labels', ''))
        if not order:
            node.parent.remove(node)
            continue

        parts: List[str] = []
        for key in order:
            val = meta.get(key, '').strip()
            if not val:
                continue
            label_base = labels_map.get(key) or _default_showmeta_label(key)
            label_display = label_base if label_base.rstrip().endswith(':') else f'{label_base}:'
            parts.append(
                f'<strong>{html_module.escape(label_display)}</strong> '
                f'{html_module.escape(val)}'
            )

        if not parts:
            node.parent.remove(node)
        else:
            inner = ' | '.join(parts)
            raw = nodes.raw(
                '',
                f'<p class="ros-page-meta-summary" data-pagefind-ignore="all">{inner}</p>',
                format='html',
            )
            node.replace_self(raw)


def setup(app):
    app.add_node(
        showmeta_node,
        html=(visit_skip_showmeta, depart_showmeta_noop),
        latex=(visit_skip_showmeta, depart_showmeta_noop),
    )
    app.add_directive('showmeta', ShowMetaDirective)
    app.connect('doctree-resolved', replace_showmeta_nodes)
    return {
        'version': '1.0.0',
        'parallel_read_safe': True,
        'parallel_write_safe': True,
    }
