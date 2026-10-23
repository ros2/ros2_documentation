# Copyright 2026 Open Robotics — Pagefind metadata for ROS 2 documentation
"""
Emit SEO ``<meta>`` tags and Pagefind ``data-pagefind-meta`` / ``data-pagefind-filter``
attributes from ``.. meta::`` fields on each page.

Facet keys and labels come from ``search_result_meta`` in ``pagefind.yml``.
Only those keys receive Pagefind filter and meta attributes. Other fields stay
plain ``<meta>`` tags for SEO.
"""

from __future__ import annotations

import html
from pathlib import PurePosixPath
from typing import Any, Dict, List, Optional

from docutils import nodes

from meta_util import all_doctree_meta, expand_all_meta_values, split_meta_values
from pagefind_config import parse_search_ui_config

# Separator for Sphinx toctree parent titles on search result cards.
_BREADCRUMB_SEPARATOR = ' › '


def _macros_flat(app) -> Dict[str, str]:
    macros = getattr(app.config, 'macros', {}) or {}
    return {str(k): str(v) for k, v in macros.items()}


def _resolved_page_meta(app, doctree: Optional[nodes.document]) -> Dict[str, str]:
    raw = all_doctree_meta(doctree)
    return expand_all_meta_values(raw, _macros_flat(app))


def _current_distro_from_config(app) -> str:
    """Build-time distro for the current Sphinx target (``macros['DISTRO']``)."""
    return str((getattr(app.config, 'macros', {}) or {}).get('DISTRO', 'rolling'))


def _breadcrumb_from_parents(parents: Any) -> str:
    """Join Sphinx ``parents`` titles (no Home, no current page) with `` › ``."""
    if not isinstance(parents, (list, tuple)):
        return ''
    titles: List[str] = []
    for parent in parents:
        if not isinstance(parent, dict):
            continue
        title = str(parent.get('title') or '').strip()
        if title:
            titles.append(title)
    return _BREADCRUMB_SEPARATOR.join(titles)


def _breadcrumb_meta_tag(crumb: str) -> str:
    """Pagefind meta tag for the result-card breadcrumb (not a facet filter)."""
    esc_val = html.escape(crumb, quote=True)
    return (
        f'<meta name="breadcrumb" content="{esc_val}" '
        f'data-pagefind-meta="breadcrumb[content]">'
    )


def _seo_and_filter_metas(fields: List[Dict[str, str]], values: Dict[str, str]) -> str:
    """One ``<meta>`` per value; facet keys get Pagefind filter and meta attrs."""
    facet_keys = {field['key'] for field in fields}
    lines: List[str] = []
    for key in sorted(values.keys()):
        esc_name = html.escape(key, quote=True)
        raw = values.get(key, '')
        # Only facet keys are comma-split into multiple tags; SEO fields stay whole.
        parts = (
            split_meta_values(raw)
            if key in facet_keys
            else ([raw.strip()] if raw and raw.strip() else [])
        )
        for value in parts:
            esc_val = html.escape(value, quote=True)
            if key in facet_keys:
                lines.append(
                    f'<meta name="{esc_name}" content="{esc_val}" '
                    f'data-pagefind-filter="{esc_name}[content]" '
                    f'data-pagefind-meta="{esc_name}[content]">'
                )
            else:
                lines.append(f'<meta name="{esc_name}" content="{esc_val}">')
    return '\n    '.join(lines)


def _pagefind_bundle_prefix(app, pagename: str) -> str:
    """Relative URL prefix from the current HTML page to the site-root ``pagefind/`` directory.

    For ``sphinx-multiversion``, each distro is built with ``pagename`` relative to that
    distro tree (for example ``index``), but HTML is served under ``/{smv_current_version}/``.
    The Pagefind bundle lives at the site root (``build/html/pagefind/``), so add one
    ``../`` when ``smv_current_version`` is set.
    """
    builder = getattr(app, 'builder', None)
    if builder is not None:
        target_uri = builder.get_target_uri(pagename, typ='html')
        depth = len(PurePosixPath(target_uri).parent.parts)
    else:
        depth = pagename.count('/')

    version = getattr(app.config, 'smv_current_version', '') or ''
    if version:
        depth += 1

    if depth == 0:
        return './pagefind/'
    return ('../' * depth) + 'pagefind/'


def _html_page_context(
    app,
    pagename: str,
    templatename: str,
    context: Dict[str, Any],
    doctree,
) -> None:
    ui = parse_search_ui_config(app.config.pagefind_search)
    fields = ui.result_meta
    context.update({
        'pagefind_enabled': bool(app.config.pagefind_enabled),
        'pagefind_seo_filter_metas': '',
        'pagefind_bundle_prefix': _pagefind_bundle_prefix(app, pagename),
        'pagefind_filter_keys_csv': ','.join(field['key'] for field in fields),
        'pagefind_result_meta_fields': fields,
        'pagefind_always_show_filters': ui.always_show_filters,
        'pagefind_current_distro': _current_distro_from_config(app),
        'pagefind_show_breadcrumb': ui.show_breadcrumb,
        'pagefind_show_metadata': ui.show_metadata,
        'pagefind_excerpt_length': ui.excerpt_length,
    })

    if app.builder.format != 'html' or templatename is None:
        return
    if not templatename.endswith('.html'):
        return

    values = _resolved_page_meta(app, doctree)
    meta_html = _seo_and_filter_metas(fields, values)
    if ui.show_breadcrumb:
        crumb = _breadcrumb_from_parents(context.get('parents'))
        if crumb:
            crumb_tag = _breadcrumb_meta_tag(crumb)
            meta_html = f'{meta_html}\n    {crumb_tag}' if meta_html else crumb_tag
    context['pagefind_seo_filter_metas'] = meta_html


def setup(app) -> Dict[str, Any]:
    app.connect('html-page-context', _html_page_context)
    return {
        'parallel_read_safe': True,
        'parallel_write_safe': True,
        'version': '1.0.0',
    }
