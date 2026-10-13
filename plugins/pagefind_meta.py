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
import re
from pathlib import PurePosixPath
from typing import Any, Dict, List, Optional, Tuple

from docutils import nodes

from meta_util import all_doctree_meta, expand_all_meta_values, split_meta_values
from pagefind_config import load_always_show_filters, load_search_result_meta_order


def _macros_flat(app) -> Dict[str, str]:
    macros = getattr(app.config, 'macros', {}) or {}
    return {str(k): str(v) for k, v in macros.items()}


def _resolved_page_meta(app, doctree: Optional[nodes.document]) -> Dict[str, str]:
    raw = all_doctree_meta(doctree)
    return expand_all_meta_values(raw, _macros_flat(app))


def _default_filter_label(key: str) -> str:
    spaced = re.sub(r'([a-z])([A-Z])', r'\1 \2', key)
    return spaced.replace('_', ' ').replace('-', ' ').strip().title()


def _parse_result_meta_fields(raw: Any) -> List[Dict[str, str]]:
    """Build an ordered ``{key, label}`` list from ``search_result_meta``."""
    if not isinstance(raw, dict):
        return []
    out: List[Dict[str, str]] = []
    for key, label in raw.items():
        k = str(key).strip()
        if not k:
            continue
        lbl = str(label).strip() if label is not None else ''
        out.append({'key': k, 'label': lbl or _default_filter_label(k)})
    return out


def _facet_filter_keys(fields: List[Dict[str, str]]) -> List[str]:
    """Facet keys in ``search_result_meta`` order."""
    return [field['key'] for field in fields if field.get('key')]


def _facet_key_set(fields: List[Dict[str, str]]) -> set[str]:
    return set(_facet_filter_keys(fields))


def _current_distro_from_config(app) -> str:
    """Build-time distro for the current Sphinx target (``macros['DISTRO']``)."""
    return str((getattr(app.config, 'macros', {}) or {}).get('DISTRO', 'rolling'))


def _search_config(app) -> Tuple[List[Dict[str, str]], List[str]]:
    """``(result meta fields, always-show filter keys)`` from ``pagefind.yml``."""
    cached = getattr(app, '_ros_pagefind_search_config', None)
    if cached is not None:
        return cached
    fields = _parse_result_meta_fields(load_search_result_meta_order(app.confdir))
    always = load_always_show_filters(app.confdir)
    cached = (fields, always)
    app._ros_pagefind_search_config = cached
    return cached


def _seo_and_filter_metas(fields: List[Dict[str, str]], values: Dict[str, str]) -> str:
    """One ``<meta>`` per value; facet keys get Pagefind filter and meta attrs."""
    facet_keys = _facet_key_set(fields)
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

    Must start with ``./`` or ``../`` so the browser resolves dynamic imports (for example
    ``import(bundlePath + 'pagefind.js')``) as URLs, not bare module specifiers.

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


def _pagefind_component_urls(app, pagename: str) -> Tuple[str, str]:
    """``(css_href, js_href)`` relative to the current page."""
    prefix = _pagefind_bundle_prefix(app, pagename)
    return prefix + 'pagefind-component-ui.css', prefix + 'pagefind-component-ui.js'


def _search_results_href(app, pagename: str) -> str:
    """Relative URL from the current page to Sphinx's ``search.html``.

    Uses the HTML builder's relative URI helper so multiversion pages under
    ``/{distro}/`` link to ``/{distro}/search.html``, not site-root
    ``/search.html`` (which may be wrong after ``make multiversion``).
    """
    builder = getattr(app, 'builder', None)
    if builder is None:
        return 'search.html'
    try:
        rel = builder.get_relative_uri(pagename, 'search')
        if rel:
            return rel
    except (AttributeError, KeyError, ValueError):
        pass
    return 'search.html'


def _html_page_context(
    app,
    pagename: str,
    templatename: str,
    context: Dict[str, Any],
    doctree,
) -> None:
    fields, always_show = _search_config(app)
    current_distro = _current_distro_from_config(app)
    context.update({
        'pagefind_seo_filter_metas': '',
        'pagefind_bundle_prefix': './pagefind/',
        'pagefind_component_css': './pagefind/pagefind-component-ui.css',
        'pagefind_component_js': './pagefind/pagefind-component-ui.js',
        'pagefind_filter_keys_csv': ','.join(_facet_filter_keys(fields)),
        'pagefind_result_meta_fields': fields,
        'pagefind_always_show_filters': always_show,
        'pagefind_search_results_href': 'search.html',
        'pagefind_current_distro': current_distro,
    })

    if app.builder.format != 'html' or templatename is None:
        return
    if not templatename.endswith('.html'):
        return

    values = _resolved_page_meta(app, doctree)
    css_href, js_href = _pagefind_component_urls(app, pagename)
    context['pagefind_seo_filter_metas'] = _seo_and_filter_metas(fields, values)
    context['pagefind_bundle_prefix'] = _pagefind_bundle_prefix(app, pagename)
    context['pagefind_component_css'] = css_href
    context['pagefind_component_js'] = js_href
    context['pagefind_search_results_href'] = _search_results_href(app, pagename)


def setup(app) -> Dict[str, Any]:
    app.connect('html-page-context', _html_page_context)
    return {
        'parallel_read_safe': True,
        'parallel_write_safe': True,
        'version': '1.0.0',
    }
