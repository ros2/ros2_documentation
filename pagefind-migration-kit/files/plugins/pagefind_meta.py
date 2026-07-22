# Copyright 2026 Open Robotics — Pagefind metadata for ROS 2 documentation
"""
Emit SEO <meta> tags, Pagefind ``data-pagefind-meta``, and ``data-pagefind-filter``
from ``.. meta::`` fields on each page.

Only keys in ``pagefind_result_meta_order`` receive ``data-pagefind-filter`` (facet
sidebar + filtering). Other meta fields are plain ``<meta>`` for SEO only.
"""

from __future__ import annotations

import html
import re
from pathlib import PurePosixPath
from typing import Any, Dict, List, Optional, Tuple

from docutils import nodes
from sphinx.util import logging

from meta_util import all_doctree_meta, expand_all_meta_values, split_meta_values

logger = logging.getLogger(__name__)


def _macros_flat(app) -> Dict[str, str]:
    macros = getattr(app.config, 'macros', {}) or {}
    return {str(k): str(v) for k, v in macros.items()}


def _resolved_page_meta(app, doctree: Optional[nodes.document]) -> Dict[str, str]:
    raw = all_doctree_meta(doctree)
    return expand_all_meta_values(raw, _macros_flat(app))


def _default_filter_label(key: str) -> str:
    spaced = re.sub(r'([a-z])([A-Z])', r'\1 \2', key)
    return spaced.replace('_', ' ').replace('-', ' ').strip().title()


def _parse_result_meta_fields(app) -> List[Dict[str, str]]:
    """Build ordered ``{key, label}`` list from ``pagefind_result_meta_order`` (dict or legacy list)."""
    raw = getattr(app.config, 'pagefind_result_meta_order', None) or {}
    out: List[Dict[str, str]] = []

    if isinstance(raw, dict):
        for key, label in raw.items():
            k = str(key).strip()
            if not k:
                continue
            lbl = str(label).strip() if label is not None else ''
            out.append({'key': k, 'label': lbl or _default_filter_label(k)})
        return out

    if isinstance(raw, (list, tuple)):
        logger.warning(
            'pagefind_result_meta_order should be a dict mapping field names to labels; '
            'list form is deprecated.',
            type='pagefind',
        )
        for item in raw:
            k = str(item).strip()
            if k:
                out.append({'key': k, 'label': _default_filter_label(k)})
    return out


def _facet_key_set(app) -> set[str]:
    return {field['key'] for field in _parse_result_meta_fields(app)}


def _current_distro_from_config(app) -> str:
    """Build-time distro for the current Sphinx target (``macros['DISTRO']``)."""
    return str((getattr(app.config, 'macros', {}) or {}).get('DISTRO', 'rolling'))


def _facet_filter_keys_for_context(app, env) -> List[str]:
    """Facet keys to expose in the UI, in ``search_result_meta`` order.

    Includes keys present in at least one document's ``.. meta::``, plus keys
    listed in ``pagefind_always_show_filters`` (empty placeholder sections).
    """
    corpus = set(_union_meta_keys(env))
    always = {
        str(k).strip()
        for k in (getattr(app.config, 'pagefind_always_show_filters', None) or [])
        if str(k).strip()
    }
    out: List[str] = []
    for field in _parse_result_meta_fields(app):
        key = field['key']
        if key in corpus or key in always:
            out.append(key)
    return out


def _seo_and_filter_metas(app, values: Dict[str, str]) -> str:
    """One <meta> per value; facet keys get Pagefind filter + meta attrs."""
    facet_keys = _facet_key_set(app)
    lines: List[str] = []
    for key in sorted(values.keys()):
        esc_name = html.escape(key, quote=True)
        for value in split_meta_values(values.get(key, '')):
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


def _ensure_meta_keys_store(env) -> Dict[str, Any]:
    if not hasattr(env, 'pagefind_meta_keys_by_doc'):
        env.pagefind_meta_keys_by_doc = {}
    return env.pagefind_meta_keys_by_doc


def _collect_meta_keys(app, doctree: nodes.document, docname: str) -> None:
    if app.builder.format != 'html':
        return
    raw = all_doctree_meta(doctree)
    store = _ensure_meta_keys_store(app.env)
    store[docname] = set(raw.keys())


def _purge_meta_keys(app, env, docname: str) -> None:
    if hasattr(env, 'pagefind_meta_keys_by_doc') and docname in env.pagefind_meta_keys_by_doc:
        del env.pagefind_meta_keys_by_doc[docname]


def _merge_meta_keys(app, env, docnames, other) -> None:
    """Merge per-document meta key sets from a parallel read worker environment."""
    if not hasattr(other, 'pagefind_meta_keys_by_doc'):
        return
    store = _ensure_meta_keys_store(env)
    for docname, keys in other.pagefind_meta_keys_by_doc.items():
        store[docname] = set(keys)


def _union_meta_keys(env) -> List[str]:
    if not hasattr(env, 'pagefind_meta_keys_by_doc'):
        return []
    union: set[str] = set()
    for keys in env.pagefind_meta_keys_by_doc.values():
        union |= set(keys)
    return sorted(union)


def _pagefind_bundle_prefix(app, pagename: str) -> str:
    """Relative URL prefix from current HTML page to the site root ``pagefind/`` directory.

    Must start with ``./`` or ``../`` so the browser resolves dynamic imports (e.g.
    ``import(bundlePath + 'pagefind.js')``) as URLs, not bare module specifiers.

    For ``sphinx-multiversion``, each distro is built with ``pagename`` relative to that
    distro tree (e.g. ``index``), but HTML is served under ``/{smv_current_version}/``.
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
    """(css_href, js_href) relative to current page."""
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


def _merge_index_entries(app, distro: str) -> List[Dict[str, Any]]:
    """Build mergeIndex list from conf (pinned docs.ros.org template)."""
    pkgs: List[str] = list(getattr(app.config, 'pagefind_merge_package_pkgs', []) or [])
    if not pkgs or not getattr(app.config, 'pagefind_merge_enabled', False):
        return []
    base = getattr(app.config, 'pagefind_merge_index_base', 'https://docs.ros.org').rstrip('/')
    overrides = getattr(app.config, 'pagefind_merge_index_overrides', {}) or {}
    out: List[Dict[str, Any]] = []
    for pkg in pkgs:
        key = f'{distro}/{pkg}'
        if key in overrides:
            bundle = overrides[key]
        else:
            bundle = f'{base}/en/{distro}/p/{pkg}/pagefind'
        entry: Dict[str, Any] = {'bundlePath': bundle}
        mf = getattr(app.config, 'pagefind_merge_filter_per_pkg', None)
        if isinstance(mf, dict) and pkg in mf:
            entry['mergeFilter'] = mf[pkg]
        iw = getattr(app.config, 'pagefind_merge_index_weight_per_pkg', None)
        if isinstance(iw, dict) and pkg in iw:
            entry['indexWeight'] = iw[pkg]
        out.append(entry)
    return out


def _html_page_context(
    app,
    pagename: str,
    templatename: str,
    context: Dict[str, Any],
    doctree,
) -> None:
    facet_keys_ordered = _facet_filter_keys_for_context(app, app.env)
    filter_csv = ','.join(facet_keys_ordered)
    result_meta_fields = _parse_result_meta_fields(app)
    current_distro = _current_distro_from_config(app)
    always_show = [
        str(k).strip()
        for k in (getattr(app.config, 'pagefind_always_show_filters', None) or [])
        if str(k).strip()
    ]

    empty = {
        'pagefind_seo_filter_metas': '',
        'pagefind_bundle_prefix': './pagefind/',
        'pagefind_component_css': './pagefind/pagefind-component-ui.css',
        'pagefind_component_js': './pagefind/pagefind-component-ui.js',
        'pagefind_merge_index': [],
        'pagefind_filter_keys_csv': filter_csv,
        'pagefind_result_meta_fields': result_meta_fields,
        'pagefind_always_show_filters': always_show,
        'pagefind_search_results_href': 'search.html',
        'pagefind_current_distro': current_distro,
    }
    context.update(empty)

    if app.builder.format != 'html' or templatename is None:
        return
    if not templatename.endswith('.html'):
        return

    default_distro = current_distro
    values = _resolved_page_meta(app, doctree)

    seo_filters = _seo_and_filter_metas(app, values)
    css_href, js_href = _pagefind_component_urls(app, pagename)
    bundle_prefix = _pagefind_bundle_prefix(app, pagename)

    merge_distro = (
        values.get('distro')
        or values.get('distribution')
        or str(default_distro)
    )
    merge = _merge_index_entries(app, merge_distro)
    context['pagefind_seo_filter_metas'] = seo_filters
    context['pagefind_bundle_prefix'] = bundle_prefix
    context['pagefind_component_css'] = css_href
    context['pagefind_component_js'] = js_href
    context['pagefind_merge_index'] = merge
    context['pagefind_search_results_href'] = _search_results_href(app, pagename)


def setup(app) -> Dict[str, Any]:
    app.add_config_value('pagefind_merge_enabled', default=False, rebuild='html')
    app.add_config_value('pagefind_merge_package_pkgs', default=[], rebuild='html')
    app.add_config_value('pagefind_merge_index_base', default='https://docs.ros.org', rebuild='html')
    app.add_config_value('pagefind_merge_index_overrides', default={}, rebuild='html')
    app.add_config_value('pagefind_merge_filter_per_pkg', default=None, rebuild='html')
    app.add_config_value('pagefind_merge_index_weight_per_pkg', default=None, rebuild='html')
    app.add_config_value('pagefind_result_meta_order', default={}, rebuild='html')
    app.add_config_value('pagefind_always_show_filters', default=[], rebuild='html')

    app.connect('html-page-context', _html_page_context)
    app.connect('doctree-resolved', _collect_meta_keys)
    app.connect('env-purge-doc', _purge_meta_keys)
    app.connect('env-merge-info', _merge_meta_keys)

    return {
        'parallel_read_safe': True,
        'parallel_write_safe': True,
        'version': '1.0.0',
    }
