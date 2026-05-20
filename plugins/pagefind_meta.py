# Copyright 2026 Open Robotics — Pagefind metadata for ROS 2 documentation
"""
Emit SEO <meta> tags, Pagefind ``data-pagefind-meta``, and ``data-pagefind-filter``
from every ``.. meta::`` field on the page (passthrough, no whitelist).

Sphinx / the HTML theme typically also emits plain ``<meta>`` tags for the same
``.. meta::`` fields. We intentionally emit an additional block with
``data-pagefind-filter`` (and split comma-separated values) so Pagefind faceting
works; crawlers may see duplicate name/content pairs for non-split fields.
"""

from __future__ import annotations

import html
import re
from pathlib import PurePosixPath
from typing import Any, Dict, List, Optional, Tuple

from docutils import nodes

from meta_util import all_doctree_meta, expand_all_meta_values, split_meta_values


def _macros_flat(app) -> Dict[str, str]:
    macros = getattr(app.config, 'macros', {}) or {}
    return {str(k): str(v) for k, v in macros.items()}


def _resolved_page_meta(app, doctree: Optional[nodes.document]) -> Dict[str, str]:
    raw = all_doctree_meta(doctree)
    return expand_all_meta_values(raw, _macros_flat(app))


def _default_filter_label(key: str) -> str:
    spaced = re.sub(r'([a-z])([A-Z])', r'\1 \2', key)
    return spaced.replace('_', ' ').replace('-', ' ').strip().title()


def _metadata_fields_for_keys(app, sorted_keys: List[str]) -> List[List[str]]:
    labels = getattr(app.config, 'pagefind_filter_labels', None) or {}
    out: List[List[str]] = []
    for k in sorted_keys:
        if isinstance(labels, dict) and labels.get(k):
            lbl = str(labels[k])
        else:
            lbl = _default_filter_label(k)
        out.append([k, lbl])
    return out


def _pagefind_data_meta_attr(values: Dict[str, str]) -> str:
    """Single data-pagefind-meta attribute value with repeated keys for multi-values."""
    parts: List[str] = []
    for key in sorted(values.keys()):
        for value in split_meta_values(values.get(key, '')):
            parts.append(f'{key}:{value}')
    inner = ', '.join(parts)
    return html.escape(inner, quote=True)


def _seo_and_filter_metas(values: Dict[str, str]) -> str:
    """One <meta> per value: SEO name/content + data-pagefind-filter (Pagefind filtering docs)."""
    lines: List[str] = []
    for key in sorted(values.keys()):
        esc_name = html.escape(key, quote=True)
        for value in split_meta_values(values.get(key, '')):
            esc_val = html.escape(value, quote=True)
            lines.append(
                f'<meta name="{esc_name}" content="{esc_val}" '
                f'data-pagefind-filter="{esc_name}[content]">'
            )
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
        return 'search'
    try:
        current = builder.get_target_uri(pagename, typ='html')
        target = builder.get_target_uri('search', typ='html')
        rel = builder.get_relative_uri(current, target)
        if rel:
            return rel
    except (AttributeError, KeyError, ValueError):
        pass
    return 'search'


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
    sorted_keys = _union_meta_keys(app.env)
    metadata_fields = _metadata_fields_for_keys(app, sorted_keys)
    filter_csv = ','.join(sorted_keys)

    empty = {
        'pagefind_seo_filter_metas': '',
        'pagefind_data_meta_attr': '',
        'pagefind_bundle_prefix': './pagefind/',
        'pagefind_component_css': './pagefind/pagefind-component-ui.css',
        'pagefind_component_js': './pagefind/pagefind-component-ui.js',
        'pagefind_merge_index': [],
        'pagefind_filter_keys_csv': filter_csv,
        'pagefind_metadata_fields': metadata_fields,
        'pagefind_result_meta_order': list(
            getattr(app.config, 'pagefind_result_meta_order', []) or []
        ),
        'pagefind_search_results_href': 'search',
    }
    context.update(empty)

    if app.builder.format != 'html' or templatename is None:
        return
    if not templatename.endswith('.html'):
        return

    default_distro = (getattr(app.config, 'macros', {}) or {}).get('DISTRO', 'rolling')
    values = _resolved_page_meta(app, doctree)

    seo_filters = _seo_and_filter_metas(values)
    data_attr = _pagefind_data_meta_attr(values)
    css_href, js_href = _pagefind_component_urls(app, pagename)
    bundle_prefix = _pagefind_bundle_prefix(app, pagename)

    merge_distro = values.get('distro') or str(default_distro)
    merge = _merge_index_entries(app, merge_distro)
    context['pagefind_seo_filter_metas'] = seo_filters
    context['pagefind_data_meta_attr'] = data_attr
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
    app.add_config_value('pagefind_filter_labels', default={}, rebuild='html')
    app.add_config_value('pagefind_result_meta_order', default=[], rebuild='html')

    app.connect('html-page-context', _html_page_context)
    app.connect('doctree-resolved', _collect_meta_keys)
    app.connect('env-purge-doc', _purge_meta_keys)
    app.connect('env-merge-info', _merge_meta_keys)

    return {
        'parallel_read_safe': True,
        'parallel_write_safe': True,
        'version': '1.0.0',
    }
