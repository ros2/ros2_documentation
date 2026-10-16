# Copyright 2026 Open Robotics — load shared Pagefind / search UI config from pagefind.yml
"""Read ``pagefind.yml`` for Sphinx-owned search metadata settings.

``conf.py`` loads the file into the ``pagefind_search`` config value, so Sphinx
rebuilds every page when the file changes.

``make html`` and ``make multiversion`` set ``pagefind_enabled`` and then index the
output. Sphinx's built-in search is deprecated for those builds. ``make serve`` still
uses it (``ROS_DOCS_SERVE=1`` and ``-D pagefind_enabled=0``) so local preview has
search without rebuilding the Pagefind index on every save.
"""

from __future__ import annotations

import os
import re
from typing import Any, Dict, List, NamedTuple

import yaml
from sphinx.util import logging

logger = logging.getLogger(__name__)

# HTML builders that ship a search page. Other builders have nothing to deprecate.
_HTML_SEARCH_BUILDERS = frozenset({'html', 'dirhtml', 'singlehtml'})

# Pagefind's built-in excerpt window when ``search_result_excerpt_length`` is absent.
_DEFAULT_EXCERPT_LENGTH = 30


class SearchUiConfig(NamedTuple):
    """Search UI settings parsed from ``pagefind.yml``."""

    # Ordered ``{key, label}`` facet fields from ``search_result_meta``.
    result_meta: List[Dict[str, str]]
    always_show_filters: List[str]
    show_breadcrumb: bool
    show_metadata: bool
    excerpt_length: int


def load_pagefind_yml(confdir: str) -> Dict[str, Any]:
    """Load ``pagefind.yml`` from the Sphinx conf directory (repo root)."""
    path = os.path.join(confdir, 'pagefind.yml')
    if not os.path.isfile(path):
        return {}
    with open(path, encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    return data if isinstance(data, dict) else {}


def _default_filter_label(key: str) -> str:
    spaced = re.sub(r'([a-z])([A-Z])', r'\1 \2', key)
    return spaced.replace('_', ' ').replace('-', ' ').strip().title()


def _result_meta(data: Dict[str, Any]) -> List[Dict[str, str]]:
    """``search_result_meta`` as an ordered ``{key, label}`` list."""
    raw = data.get('search_result_meta') or {}
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


def _always_show_filters(data: Dict[str, Any], allowed: set) -> List[str]:
    """``always_show_filters`` keys that also appear in ``search_result_meta``."""
    raw = data.get('always_show_filters') or []
    if not isinstance(raw, (list, tuple)):
        return []
    out: List[str] = []
    for item in raw:
        key = str(item).strip()
        if key in allowed and key not in out:
            out.append(key)
    return out


def _excerpt_length(data: Dict[str, Any]) -> int:
    """``search_result_excerpt_length``, or 30 when missing or invalid."""
    try:
        length = int(data.get('search_result_excerpt_length', _DEFAULT_EXCERPT_LENGTH))
    except (TypeError, ValueError):
        return _DEFAULT_EXCERPT_LENGTH
    return length if length >= 1 else _DEFAULT_EXCERPT_LENGTH


def parse_search_ui_config(data: Any) -> SearchUiConfig:
    """Parse the ``pagefind.yml`` mapping into search UI settings."""
    if not isinstance(data, dict):
        data = {}
    result_meta = _result_meta(data)
    return SearchUiConfig(
        result_meta=result_meta,
        always_show_filters=_always_show_filters(data, {f['key'] for f in result_meta}),
        show_breadcrumb=bool(data.get('search_result_breadcrumb', False)),
        show_metadata=bool(data.get('search_result_metadata', False)),
        excerpt_length=_excerpt_length(data),
    )


def _configure_search(app) -> None:
    if app.config.pagefind_enabled:
        app.add_css_file('pagefind-docsearch.css')
        return
    builder = getattr(getattr(app, 'builder', None), 'name', '')
    # make serve sets this so the live preview can keep Sphinx search quietly.
    if builder not in _HTML_SEARCH_BUILDERS or os.environ.get('ROS_DOCS_SERVE') == '1':
        return
    logger.warning(
        "Sphinx's built-in search is deprecated. "
        "Use 'make html' or 'make multiversion' for Pagefind search. "
        "Sphinx search remains available with 'make serve'.",
        type='pagefind',
        subtype='deprecated_sphinx_search',
    )


def setup(app) -> Dict[str, Any]:
    app.add_config_value('pagefind_search', {}, 'html')
    app.add_config_value('pagefind_enabled', False, 'html', types=[bool])
    app.connect('builder-inited', _configure_search)
    return {
        'version': '1.0.0',
        'parallel_read_safe': True,
        'parallel_write_safe': True,
    }
