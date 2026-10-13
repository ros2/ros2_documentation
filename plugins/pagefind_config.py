# Copyright 2026 Open Robotics — load shared Pagefind / search UI config from pagefind.yml
"""Read ``pagefind.yml`` for Sphinx-owned search metadata settings."""

from __future__ import annotations

import os
from typing import Any, Dict, List, NamedTuple, Optional

import yaml

# Pagefind's built-in excerpt window when ``search_result_excerpt_length`` is absent.
_DEFAULT_EXCERPT_LENGTH = 30


class SearchUiConfig(NamedTuple):
    """Search UI settings from one read of ``pagefind.yml``."""

    result_meta: Dict[str, str]
    always_show_filters: List[str]
    show_breadcrumb: bool
    show_metadata: bool
    excerpt_length: int


def pagefind_yml_path(confdir: str) -> str:
    return os.path.join(confdir, 'pagefind.yml')


def load_pagefind_yml(confdir: str) -> Dict[str, Any]:
    """Load ``pagefind.yml`` from the Sphinx conf directory (repo root)."""
    path = pagefind_yml_path(confdir)
    if not os.path.isfile(path):
        return {}
    with open(path, encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    return data if isinstance(data, dict) else {}


def _result_meta(data: Dict[str, Any]) -> Dict[str, str]:
    """``search_result_meta`` as ``{key: label}``."""
    raw = data.get('search_result_meta') or {}
    if not isinstance(raw, dict):
        return {}
    out: Dict[str, str] = {}
    for key, label in raw.items():
        k = str(key).strip()
        if not k:
            continue
        out[k] = str(label).strip() if label is not None else ''
    return out


def _always_show_filters(data: Dict[str, Any], meta_keys: Dict[str, str]) -> List[str]:
    """``always_show_filters`` keys that also appear in ``search_result_meta``."""
    raw = data.get('always_show_filters') or []
    if not isinstance(raw, (list, tuple)):
        return []
    allowed = set(meta_keys)
    out: List[str] = []
    seen: set = set()
    for item in raw:
        key = str(item).strip()
        if not key or key in seen or key not in allowed:
            continue
        seen.add(key)
        out.append(key)
    return out


def _excerpt_length(data: Dict[str, Any]) -> int:
    """``search_result_excerpt_length``, or 30 when missing or invalid."""
    raw = data.get('search_result_excerpt_length', _DEFAULT_EXCERPT_LENGTH)
    try:
        length = int(raw)
    except (TypeError, ValueError):
        return _DEFAULT_EXCERPT_LENGTH
    if length < 1:
        return _DEFAULT_EXCERPT_LENGTH
    return length


def load_search_ui_config(confdir: str) -> SearchUiConfig:
    """Read search UI settings from ``pagefind.yml`` in one pass."""
    data = load_pagefind_yml(confdir)
    result_meta = _result_meta(data)
    return SearchUiConfig(
        result_meta=result_meta,
        always_show_filters=_always_show_filters(data, result_meta),
        show_breadcrumb=bool(data.get('search_result_breadcrumb', False)),
        show_metadata=bool(data.get('search_result_metadata', False)),
        excerpt_length=_excerpt_length(data),
    )


def _pagefind_yml_stamp(confdir: str) -> Optional[int]:
    path = pagefind_yml_path(confdir)
    try:
        return os.stat(path).st_mtime_ns
    except OSError:
        return None


def pagefind_yml_outdated(app, env, _added, _changed, _removed) -> List[str]:
    """Re-read every page when ``pagefind.yml`` is new or has changed."""
    stamp = _pagefind_yml_stamp(app.confdir)
    if getattr(env, 'pagefind_yml_mtime', None) == stamp:
        return []
    return list(getattr(env, 'found_docs', ()) or ())


def remember_pagefind_yml(app, env) -> None:
    """Store the ``pagefind.yml`` mtime after a successful read."""
    env.pagefind_yml_mtime = _pagefind_yml_stamp(app.confdir)


def setup(app) -> Dict[str, Any]:
    app.connect('env-get-outdated', pagefind_yml_outdated)
    app.connect('env-updated', remember_pagefind_yml)
    return {
        'version': '1.0.0',
        'parallel_read_safe': True,
        'parallel_write_safe': True,
    }
