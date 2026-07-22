# Copyright 2026 Open Robotics — load shared Pagefind / search UI config from pagefind.yml
"""Read ``pagefind.yml`` for Sphinx-owned search metadata settings."""

from __future__ import annotations

import os
from typing import Any, Dict, List

import yaml


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


def load_search_result_meta_order(confdir: str) -> Dict[str, str]:
    """Return ``search_result_meta`` from ``pagefind.yml`` as ``{key: label}``."""
    raw = load_pagefind_yml(confdir).get('search_result_meta') or {}
    if not isinstance(raw, dict):
        return {}
    out: Dict[str, str] = {}
    for key, label in raw.items():
        k = str(key).strip()
        if not k:
            continue
        out[k] = str(label).strip() if label is not None else ''
    return out


def load_always_show_filters(confdir: str) -> List[str]:
    """Return ``always_show_filters`` keys from ``pagefind.yml`` (order preserved).

    Only keys that also appear in ``search_result_meta`` are kept.
    """
    data = load_pagefind_yml(confdir)
    meta_keys = set(load_search_result_meta_order(confdir))
    raw = data.get('always_show_filters') or []
    if not isinstance(raw, (list, tuple)):
        return []
    out: List[str] = []
    seen: set = set()
    for item in raw:
        key = str(item).strip()
        if not key or key in seen or key not in meta_keys:
            continue
        seen.add(key)
        out.append(key)
    return out


def note_pagefind_yml_dependency(app) -> None:
    """Register ``pagefind.yml`` so edits invalidate the build."""
    env = getattr(app, 'env', None)
    if env is None:
        return
    path = pagefind_yml_path(app.confdir)
    if os.path.isfile(path):
        env.note_dependency(path)


def setup(app) -> Dict[str, Any]:
    app.connect('builder-inited', note_pagefind_yml_dependency)
    return {
        'version': '1.0.0',
        'parallel_read_safe': True,
        'parallel_write_safe': True,
    }
