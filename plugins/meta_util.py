# Copyright 2026 Open Robotics — shared helpers for ``.. meta::`` / Pagefind
"""
Collect every ``.. meta::`` field from the doctree, sanitize keys, and expand
``{MACRO}`` placeholders using the Sphinx ``macros`` config (longest keys first).

Sphinx / the HTML theme may also emit plain ``<meta>`` tags for the same fields.
The Pagefind extension emits additional tags with ``data-pagefind-filter`` and may
split comma-separated values into multiple tags for faceted search.
"""

from __future__ import annotations

import re
from typing import Dict, List, Optional

from docutils import nodes

# HTML ``<meta name="...">`` names should be conservative; allow common patterns.
_META_NAME_RE = re.compile(r'^[A-Za-z0-9_.:-]+$')


def sanitize_meta_key(raw: str) -> Optional[str]:
    s = str(raw).strip()
    if not s or not _META_NAME_RE.match(s):
        return None
    return s


def all_doctree_meta(doctree: Optional[nodes.document]) -> Dict[str, str]:
    """Return last-wins mapping of every ``nodes.meta`` ``name``/``property`` → ``content``."""
    if doctree is None:
        return {}

    out: Dict[str, str] = {}
    for meta in doctree.findall(nodes.meta):
        if meta.get('http-equiv'):
            continue
        content = meta.get('content')
        if not content:
            continue
        key: Optional[str] = None
        name = meta.get('name')
        if name:
            key = sanitize_meta_key(str(name))
        else:
            prop = meta.get('property')
            if prop:
                key = sanitize_meta_key(str(prop))
        if not key:
            continue
        out[key] = str(content).strip()
    return out


def expand_meta_macros(text: str, macros: Dict[str, str]) -> str:
    """Expand ``{KEY}`` placeholders; longer macro names first to avoid partial matches."""
    result = text
    for key, value in sorted(macros.items(), key=lambda kv: len(kv[0]), reverse=True):
        result = result.replace(f'{{{key}}}', value)
    return result


def expand_all_meta_values(meta: Dict[str, str], macros: Dict[str, str]) -> Dict[str, str]:
    """Apply ``expand_meta_macros`` to every meta value."""
    return {k: expand_meta_macros(v, macros) for k, v in meta.items()}


def split_meta_values(value: str) -> List[str]:
    """Return comma-separated metadata values as individual Pagefind values."""
    return [part.strip() for part in value.split(',') if part.strip()]
