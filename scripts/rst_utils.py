"""
Utilities for editing reStructuredText source, in particular ``.. meta::`` directives.
"""

import logging
import os
import re

from enhance_data import (
    EnhanceData,
    calculate_metrics,
    get_results_for_file,
    mark_file_updated,
)

logger = logging.getLogger(__name__)


def _find_meta_block(content: str) -> tuple[int, int, int, str, str]:
    """
    Locate the first ``.. meta::`` directive in RST source.

    The directive block consists of the explicit marker line followed by
    contiguous indented lines; a blank line or a less-indented line ends the
    block (per reStructuredText directive block rules).

    Returns:
        Tuple of ``(start, marker_end, block_end, inner, indent)``.
        If no directive is found, ``start``, ``marker_end``, and ``block_end``
        are ``-1``, ``inner`` is ``''``, and ``indent`` defaults to three spaces.
    """
    # Explicit markup + directive name; block body starts on the following line only
    match = re.search(r"^\.\.\s+meta::\s*\n", content, re.MULTILINE)
    if not match:
        return -1, -1, -1, "", "   "

    start = match.start()  # Byte index of ``.. meta::`` (for whole-directive splice)
    marker_end = match.end()  # First character after the marker line's newline
    indent = "   "  # Default field indent when the block is empty or we prepend a new block
    inner_parts: list[str] = []
    consumed = 0  # Length of directive body in ``content`` (may omit final ``\n`` on last line)
    remainder = content[marker_end:]  # Scan forward only inside this file slice

    for line in remainder.splitlines(keepends=True):
        if line.strip() == "":
            break  # Blank line terminates the directive block
        if not line.startswith((" ", "\t")):
            break  # Body element at column 0 ends the block
        if not inner_parts:
            ws_len = len(line) - len(line.lstrip(" \t"))
            indent = line[:ws_len]  # Reuse the author's indent for new ``:name:`` lines
        inner_parts.append(line)
        consumed += len(line)

    block_end = marker_end + consumed  # Exclusive end of the directive in ``content``
    inner = "".join(inner_parts)
    # EOF without ``\n`` yields a last ``splitlines`` element with no newline—append one before new fields
    if inner and not inner.endswith("\n"):
        inner += "\n"
    return start, marker_end, block_end, inner, indent


def _get_existing_meta_names(meta_block_inner: str) -> set[str]:
    """
    Collect field names from the body of a ``.. meta::`` directive.

    Each line of the form ``:name: value`` contributes ``name`` (Docutils also
    allows forms such as ``:name attr=value:``; the captured segment matches
    that usage).
    """
    names: set[str] = set()
    # Field list lines only; group 1 is the name segment (includes ``attr=value`` forms before the final ``:``)
    for field_match in re.finditer(r"^[ \t]+:([^:\n]+?):", meta_block_inner, re.MULTILINE):
        names.add(field_match.group(1).strip())
    return names


def _normalise_meta_field_value(value: str) -> str:
    """Collapse whitespace so the meta field body stays a single logical line."""
    return " ".join(value.split())  # Docutils treats the field body as one string; keep it one physical line


def inject_metadata_to_content(content: str, metadata: dict[str, str]) -> tuple[str, bool]:
    """
    Insert or append ``.. meta::`` field entries for the given name/value pairs.

    Appends to an existing ``.. meta::`` block when present; otherwise prepends
    a new block at the start of the document (leading whitespace is stripped so
    the directive is the first element). Skips keys that already appear in the
    block.

    Returns:
        Updated source and whether any change was made.
    """
    start, marker_end, block_end, inner, indent = _find_meta_block(content)
    names = _get_existing_meta_names(inner)  # Snapshot before we add keys from this same batch
    additions: list[str] = []

    for key, raw_value in metadata.items():
        if key in names:
            logger.warning(
                "Existing meta field %r in .. meta:: block; skipping",
                key,
            )
            continue
        value = _normalise_meta_field_value(raw_value)
        additions.append(f"{indent}:{key}: {value}\n")
        names.add(key)  # Prevent duplicate inserts if ``metadata`` repeats a key

    if not additions:
        return content, False  # Nothing new to write; leave the file untouched

    new_inner = inner + "".join(additions)  # Existing fields unchanged, then appended lines

    if start >= 0:
        # Replace only the directive body slice; ``marker_end``/``block_end`` bracket the original inner
        new_content = content[:marker_end] + new_inner + content[block_end:]
    else:
        # No ``.. meta::`` yet: insert at document start; strip leading whitespace so the block is truly first
        remainder = content.lstrip()
        new_content = ".. meta::\n" + "".join(additions) + "\n" + remainder  # Blank line after block separates it from the body

    return new_content, True

