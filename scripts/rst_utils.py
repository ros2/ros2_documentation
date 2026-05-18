"""
Utilities for editing reStructuredText source, in particular ``.. meta::`` and
``.. short-description::`` directives.
"""

import logging
import re

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


def _extract_meta_names_from_block(meta_block_inner: str) -> set[str]:
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


def get_meta_names_from_content(content: str) -> set[str]:
    """
    Return the set of field names already present in the first ``.. meta::`` block.

    If no ``.. meta::`` directive exists, returns an empty set.
    """
    _start, _marker_end, _block_end, inner, _indent = _find_meta_block(content)
    return _extract_meta_names_from_block(inner)


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
    names = _extract_meta_names_from_block(inner)  # Snapshot before we add keys from this same batch
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


def _find_short_description_block(content: str) -> tuple[int, int, int, str, str]:
    """
    Locate the first ``.. short-description::`` directive in RST source.

    Uses the same block-boundary rules as ``_find_meta_block``: the body is
    contiguous indented lines until a blank line or a line starting at column 0.

    Returns:
        Tuple of ``(start, marker_end, block_end, inner, indent)``.
        If no directive is found, ``start``, ``marker_end``, and ``block_end``
        are ``-1``, ``inner`` is ``''``, and ``indent`` defaults to three spaces.
    """
    match = re.search(r"^\.\.\s+short-description::\s*\n", content, re.MULTILINE)
    if not match:
        return -1, -1, -1, "", "   "

    start = match.start()
    marker_end = match.end()
    indent = "   "
    inner_parts: list[str] = []
    consumed = 0
    remainder = content[marker_end:]

    for line in remainder.splitlines(keepends=True):
        if line.strip() == "":
            break
        if not line.startswith((" ", "\t")):
            break
        if not inner_parts:
            ws_len = len(line) - len(line.lstrip(" \t"))
            indent = line[:ws_len]
        inner_parts.append(line)
        consumed += len(line)

    block_end = marker_end + consumed
    inner = "".join(inner_parts)
    if inner and not inner.endswith("\n"):
        inner += "\n"
    return start, marker_end, block_end, inner, indent


def _short_description_inner_has_content(inner: str) -> bool:
    """True when the directive body contains non-whitespace text."""
    for line in inner.splitlines():
        if line.strip():
            return True
    return False


def has_short_description_content(content: str) -> bool:
    """
    Return whether the document already has a non-empty ``.. short-description::`` body.
    """
    _s, _m, _b, inner, _i = _find_short_description_block(content)
    return _short_description_inner_has_content(inner)


def get_short_description_body(content: str) -> str | None:
    """
    Return the normalised inner body text of the first ``.. short-description::`` block.

    Returns ``None`` if the directive is missing or the body is empty.
    """
    _s, _m, _b, inner, _i = _find_short_description_block(content)
    if not _short_description_inner_has_content(inner):
        return None
    paragraphs: list[str] = []
    current: list[str] = []
    for line in inner.splitlines():
        stripped = line.strip()
        if not stripped:
            if current:
                paragraphs.append(" ".join(current))
                current = []
            continue
        current.append(stripped)
    if current:
        paragraphs.append(" ".join(current))
    return "\n\n".join(paragraphs) if paragraphs else None


def _format_short_description_inner(text: str, indent: str) -> str:
    """Turn model output into RST directive body lines (indented paragraphs)."""
    chunks = [p.strip() for p in text.split("\n\n") if p.strip()]
    lines_out: list[str] = []
    for i, para in enumerate(chunks):
        for line in para.split("\n"):
            s = line.strip()
            if s:
                lines_out.append(f"{indent}{s}\n")
        if i < len(chunks) - 1:
            lines_out.append(f"{indent}\n")
    return "".join(lines_out)


def _find_insertion_point_after_title(content: str) -> int:
    """
    Return the index in ``content`` immediately after the first document title block.

    A title block is a non-blank text line followed by a line of ``=``, ``-``, or ``~``
    underline characters (classic reStructuredText transition marker).
    If no title is found, returns ``0``.
    """
    lines = content.splitlines(keepends=True)
    i = 0
    while i + 1 < len(lines):
        title_line = lines[i]
        underline_line = lines[i + 1]
        title_stripped = title_line.strip()
        ul_match = re.match(r"^([=\-~]+)\s*$", underline_line.rstrip("\n"))
        if title_stripped and ul_match is not None:
            ul = ul_match.group(1)
            if len(ul) >= len(title_stripped):
                pos = 0
                for j in range(i + 2):
                    pos += len(lines[j])
                return pos
        i += 1
    return 0


def inject_short_description_to_content(content: str, text: str) -> tuple[str, bool]:
    """
    Insert or fill the first ``.. short-description::`` directive with the given prose.

    If the directive exists and already has body text, logs a warning and returns
    the original content unchanged. If the directive exists with an empty body,
    fills the body. If the directive is missing, inserts a new block after the
    first detected document title (or at the start of the file if none).

    Returns:
        Updated source and whether any change was made.
    """
    start, marker_end, block_end, inner, indent = _find_short_description_block(content)
    new_inner = _format_short_description_inner(text, indent)

    if start >= 0:
        if _short_description_inner_has_content(inner):
            logger.warning(
                "Existing .. short-description:: body has content; skipping replacement",
            )
            return content, False
        new_content = content[:marker_end] + new_inner + content[block_end:]
        return new_content, True

    insert_at = _find_insertion_point_after_title(content)
    block = f"\n.. short-description::\n{new_inner}\n"
    new_content = content[:insert_at] + block + content[insert_at:]
    return new_content, True

