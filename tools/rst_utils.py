"""
Read-only utilities for detecting Sphinx directives in reStructuredText source.

Supports ``.. meta::``, ``.. short-description::``, and ``.. showmeta::``.
"""

import re


def _find_directive_block(content: str, directive: str) -> str | None:
    """
    Locate the inner body of the first ``.. <directive>::`` block in RST source.

    The directive block consists of the explicit marker line followed by
    contiguous indented lines; a blank line or a less-indented line ends the
    block (per reStructuredText directive block rules).

    Args:
        content: The RST file content to search.
        directive: Directive name without the ``..`` prefix (e.g. ``meta``).

    Returns:
        The inner body text of the directive block, or ``None`` when no block
        is found.
    """
    match = re.search(
        rf"^\.\.\s+{re.escape(directive)}::\s*\n",
        content,
        re.MULTILINE,
    )
    if not match:
        return None

    marker_end = match.end()
    inner_parts: list[str] = []
    remainder = content[marker_end:]

    for line in remainder.splitlines(keepends=True):
        if line.strip() == "":
            break
        if not line.startswith((" ", "\t")):
            break
        inner_parts.append(line)

    inner = "".join(inner_parts)
    if inner and not inner.endswith("\n"):
        inner += "\n"
    return inner if inner.strip() else None


def _extract_field_values(block_inner: str) -> dict[str, str]:
    """
    Collect field or option names and values from a directive body.

    Each line of the form ``:name: value`` contributes ``name`` (Docutils also
    allows forms such as ``:name attr=value:``; the captured segment matches
    that usage).

    Args:
        block_inner: The inner text of a directive block.

    Returns:
        Mapping from field or option name to body text (may be empty).
    """
    fields: dict[str, str] = {}
    for field_match in re.finditer(
        r"^[ \t]+:([^:\n]+?):\s*(.*)$",
        block_inner,
        re.MULTILINE,
    ):
        fields[field_match.group(1).strip()] = field_match.group(2)
    return fields


def get_meta_fields_from_content(content: str) -> dict[str, str]:
    """
    Return field names and values from the first ``.. meta::`` block.

    If no ``.. meta::`` directive exists, returns an empty mapping.

    Args:
        content: The RST file content to search.

    Returns:
        Mapping from meta field name to field body text.
    """
    inner = _find_directive_block(content, "meta")
    if not inner:
        return {}
    return _extract_field_values(inner)


def has_short_description_content(content: str) -> bool:
    """
    Return whether the document already has a non-empty ``.. short-description::`` body.

    Args:
        content: The RST file content to search.

    Returns:
        True if a non-empty short-description block exists, False otherwise.
    """
    inner = _find_directive_block(content, "short-description")
    return bool(inner and inner.strip())


def has_showmeta_with_order(content: str) -> bool:
    """
    Return whether the first ``.. showmeta::`` block has a non-empty ``:order:``.

    Args:
        content: RST source to search.

    Returns:
        True when showmeta exists with a non-blank order option.
    """
    inner = _find_directive_block(content, "showmeta")
    if not inner:
        return False
    options = _extract_field_values(inner)
    return bool(options.get("order", "").strip())
