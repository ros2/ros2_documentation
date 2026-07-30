"""
Utilities for editing reStructuredText source, in particular ``.. meta::``,
``.. short-description::``, and ``.. showmeta::`` directives.
"""

import logging
import re

logger = logging.getLogger(__name__)


def _find_directive_block(content: str, directive: str) -> tuple[int, int, int, str, str]:
    """
    Locate the first ``.. <directive>::`` block in RST source.

    The directive block consists of the explicit marker line followed by
    contiguous indented lines; a blank line or a less-indented line ends the
    block (per reStructuredText directive block rules).

    Args:
        content: The RST file content to search.
        directive: Directive name without the ``..`` prefix (e.g. ``meta``).

    Returns:
        Tuple of ``(start, marker_end, block_end, inner, indent)``.
        If no directive is found, ``start``, ``marker_end``, and ``block_end``
        are ``-1``, ``inner`` is ``''``, and ``indent`` defaults to three spaces.
    """
    match = re.search(
        rf"^\.\.\s+{re.escape(directive)}::\s*\n",
        content,
        re.MULTILINE,
    )
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


def _find_meta_block(content: str) -> tuple[int, int, int, str, str]:
    """
    Locate the first ``.. meta::`` directive in RST source.

    The directive block consists of the explicit marker line followed by
    contiguous indented lines; a blank line or a less-indented line ends the
    block (per reStructuredText directive block rules).

    Args:
        content: The RST file content to search.

    Returns:
        Tuple of ``(start, marker_end, block_end, inner, indent)``.
        If no directive is found, ``start``, ``marker_end``, and ``block_end``
        are ``-1``, ``inner`` is ``''``, and ``indent`` defaults to three spaces.
    """
    return _find_directive_block(content, "meta")


def _extract_meta_names_from_block(meta_block_inner: str) -> set[str]:
    """
    Collect field names from the body of a ``.. meta::`` directive.

    Each line of the form ``:name: value`` contributes ``name`` (Docutils also
    allows forms such as ``:name attr=value:``; the captured segment matches
    that usage).

    Args:
        meta_block_inner: The inner text of the meta block.

    Returns:
        A set of field names found in the block.
    """
    return set(_extract_meta_fields_from_block(meta_block_inner))


def _extract_meta_fields_from_block(meta_block_inner: str) -> dict[str, str]:
    """
    Collect field names and values from the body of a ``.. meta::`` directive.

    Args:
        meta_block_inner: The inner text of the meta block.

    Returns:
        Mapping from field name to field body text (may be empty).
    """
    fields: dict[str, str] = {}
    for field_match in re.finditer(
        r"^[ \t]+:([^:\n]+?):\s*(.*)$",
        meta_block_inner,
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
    _start, _marker_end, _block_end, inner, _indent = _find_meta_block(content)
    return _extract_meta_fields_from_block(inner)


def get_meta_names_from_content(content: str) -> set[str]:
    """
    Return the set of field names already present in the first ``.. meta::`` block.

    If no ``.. meta::`` directive exists, returns an empty set.

    Args:
        content: The RST file content to search.

    Returns:
        A set of field names present in the meta block.
    """
    _start, _marker_end, _block_end, inner, _indent = _find_meta_block(content)
    return _extract_meta_names_from_block(inner)


def has_meta_block(content: str) -> bool:
    """
    Return whether the document contains a ``.. meta::`` directive.

    Args:
        content: The RST file content to search.

    Returns:
        True if a ``.. meta::`` block exists, False otherwise.
    """
    start, _marker_end, _block_end, _inner, _indent = _find_meta_block(content)
    return start >= 0


def _normalise_meta_field_value(value: str) -> str:
    """
    Collapse whitespace so the meta field body stays a single logical line.

    Args:
        value: The raw field value.

    Returns:
        The normalised field value.
    """
    return " ".join(value.split())  # Docutils treats the field body as one string; keep it one physical line


def inject_metadata_to_content(
    content: str,
    metadata: dict[str, str],
) -> tuple[str, bool]:
    """
    Insert or append ``.. meta::`` field entries for the given name/value pairs.

    Appends to an existing ``.. meta::`` block when present. Otherwise inserts a
    new block at the start of the document.

    Skips keys that already have a non-empty value in the block. Fills keys that
    are missing or present with a blank value.

    Returns:
        Updated source and whether any change was made.
    """
    start, marker_end, block_end, inner, indent = _find_meta_block(content)
    existing = _extract_meta_fields_from_block(inner)
    merged: dict[str, str] = dict(existing)
    changed = False

    for key, raw_value in metadata.items():
        value = _normalise_meta_field_value(raw_value)
        if key not in merged:
            merged[key] = value
            changed = True
        elif not merged[key].strip():
            merged[key] = value
            changed = True
        else:
            logger.warning(
                "Existing meta field %r in .. meta:: block; skipping",
                key,
            )

    if not changed:
        return content, False

    ordered_keys: list[str] = list(existing.keys())
    for key in metadata:
        if key not in ordered_keys:
            ordered_keys.append(key)
    new_inner = "".join(f"{indent}:{key}: {merged[key]}\n" for key in ordered_keys)

    if start >= 0:
        # Replace only the directive body slice; ``marker_end``/``block_end`` bracket the original inner
        # Normalise trailing whitespace: one blank line after the block
        remainder = content[block_end:].lstrip()
        new_content = content[:marker_end] + new_inner + "\n" + remainder
    else:
        remainder = content.lstrip()
        new_content = ".. meta::\n" + new_inner + "\n" + remainder

    return new_content, True


def _byte_offset_to_line_number(content: str, offset: int) -> int:
    """Return the 1-based line number containing ``offset`` (or the next line at EOF)."""
    if offset <= 0:
        return 1
    if offset >= len(content):
        return content.count("\n") + (0 if content.endswith("\n") else 1)
    return content.count("\n", 0, offset) + 1


def meta_block_line_span(content: str) -> tuple[int, int] | None:
    """
    Return the inclusive 1-based line span of the first ``.. meta::`` block.

    Returns ``None`` if no meta block exists.
    """
    start, _marker_end, block_end, _inner, _indent = _find_meta_block(content)
    if start < 0:
        return None
    start_line = _byte_offset_to_line_number(content, start)
    end_offset = block_end - 1 if block_end > start else start
    end_line = _byte_offset_to_line_number(content, end_offset)
    return start_line, end_line


def _find_short_description_block(content: str) -> tuple[int, int, int, str, str]:
    """
    Locate the first ``.. short-description::`` directive in RST source.

    Uses the same block-boundary rules as ``_find_meta_block``: the body is
    contiguous indented lines until a blank line or a line starting at column 0.

    Args:
        content: The RST file content to search.

    Returns:
        Tuple of ``(start, marker_end, block_end, inner, indent)``.
        If no directive is found, ``start``, ``marker_end``, and ``block_end``
        are ``-1``, ``inner`` is ``''``, and ``indent`` defaults to three spaces.
    """
    return _find_directive_block(content, "short-description")


def _short_description_inner_has_content(inner: str) -> bool:
    """
    True when the directive body contains non-whitespace text.

    Args:
        inner: The inner text of the short-description block.

    Returns:
        True if the body has content, False otherwise.
    """
    for line in inner.splitlines():
        if line.strip():
            return True
    return False


def has_short_description_content(content: str) -> bool:
    """
    Return whether the document already has a non-empty ``.. short-description::`` body.

    Args:
        content: The RST file content to search.

    Returns:
        True if a non-empty short-description block exists, False otherwise.
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
    """
    Turn model output into RST directive body lines (indented paragraphs).

    Args:
        text: The model-generated prose.
        indent: The indentation string to use.

    Returns:
        The formatted and indented inner text for the directive.
    """
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

    A title block is a non-blank text line followed by a line of repeating
    underline characters (reStructuredText section markers such as ``=``,
    ``-``, ``~``, and other Docutils-adornment characters).
    If no title is found, returns ``0``.

    Args:
        content: The RST file content to search.

    Returns:
        The byte index where the title block ends.
    """
    lines = content.splitlines(keepends=True)
    i = 0
    while i + 1 < len(lines):
        title_line = lines[i]
        underline_line = lines[i + 1]
        title_stripped = title_line.strip()
        # Docutils section adornment characters
        ul_match = re.match(r'^([!"#$%&\'()*+,\-./:;<=>?@\[\\\]^_`{|}~]+)\s*$', underline_line.rstrip("\n"))
        if title_stripped and ul_match is not None:
            ul = ul_match.group(1)
            if len(set(ul)) == 1 and len(ul) >= len(title_stripped):
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
        # Normalise trailing whitespace: one blank line after the block
        remainder = content[block_end:].lstrip()
        new_content = content[:marker_end] + new_inner + "\n" + remainder
        return new_content, True

    insert_at = _find_insertion_point_after_title(content)
    # Normalise trailing whitespace: one blank line before and after the block
    remainder = content[insert_at:].lstrip()
    block = f"\n.. short-description::\n{new_inner}\n"
    new_content = content[:insert_at] + block + remainder
    return new_content, True


_SECTION_ADORNMENT_RE = re.compile(
    r'^([!"#$%&\'()*+,\-./:;<=>?@\[\\\]^_`{|}~]+)\s*$',
)


def _is_section_title_at(lines: list[str], index: int) -> bool:
    """
    Return whether ``lines[index]`` is an RST section title with an underline.

    Args:
        lines: Document lines (with or without trailing newlines).
        index: Zero-based line index of the candidate title line.

    Returns:
        True when the line is followed by a valid adornment underline.
    """
    if index + 1 >= len(lines):
        return False
    title_stripped = lines[index].strip()
    if not title_stripped:
        return False
    ul_match = _SECTION_ADORNMENT_RE.match(lines[index + 1].rstrip("\n"))
    if ul_match is None:
        return False
    ul = ul_match.group(1)
    return len(set(ul)) == 1 and len(ul) >= len(title_stripped)


def _line_starts_directive(line: str) -> bool:
    """Return whether ``line`` begins an explicit RST directive marker."""
    return bool(re.match(r"^\.\.\s+\S+::", line))


def _skip_past_directive_block(lines: list[str], directive_index: int) -> int:
    """
    Return the index of the first line after a directive block.

    Args:
            lines: Document lines (with or without trailing newlines).
            directive_index: Zero-based index of the ``.. directive::`` line.

    Returns:
            Index of the first line following the directive block.
    """
    i = directive_index + 1
    while i < len(lines):
        line = lines[i]
        if line.strip() == "":
            i += 1
            continue
        if not line.startswith((" ", "\t")):
            return i
        i += 1
    return i


def _title_line_span(content: str) -> tuple[int, int] | None:
    """
    Return the inclusive 1-based line span of the first document title block.

    Returns ``None`` when no title is found.
    """
    lines = content.splitlines()
    for i in range(len(lines) - 1):
        if _is_section_title_at(lines, i):
            return i + 1, i + 2
    return None


def extract_first_paragraph_after_title(
    content: str,
) -> tuple[str | None, tuple[int, int] | None]:
    """
    Find the first prose paragraph after the first document title.

    Skips blank lines, directives, indented non-prose lines (such as toctree
    entries), and section titles. Collects contiguous prose until the next
    blank line, directive, or section title.

    Args:
        content: RST source to search.

    Returns:
        Normalised paragraph text and its inclusive 1-based line span, or
        ``(None, None)`` when no prose paragraph is found.
    """
    lines = content.splitlines(keepends=True)
    stripped_lines = [line.rstrip("\n") for line in lines]
    insert_at = _find_insertion_point_after_title(content)
    if insert_at <= 0:
        start_index = 0
    else:
        start_index = content[:insert_at].count("\n")

    prose_lines: list[str] = []
    prose_start: int | None = None
    i = start_index
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()
        if not stripped:
            if prose_lines:
                break
            i += 1
            continue
        if _line_starts_directive(stripped):
            if prose_lines:
                break
            i = _skip_past_directive_block(lines, i)
            continue
        if _is_section_title_at(stripped_lines, i):
            if prose_lines:
                break
            i += 2
            continue
        if not prose_lines and line.startswith((" ", "\t")):
            i += 1
            continue
        if prose_lines and line.startswith((" ", "\t")):
            prose_lines.append(stripped)
            i += 1
            continue
        if line.startswith((" ", "\t")):
            i += 1
            continue
        if prose_start is None:
            prose_start = i
        prose_lines.append(stripped)
        i += 1

    if not prose_lines or prose_start is None:
        return None, None

    start_line = prose_start + 1
    end_line = prose_start + len(prose_lines)
    return " ".join(prose_lines), (start_line, end_line)


def wrap_first_paragraph_as_short_description(content: str) -> tuple[str, bool]:
    """
    Wrap the first prose paragraph after the title into ``.. short-description::``.

    If a non-empty short-description already exists, returns unchanged. If the
    directive exists with an empty body, fills it from the first paragraph.
    Otherwise inserts a new directive after the title and removes the paragraph
    from the body.

    Returns:
        Updated source and whether any change was made.
    """
    if has_short_description_content(content):
        return content, False

    paragraph, span = extract_first_paragraph_after_title(content)
    if paragraph is None or span is None:
        return content, False

    without_para = _remove_line_span(content, span)
    start, marker_end, block_end, _inner, indent = _find_short_description_block(without_para)
    new_inner = _format_short_description_inner(paragraph, indent)

    if start >= 0:
        remainder = without_para[block_end:].lstrip()
        new_content = without_para[:marker_end] + new_inner + "\n" + remainder
        return new_content, True

    insert_at = _find_insertion_point_after_title(without_para)
    remainder = without_para[insert_at:].lstrip()
    block = f"\n.. short-description::\n{new_inner}\n"
    new_content = without_para[:insert_at] + block + remainder
    return new_content, True


def _remove_line_span(content: str, span: tuple[int, int]) -> str:
    """
    Remove an inclusive 1-based line span from RST source.

    Args:
        content: RST source.
        span: Inclusive start and end line numbers (1-based).

    Returns:
        Source with the span removed and adjacent blank lines collapsed.
    """
    start_line, end_line = span
    lines = content.splitlines(keepends=True)
    kept = lines[: start_line - 1] + lines[end_line:]
    result = "".join(kept)
    while "\n\n\n" in result:
        result = result.replace("\n\n\n", "\n\n")
    return result


def _find_showmeta_block(content: str) -> tuple[int, int, int, str, str]:
    """Locate the first ``.. showmeta::`` directive in RST source."""
    return _find_directive_block(content, "showmeta")


def _extract_directive_options_from_block(block_inner: str) -> dict[str, str]:
    """
    Collect option names and values from a directive body.

    Each line of the form ``:name: value`` contributes ``name``.
    """
    options: dict[str, str] = {}
    for field_match in re.finditer(
        r"^[ \t]+:([^:\n]+?):\s*(.*)$",
        block_inner,
        re.MULTILINE,
    ):
        options[field_match.group(1).strip()] = field_match.group(2)
    return options


def has_showmeta_with_order(content: str) -> bool:
    """
    Return whether the first ``.. showmeta::`` block has a non-empty ``:order:``.

    Args:
        content: RST source to search.

    Returns:
        True when showmeta exists with a non-blank order option.
    """
    _s, _m, _b, inner, _i = _find_showmeta_block(content)
    if not inner.strip():
        return False
    options = _extract_directive_options_from_block(inner)
    return bool(options.get("order", "").strip())


def showmeta_line_span(content: str) -> tuple[int, int] | None:
    """
    Return the inclusive 1-based line span of the first ``.. showmeta::`` block.

    Returns ``None`` if no showmeta block exists.
    """
    start, _marker_end, block_end, _inner, _indent = _find_showmeta_block(content)
    if start < 0:
        return None
    start_line = _byte_offset_to_line_number(content, start)
    end_offset = block_end - 1 if block_end > start else start
    end_line = _byte_offset_to_line_number(content, end_offset)
    return start_line, end_line


def short_description_line_span(content: str) -> tuple[int, int] | None:
    """
    Return the inclusive 1-based line span of the first ``.. short-description::`` block.

    Returns ``None`` if no short-description block exists.
    """
    start, _marker_end, block_end, _inner, _indent = _find_short_description_block(content)
    if start < 0:
        return None
    start_line = _byte_offset_to_line_number(content, start)
    end_offset = block_end - 1 if block_end > start else start
    end_line = _byte_offset_to_line_number(content, end_offset)
    return start_line, end_line


def _insertion_point_after_short_description(content: str) -> int:
    """
    Return the byte index immediately after the first short-description block.

    Falls back to after the title when no short-description exists.
    """
    _s, _m, block_end, _inner, _indent = _find_short_description_block(content)
    if block_end >= 0:
        return block_end
    return _find_insertion_point_after_title(content)


def format_showmeta_block(options: dict[str, str], indent: str = "   ") -> str:
    """
    Build an RST ``.. showmeta::`` block for the given options.

    Args:
        options: Option name to value mapping.
        indent: Indentation for option lines.

    Returns:
        A formatted ``.. showmeta::`` directive ending with a blank line.
    """
    lines = [".. showmeta::"]
    for key, value in options.items():
        lines.append(f"{indent}:{key}: {value}")
    lines.append("")
    return "\n".join(lines)


def inject_showmeta_to_content(
    content: str,
    options: dict[str, str],
) -> tuple[str, bool]:
    """
    Insert or fill a ``.. showmeta::`` directive with the given options.

    Appends to an existing block when present. Otherwise inserts after the first
    short-description block, or after the title when none exists. Skips options
    that already have non-empty values.

    Returns:
        Updated source and whether any change was made.
    """
    start, marker_end, block_end, inner, indent = _find_showmeta_block(content)
    existing = _extract_directive_options_from_block(inner)
    merged: dict[str, str] = dict(existing)
    changed = False

    for key, raw_value in options.items():
        value = _normalise_meta_field_value(raw_value)
        if key not in merged:
            merged[key] = value
            changed = True
        elif not merged[key].strip():
            merged[key] = value
            changed = True
        else:
            logger.warning(
                "Existing showmeta option %r; skipping",
                key,
            )

    if not changed:
        return content, False

    ordered_keys: list[str] = list(existing.keys())
    for key in options:
        if key not in ordered_keys:
            ordered_keys.append(key)
    new_inner = "".join(f"{indent}:{key}: {merged[key]}\n" for key in ordered_keys)

    if start >= 0:
        remainder = content[block_end:].lstrip()
        new_content = content[:marker_end] + new_inner + "\n" + remainder
        return new_content, True

    insert_at = _insertion_point_after_short_description(content)
    remainder = content[insert_at:].lstrip()
    block = f"\n.. showmeta::\n{new_inner}\n"
    new_content = content[:insert_at] + block + remainder
    return new_content, True


def after_title_directives_line_span(content: str) -> tuple[int, int] | None:
    """
    Return the inclusive 1-based line span of the post-title directive area.

    Covers short-description and/or showmeta blocks. When neither exists, returns
    the line immediately after the title (or line 1 when no title is found).
    """
    spans: list[tuple[int, int]] = []
    for span_fn in (short_description_line_span, showmeta_line_span):
        span = span_fn(content)
        if span is not None:
            spans.append(span)

    if spans:
        return min(s[0] for s in spans), max(s[1] for s in spans)

    title_span = _title_line_span(content)
    if title_span is not None:
        return title_span[1] + 1, title_span[1] + 1

    return 1, 1

