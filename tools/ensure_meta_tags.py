#!/usr/bin/env python3
"""
Ensure configured metadata fields exist in RST ``.. meta::`` blocks.

Missing fields are injected with values from ``meta_tags.yaml``.
Existing fields are never overwritten.

When ``--diff-base`` is set, edits are only written to disk when they overlap
the pull request diff (so GitHub can offer inline suggestions). Otherwise the
review comment carries a top-of-file copy-paste fallback.
"""

from __future__ import annotations

import argparse
import logging
import re
import subprocess
import sys
from pathlib import Path

import yaml

# Allow ``python3 tools/ensure_meta_tags.py`` from the repository root.
_TOOLS_DIR = Path(__file__).resolve().parent
if str(_TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(_TOOLS_DIR))

from rst_utils import (
    first_heading_line_span,
    get_meta_names_from_content,
    has_meta_block,
    inject_metadata_to_content,
    meta_block_line_span,
)

logger = logging.getLogger(__name__)

DEFAULT_CONFIG_PATH = _TOOLS_DIR / "meta_tags.yaml"
RST_EXTENSION = ".rst"
_HUNK_HEADER = re.compile(r"^@@ -\d+(?:,\d+)? \+(\d+)(?:,(\d+))? @@")


def load_meta_config(config_path: Path) -> dict[str, str]:
    """
    Load the ``meta`` mapping from a YAML config file.

    Returns:
        Field name to value mapping.

    Raises:
        SystemExit: If the file is missing, invalid, or has no usable ``meta`` map.
    """
    if not config_path.is_file():
        logger.error("Config file not found: %s", config_path)
        raise SystemExit(1)

    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        logger.error("Invalid YAML in %s: %s", config_path, exc)
        raise SystemExit(1) from exc

    if not isinstance(raw, dict):
        logger.error("Config %s must be a YAML mapping", config_path)
        raise SystemExit(1)

    meta = raw.get("meta")
    if not isinstance(meta, dict) or not meta:
        logger.error("Config %s must contain a non-empty 'meta' mapping", config_path)
        raise SystemExit(1)

    validated: dict[str, str] = {}
    for key, value in meta.items():
        if not isinstance(key, str) or not key.strip():
            logger.error("Config %s: meta keys must be non-empty strings", config_path)
            raise SystemExit(1)
        if not isinstance(value, str):
            logger.error(
                "Config %s: meta value for %r must be a string, got %s",
                config_path,
                key,
                type(value).__name__,
            )
            raise SystemExit(1)
        validated[key] = value

    return validated


def format_meta_block(meta_config: dict[str, str], fields: list[str]) -> str:
    """Return an RST ``.. meta::`` block for the given fields."""
    lines = [".. meta::"]
    for field in fields:
        lines.append(f"   :{field}: {meta_config[field]}")
    lines.append("")
    return "\n".join(lines)


def _missing_meta_fields(content: str, meta_config: dict[str, str]) -> list[str]:
    present = get_meta_names_from_content(content)
    return [field for field in meta_config if field not in present]


def parse_diff_new_side_lines(diff_text: str) -> set[int]:
    """
    Parse a unified diff and return 1-based line numbers on the new (``+``) side.

    Both added and context lines within hunks are included so nearby suggestion
    anchors count as overlapping the pull request diff.

    File headers (``---`` / ``+++``) are only skipped outside hunks. Inside a
    hunk those prefixes are ordinary ``-`` / ``+`` lines (e.g. RST table rows of
    ``+`` characters), and must advance ``new_line`` accordingly.
    """
    lines: set[int] = set()
    new_line = 0
    in_hunk = False
    for line in diff_text.splitlines():
        match = _HUNK_HEADER.match(line)
        if match:
            in_hunk = True
            new_line = int(match.group(1))
            continue
        if not in_hunk and (line.startswith("---") or line.startswith("+++")):
            continue
        if line.startswith("\\"):
            continue
        if line.startswith("+"):
            lines.add(new_line)
            new_line += 1
        elif line.startswith("-"):
            continue
        elif line.startswith(" ") or line == "":
            if new_line > 0:
                lines.add(new_line)
                new_line += 1
        elif line.startswith("diff "):
            # Next file in a multi-file diff; subsequent ---/+++ are headers again.
            in_hunk = False
            new_line = 0
    return lines


def pr_diff_lines_for_file(diff_base: str, path: Path) -> set[int]:
    """Return PR-diff line numbers for ``path`` on the head side vs ``diff_base``."""
    result = subprocess.run(
        ["git", "diff", "-U3", f"{diff_base}...HEAD", "--", str(path)],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode not in (0, 1):
        logger.warning(
            "git diff failed for %s (exit %s): %s",
            path,
            result.returncode,
            result.stderr.strip(),
        )
        return set()
    return parse_diff_new_side_lines(result.stdout)


def _span_overlaps(span: tuple[int, int] | None, pr_lines: set[int]) -> bool:
    if span is None or not pr_lines:
        return False
    start, end = span
    return any(line in pr_lines for line in range(start, end + 1))


def choose_suggestable_placement(
    content: str,
    pr_lines: set[int],
) -> str | None:
    """
    Return how to apply an in-diff edit, or ``None`` if only fallback is possible.

    Returns:
        ``append``, ``after_heading``, ``at_top``, or ``None``.
    """
    if has_meta_block(content):
        span = meta_block_line_span(content)
        if span is None:
            return None
        start, end = span
        # Include the line after the block where fields would be appended.
        if _span_overlaps((start, end + 1), pr_lines):
            return "append"
        return None

    heading = first_heading_line_span(content)
    if heading is not None:
        start, end = heading
        if _span_overlaps((start, end + 1), pr_lines):
            return "after_heading"

    if _span_overlaps((1, 1), pr_lines):
        return "at_top"

    return None


def ensure_meta_tags_in_file(
    path: Path,
    meta_config: dict[str, str],
    *,
    pr_lines: set[int] | None = None,
) -> dict[str, object] | None:
    """
    Add missing configured meta fields, preferring pull-request-diff overlap.

    When ``pr_lines`` is provided, the file is only modified when the edit can
    land on lines already in the PR diff (inline-suggestion path). Otherwise the
    result is a fallback entry without writing the file.

    When ``pr_lines`` is ``None`` (local CLI use), behaviour is unconditional
    write using append / after-heading placement.

    Returns:
        A result dict when action is needed, otherwise ``None``.
        Keys include ``path``, ``fields``, ``mode`` (``suggestable`` or
        ``fallback``), ``placement``, and ``snippet``.
    """
    content = path.read_text(encoding="utf-8")
    missing = _missing_meta_fields(content, meta_config)
    if not missing:
        logger.info("%s: all configured meta fields present", path)
        return None

    metadata = {field: meta_config[field] for field in missing}
    snippet = format_meta_block(meta_config, missing)
    path_str = str(path).replace("\\", "/")

    if pr_lines is None:
        if has_meta_block(content):
            placement = "append"
        elif first_heading_line_span(content) is not None:
            placement = "after_heading"
        else:
            placement = "at_top"

        if placement == "append":
            new_content, changed = inject_metadata_to_content(content, metadata)
        else:
            new_content, changed = inject_metadata_to_content(
                content,
                metadata,
                new_block_placement=placement,
            )

        if not changed:
            return None
        path.write_text(new_content, encoding="utf-8")
        logger.info("%s: added meta fields %s (%s)", path, ", ".join(missing), placement)
        return {
            "path": path_str,
            "fields": missing,
            "mode": "suggestable",
            "placement": placement,
            "snippet": snippet,
        }

    placement = choose_suggestable_placement(content, pr_lines)
    if placement is None:
        logger.info(
            "%s: missing %s but edit is outside the PR diff; fallback review only",
            path,
            ", ".join(missing),
        )
        return {
            "path": path_str,
            "fields": missing,
            "mode": "fallback",
            "placement": "at_top",
            "snippet": snippet,
        }

    if placement == "append":
        new_content, changed = inject_metadata_to_content(content, metadata)
    else:
        new_content, changed = inject_metadata_to_content(
            content,
            metadata,
            new_block_placement=placement,
        )
    if not changed:
        return None

    path.write_text(new_content, encoding="utf-8")
    logger.info(
        "%s: added meta fields %s via %s (inline suggestion)",
        path,
        ", ".join(missing),
        placement,
    )
    return {
        "path": path_str,
        "fields": missing,
        "mode": "suggestable",
        "placement": placement,
        "snippet": snippet,
    }


def _collect_rst_paths(paths: list[str]) -> list[Path]:
    rst_paths: list[Path] = []
    for raw in paths:
        path = Path(raw)
        if path.suffix.lower() != RST_EXTENSION:
            logger.debug("Skipping non-RST path: %s", raw)
            continue
        if not path.is_file():
            logger.warning("Skipping missing file: %s", raw)
            continue
        rst_paths.append(path)
    return rst_paths


def build_review_comment(results: list[dict[str, object]]) -> str:
    """Build the pull request review / comment body for suggestable and fallback results."""
    suggestable = [r for r in results if r["mode"] == "suggestable"]
    fallback = [r for r in results if r["mode"] == "fallback"]

    lines = [
        "This pull request is missing configured `product` / `distribution` "
        "meta tags (defaults from `tools/meta_tags.yaml`).",
        "",
    ]

    if suggestable:
        lines.append(
            "Please **review and commit the inline suggestions**. They add the "
            "missing fields in place so the documentation metadata stays "
            "complete."
        )
        lines.append("")

    if fallback:
        lines.append(
            "GitHub can only attach suggestions to lines already in the pull "
            "request diff, so the following files could not get an inline "
            "suggestion. Please add this block at the **top of each file**:"
        )
        lines.append("")
        for result in fallback:
            lines.append(f"**`{result['path']}`**")
            lines.append("```rst")
            lines.append(str(result["snippet"]).rstrip())
            lines.append("```")
            lines.append("")

    return "\n".join(lines).rstrip() + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Ensure configured meta tags exist in RST files. "
            "Missing fields are added with values from a YAML config file."
        ),
    )
    parser.add_argument(
        "paths",
        nargs="+",
        help="One or more .rst file paths to check",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help=f"YAML config file (default: {DEFAULT_CONFIG_PATH})",
    )
    parser.add_argument(
        "--diff-base",
        help=(
            "Git commit SHA for the pull request base. When set, only writes "
            "edits that overlap the PR diff; other files become review-comment "
            "fallbacks."
        ),
    )
    parser.add_argument(
        "--status-file",
        type=Path,
        help="Write suggestable=true|false, fallback=true|false, and the review comment for CI",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Enable debug logging",
    )
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s: %(message)s",
    )

    meta_config = load_meta_config(args.config)
    rst_paths = _collect_rst_paths(args.paths)
    results: list[dict[str, object]] = []

    if not rst_paths:
        logger.info("No RST files to process")
    else:
        for path in rst_paths:
            pr_lines: set[int] | None = None
            if args.diff_base:
                pr_lines = pr_diff_lines_for_file(args.diff_base, path)
            result = ensure_meta_tags_in_file(path, meta_config, pr_lines=pr_lines)
            if result is not None:
                results.append(result)

        suggestable_count = sum(1 for r in results if r["mode"] == "suggestable")
        fallback_count = sum(1 for r in results if r["mode"] == "fallback")
        logger.info(
            "Processed %d file(s): %d suggestable write(s), %d fallback(s)",
            len(rst_paths),
            suggestable_count,
            fallback_count,
        )

    if args.status_file is not None:
        has_suggestable = any(r["mode"] == "suggestable" for r in results)
        has_fallback = any(r["mode"] == "fallback" for r in results)
        with args.status_file.open("a", encoding="utf-8") as f:
            f.write(f"suggestable={'true' if has_suggestable else 'false'}\n")
            f.write(f"fallback={'true' if has_fallback else 'false'}\n")
            if results:
                review_body = build_review_comment(results)
                f.write("comment<<EOF_META_TAGS_COMMENT\n")
                f.write(review_body)
                if not review_body.endswith("\n"):
                    f.write("\n")
                f.write("EOF_META_TAGS_COMMENT\n")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
