#!/usr/bin/env python3
"""
Ensure configured metadata fields exist in RST ``.. meta::`` blocks.

Rules are defined in ``meta_tags.yaml`` with severity and optional values.
Missing or blank fields are resolved automatically when a value is configured;
otherwise contributors must supply a non-empty value.

When ``--diff-base`` is set, edits are only written to disk when they overlap
the pull request diff (so GitHub can offer inline suggestions). Otherwise the
review comment carries copy-paste or manual instructions.

In CI (``--status-file``), emits GitHub Actions annotations per severity and
exits with code ``1`` when issues remain so the ensure step can soft-fail.
Error-severity issues also set ``has_errors`` for a final workflow gate.
"""

from __future__ import annotations

import argparse
import logging
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, TextIO

import yaml

# Allow ``python3 tools/ensure_meta_tags.py`` from the repository root.
_TOOLS_DIR = Path(__file__).resolve().parent
if str(_TOOLS_DIR) not in sys.path:
        sys.path.insert(0, str(_TOOLS_DIR))

from rst_utils import (
        get_meta_fields_from_content,
        has_meta_block,
        inject_metadata_to_content,
        meta_block_line_span,
)

logger = logging.getLogger(__name__)

DEFAULT_CONFIG_PATH = _TOOLS_DIR / "meta_tags.yaml"
RST_EXTENSION = ".rst"
_HUNK_HEADER = re.compile(r"^@@ -\d+(?:,\d+)? \+(\d+)(?:,(\d+))? @@")
Severity = Literal["warning", "error"]

# Hidden marker in review bodies so CI can find and supersede prior bot reviews.
REVIEW_MARKER_ID = "ros2-meta-tags-ensure"
REVIEW_MARKER = f"<!-- {REVIEW_MARKER_ID} -->"

# Short body for the suggest-changes review. Deliberately unstamped: that review is
# the only Conversation surface that shows live inline suggestions, so it must not be
# minimised with the summary. GitHub marks individual comments outdated when actioned.
SUGGESTION_NOTE = (
    "Inline suggestions add configured documentation metadata defaults. "
    "See the metadata review comment for the full list of fields to complete."
)


@dataclass(frozen=True)
class MetaRule:
    """
    A single metadata field rule from ``meta_tags.yaml``.

    Attributes:
            severity: Advisory ``warning`` or blocking ``error`` in CI.
            value: Default text to inject when missing or blank; empty when the
                    contributor must supply a non-empty value.
    """

    severity: Severity
    value: str

    @property
    def has_configured_value(self) -> bool:
        """
        Return whether the rule supplies a non-empty default value.

        Returns:
                ``True`` when ``value`` is non-empty after stripping whitespace.
        """
        return bool(self.value.strip())


def load_meta_config(config_path: Path) -> dict[str, MetaRule]:
    """
    Load and validate metadata rules from a YAML config file.

    Args:
            config_path: Path to the YAML configuration file.

    Returns:
            Mapping from meta field name to its rule.

    Raises:
            SystemExit: If the file is missing, invalid, or has unusable rules.
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

    validated: dict[str, MetaRule] = {}
    for key, entry in meta.items():
        if not isinstance(key, str) or not key.strip():
            logger.error("Config %s: meta keys must be non-empty strings", config_path)
            raise SystemExit(1)
        if not isinstance(entry, dict):
            logger.error(
                "Config %s: meta entry for %r must be a mapping with severity and value",
                config_path,
                key,
            )
            raise SystemExit(1)
        severity = entry.get("severity")
        if severity not in ("warning", "error"):
            logger.error(
                "Config %s: meta entry %r severity must be 'warning' or 'error', got %r",
                config_path,
                key,
                severity,
            )
            raise SystemExit(1)
        if "value" not in entry:
            logger.error("Config %s: meta entry %r must include a 'value' key", config_path, key)
            raise SystemExit(1)
        raw_value = entry.get("value")
        if raw_value is None:
            value = ""
        elif isinstance(raw_value, str):
            value = raw_value
        else:
            logger.error(
                "Config %s: meta value for %r must be a string or null, got %s",
                config_path,
                key,
                type(raw_value).__name__,
            )
            raise SystemExit(1)
        validated[key] = MetaRule(severity=severity, value=value)

    return validated


def format_meta_block(rules: dict[str, MetaRule], fields: list[str]) -> str:
    """
    Build an RST ``.. meta::`` block for auto-injectable fields.

    Args:
            rules: Configured metadata rules keyed by field name.
            fields: Field names to include in the block, in output order.

    Returns:
            A formatted ``.. meta::`` directive ending with a blank line.

    Raises:
            KeyError: If a requested field is absent from ``rules``.
    """
    lines = [".. meta::"]
    for field in fields:
        lines.append(f"   :{field}: {rules[field].value}")
    lines.append("")
    return "\n".join(lines)


def _unresolved_fields(content: str, rules: dict[str, MetaRule]) -> list[str]:
    """
    Find configured meta fields that are absent or blank in RST content.

    Args:
            content: RST source to inspect.
            rules: Configured metadata rules.

    Returns:
            Unresolved field names in configuration order.
    """
    present = get_meta_fields_from_content(content)
    return [
        name for name in rules
        if name not in present or not present[name].strip()
    ]


def parse_diff_new_side_lines(diff_text: str) -> set[int]:
    """
    Parse a unified diff and return 1-based line numbers on the new (``+``) side.

    Both added and context lines within hunks are included so nearby suggestion
    anchors count as overlapping the pull request diff.

    File headers (``---`` / ``+++``) are only skipped outside hunks. Inside a
    hunk those prefixes are ordinary ``-`` / ``+`` lines (e.g. RST table rows of
    ``+`` characters), and must advance ``new_line`` accordingly.

    Args:
            diff_text: Unified diff text to parse.

    Returns:
            One-based line numbers represented on the new side of diff hunks.
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
            in_hunk = False
            new_line = 0
    return lines


def pr_diff_lines_for_file(diff_base: str, path: Path) -> set[int]:
    """
    Find pull-request diff lines for a file on the head side.

    Args:
            diff_base: Base commit SHA used for the three-dot comparison.
            path: Repository-relative path to inspect.

    Returns:
            One-based new-side line numbers, or an empty set if ``git diff`` fails.
    """
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


def changed_rst_paths(diff_base: str) -> list[Path]:
    """
    List ``.rst`` files changed between a pull request base and ``HEAD``.

    Args:
            diff_base: Base commit SHA used for the three-dot comparison.

    Returns:
            Repository-relative paths for added, copied, modified, or renamed
            ``.rst`` files, or an empty list if ``git diff`` fails.
    """
    result = subprocess.run(
        [
            "git",
            "diff",
            "--name-only",
            "--diff-filter=ACMR",
            f"{diff_base}...HEAD",
            "--",
            "*.rst",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode not in (0, 1):
        logger.warning(
            "git diff failed listing changed RST files (exit %s): %s",
            result.returncode,
            result.stderr.strip(),
        )
        return []
    paths: list[Path] = []
    for line in result.stdout.splitlines():
        stripped = line.strip()
        if stripped:
            paths.append(Path(stripped))
    return paths


def _log_working_tree_summary(paths: list[Path]) -> None:
    """
    Log ``git status`` and ``git diff`` for processed RST paths.

    Args:
            paths: Repository-relative RST files that were checked or updated.

    Returns:
            None.
    """
    if not paths:
        return
    path_args = [str(p) for p in paths]
    logger.info("Working tree after ensure_meta_tags:")
    status = subprocess.run(
        ["git", "status", "--short", "--", *path_args],
        check=False,
        capture_output=True,
        text=True,
    )
    if status.stdout.strip():
        for line in status.stdout.splitlines():
            logger.info("%s", line)
    else:
        logger.info("(no changes)")
    diff = subprocess.run(
        ["git", "diff", "--", *path_args],
        check=False,
        capture_output=True,
        text=True,
    )
    if diff.stdout.strip():
        for line in diff.stdout.splitlines():
            logger.info("%s", line)


def _write_multiline_output(handle: TextIO, key: str, value: str) -> None:
    """
    Write a multiline GitHub Actions output using heredoc syntax.

    Args:
            handle: Open text handle for the status file.
            key: Output name.
            value: Output value, which may span several lines.

    Returns:
            None.
    """
    delimiter = f"EOF_{key.upper()}"
    handle.write(f"{key}<<{delimiter}\n")
    handle.write(value)
    if not value.endswith("\n"):
        handle.write("\n")
    handle.write(f"{delimiter}\n")


def _write_ci_status_file(
    status_file: Path,
    *,
    meta_checked: bool,
    results: list[dict[str, object]],
    rules: dict[str, MetaRule],
    has_errors: bool,
) -> None:
    """
    Append GitHub Actions output flags and optional review comment.

    Writes ``meta_checked``, ``inline_suggestions``, ``has_results``, and
    ``has_errors``, plus a multiline ``comment`` block when ``results`` is
    non-empty and ``suggestion_note`` when inline suggestions were written.

    Args:
            status_file: Path to append to (for example ``$GITHUB_OUTPUT``).
            meta_checked: Whether changed RST files were in scope for this run.
            results: Per-file result dicts from ``ensure_meta_tags_in_file``.
            rules: Configured metadata rules for building the review body.
            has_errors: Whether any result has unresolved error-severity fields.

    Returns:
            None.
    """
    has_inline_suggestions = any(r["mode"] == "suggestable" for r in results)
    has_results = bool(results)
    with status_file.open("a", encoding="utf-8") as f:
        for key, flag in (
            ("meta_checked", meta_checked),
            ("inline_suggestions", has_inline_suggestions),
            ("has_results", has_results),
            ("has_errors", has_errors),
        ):
            f.write(f"{key}={'true' if flag else 'false'}\n")
        if results:
            _write_multiline_output(f, "comment", build_review_comment(results, rules))
        if has_inline_suggestions:
            _write_multiline_output(f, "suggestion_note", SUGGESTION_NOTE)


def _span_overlaps(span: tuple[int, int] | None, pr_lines: set[int]) -> bool:
    """
    Test whether an inclusive line span overlaps pull-request lines.

    Args:
            span: Inclusive one-based start and end lines, or ``None``.
            pr_lines: One-based line numbers represented by the pull-request diff.

    Returns:
            ``True`` when at least one line overlaps; otherwise ``False``.
    """
    if span is None or not pr_lines:
        return False
    start, end = span
    return any(line in pr_lines for line in range(start, end + 1))


def _annotation_line_for_content(content: str) -> int:
    """
    Select a line for a GitHub annotation on RST content.

    Args:
            content: RST source to inspect.

    Returns:
            The first line of an existing meta block, or line 1 if none exists.
    """
    span = meta_block_line_span(content)
    if span is not None:
        return span[0]
    return 1


def _escape_workflow_command_message(message: str) -> str:
    """
    Escape a message for use in a GitHub Actions workflow command.

    Args:
            message: Unescaped annotation message.

    Returns:
            The message with workflow-command control characters escaped.
    """
    return message.replace("%", "%25").replace("\r", "%0D").replace("\n", "%0A")


def _emit_annotation(level: str, path: str, fields: list[str], line: int) -> None:
    """
    Print a GitHub Actions workflow annotation for missing meta fields.

    Args:
            level: Annotation level, typically ``warning`` or ``error``.
            path: Repository-relative path to annotate.
            fields: Missing meta field names.
            line: One-based source line to annotate.

    Returns:
            None.
    """
    if not fields:
        return
    field_list = ", ".join(fields)
    message = _escape_workflow_command_message(
        f"Missing meta fields: {field_list}",
    )
    print(f"::{level} file={path},line={line}::{message}")


def emit_github_warning(path: str, fields: list[str], line: int) -> None:
    """
    Print a GitHub Actions warning annotation for missing meta fields.

    Args:
            path: Repository-relative path to annotate.
            fields: Missing meta field names.
            line: One-based source line to annotate.

    Returns:
            None.
    """
    _emit_annotation("warning", path, fields, line)


def emit_github_error(path: str, fields: list[str], line: int) -> None:
    """
    Print a GitHub Actions error annotation for missing meta fields.

    Args:
            path: Repository-relative path to annotate.
            fields: Missing meta field names.
            line: One-based source line to annotate.

    Returns:
            None.
    """
    _emit_annotation("error", path, fields, line)


def can_suggest_inline(content: str, pr_lines: set[int]) -> bool:
    """
    Return whether a meta-tag edit can be anchored to the pull request diff.

    Existing ``.. meta::`` blocks are suggestable when the block's inclusive line
    span overlaps the diff. New blocks are only suggestable when line 1 is in
    the diff, since inserts always go at the top of the file.

    Args:
            content: RST source before metadata is injected.
            pr_lines: One-based lines represented by the pull-request diff.

    Returns:
            ``True`` if GitHub can anchor the metadata edit to the diff.
    """
    if has_meta_block(content):
        span = meta_block_line_span(content)
        if span is None:
            return False
        return _span_overlaps(span, pr_lines)

    return _span_overlaps((1, 1), pr_lines)


def _severity_fields(
    field_names: list[str],
    rules: dict[str, MetaRule],
    severity: Severity,
) -> list[str]:
    """
    Return field names that use the given severity in ``rules``.

    Args:
            field_names: Candidate meta field names.
            rules: Configured metadata rules keyed by field name.
            severity: Severity label to match.

    Returns:
            Names from ``field_names`` whose rule uses ``severity``.
    """
    return [name for name in field_names if rules[name].severity == severity]


def ensure_meta_tags_in_file(
    path: Path,
    rules: dict[str, MetaRule],
    *,
    pr_lines: set[int] | None = None,
) -> dict[str, object] | None:
    """
    Resolve missing metadata in one RST file where rules allow automatic fixes.

    When ``pr_lines`` is provided, automatic edits are only written when they can
    land on lines already in the PR diff. Fields without configured values always
    require manual input.

    Args:
            path: RST file to inspect and, where permitted, update.
            rules: Configured metadata rules keyed by field name.
            pr_lines: One-based pull-request diff lines, or ``None`` for local mode.

    Returns:
            A result dict when fields were missing in the file as read, otherwise
            ``None``. Each result includes ``path``, ``line``, ``mode``
            (``suggestable``, ``snippet``, or ``manual_fields``), ``snippet`` (RST
            for copy-paste when relevant), ``auto_fields`` (configured values the
            tool can fill), ``manual_fields``, ``warning_fields``, and
            ``error_fields``. The severity lists cover every field that was missing
            or blank before any automatic injection.

    Raises:
            OSError: If the RST file cannot be read or an eligible edit cannot be written.
            UnicodeError: If the RST file cannot be decoded or encoded as UTF-8.
    """
    content = path.read_text(encoding="utf-8")
    unresolved = _unresolved_fields(content, rules)
    if not unresolved:
        logger.info("%s: all configured meta fields present", path)
        return None

    path_str = str(path).replace("\\", "/")
    annotation_line = _annotation_line_for_content(content)

    auto_fields = [name for name in unresolved if rules[name].has_configured_value]
    auto_metadata = {name: rules[name].value for name in auto_fields}

    mode: str | None = None
    snippet = ""

    if auto_metadata:
        if pr_lines is not None and not can_suggest_inline(content, pr_lines):
            mode = "snippet"
            snippet = format_meta_block(rules, auto_fields)
            logger.info(
                "%s: missing %s but auto-fix is outside the PR diff; snippet review only",
                path,
                ", ".join(auto_fields),
            )
        else:
            new_content, changed = inject_metadata_to_content(content, auto_metadata)
            if changed:
                path.write_text(new_content, encoding="utf-8")
                content = new_content
                mode = "suggestable"
                snippet = format_meta_block(rules, auto_fields)
                if pr_lines is None:
                    logger.info("%s: added meta fields %s", path, ", ".join(auto_fields))
                else:
                    logger.info(
                        "%s: added meta fields %s (inline suggestion)",
                        path,
                        ", ".join(auto_fields),
                    )

    still_unresolved = _unresolved_fields(content, rules)
    if not still_unresolved and mode is None:
        return None

    # Annotations and severity describe the pull request as pushed. Auto-injected
    # values only exist in the CI working tree until the suggestion is committed,
    # so they are reported from ``unresolved`` rather than ``still_unresolved``.
    manual_fields = [name for name in unresolved if not rules[name].has_configured_value]
    warning_fields = _severity_fields(unresolved, rules, "warning")
    error_fields = _severity_fields(unresolved, rules, "error")

    if mode is None:
        mode = "manual_fields"

    return {
        "path": path_str,
        "line": annotation_line,
        "mode": mode,
        "snippet": snippet,
        "auto_fields": auto_fields,
        "manual_fields": manual_fields,
        "warning_fields": warning_fields,
        "error_fields": error_fields,
    }


def _collect_rst_paths(paths: list[str]) -> list[Path]:
    """
    Collect existing RST files from command-line path strings.

    Args:
            paths: Candidate filesystem paths.

    Returns:
            Existing paths whose suffix is ``.rst`` (case-insensitive).
    """
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


def stamp_review_comment(body: str) -> str:
    """
    Stamp a review comment so CI can supersede it later.

    Args:
            body: Unstamped review comment body.

    Returns:
            The body with the hidden review marker appended.
    """
    return body.rstrip() + f"\n\n{REVIEW_MARKER}\n"


def _field_list_markdown(field_names: list[str], rules: dict[str, MetaRule]) -> str:
    """
    Format field names with severity hints for review text.

    Args:
            field_names: Meta field names to list.
            rules: Configured metadata rules keyed by field name.

    Returns:
            A comma-separated Markdown fragment such as `` `area` (required) ``.
    """
    parts: list[str] = []
    for name in field_names:
        label = "required" if rules[name].severity == "error" else "warning"
        parts.append(f"`{name}` ({label})")
    return ", ".join(parts)


def build_review_comment(
    results: list[dict[str, object]],
    rules: dict[str, MetaRule],
) -> str:
    """
    Build a pull-request review body from metadata check results.

    Args:
            results: Per-file result dictionaries from ``ensure_meta_tags_in_file``.
            rules: Configured metadata rules.

    Returns:
            A stamped Markdown review body containing the relevant instructions.
    """
    inline_modes = [r for r in results if r["mode"] == "suggestable"]
    snippet_modes = [r for r in results if r["mode"] == "snippet"]
    manual_fields_modes = [r for r in results if r["mode"] == "manual_fields"]

    lines = [
        "This pull request is missing configured documentation metadata "
        "(see `tools/meta_tags.yaml`).",
        "",
    ]

    if inline_modes:
        lines.append(
            "Please **review and commit the inline suggestions**. They add these "
            "configured default values in place so metadata stays complete:",
        )
        lines.append("")
        for result in inline_modes:
            auto = ", ".join(f"`{name}`" for name in result["auto_fields"])
            lines.append(f"**`{result['path']}`**: {auto}")
        lines.append("")

    if snippet_modes:
        lines.append(
            "GitHub can only attach suggestions to lines already in the pull request "
            "diff, so the following files could not get an inline suggestion for "
            "auto-filled fields. Please add this block at the **top of each file** "
            "(or append the listed fields to an existing `.. meta::` block):",
        )
        lines.append("")
        for result in snippet_modes:
            lines.append(f"**`{result['path']}`**")
            lines.append("```rst")
            lines.append(str(result["snippet"]).rstrip())
            lines.append("```")
            lines.append("")

    manual_results = [r for r in results if r.get("manual_fields")]
    if manual_results:
        lines.append(
            "The following fields must be provided with **non-empty** values in each "
            "file's `.. meta::` block:",
        )
        lines.append("")
        for result in manual_results:
            manual = list(result["manual_fields"])
            lines.append(f"**`{result['path']}`**: {_field_list_markdown(manual, rules)}")
        lines.append("")

    if manual_fields_modes and not inline_modes and not snippet_modes:
        lines.append(
            "Add or complete a `.. meta::` block at the top of each affected file.",
        )
        lines.append("")

    body = "\n".join(lines).rstrip() + "\n"
    return stamp_review_comment(body)


def main(argv: list[str] | None = None) -> int:
    """
    Run the command-line metadata check.

    When no ``paths`` are given, ``--diff-base`` must be set so changed ``.rst``
    files are discovered with ``git diff``. With ``--status-file``, writes CI
    outputs and exits ``1`` when any per-file results remain; without it, exits
    ``1`` only for unresolved error-severity fields.

    Args:
            argv: Command-line arguments excluding the executable name, or ``None``
                    to read them from ``sys.argv``.

    Returns:
            Process exit code (``0`` on success, ``1`` when issues remain per mode
            above).

    Raises:
            SystemExit: If command-line arguments or metadata configuration are invalid.
    """
    parser = argparse.ArgumentParser(
        description=(
            "Ensure configured meta tags exist in RST files using rules from a YAML "
            "config file."
        ),
    )
    parser.add_argument(
        "paths",
        nargs="*",
        help=(
            "One or more .rst file paths to check; when omitted, "
            "--diff-base must be set to discover changed files"
        ),
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
            "Git commit SHA for the pull request base. When set, only writes edits "
            "that overlap the PR diff; other files receive a review comment instead."
        ),
    )
    parser.add_argument(
        "--status-file",
        type=Path,
        help=(
            "Write meta_checked, inline_suggestions, has_results, has_errors, "
            "the review comment body, and the suggestion note for CI"
        ),
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

    if not args.paths and not args.diff_base:
        parser.error("provide at least one .rst path or set --diff-base to discover changes")

    rules = load_meta_config(args.config)
    checked_pull_request_rst = False

    if not args.paths:
        assert args.diff_base is not None
        discovered = changed_rst_paths(args.diff_base)
        if not discovered:
            logger.info("No changed RST files in this pull request.")
            if args.status_file is not None:
                _write_ci_status_file(
                    args.status_file,
                    meta_checked=False,
                    results=[],
                    rules=rules,
                    has_errors=False,
                )
            return 0
        checked_pull_request_rst = True
        rst_paths = _collect_rst_paths([str(p) for p in discovered])
    else:
        checked_pull_request_rst = True
        rst_paths = _collect_rst_paths(args.paths)

    results: list[dict[str, object]] = []

    if not rst_paths:
        logger.info("No RST files to process")
    else:
        for path in rst_paths:
            pr_lines: set[int] | None = None
            if args.diff_base:
                pr_lines = pr_diff_lines_for_file(args.diff_base, path)
            result = ensure_meta_tags_in_file(path, rules, pr_lines=pr_lines)
            if result is not None:
                results.append(result)

        inline_count = sum(1 for r in results if r["mode"] == "suggestable")
        snippet_count = sum(1 for r in results if r["mode"] == "snippet")
        manual_fields_count = sum(1 for r in results if r["mode"] == "manual_fields")
        logger.info(
            "Processed %d file(s): %d inline, %d snippet, %d manual_fields",
            len(rst_paths),
            inline_count,
            snippet_count,
            manual_fields_count,
        )
        _log_working_tree_summary(rst_paths)

    for result in results:
        path = str(result["path"])
        line = int(result["line"])
        emit_github_warning(path, list(result["warning_fields"]), line)
        emit_github_error(path, list(result["error_fields"]), line)

    has_errors = any(result["error_fields"] for result in results)

    if args.status_file is not None:
        _write_ci_status_file(
            args.status_file,
            meta_checked=checked_pull_request_rst,
            results=results,
            rules=rules,
            has_errors=has_errors,
        )

    if args.status_file is not None and results:
        return 1
    if args.status_file is None and has_errors:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
