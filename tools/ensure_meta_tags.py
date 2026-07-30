#!/usr/bin/env python3
"""
Ensure configured documentation enhancements exist in RST source files.

Rules are defined in ``enhance.yaml``: ``meta`` field rules for ``.. meta::``
blocks and ``after_title`` rules for post-heading directives such as
``.. short-description::`` and ``.. showmeta::``.

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
        after_title_directives_line_span,
        extract_first_paragraph_after_title,
        format_showmeta_block,
        get_meta_fields_from_content,
        has_meta_block,
        has_short_description_content,
        has_showmeta_with_order,
        inject_metadata_to_content,
        inject_showmeta_to_content,
        meta_block_line_span,
        wrap_first_paragraph_as_short_description,
)

logger = logging.getLogger(__name__)

DEFAULT_CONFIG_PATH = _TOOLS_DIR / "enhance.yaml"
RST_EXTENSION = ".rst"
_HUNK_HEADER = re.compile(r"^@@ -\d+(?:,\d+)? \+(\d+)(?:,(\d+))? @@")
Severity = Literal["warning", "error"]

# Hidden marker in review bodies so CI can find and supersede prior bot reviews.
REVIEW_MARKER_ID = "ros2-meta-tags-ensure"
REVIEW_MARKER = f"<!-- {REVIEW_MARKER_ID} -->"

# Titles and section headings for pull request review bodies (GitHub Markdown).
SUMMARY_REVIEW_TITLE = "## Documentation enhancements"
SUGGESTION_REVIEW_TITLE = "## Inline documentation suggestions"
SECTION_INLINE_SUGGESTIONS = "### Commit inline suggestions"
SECTION_COPY_PASTE_BLOCKS = "### Copy-paste `.. meta::` blocks"
SECTION_COPY_PASTE_AFTER_TITLE = "### Copy-paste after-title directives"
SECTION_NON_EMPTY_VALUES = "### Provide non-empty values"
SECTION_AFTER_TITLE_MANUAL = "### Add after-title directives"

# Short body for the suggest-changes review. Deliberately unstamped: that review is
# the only Conversation surface that shows live inline suggestions, so it must not be
# minimised with the summary. GitHub marks individual comments outdated when actioned.
SUGGESTION_NOTE = (
    f"{SUGGESTION_REVIEW_TITLE}\n"
    "\n"
    "Each **Commit suggestion** below applies configured documentation "
    "enhancements from `tools/enhance.yaml`.\n"
    "\n"
    "For copy-paste blocks, required fields, and the full per-file breakdown, "
    "see the **Documentation enhancements** review comment."
)


@dataclass(frozen=True)
class AfterTitleRule:
    """
    A single after-title directive rule from ``enhance.yaml``.

    Attributes:
            directive: Directive name (e.g. ``short-description``, ``showmeta``).
            severity: Advisory ``warning`` or blocking ``error`` in CI.
            content: Content source for body directives (e.g. ``first_paragraph``).
            options: Option name/value pairs for option-only directives.
            required_options: Option names that must be non-empty when present.
    """

    directive: str
    severity: Severity
    content: str | None = None
    options: dict[str, str] | None = None
    required_options: tuple[str, ...] = ()


@dataclass(frozen=True)
class EnhanceConfig:
    """Full enhancement configuration loaded from ``enhance.yaml``."""

    meta: dict[str, MetaRule]
    after_title: tuple[AfterTitleRule, ...]


@dataclass(frozen=True)
class MetaRule:
    """
    A single metadata field rule from ``enhance.yaml``.

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


def _parse_meta_rules(meta: dict, config_path: Path) -> dict[str, MetaRule]:
    """Validate and parse the ``meta`` mapping from config YAML."""
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


def _parse_after_title_rules(
    raw_list: object,
    config_path: Path,
) -> tuple[AfterTitleRule, ...]:
    """Validate and parse the ``after_title`` list from config YAML."""
    if raw_list is None:
        return ()
    if not isinstance(raw_list, list):
        logger.error("Config %s: 'after_title' must be a list", config_path)
        raise SystemExit(1)

    supported_directives = {"short-description", "showmeta"}
    validated: list[AfterTitleRule] = []
    for index, entry in enumerate(raw_list):
        if not isinstance(entry, dict):
            logger.error(
                "Config %s: after_title[%d] must be a mapping",
                config_path,
                index,
            )
            raise SystemExit(1)
        directive = entry.get("directive")
        if not isinstance(directive, str) or not directive.strip():
            logger.error(
                "Config %s: after_title[%d] must include a non-empty 'directive'",
                config_path,
                index,
            )
            raise SystemExit(1)
        if directive not in supported_directives:
            logger.error(
                "Config %s: after_title[%d] directive %r is not supported",
                config_path,
                index,
                directive,
            )
            raise SystemExit(1)
        severity = entry.get("severity")
        if severity not in ("warning", "error"):
            logger.error(
                "Config %s: after_title[%d] severity must be 'warning' or 'error', got %r",
                config_path,
                index,
                severity,
            )
            raise SystemExit(1)

        content = entry.get("content")
        if content is not None and not isinstance(content, str):
            logger.error(
                "Config %s: after_title[%d] content must be a string",
                config_path,
                index,
            )
            raise SystemExit(1)

        raw_options = entry.get("options")
        options: dict[str, str] | None = None
        if raw_options is not None:
            if not isinstance(raw_options, dict):
                logger.error(
                    "Config %s: after_title[%d] options must be a mapping",
                    config_path,
                    index,
                )
                raise SystemExit(1)
            options = {}
            for opt_key, opt_value in raw_options.items():
                if not isinstance(opt_key, str) or not isinstance(opt_value, str):
                    logger.error(
                        "Config %s: after_title[%d] option keys and values must be strings",
                        config_path,
                        index,
                    )
                    raise SystemExit(1)
                options[opt_key] = opt_value

        raw_required = entry.get("required_options")
        required_options: tuple[str, ...] = ()
        if raw_required is not None:
            if not isinstance(raw_required, list):
                logger.error(
                    "Config %s: after_title[%d] required_options must be a list",
                    config_path,
                    index,
                )
                raise SystemExit(1)
            required_options = tuple(str(item) for item in raw_required)

        if directive == "short-description":
            if content != "first_paragraph":
                logger.error(
                    "Config %s: short-description rule must use content: first_paragraph",
                    config_path,
                )
                raise SystemExit(1)
        elif directive == "showmeta":
            if not options or not options.get("order", "").strip():
                logger.error(
                    "Config %s: showmeta rule must include options.order",
                    config_path,
                )
                raise SystemExit(1)
            if "order" not in required_options:
                logger.error(
                    "Config %s: showmeta rule must list order in required_options",
                    config_path,
                )
                raise SystemExit(1)

        validated.append(
            AfterTitleRule(
                directive=directive,
                severity=severity,
                content=content,
                options=options,
                required_options=required_options,
            ),
        )
    return tuple(validated)


def load_enhance_config(config_path: Path) -> EnhanceConfig:
    """
    Load and validate enhancement rules from a YAML config file.

    Args:
            config_path: Path to the YAML configuration file.

    Returns:
            Parsed enhancement configuration.

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

    return EnhanceConfig(
        meta=_parse_meta_rules(meta, config_path),
        after_title=_parse_after_title_rules(raw.get("after_title"), config_path),
    )


def load_meta_config(config_path: Path) -> dict[str, MetaRule]:
    """
    Load metadata rules from a YAML config file.

    Deprecated alias for ``load_enhance_config(...).meta``.
    """
    return load_enhance_config(config_path).meta


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
            The first line of an existing meta block, the after-title area, or line 1.
    """
    span = meta_block_line_span(content)
    if span is not None:
        return span[0]
    after_title = after_title_directives_line_span(content)
    if after_title is not None:
        return after_title[0]
    return 1


def _emit_after_title_annotation(
    level: str,
    path: str,
    directives: list[str],
    line: int,
) -> None:
    """Print a GitHub Actions annotation for missing after-title directives."""
    if not directives:
        return
    directive_list = ", ".join(directives)
    message = _escape_workflow_command_message(
        f"Missing after-title directives: {directive_list}",
    )
    print(f"::{level} file={path},line={line}::{message}")


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


def can_suggest_after_title_inline(
    content: str,
    pr_lines: set[int],
    *,
    paragraph_span: tuple[int, int] | None = None,
) -> bool:
    """
    Return whether after-title directive edits can anchor to the PR diff.

    Args:
            content: RST source before after-title injections.
            pr_lines: One-based lines represented by the pull-request diff.
            paragraph_span: Optional line span of the paragraph to wrap.

    Returns:
            ``True`` when the after-title area or paragraph span overlaps the diff.
    """
    if paragraph_span is not None and _span_overlaps(paragraph_span, pr_lines):
        return True
    return _span_overlaps(after_title_directives_line_span(content), pr_lines)


def _after_title_rule_satisfied(content: str, rule: AfterTitleRule) -> bool:
    """Return whether an after-title rule is already satisfied in ``content``."""
    if rule.directive == "short-description":
        return has_short_description_content(content)
    if rule.directive == "showmeta":
        return has_showmeta_with_order(content)
    return True


def _format_short_description_snippet(paragraph: str) -> str:
    """Build a copy-paste ``.. short-description::`` block for a paragraph."""
    lines = [".. short-description::"]
    for chunk in paragraph.split("\n\n"):
        for line in chunk.split("\n"):
            stripped = line.strip()
            if stripped:
                lines.append(f"   {stripped}")
    lines.append("")
    return "\n".join(lines)


def _process_after_title_rules(
    path: Path,
    content: str,
    after_title_rules: tuple[AfterTitleRule, ...],
    *,
    pr_lines: set[int] | None,
) -> tuple[str, dict[str, object]]:
    """
    Apply configured after-title directive rules to RST content.

    Returns:
            Updated content and a dict of after-title result fields.
    """
    unresolved_rules = [rule for rule in after_title_rules if not _after_title_rule_satisfied(content, rule)]
    if not unresolved_rules:
        return content, {}

    after_title_auto: list[str] = []
    after_title_manual: list[str] = []
    after_title_warning: list[str] = []
    after_title_error: list[str] = []
    after_title_snippets: list[dict[str, str]] = []
    suggestable = False
    snippet_only = False

    for rule in unresolved_rules:
        paragraph_span: tuple[int, int] | None = None
        if rule.directive == "short-description":
            paragraph, paragraph_span = extract_first_paragraph_after_title(content)
            if paragraph is None:
                after_title_manual.append(rule.directive)
                if rule.severity == "error":
                    after_title_error.append(rule.directive)
                else:
                    after_title_warning.append(rule.directive)
                continue

            can_suggest = pr_lines is None or can_suggest_after_title_inline(
                content,
                pr_lines,
                paragraph_span=paragraph_span,
            )
            snippet_text = _format_short_description_snippet(paragraph)

            if can_suggest:
                new_content, changed = wrap_first_paragraph_as_short_description(content)
                if changed:
                    content = new_content
                    after_title_auto.append(rule.directive)
                    suggestable = True
                    if rule.severity == "error":
                        after_title_error.append(rule.directive)
                    else:
                        after_title_warning.append(rule.directive)
            else:
                after_title_snippets.append({"directive": rule.directive, "snippet": snippet_text})
                snippet_only = True
                if rule.severity == "error":
                    after_title_error.append(rule.directive)
                else:
                    after_title_warning.append(rule.directive)

        elif rule.directive == "showmeta":
            assert rule.options is not None
            can_suggest = pr_lines is None or can_suggest_after_title_inline(content, pr_lines)
            snippet_text = format_showmeta_block(rule.options)

            if can_suggest:
                new_content, changed = inject_showmeta_to_content(content, rule.options)
                if changed:
                    content = new_content
                    after_title_auto.append(rule.directive)
                    suggestable = True
                    if rule.severity == "error":
                        after_title_error.append(rule.directive)
                    else:
                        after_title_warning.append(rule.directive)
            else:
                after_title_snippets.append({"directive": rule.directive, "snippet": snippet_text})
                snippet_only = True
                if rule.severity == "error":
                    after_title_error.append(rule.directive)
                else:
                    after_title_warning.append(rule.directive)

    if not (after_title_auto or after_title_manual or after_title_snippets):
        return content, {}

    mode = "suggestable" if suggestable else ("snippet" if snippet_only else "manual_fields")
    return content, {
        "after_title_auto": after_title_auto,
        "after_title_manual": after_title_manual,
        "after_title_warning": after_title_warning,
        "after_title_error": after_title_error,
        "after_title_snippets": after_title_snippets,
        "after_title_mode": mode,
    }


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
    config: EnhanceConfig | dict[str, MetaRule],
    *,
    pr_lines: set[int] | None = None,
) -> dict[str, object] | None:
    """
    Resolve missing documentation enhancements in one RST file.

    When ``pr_lines`` is provided, automatic edits are only written when they can
    land on lines already in the PR diff. Fields without configured values always
    require manual input.

    Args:
            path: RST file to inspect and, where permitted, update.
            config: Enhancement configuration, or a legacy meta-rules mapping.
            pr_lines: One-based pull-request diff lines, or ``None`` for local mode.

    Returns:
            A result dict when issues were found in the file as read, otherwise
            ``None``.

    Raises:
            OSError: If the RST file cannot be read or an eligible edit cannot be written.
            UnicodeError: If the RST file cannot be decoded or encoded as UTF-8.
    """
    if isinstance(config, dict):
        enhance_config = EnhanceConfig(meta=config, after_title=())
    else:
        enhance_config = config
    rules = enhance_config.meta

    content = path.read_text(encoding="utf-8")
    path_str = str(path).replace("\\", "/")

    unresolved = _unresolved_fields(content, rules)
    after_title_unresolved = [
        rule
        for rule in enhance_config.after_title
        if not _after_title_rule_satisfied(content, rule)
    ]

    if not unresolved and not after_title_unresolved:
        logger.info("%s: all configured enhancements present", path)
        return None

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

    after_title_result: dict[str, object] = {}
    if enhance_config.after_title:
        content_before_after_title = content
        content, after_title_result = _process_after_title_rules(
            path,
            content,
            enhance_config.after_title,
            pr_lines=pr_lines,
        )
        if after_title_result and content != content_before_after_title:
            path.write_text(content, encoding="utf-8")

    manual_fields = [name for name in unresolved if not rules[name].has_configured_value]
    warning_fields = _severity_fields(unresolved, rules, "warning")
    error_fields = _severity_fields(unresolved, rules, "error")

    after_title_mode = after_title_result.get("after_title_mode")
    if after_title_mode == "suggestable":
        mode = "suggestable"
    elif after_title_mode == "snippet" and mode != "suggestable":
        mode = "snippet"
    elif mode is None and (unresolved or after_title_result):
        mode = "manual_fields"

    if not unresolved and not after_title_result and mode is None:
        return None

    return {
        "path": path_str,
        "line": annotation_line,
        "mode": mode,
        "snippet": snippet,
        "auto_fields": auto_fields,
        "manual_fields": manual_fields,
        "warning_fields": warning_fields,
        "error_fields": error_fields,
        "after_title_auto": after_title_result.get("after_title_auto", []),
        "after_title_manual": after_title_result.get("after_title_manual", []),
        "after_title_warning": after_title_result.get("after_title_warning", []),
        "after_title_error": after_title_result.get("after_title_error", []),
        "after_title_snippets": after_title_result.get("after_title_snippets", []),
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
        label = "required" if rules[name].severity == "error" else "optional"   
        parts.append(f"`{name}` ({label})")
    return ", ".join(parts)


def build_review_comment(
    results: list[dict[str, object]],
    rules: dict[str, MetaRule],
) -> str:
    """
    Build a pull-request review body from enhancement check results.

    Args:
            results: Per-file result dictionaries from ``ensure_meta_tags_in_file``.
            rules: Configured metadata rules.

    Returns:
            A stamped Markdown review body containing the relevant instructions.
    """
    inline_modes = [r for r in results if r["mode"] == "suggestable"]
    snippet_modes = [r for r in results if r["mode"] == "snippet"]
    manual_fields_modes = [r for r in results if r["mode"] == "manual_fields"]
    after_title_snippet_results = [
        r for r in results if r.get("after_title_snippets")
    ]
    after_title_manual_results = [
        r for r in results if r.get("after_title_manual")
    ]

    lines = [
        SUMMARY_REVIEW_TITLE,
        "",
        "This pull request is missing configured documentation enhancements "
        "(see `tools/enhance.yaml`).",
        "",
    ]

    if inline_modes:
        lines.extend(
            [
                SECTION_INLINE_SUGGESTIONS,
                "",
                "Please **review and commit the inline suggestions** on the "
                "**Files changed** tab (or use the separate *Inline documentation "
                "suggestions* review). They apply these configured defaults:",
                "",
            ]
        )
        for result in inline_modes:
            parts: list[str] = []
            auto = result.get("auto_fields") or []
            if auto:
                parts.append(", ".join(f"`{name}`" for name in auto))
            after_auto = result.get("after_title_auto") or []
            if after_auto:
                parts.append(", ".join(f"`{name}`" for name in after_auto))
            lines.append(f"- **`{result['path']}`**: {', '.join(parts)}")
        lines.append("")

    if snippet_modes:
        meta_snippet_modes = [r for r in snippet_modes if r.get("snippet")]
        if meta_snippet_modes:
            lines.extend(
                [
                    SECTION_COPY_PASTE_BLOCKS,
                    "",
                    "These files could not receive inline suggestions because the edits are "
                    "outside the pull request diff. Add this block at the **top of each "
                    "file** (or append the listed fields to an existing `.. meta::` block):",
                    "",
                ]
            )
            for result in meta_snippet_modes:
                lines.append(f"**`{result['path']}`**")
                lines.append("```rst")
                lines.append(str(result["snippet"]).rstrip())
                lines.append("```")
                lines.append("")

    if after_title_snippet_results:
        lines.extend(
            [
                SECTION_COPY_PASTE_AFTER_TITLE,
                "",
                "Add these directives **after the first document title** in each file:",
                "",
            ]
        )
        for result in after_title_snippet_results:
            lines.append(f"**`{result['path']}`**")
            for entry in result.get("after_title_snippets") or []:
                lines.append("```rst")
                lines.append(str(entry["snippet"]).rstrip())
                lines.append("```")
            lines.append("")

    manual_results = [r for r in results if r.get("manual_fields")]
    if manual_results:
        lines.extend(
            [
                SECTION_NON_EMPTY_VALUES,
                "",
                "These fields must have **non-empty** values in each file's "
                "`.. meta::` block:",
                "",
            ]
        )
        for result in manual_results:
            manual = list(result["manual_fields"])
            lines.append(
                f"- **`{result['path']}`**: {_field_list_markdown(manual, rules)}"
            )
        lines.append("")

    if after_title_manual_results:
        lines.extend(
            [
                SECTION_AFTER_TITLE_MANUAL,
                "",
                "These files need after-title directives that could not be added "
                "automatically (for example, no prose paragraph to wrap):",
                "",
            ]
        )
        for result in after_title_manual_results:
            manual = ", ".join(f"`{name}`" for name in result.get("after_title_manual") or [])
            lines.append(f"- **`{result['path']}`**: {manual}")
        lines.append("")

    if manual_fields_modes and not inline_modes and not snippet_modes and not after_title_snippet_results:
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

    rules = load_enhance_config(args.config)
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
                    rules=rules.meta,
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
        _emit_after_title_annotation(
            "warning",
            path,
            list(result.get("after_title_warning") or []),
            line,
        )
        _emit_after_title_annotation(
            "error",
            path,
            list(result.get("after_title_error") or []),
            line,
        )

    has_errors = any(
        result["error_fields"] or result.get("after_title_error")
        for result in results
    )

    if args.status_file is not None:
        _write_ci_status_file(
            args.status_file,
            meta_checked=checked_pull_request_rst,
            results=results,
            rules=rules.meta,
            has_errors=has_errors,
        )

    if args.status_file is not None and results:
        return 1
    if args.status_file is None and has_errors:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
