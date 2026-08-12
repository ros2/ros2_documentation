#!/usr/bin/env python3
"""
Ensure configured documentation enhancements exist in RST source files.

Rules are defined in ``enhance.yaml``: ``meta`` and ``after_title`` mappings
for ``.. meta::`` fields and post-heading directives such as
``.. short-description::`` and ``.. showmeta::``.

In CI (``--status-file``), writes GitHub Actions outputs and exits with code
``1`` when issues remain so the ensure step can soft-fail. Error-severity
issues also set ``has_errors`` for a final workflow gate.
"""

from __future__ import annotations

import argparse
import logging
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, TextIO

import yaml

# Allow ``python3 tools/ensure_enhancements.py`` from the repository root.
_TOOLS_DIR = Path(__file__).resolve().parent
if str(_TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(_TOOLS_DIR))

from rst_utils import (  # noqa: E402
    get_meta_fields_from_content,
    has_short_description_content,
    has_showmeta_with_order,
)

logger = logging.getLogger(__name__)

DEFAULT_CONFIG_PATH = _TOOLS_DIR / "enhance.yaml"
RST_EXTENSION = ".rst"
Severity = Literal["warning", "error"]

# Hidden marker in review bodies so CI can find and supersede prior bot reviews.
REVIEW_MARKER_ID = "ros2-doc-enhance-ensure"
REVIEW_MARKER = f"<!-- {REVIEW_MARKER_ID} -->"

# Title for pull request review bodies (GitHub Markdown).
SUMMARY_REVIEW_TITLE = "## Documentation enhancements"


@dataclass(frozen=True)
class AfterTitleRule:
    """
    A single after-title directive rule from ``enhance.yaml``.

    The directive name is the key in the ``after_title`` mapping (like ``meta``).

    Attributes:
        severity: Advisory ``warning`` or blocking ``error`` in CI.
        content: Content source for body directives (e.g. ``first_paragraph``).
        options: Option name/value pairs for option-only directives.
        required_options: Option names that must be non-empty when present.
    """

    severity: Severity
    content: str | None = None
    options: dict[str, str] | None = None
    required_options: tuple[str, ...] = ()


@dataclass(frozen=True)
class EnhanceConfig:
    """Full enhancement configuration loaded from ``enhance.yaml``."""

    meta: dict[str, MetaRule]
    after_title: dict[str, AfterTitleRule]


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
    raw: object,
    config_path: Path,
) -> dict[str, AfterTitleRule]:
    """Validate and parse the ``after_title`` mapping from config YAML."""
    if raw is None:
        return {}
    if not isinstance(raw, dict):
        logger.error(
            "Config %s: 'after_title' must be a mapping keyed by directive name",
            config_path,
        )
        raise SystemExit(1)

    supported_directives = {"short-description", "showmeta"}
    validated: dict[str, AfterTitleRule] = {}
    for directive, entry in raw.items():
        if not isinstance(directive, str) or not directive.strip():
            logger.error(
                "Config %s: after_title keys must be non-empty directive names",
                config_path,
            )
            raise SystemExit(1)
        if directive not in supported_directives:
            logger.error(
                "Config %s: after_title directive %r is not supported",
                config_path,
                directive,
            )
            raise SystemExit(1)
        if not isinstance(entry, dict):
            logger.error(
                "Config %s: after_title entry for %r must be a mapping",
                config_path,
                directive,
            )
            raise SystemExit(1)
        severity = entry.get("severity")
        if severity not in ("warning", "error"):
            logger.error(
                "Config %s: after_title entry %r severity must be 'warning' or 'error', got %r",
                config_path,
                directive,
                severity,
            )
            raise SystemExit(1)

        content = entry.get("content")
        if content is not None and not isinstance(content, str):
            logger.error(
                "Config %s: after_title entry %r content must be a string",
                config_path,
                directive,
            )
            raise SystemExit(1)

        raw_options = entry.get("options")
        options: dict[str, str] | None = None
        if raw_options is not None:
            if not isinstance(raw_options, dict):
                logger.error(
                    "Config %s: after_title entry %r options must be a mapping",
                    config_path,
                    directive,
                )
                raise SystemExit(1)
            options = {}
            for opt_key, opt_value in raw_options.items():
                if not isinstance(opt_key, str) or not isinstance(opt_value, str):
                    logger.error(
                        "Config %s: after_title entry %r option keys and values must be strings",
                        config_path,
                        directive,
                    )
                    raise SystemExit(1)
                options[opt_key] = opt_value

        raw_required = entry.get("required_options")
        required_options: tuple[str, ...] = ()
        if raw_required is not None:
            if not isinstance(raw_required, list):
                logger.error(
                    "Config %s: after_title entry %r required_options must be a list",
                    config_path,
                    directive,
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

        validated[directive] = AfterTitleRule(
            severity=severity,
            content=content,
            options=options,
            required_options=required_options,
        )
    return validated


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
    enhancements_checked: bool,
    results: list[dict[str, object]],
    config: EnhanceConfig,
    has_errors: bool,
) -> None:
    """
    Append GitHub Actions output flags and optional review comment.

    Writes ``enhancements_checked``, ``has_results``, and ``has_errors``, plus
    a multiline ``comment`` block when ``results`` is non-empty.

    Args:
        status_file: Path to append to (for example ``$GITHUB_OUTPUT``).
        enhancements_checked: Whether changed RST files were in scope for this run.
        results: Per-file result dicts from ``ensure_enhancements_in_file``.
        config: Enhancement configuration used to build the review comment.
        has_errors: Whether any result has unresolved error-severity fields.

    Returns:
        None.
    """
    has_results = bool(results)
    with status_file.open("a", encoding="utf-8") as f:
        for key, flag in (
            ("enhancements_checked", enhancements_checked),
            ("has_results", has_results),
            ("has_errors", has_errors),
        ):
            f.write(f"{key}={'true' if flag else 'false'}\n")
        if results:
            _write_multiline_output(
                f,
                "comment",
                build_review_comment(results, config=config),
            )


def _after_title_rule_satisfied(content: str, directive: str) -> bool:
    """Return whether an after-title rule is already satisfied in ``content``."""
    if directive == "short-description":
        return has_short_description_content(content)
    if directive == "showmeta":
        return has_showmeta_with_order(content)
    return True


def _severity_fields(
    field_names: list[str],
    rules: dict[str, MetaRule] | dict[str, AfterTitleRule],
    severity: Severity,
) -> list[str]:
    """
    Return field or directive names that use the given severity in ``rules``.

    Args:
        field_names: Candidate meta field or directive names.
        rules: Configured metadata or after-title rules keyed by name.
        severity: Severity label to match.

    Returns:
        Names from ``field_names`` whose rule uses ``severity``.
    """
    return [name for name in field_names if rules[name].severity == severity]


def ensure_enhancements_in_file(
    path: Path,
    config: EnhanceConfig,
) -> dict[str, object] | None:
    """
    Check one RST file for missing documentation enhancements.

    Args:
        path: RST file to inspect.
        config: Enhancement configuration.

    Returns:
        A result dict when issues were found, otherwise ``None``.

    Raises:
        OSError: If the RST file cannot be read.
        UnicodeError: If the RST file cannot be decoded as UTF-8.
    """
    content = path.read_text(encoding="utf-8")
    path_str = str(path).replace("\\", "/")

    unresolved = _unresolved_fields(content, config.meta)
    after_title_unresolved = [
        directive
        for directive in config.after_title
        if not _after_title_rule_satisfied(content, directive)
    ]

    if not unresolved and not after_title_unresolved:
        logger.info("%s: all configured enhancements present", path)
        return None

    return {
        "path": path_str,
        "meta_required": _severity_fields(unresolved, config.meta, "error"),
        "meta_optional": _severity_fields(unresolved, config.meta, "warning"),
        "after_title_required": _severity_fields(
            after_title_unresolved,
            config.after_title,
            "error",
        ),
        "after_title_optional": _severity_fields(
            after_title_unresolved,
            config.after_title,
            "warning",
        ),
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


def _meta_field_hint(name: str, rule: MetaRule) -> str:
    """Format one missing meta field for the review comment."""
    severity = "required" if rule.severity == "error" else "optional"
    if rule.has_configured_value:
        return f"`{name}` ({severity}, suggested value `{rule.value}`)"
    return f"`{name}` ({severity})"


def _after_title_hint(name: str, rule: AfterTitleRule) -> str:
    """Format one missing after-title directive for the review comment."""
    severity = "required" if rule.severity == "error" else "optional"
    if name == "short-description":
        return f"`{name}` ({severity}, wrap the opening paragraph)"
    if name == "showmeta" and rule.options:
        order = rule.options.get("order", "")
        return f"`{name}` ({severity}, add with `:order: {order}`)"
    return f"`{name}` ({severity})"


def build_review_comment(
    results: list[dict[str, object]],
    *,
    config: EnhanceConfig,
) -> str:
    """
    Build a pull-request review body from enhancement check results.

    Args:
        results: Per-file result dictionaries from ``ensure_enhancements_in_file``.
        config: Enhancement configuration used to format hints.

    Returns:
        A stamped Markdown review body listing missing items per file.
    """
    lines = [
        SUMMARY_REVIEW_TITLE,
        "",
        "This pull request is missing configured documentation enhancements "
        "(see `tools/enhance.yaml`).",
        "",
    ]

    for result in results:
        path = str(result["path"])
        lines.append(f"### `{path}`")

        meta_required = list(result.get("meta_required") or [])
        meta_optional = list(result.get("meta_optional") or [])
        if meta_required or meta_optional:
            hints = [
                _meta_field_hint(name, config.meta[name])
                for name in meta_required + meta_optional
            ]
            lines.append(f"- Missing `.. meta::` fields: {', '.join(hints)}")

        after_title_required = list(result.get("after_title_required") or [])
        after_title_optional = list(result.get("after_title_optional") or [])
        if after_title_required or after_title_optional:
            hints = [
                _after_title_hint(name, config.after_title[name])
                for name in after_title_required + after_title_optional
            ]
            lines.append(f"- Missing after-title directives: {', '.join(hints)}")

        lines.append("")

    return "\n".join(lines).rstrip() + f"\n\n{REVIEW_MARKER}\n"


def main(argv: list[str] | None = None) -> int:
    """
    Run the command-line documentation enhancement check.

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
        SystemExit: If command-line arguments or enhancement configuration are invalid.
    """
    parser = argparse.ArgumentParser(
        description=(
            "Ensure configured documentation enhancements exist in RST files "
            "using rules from a YAML config file."
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
            "Git commit SHA for the pull request base. When set, changed "
            "`.rst` files are discovered automatically."
        ),
    )
    parser.add_argument(
        "--status-file",
        type=Path,
        help=(
            "Write enhancements_checked, has_results, has_errors, and the "
            "review comment body for CI"
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

    config = load_enhance_config(args.config)
    checked_pull_request_rst = False

    if not args.paths:
        assert args.diff_base is not None
        discovered = changed_rst_paths(args.diff_base)
        if not discovered:
            logger.info("No changed RST files in this pull request.")
            if args.status_file is not None:
                _write_ci_status_file(
                    args.status_file,
                    enhancements_checked=False,
                    results=[],
                    config=config,
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
            result = ensure_enhancements_in_file(path, config)
            if result is not None:
                results.append(result)

        logger.info(
            "Processed %d file(s): %d with missing enhancements",
            len(rst_paths),
            len(results),
        )

    has_errors = any(
        result["meta_required"] or result.get("after_title_required")
        for result in results
    )

    if args.status_file is not None:
        _write_ci_status_file(
            args.status_file,
            enhancements_checked=checked_pull_request_rst,
            results=results,
            config=config,
            has_errors=has_errors,
        )

    if args.status_file is not None and results:
        return 1
    if args.status_file is None and has_errors:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
