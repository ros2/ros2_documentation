"""
Load and validate documentation enhancement rules from ``enhance.yaml``.

Defines the configuration schema (``meta`` and ``after_title`` mappings) used by
``ensure_enhancements.py``.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import yaml

_TOOLS_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = _TOOLS_DIR / "enhance.yaml"

logger = logging.getLogger(__name__)

Severity = Literal["warning", "error"]


@dataclass(frozen=True)
class MetaRule:
    """
    A single metadata field rule from ``enhance.yaml``.

    Attributes:
        severity: Advisory ``warning`` or blocking ``error`` in CI.
        value: Suggested default text when missing or blank; empty when the
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
