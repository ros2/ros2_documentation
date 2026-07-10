#!/usr/bin/env python3
"""
Ensure configured metadata fields exist in RST ``.. meta::`` blocks.

Missing fields are injected with values from ``meta_tags.yaml``.
Existing fields are never overwritten.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import yaml

# Allow ``python3 tools/ensure_meta_tags.py`` from the repository root.
_TOOLS_DIR = Path(__file__).resolve().parent
if str(_TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(_TOOLS_DIR))

from rst_utils import get_meta_names_from_content, inject_metadata_to_content

logger = logging.getLogger(__name__)

DEFAULT_CONFIG_PATH = _TOOLS_DIR / "meta_tags.yaml"
RST_EXTENSION = ".rst"


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


def _missing_meta_fields(content: str, meta_config: dict[str, str]) -> list[str]:
    present = get_meta_names_from_content(content)
    return [field for field in meta_config if field not in present]


def ensure_meta_tags_in_file(path: Path, meta_config: dict[str, str]) -> bool:
    """
    Add missing configured meta fields to one RST file.

    Returns:
        True if the file was updated, False otherwise.
    """
    content = path.read_text(encoding="utf-8")
    missing = _missing_meta_fields(content, meta_config)
    if not missing:
        logger.info("%s: all configured meta fields present", path)
        return False

    metadata = {field: meta_config[field] for field in missing}
    new_content, changed = inject_metadata_to_content(content, metadata)
    if not changed:
        return False

    path.write_text(new_content, encoding="utf-8")
    logger.info("%s: added meta fields %s", path, ", ".join(missing))
    return True


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
    if not rst_paths:
        logger.info("No RST files to process")
        return 0

    updated = 0
    for path in rst_paths:
        if ensure_meta_tags_in_file(path, meta_config):
            updated += 1

    logger.info("Updated %d of %d file(s)", updated, len(rst_paths))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
