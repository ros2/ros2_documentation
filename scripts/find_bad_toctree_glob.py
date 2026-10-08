#!/usr/bin/env python3
"""Find Sphinx toctree directives that use :glob: without wildcards.

:glob: should only be present when TOC entries contain wildcards
(*, ?, or [...]). See:
https://www.sphinx-doc.org/en/master/usage/restructuredtext/directives.html#directive-option-toctree-glob
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

TOCTREE_RE = re.compile(
    r"(?m)^\.\. toctree::\s*\n"
    r"(?P<header>(?:[ \t]+:[^\n]+\n)*)"
    # Blank line(s) often separate options from entries; keep reading
    # indented lines (entries or blanks) until a non-indented line.
    r"(?P<body>(?:[ \t]*\n|[ \t]+[^\n]*\n)*)",
)
WILDCARD_RE = re.compile(r"[*?[]")


def find_toctrees(root: Path) -> tuple[list[dict], list[dict]]:
    bad: list[dict] = []
    good: list[dict] = []

    for path in sorted(root.rglob("*.rst")):
        text = path.read_text(encoding="utf-8")
        for match in TOCTREE_RE.finditer(text):
            header = match.group("header")
            body = match.group("body")
            if ":glob:" not in header:
                continue

            entries: list[str] = []
            for line in body.splitlines():
                stripped = line.strip()
                if not stripped or stripped.startswith(":") or stripped.startswith(".."):
                    continue
                entries.append(stripped)

            line_no = text.count("\n", 0, match.start()) + 1
            rel = path.relative_to(root).as_posix()
            info = {"file": rel, "line": line_no, "entries": entries}
            if any(WILDCARD_RE.search(entry) for entry in entries):
                good.append(info)
            else:
                bad.append(info)

    return bad, good


def print_report(items: list[dict], title: str) -> None:
    print(f"=== {title} ===")
    print(f"Count: {len(items)}\n")
    for item in items:
        print(f"{item['file']}:{item['line']}")
        if item["entries"]:
            for entry in item["entries"]:
                print(f"    {entry}")
        else:
            print("    (no entries)")
        print()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "source",
        help="Docs root to scan (default: <repo>/source)",
    )
    parser.add_argument(
        "--show-ok",
        action="store_true",
        help="Also list :glob: usages that correctly include wildcards",
    )
    args = parser.parse_args()

    root = args.root.resolve()
    if not root.is_dir():
        raise SystemExit(f"Docs root not found: {root}")

    bad, good = find_toctrees(root)
    print_report(bad, "WRONG: :glob: without wildcards in TOC entries")
    if args.show_ok:
        print_report(good, "OK: :glob: with wildcards")
    else:
        print(f"(OK reference count with wildcards: {len(good)}; pass --show-ok to list)")

    return 0 if not bad else 1


if __name__ == "__main__":
    raise SystemExit(main())
