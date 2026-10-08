#!/usr/bin/env python3
"""Find Sphinx local TOC (.. contents::) variants that don't match the standard.

Expected form:

.. contents:: Contents
   :depth: 2
   :local:
"""

from __future__ import annotations

import argparse
import re
from collections import Counter
from pathlib import Path

CONTENTS_RE = re.compile(
    r"(?m)^(?P<indent>[ \t]*)\.\. contents::(?P<title>[^\n]*)\n"
    r"(?P<body>(?:(?P=indent)[ \t]+[^\n]*\n)*)",
)

EXPECTED_TITLE = "Contents"
EXPECTED_DEPTH = "2"
EXPECTED_LOCAL = True


def parse_options(body: str) -> dict[str, str | bool]:
    options: dict[str, str | bool] = {}
    for line in body.splitlines():
        stripped = line.strip()
        if not stripped.startswith(":"):
            continue
        # :name: value  or  :flag:
        m = re.match(r":([^:]+):(.*)$", stripped)
        if not m:
            continue
        name = m.group(1).strip()
        value = m.group(2).strip()
        options[name] = value if value else True
    return options


def classify(title: str, options: dict[str, str | bool]) -> list[str]:
    reasons: list[str] = []
    title_norm = title.strip()
    if title_norm != EXPECTED_TITLE:
        reasons.append(f"title={title_norm!r} (expected {EXPECTED_TITLE!r})")

    depth = options.get("depth")
    if depth is None:
        reasons.append("missing :depth:")
    elif str(depth) != EXPECTED_DEPTH:
        reasons.append(f"depth={depth!r} (expected {EXPECTED_DEPTH!r})")

    if "local" not in options:
        reasons.append("missing :local:")
    elif options["local"] not in (True, ""):
        reasons.append(f"local={options['local']!r}")

    extra = sorted(k for k in options if k not in {"depth", "local"})
    if extra:
        reasons.append(f"extra options: {', '.join(extra)}")

    return reasons


def scan(root: Path) -> tuple[list[dict], list[dict]]:
    bad: list[dict] = []
    good: list[dict] = []

    for path in sorted(root.rglob("*.rst")):
        text = path.read_text(encoding="utf-8")
        for match in CONTENTS_RE.finditer(text):
            title = match.group("title")
            body = match.group("body")
            options = parse_options(body)
            line_no = text.count("\n", 0, match.start()) + 1
            rel = path.relative_to(root).as_posix()
            reasons = classify(title, options)
            info = {
                "file": rel,
                "line": line_no,
                "title": title.strip(),
                "options": options,
                "reasons": reasons,
                "snippet": match.group(0).rstrip(),
            }
            if reasons:
                bad.append(info)
            else:
                good.append(info)

    return bad, good


def print_bad(items: list[dict]) -> None:
    print("=== NON-STANDARD .. contents:: ===")
    print(f"Count: {len(items)}\n")

    by_reason = Counter()
    for item in items:
        for reason in item["reasons"]:
            # group title variants coarsely
            key = reason.split(" (expected")[0]
            by_reason[key] += 1

    print("--- Summary by reason ---")
    for reason, count in by_reason.most_common():
        print(f"  {count:4d}  {reason}")
    print()

    print("--- Per file ---")
    for item in items:
        opts = item["options"]
        opt_bits = []
        if "depth" in opts:
            opt_bits.append(f"depth={opts['depth']}")
        else:
            opt_bits.append("no-depth")
        if "local" in opts:
            opt_bits.append("local")
        else:
            opt_bits.append("no-local")
        extras = [k for k in opts if k not in {"depth", "local"}]
        if extras:
            opt_bits.append("extra=" + ",".join(extras))
        print(f"{item['file']}:{item['line']}")
        print(f"  title={item['title']!r}  [{', '.join(opt_bits)}]")
        for reason in item["reasons"]:
            print(f"  - {reason}")
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
        help="Also list matching standard contents directives",
    )
    args = parser.parse_args()

    root = args.root.resolve()
    if not root.is_dir():
        raise SystemExit(f"Docs root not found: {root}")

    bad, good = scan(root)
    print_bad(bad)
    if args.show_ok:
        print(f"=== STANDARD (OK) ===\nCount: {len(good)}\n")
        for item in good:
            print(f"{item['file']}:{item['line']}")
    else:
        print(f"(OK / standard count: {len(good)}; pass --show-ok to list)")

    return 0 if not bad else 1


if __name__ == "__main__":
    raise SystemExit(main())
