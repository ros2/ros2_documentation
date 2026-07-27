# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import sys
import tempfile
import textwrap
import unittest
from pathlib import Path
from unittest import mock

_TOOLS_DIR = Path(__file__).resolve().parent.parent
if str(_TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(_TOOLS_DIR))

from ensure_meta_tags import (  # noqa: E402
    MetaRule,
    _unresolved_fields,
    build_review_comment,
    can_suggest_inline,
    changed_rst_paths,
    ensure_meta_tags_in_file,
    load_meta_config,
    main,
)
from rst_utils import get_meta_fields_from_content, inject_metadata_to_content  # noqa: E402

SAMPLE_CONFIG = textwrap.dedent(
    """
    meta:
      product:
        severity: warning
        value: "{PRODUCT}"
      area:
        severity: error
        value:
      experience:
        severity: warning
        value:
    """
).strip()


class TestMetaConfig(unittest.TestCase):
    def test_load_meta_config_parses_rules(self) -> None:
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as handle:
            handle.write(SAMPLE_CONFIG)
            path = Path(handle.name)
        try:
            rules = load_meta_config(path)
        finally:
            path.unlink()
        self.assertEqual(rules["product"].severity, "warning")
        self.assertEqual(rules["product"].value, "{PRODUCT}")
        self.assertTrue(rules["product"].has_configured_value)
        self.assertFalse(rules["area"].has_configured_value)
        self.assertEqual(rules["area"].severity, "error")


class TestCanSuggestInline(unittest.TestCase):
    def test_meta_block_overlap_uses_inclusive_span_only(self) -> None:
        content = textwrap.dedent(
            """
            .. meta::
               :product: x

            Title
            =====
            """
        ).lstrip()
        # Meta block is lines 1-2; line 3 is blank after the block.
        self.assertFalse(can_suggest_inline(content, {3}))
        self.assertTrue(can_suggest_inline(content, {2}))


class TestUnresolvedFields(unittest.TestCase):
    def test_missing_and_blank_count_as_unresolved(self) -> None:
        rules = {
            "product": MetaRule("warning", "{PRODUCT}"),
            "area": MetaRule("error", ""),
        }
        content = textwrap.dedent(
            """
            .. meta::
               :area:

            Title
            =====
            """
        )
        self.assertEqual(_unresolved_fields(content, rules), ["product", "area"])

    def test_inject_fills_blank_configured_value(self) -> None:
        content = textwrap.dedent(
            """
            .. meta::
               :product:

            Title
            =====
            """
        )
        updated, changed = inject_metadata_to_content(content, {"product": "{PRODUCT}"})
        self.assertTrue(changed)
        fields = get_meta_fields_from_content(updated)
        self.assertEqual(fields["product"], "{PRODUCT}")


class TestEnsureMetaTagsInFile(unittest.TestCase):
    def test_local_auto_inject_clears_configured_fields(self) -> None:
        rules = {
            "product": MetaRule("warning", "{PRODUCT}"),
            "area": MetaRule("error", ""),
        }
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "page.rst"
            path.write_text("Title\n=====\n", encoding="utf-8")
            result = ensure_meta_tags_in_file(path, rules)
            self.assertIsNotNone(result)
            self.assertEqual(result["mode"], "suggestable")
            self.assertIn("area", result["manual_fields"])
            self.assertIn("area", result["error_fields"])
            fields = get_meta_fields_from_content(path.read_text(encoding="utf-8"))
            self.assertEqual(fields["product"], "{PRODUCT}")


class TestReviewAndExit(unittest.TestCase):
    def test_build_review_comment_lists_manual_fields(self) -> None:
        rules = {
            "area": MetaRule("error", ""),
            "experience": MetaRule("warning", ""),
        }
        results = [
            {
                "path": "source/Page.rst",
                "mode": "manual_fields",
                "snippet": "",
                "manual_fields": ["area", "experience"],
                "warning_fields": ["experience"],
                "error_fields": ["area"],
                "line": 1,
            },
        ]
        body = build_review_comment(results, rules)
        self.assertIn("area", body)
        self.assertIn("required", body)
        self.assertIn("experience", body)

    def test_local_exit_nonzero_only_for_error_severity(self) -> None:
        warning_config = textwrap.dedent(
            """
            meta:
              product:
                severity: warning
                value: "{PRODUCT}"
            """
        ).strip()
        rules_path = Path(tempfile.mkdtemp()) / "meta.yaml"
        rules_path.write_text(warning_config, encoding="utf-8")
        with tempfile.TemporaryDirectory() as tmp:
            warning_only = Path(tmp) / "warn.rst"
            warning_only.write_text("Title\n=====\n", encoding="utf-8")
            code = main(
                [
                    str(warning_only),
                    "--config",
                    str(rules_path),
                ],
            )
            self.assertEqual(code, 0)

        rules_path.write_text(SAMPLE_CONFIG, encoding="utf-8")
        with tempfile.TemporaryDirectory() as tmp:
            error_file = Path(tmp) / "err.rst"
            error_file.write_text("Title\n=====\n", encoding="utf-8")
            code = main(
                [
                    str(error_file),
                    "--config",
                    str(rules_path),
                ],
            )
            self.assertEqual(code, 1)


class TestChangedRstPaths(unittest.TestCase):
    def test_parses_git_diff_output(self) -> None:
        completed = mock.Mock(returncode=0, stdout="source/A.rst\nsource/B.rst\n", stderr="")
        with mock.patch("ensure_meta_tags.subprocess.run", return_value=completed) as run:
            paths = changed_rst_paths("abc123")
        self.assertEqual(paths, [Path("source/A.rst"), Path("source/B.rst")])
        run.assert_called_once()
        call_args = run.call_args[0][0]
        self.assertEqual(call_args[:4], ["git", "diff", "--name-only", "--diff-filter=ACMR"])
        self.assertIn("abc123...HEAD", call_args[4])


class TestMainDiscovery(unittest.TestCase):
    def test_requires_paths_or_diff_base(self) -> None:
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as handle:
            handle.write(SAMPLE_CONFIG)
            config_path = Path(handle.name)
        try:
            with self.assertRaises(SystemExit) as ctx:
                main(["--config", str(config_path)])
            self.assertEqual(ctx.exception.code, 2)
        finally:
            config_path.unlink()

    def test_empty_discovery_writes_meta_checked_false(self) -> None:
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as handle:
            handle.write(SAMPLE_CONFIG)
            config_path = Path(handle.name)
        try:
            with tempfile.NamedTemporaryFile("w", delete=False) as status_handle:
                status_path = Path(status_handle.name)
            try:
                with mock.patch(
                    "ensure_meta_tags.changed_rst_paths",
                    return_value=[],
                ):
                    code = main(
                        [
                            "--config",
                            str(config_path),
                            "--diff-base",
                            "base-sha",
                            "--status-file",
                            str(status_path),
                        ],
                    )
                self.assertEqual(code, 0)
                status_text = status_path.read_text(encoding="utf-8")
                self.assertIn("meta_checked=false", status_text)
                self.assertIn("has_errors=false", status_text)
            finally:
                status_path.unlink()
        finally:
            config_path.unlink()


if __name__ == "__main__":
    unittest.main()
