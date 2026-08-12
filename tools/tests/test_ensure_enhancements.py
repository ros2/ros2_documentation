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

from ensure_enhancements import (  # noqa: E402
    REVIEW_MARKER,
    SUMMARY_REVIEW_TITLE,
    AfterTitleRule,
    EnhanceConfig,
    MetaRule,
    _unresolved_fields,
    build_review_comment,
    changed_rst_paths,
    ensure_enhancements_in_file,
    load_enhance_config,
    main,
)

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
    after_title:
      short-description:
        severity: warning
        content: first_paragraph
      showmeta:
        severity: warning
        options:
          order: area, content-type, experience
        required_options:
          - order
    """
).strip()

AFTER_TITLE_RULES = {
    "short-description": AfterTitleRule(
        severity="warning",
        content="first_paragraph",
    ),
    "showmeta": AfterTitleRule(
        severity="warning",
        options={"order": "area, content-type, experience"},
        required_options=("order",),
    ),
}

META_ONLY_CONFIG = textwrap.dedent(
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


class TestEnhanceConfig(unittest.TestCase):
    def test_load_enhance_config_parses_meta_rules(self) -> None:
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as handle:
            handle.write(SAMPLE_CONFIG)
            path = Path(handle.name)
        try:
            config = load_enhance_config(path)
        finally:
            path.unlink()
        rules = config.meta
        self.assertEqual(rules["product"].severity, "warning")
        self.assertEqual(rules["product"].value, "{PRODUCT}")
        self.assertTrue(rules["product"].has_configured_value)
        self.assertFalse(rules["area"].has_configured_value)
        self.assertEqual(rules["area"].severity, "error")

    def test_load_enhance_config_parses_after_title(self) -> None:
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as handle:
            handle.write(SAMPLE_CONFIG)
            path = Path(handle.name)
        try:
            config = load_enhance_config(path)
        finally:
            path.unlink()
        self.assertEqual(len(config.after_title), 2)
        self.assertEqual(list(config.after_title.keys()), ["short-description", "showmeta"])
        self.assertEqual(config.after_title["short-description"].content, "first_paragraph")
        self.assertEqual(
            config.after_title["showmeta"].options,
            {"order": "area, content-type, experience"},
        )


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


class TestEnsureEnhancementsInFile(unittest.TestCase):
    def test_reports_missing_meta_fields(self) -> None:
        config = EnhanceConfig(
            meta={
                "product": MetaRule("warning", "{PRODUCT}"),
                "area": MetaRule("error", ""),
            },
            after_title={},
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "page.rst"
            path.write_text("Title\n=====\n", encoding="utf-8")
            result = ensure_enhancements_in_file(path, config)
            self.assertIsNotNone(result)
            assert result is not None
            self.assertIn("product", result["meta_optional"])
            self.assertIn("area", result["meta_required"])
            self.assertEqual(result["after_title_optional"], [])
            self.assertEqual(result["after_title_required"], [])

    def test_no_result_when_all_fields_present(self) -> None:
        config = EnhanceConfig(
            meta={"product": MetaRule("warning", "{PRODUCT}")},
            after_title={},
        )
        content = textwrap.dedent(
            """
            .. meta::
               :product: ROS 2

            Title
            =====
            """
        ).lstrip()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "page.rst"
            path.write_text(content, encoding="utf-8")
            self.assertIsNone(ensure_enhancements_in_file(path, config))

    def test_does_not_modify_files(self) -> None:
        config = EnhanceConfig(
            meta={"product": MetaRule("warning", "{PRODUCT}")},
            after_title={},
        )
        original = "Title\n=====\n"
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "page.rst"
            path.write_text(original, encoding="utf-8")
            ensure_enhancements_in_file(path, config)
            self.assertEqual(path.read_text(encoding="utf-8"), original)


class TestAfterTitleEnhancements(unittest.TestCase):
    def test_reports_missing_after_title_directives(self) -> None:
        config = EnhanceConfig(meta={}, after_title=AFTER_TITLE_RULES)
        content = textwrap.dedent(
            """
            Title
            =====

            Opening paragraph for the page.
            """
        ).lstrip()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "page.rst"
            path.write_text(content, encoding="utf-8")
            result = ensure_enhancements_in_file(path, config)
            self.assertIsNotNone(result)
            assert result is not None
            self.assertIn("short-description", result["after_title_optional"])
            self.assertIn("showmeta", result["after_title_optional"])


class TestReviewAndExit(unittest.TestCase):
    def test_build_review_comment_lists_missing_items(self) -> None:
        config = EnhanceConfig(
            meta={
                "product": MetaRule("warning", "{PRODUCT}"),
                "area": MetaRule("error", ""),
                "experience": MetaRule("warning", ""),
            },
            after_title=AFTER_TITLE_RULES,
        )
        results = [
            {
                "path": "source/Page.rst",
                "meta_required": ["area"],
                "meta_optional": ["product", "experience"],
                "after_title_required": [],
                "after_title_optional": ["short-description", "showmeta"],
            },
        ]
        body = build_review_comment(results, config=config)
        self.assertIn(SUMMARY_REVIEW_TITLE, body)
        self.assertIn("source/Page.rst", body)
        self.assertIn("Missing `.. meta::` fields", body)
        self.assertIn("Missing after-title directives", body)
        self.assertIn("required", body)
        self.assertIn("{PRODUCT}", body)
        self.assertIn(":order: area, content-type, experience", body)
        self.assertIn(REVIEW_MARKER, body)

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


def _extract_multiline_output(status: str, key: str) -> str | None:
    """Return the body of a GitHub Actions heredoc output block, or None if absent."""
    delimiter = f"EOF_{key.upper()}"
    header = f"{key}<<{delimiter}\n"
    start = status.find(header)
    if start < 0:
        return None
    start += len(header)
    end = status.find(f"\n{delimiter}\n", start)
    if end < 0:
        return None
    return status[start:end]


class TestCiStatusOutputs(unittest.TestCase):
    """The workflow gates steps on these outputs, so keep them self-consistent."""

    def _run_with_status_file(self, content: str, *, config: str = META_ONLY_CONFIG) -> str:
        rules_path = Path(tempfile.mkdtemp()) / "meta.yaml"
        rules_path.write_text(config, encoding="utf-8")
        with tempfile.TemporaryDirectory() as tmp:
            page = Path(tmp) / "page.rst"
            page.write_text(content, encoding="utf-8")
            status_path = Path(tmp) / "status.txt"
            main(
                [
                    str(page),
                    "--config",
                    str(rules_path),
                    "--status-file",
                    str(status_path),
                ],
            )
            return status_path.read_text(encoding="utf-8")

    def test_status_file_writes_comment_when_issues_remain(self) -> None:
        status = self._run_with_status_file("Title\n=====\n")
        self.assertIn("has_results=true", status)
        self.assertIn("has_errors=true", status)
        comment = _extract_multiline_output(status, "comment")
        self.assertIsNotNone(comment)
        self.assertIn(SUMMARY_REVIEW_TITLE, comment or "")
        self.assertIn(REVIEW_MARKER, comment or "")

    def test_clean_file_writes_no_review_body(self) -> None:
        content = textwrap.dedent(
            """
            .. meta::
               :product: ROS 2
               :area: docs
               :experience: beginner

            Title
            =====

            .. short-description::
               Summary for the page.

            .. showmeta::
               :order: area, content-type, experience
            """
        ).lstrip()
        status = self._run_with_status_file(content)
        self.assertIn("has_results=false", status)
        self.assertIn("has_errors=false", status)
        self.assertNotIn("comment<<", status)


class TestChangedRstPaths(unittest.TestCase):
    def test_parses_git_diff_output(self) -> None:
        completed = mock.Mock(returncode=0, stdout="source/A.rst\nsource/B.rst\n", stderr="")
        with mock.patch("ensure_enhancements.subprocess.run", return_value=completed) as run:
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

    def test_empty_discovery_writes_enhancements_checked_false(self) -> None:
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as handle:
            handle.write(SAMPLE_CONFIG)
            config_path = Path(handle.name)
        try:
            with tempfile.NamedTemporaryFile("w", delete=False) as status_handle:
                status_path = Path(status_handle.name)
            try:
                with mock.patch(
                    "ensure_enhancements.changed_rst_paths",
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
                self.assertIn("enhancements_checked=false", status_text)
                self.assertIn("has_errors=false", status_text)
            finally:
                status_path.unlink()
        finally:
            config_path.unlink()


if __name__ == "__main__":
    unittest.main()
