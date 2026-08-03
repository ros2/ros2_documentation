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
    SECTION_COPY_PASTE_AFTER_TITLE,
    SECTION_INLINE_SUGGESTIONS,
    SECTION_NON_EMPTY_VALUES,
    SUMMARY_REVIEW_TITLE,
    AfterTitleRule,
    EnhanceConfig,
    MetaRule,
    _unresolved_fields,
    build_review_comment,
    can_suggest_after_title_inline,
    can_suggest_inline,
    changed_rst_paths,
    ensure_enhancements_in_file,
    load_enhance_config,
    main,
    _annotation_line_for_after_title,
    _annotation_line_for_meta,
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

    def test_after_title_overlap_uses_paragraph_span(self) -> None:
        content = textwrap.dedent(
            """
            Title
            =====

            Opening paragraph.
            """
        ).lstrip()
        self.assertTrue(can_suggest_after_title_inline(content, {4}, paragraph_span=(4, 4)))
        self.assertFalse(can_suggest_after_title_inline(content, {8}, paragraph_span=(4, 4)))


class TestAnnotationLines(unittest.TestCase):
    def test_after_title_line_uses_paragraph_when_meta_is_at_top(self) -> None:
        content = textwrap.dedent(
            """
            .. meta::
               :product: x

            Title
            =====

            Opening paragraph beneath the title.

            More body.
            """
        ).lstrip()
        self.assertEqual(_annotation_line_for_meta(content), 1)
        self.assertEqual(_annotation_line_for_after_title(content), 7)

    def test_result_includes_separate_after_title_line(self) -> None:
        config = EnhanceConfig(meta={}, after_title=AFTER_TITLE_RULES)
        content = textwrap.dedent(
            """
            .. meta::
               :product: x

            Title
            =====

            Opening paragraph beneath the title.
            """
        ).lstrip()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "page.rst"
            path.write_text(content, encoding="utf-8")
            result = ensure_enhancements_in_file(path, config)
            self.assertIsNotNone(result)
            assert result is not None
            self.assertEqual(result["line"], 1)
            self.assertEqual(result["after_title_line"], 7)


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


class TestEnsureEnhancementsInFile(unittest.TestCase):
    def test_local_auto_inject_clears_configured_fields(self) -> None:
        rules = {
            "product": MetaRule("warning", "{PRODUCT}"),
            "area": MetaRule("error", ""),
        }
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "page.rst"
            path.write_text("Title\n=====\n", encoding="utf-8")
            result = ensure_enhancements_in_file(path, rules)
            self.assertIsNotNone(result)
            self.assertEqual(result["mode"], "suggestable")
            self.assertIn("area", result["manual_fields"])
            self.assertIn("area", result["error_fields"])
            fields = get_meta_fields_from_content(path.read_text(encoding="utf-8"))
            self.assertEqual(fields["product"], "{PRODUCT}")

    def test_auto_injected_fields_are_still_annotated(self) -> None:
        rules = {
            "product": MetaRule("warning", "{PRODUCT}"),
            "area": MetaRule("error", ""),
        }
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "page.rst"
            path.write_text("Title\n=====\n", encoding="utf-8")
            result = ensure_enhancements_in_file(path, rules)
            self.assertIsNotNone(result)
            self.assertIn("product", result["warning_fields"])
            self.assertNotIn("product", result["manual_fields"])

    def test_result_returned_when_only_configured_fields_missing(self) -> None:
        rules = {"product": MetaRule("warning", "{PRODUCT}")}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "page.rst"
            path.write_text("Title\n=====\n", encoding="utf-8")
            result = ensure_enhancements_in_file(path, rules)
            self.assertIsNotNone(result)
            self.assertEqual(result["mode"], "suggestable")
            self.assertEqual(result["warning_fields"], ["product"])
            self.assertEqual(result["manual_fields"], [])

    def test_no_result_when_all_fields_present(self) -> None:
        rules = {"product": MetaRule("warning", "{PRODUCT}")}
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
            self.assertIsNone(ensure_enhancements_in_file(path, rules))


class TestAfterTitleEnhancements(unittest.TestCase):
    def test_inserts_short_description_and_showmeta(self) -> None:
        config = EnhanceConfig(
            meta={"product": MetaRule("warning", "{PRODUCT}")},
            after_title=AFTER_TITLE_RULES,
        )
        content = textwrap.dedent(
            """
            Title
            =====

            Opening paragraph for the page.

            More content.
            """
        ).lstrip()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "page.rst"
            path.write_text(content, encoding="utf-8")
            result = ensure_enhancements_in_file(path, config)
            self.assertIsNotNone(result)
            updated = path.read_text(encoding="utf-8")
            self.assertIn(".. short-description::", updated)
            self.assertIn(".. showmeta::", updated)
            self.assertIn(":order: area, content-type, experience", updated)
            self.assertIn("Opening paragraph for the page.", updated)
            self.assertIn("More content.", updated)

    def test_toctree_before_paragraph_wraps_correct_paragraph(self) -> None:
        config = EnhanceConfig(meta={}, after_title=AFTER_TITLE_RULES)
        content = textwrap.dedent(
            """
            Title
            =====

            .. toctree::
               Page

            First paragraph here.

            Second paragraph.
            """
        ).lstrip()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "page.rst"
            path.write_text(content, encoding="utf-8")
            result = ensure_enhancements_in_file(path, config)
            self.assertIsNotNone(result)
            updated = path.read_text(encoding="utf-8")
            self.assertIn("First paragraph here.", updated)
            self.assertIn(".. showmeta::", updated)
            toctree_pos = updated.index(".. toctree::")
            showmeta_pos = updated.index(".. showmeta::")
            self.assertLess(showmeta_pos, toctree_pos)

    def test_build_review_comment_includes_after_title_snippets(self) -> None:
        rules = {"area": MetaRule("error", "")}
        results = [
            {
                "path": "source/Page.rst",
                "mode": "snippet",
                "snippet": "",
                "auto_fields": [],
                "manual_fields": [],
                "warning_fields": ["short-description"],
                "error_fields": [],
                "line": 1,
                "after_title_auto": [],
                "after_title_manual": [],
                "after_title_warning": ["short-description"],
                "after_title_error": [],
                "after_title_snippets": [
                    {
                        "directive": "showmeta",
                        "snippet": ".. showmeta::\n   :order: area\n",
                    },
                ],
            },
        ]
        body = build_review_comment(results, rules)
        self.assertIn(SECTION_COPY_PASTE_AFTER_TITLE, body)
        self.assertIn(".. showmeta::", body)


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
                "auto_fields": [],
                "manual_fields": ["area", "experience"],
                "warning_fields": ["experience"],
                "error_fields": ["area"],
                "line": 1,
            },
        ]
        body = build_review_comment(results, rules)
        self.assertIn(SUMMARY_REVIEW_TITLE, body)
        self.assertIn(SECTION_NON_EMPTY_VALUES, body)
        self.assertIn("area", body)
        self.assertIn("required", body)
        self.assertIn("experience", body)

    def test_build_review_comment_lists_inline_suggestion_fields(self) -> None:
        rules = {
            "product": MetaRule("warning", "{PRODUCT}"),
            "experience": MetaRule("warning", ""),
        }
        results = [
            {
                "path": "source/Page.rst",
                "mode": "suggestable",
                "snippet": "",
                "auto_fields": ["product"],
                "manual_fields": ["experience"],
                "warning_fields": ["product", "experience"],
                "error_fields": [],
                "line": 1,
            },
        ]
        body = build_review_comment(results, rules)
        self.assertIn(SUMMARY_REVIEW_TITLE, body)
        self.assertIn(SECTION_INLINE_SUGGESTIONS, body)
        self.assertIn(SECTION_NON_EMPTY_VALUES, body)
        self.assertIn("- **`source/Page.rst`**: `product`", body)
        self.assertIn("`experience` (optional)", body)

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

    def test_suggestion_note_written_with_inline_suggestions(self) -> None:
        status = self._run_with_status_file("Title\n=====\n")
        self.assertIn("inline_suggestions=true", status)
        self.assertIn("suggestion_note<<", status)
        suggestion_note = _extract_multiline_output(status, "suggestion_note")
        comment = _extract_multiline_output(status, "comment")
        self.assertIsNotNone(suggestion_note)
        self.assertIsNotNone(comment)
        self.assertIn("## Inline documentation suggestions", suggestion_note or "")
        self.assertNotIn(REVIEW_MARKER, suggestion_note or "")
        self.assertIn(SUMMARY_REVIEW_TITLE, comment or "")
        self.assertIn(REVIEW_MARKER, comment or "")

    def test_no_suggestion_note_without_inline_suggestions(self) -> None:
        content = textwrap.dedent(
            """
            .. meta::
               :product: ROS 2

            Title
            =====
            """
        ).lstrip()
        status = self._run_with_status_file(content)
        self.assertIn("inline_suggestions=false", status)
        self.assertNotIn("suggestion_note<<", status)
        self.assertIn("comment<<", status)

    def test_clean_file_writes_no_review_bodies(self) -> None:
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
        self.assertNotIn("suggestion_note<<", status)


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
