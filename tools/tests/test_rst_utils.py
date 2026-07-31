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
import textwrap
import unittest
from pathlib import Path

_TOOLS_DIR = Path(__file__).resolve().parent.parent
if str(_TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(_TOOLS_DIR))

from rst_utils import (  # noqa: E402
    extract_first_paragraph_after_title,
    format_showmeta_block,
    has_short_description_content,
    has_showmeta_with_order,
    inject_showmeta_to_content,
    wrap_first_paragraph_as_short_description,
)


class TestExtractFirstParagraph(unittest.TestCase):
    def test_skips_directives_before_prose(self) -> None:
        content = textwrap.dedent(
            """
            Title
            =====

            .. toctree::
               :maxdepth: 1

               Page

            First paragraph here.

            Second paragraph.
            """
        ).lstrip()
        paragraph, span = extract_first_paragraph_after_title(content)
        self.assertEqual(paragraph, "First paragraph here.")
        self.assertEqual(span, (9, 9))

    def test_finds_paragraph_after_dash_title(self) -> None:
        content = textwrap.dedent(
            """
            Summary
            -------

            Opening prose for the page.
            """
        ).lstrip()
        paragraph, span = extract_first_paragraph_after_title(content)
        self.assertEqual(paragraph, "Opening prose for the page.")
        self.assertEqual(span, (4, 4))

    def test_returns_none_when_no_prose(self) -> None:
        content = textwrap.dedent(
            """
            Title
            =====

            .. toctree::
               Page
            """
        ).lstrip()
        paragraph, span = extract_first_paragraph_after_title(content)
        self.assertIsNone(paragraph)
        self.assertIsNone(span)


class TestWrapShortDescription(unittest.TestCase):
    def test_wraps_first_paragraph_after_equals_title(self) -> None:
        content = textwrap.dedent(
            """
            Title
            =====

            Opening paragraph for the article.

            More content.
            """
        ).lstrip()
        updated, changed = wrap_first_paragraph_as_short_description(content)
        self.assertTrue(changed)
        self.assertTrue(has_short_description_content(updated))
        self.assertIn(".. short-description::", updated)
        self.assertIn("Opening paragraph for the article.", updated)
        self.assertIn("More content.", updated)
        body_after_directive = updated.split(".. short-description::", 1)[1]
        self.assertNotIn("Opening paragraph for the article.", body_after_directive.split("More content.", 1)[1])

    def test_does_not_replace_existing_short_description(self) -> None:
        content = textwrap.dedent(
            """
            Title
            =====

            .. short-description::
               Existing summary.

            Body paragraph.
            """
        ).lstrip()
        updated, changed = wrap_first_paragraph_as_short_description(content)
        self.assertFalse(changed)
        self.assertEqual(updated, content)

    def test_fills_empty_short_description_from_first_paragraph(self) -> None:
        content = textwrap.dedent(
            """
            Title
            =====

            .. short-description::

            Body paragraph here.
            """
        ).lstrip()
        updated, changed = wrap_first_paragraph_as_short_description(content)
        self.assertTrue(changed)
        self.assertTrue(has_short_description_content(updated))
        self.assertEqual(updated.count("Body paragraph here."), 1)


class TestShowmetaHelpers(unittest.TestCase):
    def test_inject_showmeta_after_short_description(self) -> None:
        content = textwrap.dedent(
            """
            Title
            =====

            .. short-description::
               Summary text.

            Body content.
            """
        ).lstrip()
        updated, changed = inject_showmeta_to_content(
            content,
            {"order": "area, content-type, experience"},
        )
        self.assertTrue(changed)
        self.assertTrue(has_showmeta_with_order(updated))
        short_desc_pos = updated.index(".. short-description::")
        showmeta_pos = updated.index(".. showmeta::")
        body_pos = updated.index("Body content.")
        self.assertLess(short_desc_pos, showmeta_pos)
        self.assertLess(showmeta_pos, body_pos)

    def test_fills_missing_order_on_existing_showmeta(self) -> None:
        content = textwrap.dedent(
            """
            Title
            =====

            .. showmeta::
               :order:

            Body content.
            """
        ).lstrip()
        updated, changed = inject_showmeta_to_content(content, {"order": "area"})
        self.assertTrue(changed)
        self.assertIn(":order: area", updated)

    def test_does_not_overwrite_existing_order(self) -> None:
        content = textwrap.dedent(
            """
            Title
            =====

            .. showmeta::
               :order: area, experience

            Body content.
            """
        ).lstrip()
        updated, changed = inject_showmeta_to_content(
            content,
            {"order": "area, content-type, experience"},
        )
        self.assertFalse(changed)
        self.assertEqual(updated, content)

    def test_format_showmeta_block(self) -> None:
        block = format_showmeta_block({"order": "area, content-type, experience"})
        self.assertIn(".. showmeta::", block)
        self.assertIn(":order: area, content-type, experience", block)


if __name__ == "__main__":
    unittest.main()
