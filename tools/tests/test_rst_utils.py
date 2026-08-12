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
    get_meta_fields_from_content,
    has_short_description_content,
    has_showmeta_with_order,
)


class TestMetaFields(unittest.TestCase):
    def test_get_meta_fields_from_content(self) -> None:
        content = textwrap.dedent(
            """
            .. meta::
               :product: ROS 2
               :area: docs

            Title
            =====
            """
        ).lstrip()
        fields = get_meta_fields_from_content(content)
        self.assertEqual(fields["product"], "ROS 2")
        self.assertEqual(fields["area"], "docs")

    def test_returns_empty_when_no_meta_block(self) -> None:
        content = "Title\n=====\n"
        self.assertEqual(get_meta_fields_from_content(content), {})


class TestShortDescription(unittest.TestCase):
    def test_has_short_description_content(self) -> None:
        content = textwrap.dedent(
            """
            Title
            =====

            .. short-description::
               Existing summary.

            Body paragraph.
            """
        ).lstrip()
        self.assertTrue(has_short_description_content(content))

    def test_empty_short_description_is_not_present(self) -> None:
        content = textwrap.dedent(
            """
            Title
            =====

            .. short-description::

            Body paragraph.
            """
        ).lstrip()
        self.assertFalse(has_short_description_content(content))

    def test_missing_short_description(self) -> None:
        content = "Title\n=====\n\nBody paragraph.\n"
        self.assertFalse(has_short_description_content(content))


class TestShowmeta(unittest.TestCase):
    def test_has_showmeta_with_order(self) -> None:
        content = textwrap.dedent(
            """
            Title
            =====

            .. showmeta::
               :order: area, content-type, experience

            Body content.
            """
        ).lstrip()
        self.assertTrue(has_showmeta_with_order(content))

    def test_blank_order_is_not_present(self) -> None:
        content = textwrap.dedent(
            """
            Title
            =====

            .. showmeta::
               :order:

            Body content.
            """
        ).lstrip()
        self.assertFalse(has_showmeta_with_order(content))

    def test_missing_showmeta(self) -> None:
        content = "Title\n=====\n\nBody content.\n"
        self.assertFalse(has_showmeta_with_order(content))


if __name__ == "__main__":
    unittest.main()
