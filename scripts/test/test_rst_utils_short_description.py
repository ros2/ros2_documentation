"""Tests for ``.. short-description::`` helpers in ``rst_utils``."""

import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rst_utils import (
    get_short_description_body,
    has_short_description_content,
    inject_short_description_to_content,
)


def test_has_short_description_content_false_when_missing() -> None:
    src = "Title\n=====\n\nBody.\n"
    assert has_short_description_content(src) is False


def test_has_short_description_content_true_when_populated() -> None:
    src = """Title
=====

.. short-description::
   First sentence here.
   Second sentence here.

Next section
------------
"""
    assert has_short_description_content(src) is True


def test_get_short_description_body_normalises() -> None:
    src = """.. short-description::
   Line one continued
   same paragraph.
   New paragraph line.

Body.
"""
    body = get_short_description_body(src)
    assert body is not None
    assert "Line one" in body


def test_inject_fills_empty_directive() -> None:
    src = """Title
=====

.. short-description::

**Area:** x

"""
    new_src, changed = inject_short_description_to_content(src, "One paragraph.\n\nTwo paragraph.")
    assert changed is True
    assert "One paragraph." in new_src
    assert ".. short-description::" in new_src
    assert "**Area:**" in new_src


def test_inject_skips_when_body_present() -> None:
    src = """Title
=====

.. short-description::
   Original text.

"""
    new_src, changed = inject_short_description_to_content(src, "New prose.")
    assert changed is False
    assert new_src == src


def test_inject_inserts_after_title_when_missing() -> None:
    src = """My Doc
======

Some intro text.
"""
    new_src, changed = inject_short_description_to_content(src, "Intro summary.")
    assert changed is True
    assert ".. short-description::" in new_src
    assert "Intro summary." in new_src
    assert new_src.index("======") < new_src.index(".. short-description::")
