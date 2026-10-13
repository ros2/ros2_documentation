# Copyright 2026 Open Robotics
"""Tests for pagefind.yml loading."""

from __future__ import annotations

import os
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, 'plugins')

from pagefind_config import load_search_ui_config  # noqa: E402
from pagefind_config import pagefind_yml_outdated  # noqa: E402
from pagefind_config import remember_pagefind_yml  # noqa: E402


def test_load_search_ui_config_from_repo_pagefind_yml() -> None:
    confdir = str(Path(__file__).resolve().parent.parent)
    config = load_search_ui_config(confdir)
    order = config.result_meta
    assert 'product' not in order
    assert order['distribution'] == 'Distribution'
    assert order['area'] == 'Area'
    assert order['experience'] == 'Level'
    assert list(order.keys()).index('distribution') < list(order.keys()).index('area')
    assert config.always_show_filters == ['distribution', 'area', 'contentType', 'experience']
    assert config.always_show_filters.index('distribution') < config.always_show_filters.index('experience')
    assert config.show_breadcrumb is True
    assert config.show_metadata is False
    assert config.excerpt_length == 50


def test_load_search_ui_config_missing_file_uses_defaults() -> None:
    config = load_search_ui_config('/nonexistent/path')
    assert config.result_meta == {}
    assert config.always_show_filters == []
    assert config.show_breadcrumb is False
    assert config.show_metadata is False
    assert config.excerpt_length == 30


def test_load_search_ui_config_drops_invalid_excerpt_and_unknown_filters(tmp_path) -> None:
    (tmp_path / 'pagefind.yml').write_text(
        """
search_result_meta:
  area: Area
always_show_filters:
  - area
  - missing
search_result_excerpt_length: 0
search_result_breadcrumb: true
search_result_metadata: true
""",
        encoding='utf-8',
    )
    config = load_search_ui_config(str(tmp_path))
    assert config.always_show_filters == ['area']
    assert config.excerpt_length == 30
    assert config.show_breadcrumb is True
    assert config.show_metadata is True


def test_pagefind_yml_outdated_until_mtime_is_remembered(tmp_path) -> None:
    yml = tmp_path / 'pagefind.yml'
    yml.write_text('search_result_meta: {}\n', encoding='utf-8')
    app = SimpleNamespace(confdir=str(tmp_path))
    env = SimpleNamespace(found_docs={'index', 'about'}, pagefind_yml_mtime=None)
    assert set(pagefind_yml_outdated(app, env, set(), set(), set())) == {'index', 'about'}
    remember_pagefind_yml(app, env)
    assert pagefind_yml_outdated(app, env, set(), set(), set()) == []
    yml.write_text('search_result_meta:\n  area: Area\n', encoding='utf-8')
    yml_stat = yml.stat()
    os.utime(yml, ns=(yml_stat.st_atime_ns, yml_stat.st_mtime_ns + 1))
    assert set(pagefind_yml_outdated(app, env, set(), set(), set())) == {'index', 'about'}
