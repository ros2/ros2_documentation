# Copyright 2026 Open Robotics
"""Tests for pagefind.yml loading."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, 'plugins')

from pagefind_config import load_pagefind_yml  # noqa: E402
from pagefind_config import parse_search_ui_config  # noqa: E402


def load_search_ui_config(confdir: str):
    return parse_search_ui_config(load_pagefind_yml(confdir))


def test_load_search_ui_config_from_repo_pagefind_yml() -> None:
    confdir = str(Path(__file__).resolve().parent.parent)
    config = load_search_ui_config(confdir)
    labels = {field['key']: field['label'] for field in config.result_meta}
    keys = [field['key'] for field in config.result_meta]
    assert 'product' not in labels
    assert labels['distribution'] == 'Distribution'
    assert labels['area'] == 'Area'
    assert labels['experience'] == 'Level'
    assert keys.index('distribution') < keys.index('area')
    assert config.always_show_filters == ['distribution', 'area', 'contentType', 'experience']
    assert config.always_show_filters.index('distribution') < config.always_show_filters.index('experience')
    assert config.show_breadcrumb is True
    assert config.show_metadata is False
    assert config.excerpt_length == 50


def test_load_search_ui_config_missing_file_uses_defaults() -> None:
    config = load_search_ui_config('/nonexistent/path')
    assert config.result_meta == []
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



def test_parse_result_meta_preserves_order_and_labels() -> None:
    config = parse_search_ui_config({
        'search_result_meta': {
            'contentType': 'Content type',
            'product': 'Product',
            'distribution': 'Distribution',
        },
    })
    assert config.result_meta == [
        {'key': 'contentType', 'label': 'Content type'},
        {'key': 'product', 'label': 'Product'},
        {'key': 'distribution', 'label': 'Distribution'},
    ]


def test_parse_result_meta_empty_label_uses_default() -> None:
    config = parse_search_ui_config({'search_result_meta': {'contentType': ''}})
    assert config.result_meta == [{'key': 'contentType', 'label': 'Content Type'}]


def test_parse_result_meta_ignores_non_dict() -> None:
    assert parse_search_ui_config({'search_result_meta': ['area']}).result_meta == []
    assert parse_search_ui_config(None).result_meta == []
