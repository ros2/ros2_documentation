# Copyright 2026 Open Robotics
"""Tests for Pagefind result metadata config parsing."""

from __future__ import annotations

import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, 'plugins')

from pagefind_meta import (  # noqa: E402
    _parse_result_meta_fields,
    _seo_and_filter_metas,
)


def _app(result_meta_order):
    return SimpleNamespace(config=SimpleNamespace(pagefind_result_meta_order=result_meta_order))


def test_parse_result_meta_fields_dict_preserves_order_and_labels() -> None:
    app = _app(
        {
            'contentType': 'Content type',
            'product': 'Product',
            'distribution': 'Distribution',
        },
    )
    assert _parse_result_meta_fields(app) == [
        {'key': 'contentType', 'label': 'Content type'},
        {'key': 'product', 'label': 'Product'},
        {'key': 'distribution', 'label': 'Distribution'},
    ]


def test_parse_result_meta_fields_dict_empty_label_uses_default() -> None:
    app = _app({'area': ''})
    parsed = _parse_result_meta_fields(app)
    assert len(parsed) == 1
    assert parsed[0]['key'] == 'area'
    assert parsed[0]['label'] == 'Area'


def test_parse_result_meta_fields_list_deprecated_shim() -> None:
    app = _app(['product', 'area'])
    parsed = _parse_result_meta_fields(app)
    assert [p['key'] for p in parsed] == ['product', 'area']
    assert parsed[0]['label'] == 'Product'
    assert parsed[1]['label'] == 'Area'


def test_parse_result_meta_fields_allowlist_only_configured_keys() -> None:
    app = _app({'product': 'Product'})
    parsed = _parse_result_meta_fields(app)
    keys = {p['key'] for p in parsed}
    assert keys == {'product'}
    assert 'description' not in keys


def test_parse_result_meta_fields_empty_config() -> None:
    app = _app({})
    assert _parse_result_meta_fields(app) == []


def test_seo_and_filter_metas_facet_allowlist() -> None:
    app = _app({'product': 'Product', 'area': 'Area'})
    html = _seo_and_filter_metas(
        app,
        {
            'product': 'ROS 2',
            'description': 'Long overview text',
            'area': 'framework',
        },
    )
    assert 'data-pagefind-filter="product[content]"' in html
    assert 'data-pagefind-filter="area[content]"' in html
    assert 'name="description"' in html
    assert 'data-pagefind-filter="description' not in html


def test_seo_and_filter_metas_no_facets_when_config_empty() -> None:
    app = _app({})
    html = _seo_and_filter_metas(app, {'product': 'ROS 2'})
    assert 'name="product"' in html
    assert 'data-pagefind-filter' not in html
