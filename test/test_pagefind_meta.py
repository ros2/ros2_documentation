# Copyright 2026 Open Robotics
"""Tests for Pagefind result metadata config parsing."""

from __future__ import annotations

import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, 'plugins')

from pagefind_meta import (  # noqa: E402
    _current_distro_from_config,
    _facet_filter_keys_for_context,
    _parse_result_meta_fields,
    _seo_and_filter_metas,
)


def _app(result_meta_order, always_show_filters=None):
    return SimpleNamespace(
        config=SimpleNamespace(
            pagefind_result_meta_order=result_meta_order,
            pagefind_always_show_filters=always_show_filters or [],
        ),
    )


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
    assert 'data-pagefind-meta="product[content]"' in html
    assert 'data-pagefind-meta="area[content]"' in html
    assert 'name="description"' in html
    assert 'data-pagefind-filter="description' not in html
    assert 'data-pagefind-meta="description' not in html
    assert 'pagefind-page-meta' not in html


def test_seo_and_filter_metas_splits_facets_keeps_seo_whole() -> None:
    app = _app({'area': 'Area'})
    html = _seo_and_filter_metas(
        app,
        {
            'area': 'framework, middleware',
            'description': 'Alpha, beta, and gamma overview',
        },
    )
    assert html.count('data-pagefind-filter="area[content]"') == 2
    assert 'content="framework"' in html
    assert 'content="middleware"' in html
    assert html.count('name="description"') == 1
    assert 'content="Alpha, beta, and gamma overview"' in html


def test_facet_filter_keys_for_context_order_and_corpus() -> None:
    app = _app({'product': 'Product', 'area': 'Area', 'tool': 'Tool'})
    env = SimpleNamespace(
        pagefind_meta_keys_by_doc={
            'a': {'product', 'area'},
            'b': {'description'},
        },
    )
    assert _facet_filter_keys_for_context(app, env) == ['product', 'area']


def test_facet_filter_keys_include_always_show_without_corpus() -> None:
    app = _app(
        {'product': 'Product', 'area': 'Area', 'tool': 'Tool'},
        always_show_filters=['tool', 'product'],
    )
    env = SimpleNamespace(pagefind_meta_keys_by_doc={})
    # Order follows search_result_meta, not always_show_filters list order.
    assert _facet_filter_keys_for_context(app, env) == ['product', 'tool']


def test_facet_filter_keys_union_corpus_and_always_show() -> None:
    app = _app(
        {'product': 'Product', 'area': 'Area', 'tool': 'Tool'},
        always_show_filters=['tool'],
    )
    env = SimpleNamespace(
        pagefind_meta_keys_by_doc={'a': {'product'}},
    )
    assert _facet_filter_keys_for_context(app, env) == ['product', 'tool']


def test_current_distro_from_config_uses_macros_distro() -> None:
    app = SimpleNamespace(config=SimpleNamespace(macros={'DISTRO': 'humble'}))
    assert _current_distro_from_config(app) == 'humble'


def test_current_distro_from_config_defaults_to_rolling() -> None:
    app = SimpleNamespace(config=SimpleNamespace(macros={}))
    assert _current_distro_from_config(app) == 'rolling'
