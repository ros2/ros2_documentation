# Copyright 2026 Open Robotics
"""Tests for Pagefind result metadata config parsing."""

from __future__ import annotations

import sys
from types import SimpleNamespace

sys.path.insert(0, 'plugins')

from pagefind_meta import (  # noqa: E402
    _breadcrumb_from_parents,
    _breadcrumb_meta_tag,
    _current_distro_from_config,
    _html_page_context,
    _seo_and_filter_metas,
)


def _parse_result_meta_fields(raw: dict) -> list:
    return [{'key': key, 'label': label} for key, label in raw.items()]


def test_seo_and_filter_metas_facet_allowlist() -> None:
    fields = _parse_result_meta_fields({'product': 'Product', 'area': 'Area'})
    html = _seo_and_filter_metas(
        fields,
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
    fields = _parse_result_meta_fields({'area': 'Area'})
    html = _seo_and_filter_metas(
        fields,
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


def test_current_distro_from_config_uses_macros_distro() -> None:
    app = SimpleNamespace(config=SimpleNamespace(macros={'DISTRO': 'humble'}))
    assert _current_distro_from_config(app) == 'humble'


def test_current_distro_from_config_defaults_to_rolling() -> None:
    app = SimpleNamespace(config=SimpleNamespace(macros={}))
    assert _current_distro_from_config(app) == 'rolling'


def test_breadcrumb_from_parents_joins_titles() -> None:
    parents = [
        {'link': 'Get-Started.html', 'title': 'Get started'},
        {'link': 'Installation.html', 'title': 'Installation'},
    ]
    assert _breadcrumb_from_parents(parents) == 'Get started › Installation'


def test_breadcrumb_from_parents_skips_empty_and_non_dict() -> None:
    assert _breadcrumb_from_parents([]) == ''
    assert _breadcrumb_from_parents(None) == ''
    assert _breadcrumb_from_parents([{'title': ''}, 'x', {'title': 'Tools'}]) == 'Tools'


def _app_for_context(fmt: str) -> SimpleNamespace:
    def get_target_uri(pagename: str, typ: str | None = None) -> str:
        del typ
        return f'{pagename}.html'

    return SimpleNamespace(
        config=SimpleNamespace(
            macros={},
            smv_current_version='',
            pagefind_enabled=False,
            pagefind_search={'search_result_meta': {'area': 'Area'}},
        ),
        builder=SimpleNamespace(format=fmt, get_target_uri=get_target_uri),
    )


def test_non_html_builder_still_gets_page_relative_pagefind_paths() -> None:
    context: dict = {}
    _html_page_context(
        _app_for_context('epub'),
        'Guides/Example',
        'page.tex',
        context,
        None,
    )
    assert context['pagefind_bundle_prefix'] == '../pagefind/'
    assert context['pagefind_filter_keys_csv'] == 'area'
    assert context['pagefind_enabled'] is False
    assert context['pagefind_seo_filter_metas'] == ''


def test_breadcrumb_meta_tag_is_meta_not_filter() -> None:
    tag = _breadcrumb_meta_tag('Get started › Installation')
    assert 'name="breadcrumb"' in tag
    assert 'content="Get started › Installation"' in tag
    assert 'data-pagefind-meta="breadcrumb[content]"' in tag
    assert 'data-pagefind-filter' not in tag
