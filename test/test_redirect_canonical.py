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

"""Regression tests for canonical URLs (https://github.com/ros2/ros2_documentation/issues/6112)."""

import re
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock
from xml.etree.ElementTree import parse

from sphinx.builders.html import StandaloneHTMLBuilder
from sphinx.util.osutil import relative_uri

# Workaround to be able to import conf without it being a proper module
sys.path.append('..')

import conf
from conf import RedirectFrom
from conf import smv_rewrite_configs
from make_sitemapindex import make_sitemapindex


def test_default_baseurl_includes_rolling() -> None:
    assert conf.html_baseurl == 'https://docs.ros.org/en/rolling'


def test_multiversion_overrides_baseurl() -> None:
    app = MagicMock()
    app.config.html_baseurl = conf.html_baseurl
    app.config.smv_current_version = 'humble'
    smv_rewrite_configs(app, app.config)
    assert app.config.html_baseurl == 'https://docs.ros.org/en/humble'


def test_redirect_canonical_is_absolute(monkeypatch) -> None:
    builder = MagicMock(spec=StandaloneHTMLBuilder)
    builder.get_target_uri.side_effect = lambda docname: docname + '.html'
    builder.get_relative_uri.side_effect = (
        lambda from_, to: relative_uri(from_ + '.html', to + '.html'))
    app = SimpleNamespace(
        builder=builder,
        srcdir='/src',
        config=SimpleNamespace(html_baseurl='https://docs.ros.org/en/rolling'),
    )
    monkeypatch.setattr(RedirectFrom, 'redirections', {
        '/src/How-To-Guides/Ament-CMake-Documentation.rst': {'Guides/Ament-CMake-Documentation'},
    })

    [(redirect_url, context, _)] = list(RedirectFrom.generate(app))
    metatags = context['metatags']

    assert redirect_url == 'Guides/Ament-CMake-Documentation'
    canonical = re.search(r'<link rel="canonical" href="([^"]+)"', metatags).group(1)
    assert canonical == \
        'https://docs.ros.org/en/rolling/How-To-Guides/Ament-CMake-Documentation.html'
    # The in-site redirect stays relative.
    assert 'url=../How-To-Guides/Ament-CMake-Documentation.html' in metatags


def test_sitemapindex_urls(tmp_path) -> None:
    sitemap_file = tmp_path / 'sitemap.xml'
    make_sitemapindex(str(sitemap_file))
    ns = {'sm': 'http://www.sitemaps.org/schemas/sitemap/0.9'}
    locs = [loc.text for loc in parse(sitemap_file).getroot().findall('sm:sitemap/sm:loc', ns)]
    assert locs == [
        f'https://docs.ros.org/en/{distro}/sitemap.xml' for distro in conf.distro_full_names
    ]
