# Copyright 2026 Open Robotics and contributors.
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

"""Build tests for the ``.. ros-related-articles::`` directive."""

import io
import os

from sphinx.application import Sphinx

PLUGINS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'plugins'))


def _page(area, title):
    return (
        '.. meta::\n'
        f'   :area: {area}\n'
        '   :contentType: about\n'
        '\n'
        f'{title}\n'
        f'{"=" * len(title)}\n'
        '\n'
        'Text.\n'
        '\n'
        'Related content\n'
        '---------------\n'
        '\n'
        'Related articles:\n'
        '\n'
        '.. ros-related-articles::\n'
    )


def _build(tmp_path, pages):
    """Build *pages* ({docname: (area, title)}) and return (warnings, outdir)."""
    srcdir = tmp_path / 'src'
    outdir = tmp_path / 'out'
    srcdir.mkdir()
    (srcdir / 'conf.py').write_text(
        'import sys\n'
        f'sys.path.insert(0, {PLUGINS_DIR!r})\n'
        "extensions = ['ros_related_articles']\n"
    )
    toctree = ''.join(f'   {name}\n' for name in pages)
    (srcdir / 'index.rst').write_text(f'Index\n=====\n\n.. toctree::\n\n{toctree}')
    for name, (area, title) in pages.items():
        (srcdir / f'{name}.rst').write_text(_page(area, title))

    warnings = io.StringIO()
    app = Sphinx(
        str(srcdir), str(srcdir), str(outdir), str(tmp_path / 'doctrees'), 'html',
        status=None, warning=warnings, freshenv=True,
    )
    app.build()
    return warnings.getvalue(), outdir


def test_links_pages_that_share_the_primary_area(tmp_path):
    warnings, outdir = _build(tmp_path, {
        'a': ('nodes, framework', 'Page A'),
        'b': ('nodes', 'Page B'),
    })

    assert 'ros-related-articles' not in warnings
    assert 'href="b.html"' in (outdir / 'a.html').read_text()
    assert 'href="a.html"' in (outdir / 'b.html').read_text()


def test_warns_when_no_other_page_shares_the_area(tmp_path):
    warnings, _ = _build(tmp_path, {
        'a': ('nodes', 'Page A'),
        'b': ('nodes', 'Page B'),
        'c': ('perception', 'Page C'),
    })

    assert 'c.rst' in warnings
    assert "no other page has the area 'perception'" in warnings
    assert 'a.rst' not in warnings
    assert 'b.rst' not in warnings
