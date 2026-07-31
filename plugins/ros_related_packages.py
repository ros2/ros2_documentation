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

"""Sphinx directive for runtime ROS distro package lists (filtered in the browser)."""

from __future__ import annotations

import html
import os
import urllib.error
import urllib.request
from typing import List

from docutils import nodes
from docutils.parsers.rst import directives
from sphinx.util import logging as sphinx_logging
from sphinx.util.docutils import SphinxDirective

LOGGER = sphinx_logging.getLogger(__name__)

ROSDISTRO_CACHE_TEMPLATE = (
    'https://repo.ros2.org/rosdistro_cache/{distro}-cache.yaml.gz'
)

DEFAULT_RELATED_PACKAGES_MAX = 25
DEFAULT_RELATED_PACKAGES_VISIBLE_MAX = 7


def _normalize_field_name(raw: str) -> str:
    """Normalize a docinfo field label for comparison (e.g. ``Area`` → ``area``)."""
    name = raw.strip().lower().rstrip(':')
    return name.replace(' ', '-')


def _field_value_from_doctree(document: nodes.document, wanted: str) -> str | None:
    """Return the body of the first matching docinfo/rst field in the document."""
    wanted_norm = _normalize_field_name(wanted)
    for field in document.traverse(nodes.field):
        children = getattr(field, 'children', ()) or ()
        if len(children) < 2:
            continue
        label = children[0].astext()
        if _normalize_field_name(label) != wanted_norm:
            continue
        return children[1].astext().strip()
    return None


def _meta_get(metadata: dict, *names: str) -> str | None:
    """Look up document metadata using several possible keys (Sphinx/docutils variants)."""
    for name in names:
        for key, val in metadata.items():
            if not val:
                continue
            if _normalize_field_name(str(key)) == _normalize_field_name(name):
                return str(val).strip()
    return None


def _meta_content_from_docutils(document: nodes.document, meta_name: str) -> str | None:
    """Read ``docutils.nodes.meta`` emitted by ``.. meta::`` (typically ``<head>`` HTML meta tags).

    Field names work in RST as ``.. meta::`` fields, e.g. ``:area: nodes, framework``.
    """
    for node in document.traverse(nodes.meta):
        if node.get('name') != meta_name:
            continue
        raw = node.get('content')
        if raw:
            return str(raw).strip()
    return None


def _bundled_cache_href(docname: str, distro: str) -> str:
    """Relative URL from this page's HTML file to the downloaded gzip in ``_static/``.

    Sphinx emits sibling paths like ``_static/`` under the HTML root (including per-version
    directories for multiversion builds). Depth follows ``docname`` segments (slashes).
    """
    depth = docname.count('/')
    return ('../' * depth) + f'_static/rosdistro_cache/{distro}-cache.yaml.gz'


def _proxy_cache_href(proxy_template: str, distro: str) -> str:
    """Build runtime proxy URL from template, replacing ``{distro}``."""
    if not proxy_template:
        return ''
    return proxy_template.replace('{distro}', distro)


def _positive_int_option(argument: str) -> int:
    """Parse a positive integer option for the directive."""
    if argument is None:
        raise ValueError('option requires a number')
    value = int(argument)
    if value < 1:
        raise ValueError('must be positive')
    return value


class RosRelatedPackagesDirective(SphinxDirective):
    """Emit a placeholder ``div`` filled at runtime by ``related_packages.js``.

    Write the section intro (e.g. ``Packages/reference:``) in the RST source
    before this directive. Optional bullet items immediately before or after
    the directive are merged into the same list at runtime.
    Auto-generated entries that duplicate a manual package link are omitted.

    By default up to 25 packages are loaded (alphabetical); the first 7 are
    shown with a control to reveal the rest.

    Matching rules
    --------------
    The page declares ``area`` via ``.. meta::`` (the same field the related
    article lists use, so one value drives both lists). A package is listed when
    the first value of its ``<area>`` export in ``package.xml`` equals the first
    value of the page's ``area``. Order ``area`` so that the most specific value
    comes first (for example ``nodes, framework``); matching keys on that first
    value and does not pull in packages that only share a broader parent.

    Package metadata comes from the rosdistro cache: each package embeds its
    ``package.xml``, and the browser reads the ``<area>`` export::

        <export>
          <build_type>ament_cmake</build_type>
          <area>nodes, framework</area>
        </export>

    Several values may be given either as one ``<area>`` with values separated
    by commas, or as repeated ``<area>`` elements, ordered so the most specific
    value comes first. Only the first value is used for matching.

    Declare the page area like this::

        .. meta::
           :area: nodes, framework

    Fallbacks: Sphinx ``env.metadata`` / a visible RST field list ``:area:``.
    Optional ``:area:`` on this directive overrides document metadata.
    """

    has_content = False
    required_arguments = 0
    optional_arguments = 0
    option_spec = {
        'area': directives.unchanged,
        'max': _positive_int_option,
    }

    def run(self) -> List[nodes.Node]:
        area_opt = self.options.get('area')
        if area_opt:
            area = area_opt.strip()
        else:
            meta = self.env.metadata.get(self.env.docname, {})
            area = (
                _meta_content_from_docutils(self.state.document, 'area')
                or _meta_get(meta, 'area')
                or _field_value_from_doctree(self.state.document, 'area')
                or ''
            )

        if not area:
            raise self.error(
                'ros-related-packages: define `area` with `.. meta::` '
                '(recommended), or a `:area:` field list, or pass `:area:` on '
                'this directive.'
            )

        max_pkgs = self.options.get('max', DEFAULT_RELATED_PACKAGES_MAX)

        macros = getattr(self.env.config, 'macros', {}) or {}
        distro = macros.get('DISTRO', 'rolling')

        escaped_area = html.escape(area, quote=True)
        escaped_distro = html.escape(distro, quote=True)
        bundled_href = _bundled_cache_href(self.env.docname, distro)
        escaped_bundled = html.escape(bundled_href, quote=True)
        proxy_template = getattr(self.env.config, 'ros_related_packages_proxy_url', '')
        proxy_href = _proxy_cache_href(proxy_template, distro)
        escaped_proxy = html.escape(proxy_href, quote=True)

        html_body = (
            '<div class="related-packages related-packages--loading js-related-packages" '
            f'data-area="{escaped_area}" '
            f'data-max="{int(max_pkgs)}" '
            f'data-visible-max="{DEFAULT_RELATED_PACKAGES_VISIBLE_MAX}" '
            f'data-distro="{escaped_distro}" '
            f'data-bundled-cache-href="{escaped_bundled}" '
            f'data-proxy-cache-href="{escaped_proxy}" '
            'role="region" aria-live="polite">'
            '<p class="related-packages__status">Loading related packages…</p>'
            '</div>'
        )
        return [nodes.raw('', html_body, format='html')]


def download_rosdistro_cache(app) -> None:
    """Fetch the gzipped rosdistro cache into ``source/_static`` for same-origin loads.

    Sphinx 8+ passes only ``app`` to ``builder-inited``; the builder is ``app.builder``.
    """
    builder = app.builder
    if builder is None or builder.format != 'html':
        return

    macros = getattr(app.config, 'macros', {}) or {}
    distro = macros.get('DISTRO', 'rolling')

    dest_dir = os.path.join(app.confdir, 'source', '_static', 'rosdistro_cache')
    os.makedirs(dest_dir, exist_ok=True)
    dest_path = os.path.join(dest_dir, f'{distro}-cache.yaml.gz')
    url = ROSDISTRO_CACHE_TEMPLATE.format(distro=distro)

    request = urllib.request.Request(url, headers={'User-Agent': 'ros2-documentation-build/1.0'})
    try:
        with urllib.request.urlopen(request, timeout=120) as response:
            data = response.read()
        with open(dest_path, 'wb') as handle:
            handle.write(data)
    except (urllib.error.URLError, OSError, TimeoutError) as exc:
        LOGGER.warning(
            'Could not download rosdistro cache from %s (%s). '
            'Related package lists may not work until the file exists at %s',
            url,
            exc,
            dest_path,
        )


def setup(app) -> dict:
    """Register the directive, config value and cache download hook with Sphinx."""
    app.add_config_value('ros_related_packages_proxy_url', '', 'html')
    app.add_directive('ros-related-packages', RosRelatedPackagesDirective)
    app.connect('builder-inited', download_rosdistro_cache)
    return {
        'parallel_read_safe': True,
        'parallel_write_safe': True,
    }
