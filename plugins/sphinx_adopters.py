# Copyright 2026 Sony Group Corporation.
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

"""Sphinx extension to render and validate the ROS 2 adopters YAML file. """

import os

import yaml
from docutils import nodes
from docutils.parsers.rst import Directive
from sphinx.errors import ExtensionError

from adopters_schema import validate_adopters, validate_adopter_urls


def _escape(text):
    """Escape HTML special characters."""
    if text is None:
        return ''
    return (
        str(text)
        .replace('&', '&amp;')
        .replace('<', '&lt;')
        .replace('>', '&gt;')
        .replace('"', '&quot;')
    )


def _make_link(text, url):
    """Create an HTML link if url is provided, otherwise plain text."""
    escaped_text = _escape(text)
    if url:
        escaped_url = _escape(url)
        return f'<a href="{escaped_url}" target="_blank" rel="noopener">{escaped_text}</a>'
    return escaped_text


class AdoptersTableDirective(Directive):
    """Directive to render the adopters YAML as a filterable HTML table."""

    has_content = False
    required_arguments = 0
    optional_arguments = 0
    option_spec = {}

    def run(self):
        env = self.state.document.settings.env
        # Locate adopters.yaml relative to the source file containing the directive.
        source_dir = os.path.dirname(env.doc2path(env.docname))
        yaml_path = os.path.join(source_dir, 'Adopters', 'adopters.yaml')

        if not os.path.isfile(yaml_path):
            raise ExtensionError(
                f'adopters.yaml not found at {yaml_path}'
            )

        # Register adopters.yaml as a dependency so incremental builds
        # detect changes and re-read the directive.
        env.note_dependency(yaml_path)

        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        adopters = data.get('adopters', [])
        errors = validate_adopters(adopters)
        errors.extend(validate_adopter_urls(adopters))
        if errors:
            raise ExtensionError(
                'Adopters YAML validation failed:\n' + '\n'.join(f'  - {e}' for e in errors)
            )

        # Sort by organization name (A-Z), then date_added descending (newest first).
        adopters.sort(key=lambda a: a.get('date_added', ''), reverse=True)
        adopters.sort(key=lambda a: a.get('organization', '').lower())

        # Collect unique values for filters.
        all_domains = sorted({d for a in adopters for d in a.get('domain', [])})
        all_countries = sorted({c for a in adopters for c in a.get('country', [])})

        # Build HTML.
        html_parts = []

        # Filter controls.
        html_parts.append('<div class="adopters-filters">')
        html_parts.append('<label for="adopters-filter-domain">Domain:</label>')
        html_parts.append('<select id="adopters-filter-domain">')
        html_parts.append('<option value="">All</option>')
        for d in all_domains:
            html_parts.append(f'<option value="{_escape(d)}">{_escape(d)}</option>')
        html_parts.append('</select>')

        html_parts.append('<label for="adopters-filter-country">Country:</label>')
        html_parts.append('<select id="adopters-filter-country">')
        html_parts.append('<option value="">All</option>')
        for c in all_countries:
            html_parts.append(f'<option value="{_escape(c)}">{_escape(c)}</option>')
        html_parts.append('</select>')

        html_parts.append('<label for="adopters-filter-search">Search:</label>')
        html_parts.append(
            '<input type="text" id="adopters-filter-search" placeholder="Filter by keyword...">'
        )

        # Show-all-history toggle.
        html_parts.append(
            '<label class="adopters-toggle-label">'
            '<input type="checkbox" id="adopters-show-all">'
            ' Show all history'
            '</label>'
            '<span class="adopters-filter-note">'
            'Showing entries from the past 3 years. '
            'Check &ldquo;Show all history&rdquo; to see all entries.'
            '</span>'
        )

        html_parts.append('</div>')

        # Table.
        html_parts.append('<table class="adopters-table">')
        html_parts.append('<thead><tr>')
        html_parts.append('<th>Organization</th>')
        html_parts.append('<th>Project</th>')
        html_parts.append('<th>Domain</th>')
        html_parts.append('<th>Date Added</th>')
        html_parts.append('<th>Country</th>')
        html_parts.append('<th>Description</th>')
        html_parts.append('</tr></thead>')
        html_parts.append('<tbody>')

        for adopter in adopters:
            org = _make_link(
                adopter.get('organization', ''),
                adopter.get('organization_url'),
            )
            project = _make_link(
                adopter.get('project', ''),
                adopter.get('project_url'),
            )
            domains = ', '.join(adopter.get('domain', []))
            date_added = adopter.get('date_added', '')
            countries = adopter.get('country', [])
            country_str = ', '.join(countries)
            description = _escape(adopter.get('description', ''))

            # Data attributes for filtering.
            data_domains = ' '.join(_escape(d) for d in adopter.get('domain', []))
            data_countries = ' '.join(_escape(c) for c in countries)
            html_parts.append(
                f'<tr data-domains="{data_domains}" '
                f'data-date-added="{_escape(str(date_added))}" '
                f'data-countries="{data_countries}">'
            )
            html_parts.append(f'<td>{org}</td>')
            html_parts.append(f'<td>{project}</td>')
            html_parts.append(f'<td>{_escape(domains)}</td>')
            html_parts.append(f'<td>{_escape(str(date_added))}</td>')
            html_parts.append(f'<td>{_escape(country_str)}</td>')
            html_parts.append(f'<td>{description}</td>')
            html_parts.append('</tr>')

        html_parts.append('</tbody></table>')

        raw_node = nodes.raw('', '\n'.join(html_parts), format='html')
        return [raw_node]


_ADOPTERS_DOCNAME = 'The-ROS2-Project/Adopters/Adopters'


def _get_outdated(app, env, _added, _changed, _removed):
    """Force rebuild of the Adopters page when adopters.yaml changes."""
    yaml_path = os.path.join(app.srcdir, 'The-ROS2-Project', 'Adopters', 'adopters.yaml')
    if not os.path.isfile(yaml_path):
        return []
    doctree_path = os.path.join(app.doctreedir, _ADOPTERS_DOCNAME + '.doctree')
    if os.path.isfile(doctree_path):
        if os.path.getmtime(yaml_path) > os.path.getmtime(doctree_path):
            return [_ADOPTERS_DOCNAME]
    return []


def setup(app):
    app.add_directive('adopters-table', AdoptersTableDirective)
    app.connect('env-get-outdated', _get_outdated)
    return {
        'version': '0.1',
        'parallel_read_safe': True,
        'parallel_write_safe': True,
    }
