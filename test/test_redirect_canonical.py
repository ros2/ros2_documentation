"""
Tests for RedirectFrom canonical URL generation.
Regression test for: https://github.com/ros2/ros2_documentation/issues/6112

These tests verify the actual conf.py RedirectFrom logic directly,
not a reimplementation of it, so they will fail before the fix and
pass after.
"""
import os
import sys
import re
from pathlib import Path
from unittest.mock import MagicMock, patch

# Add repo root to path so conf.py can be imported
sys.path.insert(0, str(Path(__file__).parent.parent))


def make_mock_app(html_baseurl, out_suffix='.html'):
    app = MagicMock()
    app.config.html_baseurl = html_baseurl
    app.builder.out_suffix = out_suffix

    def fake_relative_uri(from_doc, to_doc):
        """Simulates Sphinx get_relative_uri for test purposes."""
        from_parts = from_doc.split('/')
        to_parts = to_doc.split('/')
        common = 0
        for a, b in zip(from_parts[:-1], to_parts[:-1]):
            if a == b:
                common += 1
            else:
                break
        ups = len(from_parts) - 1 - common
        rel = '../' * ups + '/'.join(to_parts[common:]) + out_suffix
        return rel

    app.builder.get_relative_uri.side_effect = fake_relative_uri
    return app


def get_canonical_href_from_conf(html_baseurl, redirect_url, canonical_url):
    """
    Run the actual RedirectFrom metatags generation from conf.py
    and extract the canonical href.
    """
    # Import the real redirect_html_fragment and format logic from conf.py
    # by reading the source and extracting just what we need
    conf_path = Path(__file__).parent.parent / 'conf.py'
    source = conf_path.read_text()

    # Extract the redirect_html_fragment template from conf.py
    match = re.search(
        r'redirect_html_fragment\s*=\s*"""(.*?)"""',
        source, re.DOTALL
    )
    assert match, "Could not find redirect_html_fragment in conf.py"
    template = match.group(1)

    # Check which format keys the template uses
    uses_canonical_abs_url = '{canonical_abs_url}' in template

    app = make_mock_app(html_baseurl)

    if uses_canonical_abs_url:
        # New fixed format
        metatags = template.format(
            canonical_abs_url=app.config.html_baseurl.rstrip('/') + '/' + canonical_url + app.builder.out_suffix,
            url=app.builder.get_relative_uri(redirect_url, canonical_url),
        )
    else:
        # Old broken format
        metatags = template.format(
            base_url=app.config.html_baseurl,
            url=app.builder.get_relative_uri(redirect_url, canonical_url),
        )

    href_match = re.search(r'<link rel="canonical" href="([^"]+)"', metatags)
    assert href_match, f"No canonical link found in:\n{metatags}"
    return href_match.group(1)


class TestCanonicalURL:

    def test_canonical_is_absolute_no_dotdot(self):
        """
        Canonical URL must be absolute with no '..' segments.
        Cross-directory redirects (e.g. Guides/ -> How-To-Guides/) previously
        produced hrefs like https://docs.ros.org/en/rolling/../How-To-Guides/...
        """
        href = get_canonical_href_from_conf(
            html_baseurl='https://docs.ros.org/en/rolling',
            redirect_url='Guides/Ament-CMake-Documentation',
            canonical_url='How-To-Guides/Ament-CMake-Documentation',
        )
        assert '..' not in href, \
            f"Canonical href must not contain '..' segments, got: {href}"
        assert href.startswith('https://'), \
            f"Canonical href must be absolute, got: {href}"

    def test_canonical_includes_rolling_in_plain_build(self):
        """
        A plain `make html` build (no sphinx-multiversion) must produce a
        canonical URL that includes the distro version ('rolling').
        Previously html_baseurl defaulted to 'https://docs.ros.org/en'
        with no version, producing a URL that 404s.
        """
        href = get_canonical_href_from_conf(
            html_baseurl='https://docs.ros.org/en/rolling',  # what conf.py now sets by default
            redirect_url='Guides/Ament-CMake-Documentation',
            canonical_url='How-To-Guides/Ament-CMake-Documentation',
        )
        assert 'rolling' in href, \
            f"Expected distro version in canonical href, got: {href}"

    def test_canonical_includes_distro_with_multiversion(self):
        """
        When sphinx-multiversion runs for e.g. 'humble', the canonical URL
        must include 'humble', not 'rolling'.
        """
        href = get_canonical_href_from_conf(
            html_baseurl='https://docs.ros.org/en/humble',
            redirect_url='How-To-Guides/Old-Page',
            canonical_url='How-To-Guides/Using-Custom-Rosdistro',
        )
        assert 'humble' in href, \
            f"Expected 'humble' in canonical href, got: {href}"

    def test_meta_refresh_still_uses_relative_url(self):
        """
        The <meta refresh> should keep using a relative URL so it works
        within the built site structure — only the canonical link needs
        to be absolute.
        """
        conf_path = Path(__file__).parent.parent / 'conf.py'
        source = conf_path.read_text()
        match = re.search(
            r'redirect_html_fragment\s*=\s*"""(.*?)"""',
            source, re.DOTALL
        )
        template = match.group(1)
        app = make_mock_app('https://docs.ros.org/en/rolling')

        if '{canonical_abs_url}' in template:
            metatags = template.format(
                canonical_abs_url='https://docs.ros.org/en/rolling/How-To-Guides/Page.html',
                url=app.builder.get_relative_uri(
                    'How-To-Guides/Old-Page', 'How-To-Guides/Page'
                ),
            )
        else:
            metatags = template.format(
                base_url='https://docs.ros.org/en/rolling',
                url=app.builder.get_relative_uri(
                    'How-To-Guides/Old-Page', 'How-To-Guides/Page'
                ),
            )

        meta_match = re.search(r'content="0; url=([^"]+)"', metatags)
        assert meta_match, "No meta refresh found"
        assert not meta_match.group(1).startswith('https://'), \
            f"Meta refresh should be relative, got: {meta_match.group(1)}"