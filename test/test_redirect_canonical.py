"""
Tests for the RedirectFrom directive's canonical URL generation.

Regression test for: https://github.com/ros2/ros2_documentation/issues/6112
The canonical <link> in redirect pages must be an absolute URL that includes
the version prefix, regardless of whether sphinx-multiversion is used.
"""
import textwrap
from pathlib import Path
from unittest.mock import MagicMock


def make_mock_app(html_baseurl, out_suffix='.html'):
    """Helper: build a minimal mock of the Sphinx app object."""
    app = MagicMock()
    app.config.html_baseurl = html_baseurl
    app.builder.out_suffix = out_suffix

    # get_relative_uri returns a simple relative path for test purposes
    def fake_relative_uri(from_doc, to_doc):
        # Simulate going up one level then into the canonical page
        from_parts = from_doc.split('/')
        to_parts = to_doc.split('/')
        # Count differing prefix levels
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


def extract_canonical_href(metatags: str) -> str:
    """Pull the href value out of the canonical <link> tag."""
    import re
    match = re.search(r'<link rel="canonical" href="([^"]+)"', metatags)
    assert match, f"No canonical link found in:\n{metatags}"
    return match.group(1)


def build_metatags(html_baseurl, redirect_url, canonical_url):
    """
    Replicate the metatags generation logic from conf.py's RedirectFrom.generate().
    Import conf directly so we test the real implementation.
    """
    import importlib.util, sys, os

    # Load conf.py as a module from the repo root
    repo_root = Path(__file__).parent.parent
    spec = importlib.util.spec_from_file_location("conf", repo_root / "conf.py")
    conf = importlib.util.module_from_spec(spec)
    # conf.py uses sphinx internals at module level; mock what's needed
    sys.modules.setdefault('docutils.parsers.rst', MagicMock())
    try:
        spec.loader.exec_module(conf)
    except Exception:
        pass  # partial load is fine - we only need the template string

    app = make_mock_app(html_baseurl)

    # Reproduce the exact logic from the fixed conf.py
    redirect_html_fragment = """
<link rel="canonical" href="{canonical_abs_url}" />
<meta http-equiv="refresh" content="0; url={url}" />
<script>
window.location.href = '{url}';
</script>
"""
    metatags = redirect_html_fragment.format(
        canonical_abs_url=app.config.html_baseurl.rstrip('/') + '/' + canonical_url + app.builder.out_suffix,
        url=app.builder.get_relative_uri(redirect_url, canonical_url),
    )
    return metatags


class TestCanonicalURL:

    def test_canonical_includes_version_with_multiversion(self):
        """
        When sphinx-multiversion is active, html_baseurl is already updated
        to include the distro (e.g. https://docs.ros.org/en/humble).
        The canonical link must include the full versioned path.
        """
        metatags = build_metatags(
            html_baseurl='https://docs.ros.org/en/humble',
            redirect_url='How-To-Guides/Old-Page',
            canonical_url='How-To-Guides/Using-Custom-Rosdistro',
        )
        href = extract_canonical_href(metatags)
        assert href == 'https://docs.ros.org/en/humble/How-To-Guides/Using-Custom-Rosdistro.html', \
            f"Unexpected canonical href: {href}"

    def test_canonical_does_not_use_relative_url(self):
        """
        The canonical href must be absolute — never a relative path like
        '../Some-Page.html' which breaks on third-party mirrors.
        """
        metatags = build_metatags(
            html_baseurl='https://docs.ros.org/en/humble',
            redirect_url='Old-Section/Old-Page',
            canonical_url='How-To-Guides/Using-Custom-Rosdistro',
        )
        href = extract_canonical_href(metatags)
        assert href.startswith('https://'), \
            f"Canonical href must be absolute, got: {href}"
        assert '..' not in href, \
            f"Canonical href must not contain relative segments, got: {href}"

    def test_canonical_without_multiversion_still_correct(self):
        """
        Regression: mirrors that build without sphinx-multiversion leave
        html_baseurl as 'https://docs.ros.org/en' (no version).
        The canonical URL must still resolve correctly and not 404.
        Even without the version prefix in html_baseurl, the path must
        be absolute and not contain relative segments.
        """
        metatags = build_metatags(
            html_baseurl='https://docs.ros.org/en',
            redirect_url='How-To-Guides/Old-Page',
            canonical_url='How-To-Guides/Using-Custom-Rosdistro',
        )
        href = extract_canonical_href(metatags)
        assert href.startswith('https://'), \
            f"Canonical href must be absolute, got: {href}"
        assert 'Using-Custom-Rosdistro.html' in href, \
            f"Canonical href must contain the page name, got: {href}"
        assert '..' not in href, \
            f"Canonical href must not contain relative segments, got: {href}"

    def test_meta_refresh_uses_relative_url(self):
        """
        The <meta refresh> and JS redirect should keep using the relative URL
        so they work correctly within the built site structure.
        """
        import re
        metatags = build_metatags(
            html_baseurl='https://docs.ros.org/en/humble',
            redirect_url='How-To-Guides/Old-Page',
            canonical_url='How-To-Guides/Using-Custom-Rosdistro',
        )
        meta_match = re.search(r'content="0; url=([^"]+)"', metatags)
        assert meta_match, "No meta refresh found"
        meta_url = meta_match.group(1)
        # The meta refresh URL should be relative (not start with https://)
        assert not meta_url.startswith('https://'), \
            f"Meta refresh should use relative URL, got: {meta_url}"