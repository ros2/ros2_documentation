# Copyright (c) 2013 Michael Dowling <mtdowling@gmail.com>
# Copyright (c) 2017 Jared Dillard <jared.dillard@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.


'''
The implementation of this sphinx extensions is largely borrowed from the sphinx_sitemap extension
(version: https://github.com/jdillard/sphinx-sitemap/blob/3bd87ef166bc9f3758d360ad4d23b524668b161d/sphinx_sitemap/__init__.py)
with modifications made to add_html_link() to ignore any pages whose context contains the
'skip_sitemap' key.
'''


import os
import queue
from multiprocessing import Manager
from typing import Any, Dict, List, Optional
from xml.etree import ElementTree

from sphinx.application import Sphinx
from sphinx.util.logging import getLogger

__version__ = "0.0.1"

logger = getLogger(__name__)


def setup(app: Sphinx) -> Dict[str, Any]:
    """
    Sphinx extension setup function.
    It adds config values and connects Sphinx events to the sitemap builder.

    :param app: The Sphinx Application instance
    :return: A dict of Sphinx extension options
    """
    app.add_config_value("site_url", default=None, rebuild="")
    app.add_config_value(
        "sitemap_url_scheme", default="{lang}{version}{link}", rebuild=""
    )
    app.add_config_value("sitemap_locales", default=None, rebuild="")

    app.add_config_value("sitemap_filename", default="sitemap.xml", rebuild="")

    try:
        app.add_config_value("html_baseurl", default=None, rebuild="")
    except BaseException:
        pass

    app.connect("builder-inited", record_builder_type)
    app.connect("html-page-context", add_html_link)
    app.connect("build-finished", create_sitemap)

    return {
        "parallel_read_safe": True,
        "parallel_write_safe": True,
        "version": __version__,
    }


def get_locales(app: Sphinx) -> List[str]:
    """
    Get a list of locales from the extension config or automatically detect based
    on Sphinx Application config.

    :param app: The Sphinx Application instance
    :return: A list of locales
    """
    # Manually configured list of locales
    sitemap_locales: Optional[List[str]] = app.builder.config.sitemap_locales
    if sitemap_locales:
        # special value to add nothing -> use primary language only
        if sitemap_locales == [None]:
            return []

        # otherwise, add each locale
        return [locale for locale in sitemap_locales]

    # Or autodetect locales
    locales = []
    for locale_dir in app.builder.config.locale_dirs:
        locale_dir = os.path.join(app.confdir, locale_dir)
        if os.path.isdir(locale_dir):
            for locale in os.listdir(locale_dir):
                if os.path.isdir(os.path.join(locale_dir, locale)):
                    locales.append(locale)
    return locales


def record_builder_type(app: Sphinx):
    """
    Determine if the Sphinx Builder is an instance of DirectoryHTMLBuilder and store that in the
    application environment.

    :param app: The Sphinx Application instance
    """
    # builder isn't initialized in the setup so we do it here
    builder = getattr(app, "builder", None)
    if builder is None:
        return
    builder.env.is_directory_builder = type(builder).__name__ == "DirectoryHTMLBuilder"
    builder.env.app.sitemap_links = Manager().Queue()


def hreflang_formatter(lang: str) -> str:
    """
    Format the supplied locale code into a string that is compatible with `hreflang`.
    See also:

    - https://en.wikipedia.org/wiki/Hreflang#Common_Mistakes
    - https://github.com/readthedocs/readthedocs.org/pull/5638

    :param lang: The locale string to format
    :return: The formatted locale string
    """
    if "_" in lang:
        return lang.replace("_", "-")
    return lang


def add_html_link(app: Sphinx, pagename: str, templatename, context, doctree):
    """
    As each page is built, collect page names for the sitemap

    :param app: The Sphinx Application instance
    :param pagename: The current page being built
    """

    # Return if the context contains `skip_sitemap`.
    if 'skip_sitemap' in context.keys():
        logger.debug(f"sphinx-sitemap-ros: Ignoring sitemap entry for {pagename} with reason \"{context['skip_sitemap']}\"")
        return

    env = app.builder.env
    if app.builder.config.html_file_suffix is None:
        file_suffix = ".html"
    else:
        file_suffix = app.builder.config.html_file_suffix

    # Support DirectoryHTMLBuilder path structure
    # where generated links between pages omit the index.html
    if env.is_directory_builder:  # type: ignore
        if pagename == "index":
            sitemap_link = ""
        elif pagename.endswith("/index"):
            sitemap_link = pagename[:-6] + "/"
        else:
            sitemap_link = pagename + "/"
    else:
        sitemap_link = pagename + file_suffix

    env.app.sitemap_links.put(sitemap_link)  # type: ignore


def create_sitemap(app: Sphinx, exception):
    """
    Generates the sitemap.xml from the collected HTML page links.

    :param app: The Sphinx Application instance
    """
    site_url = app.builder.config.site_url or app.builder.config.html_baseurl
    if site_url:
        site_url.rstrip("/") + "/"
    else:
        logger.warning(
            "sphinx-sitemap-ros: html_baseurl is required in conf.py." "Sitemap not built.",
            type="sitemap",
            subtype="configuration",
        )
        return

    if app.env.app.sitemap_links.empty():  # type: ignore
        logger.info(
            "sphinx-sitemap-ros: No pages generated for %s" % app.config.sitemap_filename,
            type="sitemap",
            subtype="information",
        )
        return

    ElementTree.register_namespace("xhtml", "http://www.w3.org/1999/xhtml")

    root = ElementTree.Element(
        "urlset", xmlns="http://www.sitemaps.org/schemas/sitemap/0.9"
    )

    locales = get_locales(app)

    if app.builder.config.version:
        version = app.builder.config.version + "/"
    else:
        version = ""

    while True:
        try:
            link = app.env.app.sitemap_links.get_nowait()  # type: ignore
        except queue.Empty:
            break

        url = ElementTree.SubElement(root, "url")
        scheme = app.config.sitemap_url_scheme
        if app.builder.config.language:
            lang = app.builder.config.language + "/"
        else:
            lang = ""

        ElementTree.SubElement(url, "loc").text = site_url + scheme.format(
            lang=lang, version=version, link=link
        )

        for lang in locales:
            lang = lang + "/"
            ElementTree.SubElement(
                url,
                "{http://www.w3.org/1999/xhtml}link",
                rel="alternate",
                hreflang=hreflang_formatter(lang.rstrip("/")),
                href=site_url + scheme.format(lang=lang, version=version, link=link),
            )

    filename = app.outdir + "/" + app.config.sitemap_filename
    ElementTree.ElementTree(root).write(
        filename, xml_declaration=True, encoding="utf-8", method="xml"
    )

    logger.info(
        "sphinx-sitemap-ros: %s was generated for URL %s in %s"
        % (app.config.sitemap_filename, site_url, filename),
        type="sitemap",
        subtype="information",
    )
