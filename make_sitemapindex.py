from xml.etree.ElementTree import Element, SubElement, ElementTree
from conf import distro_full_names, html_baseurl


def make_sitemapindex(sitemap_file):

    sitemapindex = Element('sitemapindex')
    sitemapindex.set('xmlns', 'http://www.sitemaps.org/schemas/sitemap/0.9')
    for distro in distro_full_names.keys():
        node = SubElement(sitemapindex, 'sitemap')
        SubElement(node, 'loc').text = f'{html_baseurl}/{distro}/sitemap.xml'

    ElementTree(sitemapindex).write(sitemap_file, encoding='utf-8', xml_declaration=True)

if __name__ == '__main__':
    sitemap_file = 'build/html/sitemap.xml'
    make_sitemapindex(sitemap_file)
