from xml.etree.ElementTree import Element, SubElement, ElementTree


class SitemapGenerator:
    def __init__(self):
        self.sitemapindex = Element('sitemapindex')
        self.sitemapindex.set('xmlns', 'http://www.sitemaps.org/schemas/sitemap/0.9')

    def generate_url(self, loc):
        url = Element('url')
        SubElement(url, 'loc').text = loc
        return url

    def add_sitemap(self, distro, base_url):
        node = SubElement(self.sitemapindex, 'sitemap')
        SubElement(node, 'loc').text = f'{base_url}/{distro}/sitemap.xml'

    def generate_sitemap_index(self):
        ElementTree(self.sitemapindex).write('build/html/sitemap.xml', encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    from conf import distro_full_names, html_baseurl

    sitemap_generator = SitemapGenerator()
    for distro in distro_full_names.keys():
        sitemap_url = f'{html_baseurl}/{distro}/sitemap.xml'
        sitemap_generator.add_sitemap(distro, html_baseurl)
    
    sitemap_generator.generate_sitemap_index()
