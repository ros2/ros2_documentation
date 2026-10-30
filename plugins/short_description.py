from __future__ import annotations

from docutils import nodes
from sphinx.util.docutils import SphinxDirective


class ShortDescriptionDirective(SphinxDirective):
    """Directive to render the short description of an article."""

    has_content = True
    required_arguments = 0
    optional_arguments = 0
    option_spec = {}

    def run(self) -> list[nodes.Node]:
        # Create a container node to hold the parsed content
        node = nodes.container()
        node['classes'].append('short-description')

        # Parse the directive content into the container node
        self.state.nested_parse(self.content, self.content_offset, node)

        return [node]


def setup(app):
    app.add_directive('short-description', ShortDescriptionDirective)
    return {
        'parallel_read_safe': True,
        'parallel_write_safe': True,
        'version': '0.1.0',
    }