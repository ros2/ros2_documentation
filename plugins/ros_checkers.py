#!/usr/bin/python3
from sphinxlint.checkers import checker
from sphinxlint.utils import paragraphs
from sphinx_tamer import get_lines
from sphinx_tamer.sphinx_wrapper import load_sphinx, SphinxFile
from sphinx_tamer.sentences import split_into_sentences, register_extra_pattern
from sphinx_tamer.sentence_scan import is_ignorable
import pathlib
import re
import yaml


root_path = pathlib.Path('.')
config_path = root_path / '.sphinx_tamer.yaml'
src_path = root_path / 'source'
ignorable_prefixes = []
if config_path.exists():
    config = yaml.safe_load(open(config_path))
    scan_config = config.get('sentence_scan', {})
    for pattern_text in scan_config.get('extra_patterns', []):
        register_extra_pattern(pattern_text)
    ignorable_prefixes += scan_config.get('ignorable_prefixes', [])

settings = None
reporter = None
parser = None


@checker('.rst', '.md')
def check_sentence_count(file, lines, options=None):
    global settings, reporter, parser, ignorable_prefixes

    if settings is None:
        settings, reporter, parser = load_sphinx(
            src_path=src_path,
            conf_path=root_path
        )

    file_path = pathlib.Path(file)
    rel_path = file_path.relative_to(src_path)

    if is_ignorable(str(rel_path), ignorable_prefixes):
        return

    sphinx_file = SphinxFile(file_path, rel_path, settings, reporter, parser)

    for line in get_lines(sphinx_file.parse()):
        sentences = split_into_sentences(line.get_text())
        if len(sentences) <= 1:
            continue

        src_sentences = line.get_source_sentences(sentences)
        for first, second in zip(src_sentences, src_sentences[1:]):
            words0 = ' '.join(first.split()[-3:])  # last three words
            words1 = ' '.join(second.split()[:3])  # first three words
            yield line.line_num, f'Each sentence must start on a new line. Break between "{words0}" and "{words1}"'
