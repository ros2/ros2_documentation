#!/usr/bin/python3
from sphinxlint.checkers import checker
from sphinxlint.utils import paragraphs
from sphinx_tamer.sentences import split_into_sentences, register_extra_pattern
from sphinx_tamer.sentence_scan import is_ignorable
import pathlib
import re
import yaml


config_path = pathlib.Path('.sphinx_tamer.yaml')
ignorable_prefixes = []
if config_path.exists():
    config = yaml.safe_load(open(config_path))
    scan_config = config.get('sentence_scan', {})
    for pattern_text in scan_config.get('extra_patterns', []):
        register_extra_pattern(pattern_text)
    ignorable_prefixes += scan_config.get('ignorable_prefixes', [])


@checker('.rst', '.md')
def check_sentence_count(file, lines, options=None):
    if is_ignorable(file[7:], ignorable_prefixes):
        return

    for paragraph_lno, paragraph in paragraphs(lines):
        if paragraph.lstrip().startswith('.. '):
            continue

        for i, line in enumerate(paragraph.split('\n')):
            sentences = split_into_sentences(line)
            if len(sentences) <= 1:
                continue

            sentence0_words = ' '.join(sentences[0].split(' ')[-3:])
            sentence1_words = ' '.join(sentences[1].split(' ')[:3])
            yield paragraph_lno + i, f'Each sentence must start on a new line. Break between "{sentence0_words}" and "{sentence1_words}"'
