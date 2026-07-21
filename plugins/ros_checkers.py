#!/usr/bin/env python3
from sphinxlint.checkers import checker
import subprocess
import sys
import yaml


@checker('.rst', '.md')
def check_sentence_count(file, lines, options=None):
    output = subprocess.check_output(['sphinx_sentence_scan_single', '.', file])
    results = yaml.safe_load(output)
    for result in results:
        try:
            sentences = result['sentences']
        except TypeError:
            print(f'Unexpected structure in {file}:\n{results}\n', file=sys.stderr)
            raise
        for first, second in zip(sentences, sentences[1:]):
            words0 = ' '.join(first.split()[-3:])  # last three words
            words1 = ' '.join(second.split()[:3])  # first three words
            yield result['line_num'], f'Each sentence must start on a new line. Break between "{words0}" and "{words1}"'
