#!/usr/bin/python3
from sphinxlint.checkers import checker
from sphinxlint.utils import paragraphs
import re

# Derived from https://stackoverflow.com/a/31505798
DOT = '•'
QUESTION = '✇'
EXCLAM = '‼'
SP_LOOKUP = {
    '.': DOT,
    '?': QUESTION,
    '!': EXCLAM,
}
STOP = '¶'

# Classes and patterns
ALPHABETIC = r'([A-Za-z])'
DIGITS = '([0-9])'
NOT_BREAK = r'[^!.?]'
BREAK = r'([!.?])'
PREFIXES = r'(Mr|St|Mrs|Ms|Dr|etc|vol|cf|et al|vs|eg|Proc)\.'
SUFFIXES = r'(Inc|Ltd|Jr|Sr|Co)'
WEBSITES = r'\.(com|net|org|io|gov|edu|me|ros)'
EXTENSIONS = r'\.(xml|cfg|py|launch|frame_id|ini|md|log|h|cpp|bash|patch|gz|yaml|txt|NET|js|msg|srv|action|rst|exe)'
MULTIPLE_DOTS = r'\.{2,}'
STARTERS = r'(Mr|Mrs|Ms|Dr|Prof|Capt|Cpt|Lt|He\s|She\s|It\s|They\s|Their\s|Our\s|We\s|But\s|However\s|That\s|This\s|Wherever)'
ACRONYMS = r'([A-Z][.][A-Z][.](?:[A-Z][.])?)'
DIGITS_DOT_DIGITS = re.compile(DIGITS + '([.])' + DIGITS)
SINGLE_LETTER = re.compile(r'(\s[A-Za-z])(\.)(\s)')
DOT_PAREN = re.compile(r'\.( \()')

# RST Formatting Patterns
HYPERLINK = re.compile(r'(<[^>.!?]*)' + BREAK + r'([^>]*>)')
BACKTICK = re.compile(r'(`[^`.!?]*)' + BREAK + r'([^`]*`)')
LIST_PREFIX = re.compile(r'^(\s*[\d#]+)(\.)(.*)')
TRAILING_FORMATTING = re.compile(r'()' + BREAK + r'(\s*[\*\)]*\s*)$')


def split_into_sentences(text):
    """
    Split the text into sentences.

    Assumes text does not contain the special characters DOT or STOP

    :param text: text to be split into sentences
    :type text: str

    :return: list of sentences
    :rtype: list[str]
    """

    text = ' ' + text + '  '

    # Convert nonbreaking punctuation to special characters
    for pattern in [DIGITS_DOT_DIGITS, SINGLE_LETTER,
                                 HYPERLINK, BACKTICK, LIST_PREFIX, TRAILING_FORMATTING
                                 ]:
        m = pattern.search(text)
        while m:
            text = text.replace(m.group(0), m.group(1) + SP_LOOKUP[m.group(2)] + m.group(3))
            m = pattern.search(text)

    for pattern in [PREFIXES, SUFFIXES]:
        text = re.sub(pattern, '\\1' + DOT, text)
    for pattern in [DOT_PAREN, WEBSITES, EXTENSIONS]:
        text = re.sub(pattern, DOT + '\\1', text)

    text = re.sub(MULTIPLE_DOTS, lambda match: DOT * len(match.group(0)) + STOP, text)
    text = re.sub('Ph\.D\.', f'Ph{DOT}D{DOT}', text)
    text = re.sub(ACRONYMS + ' ' + STARTERS,
                  f'\\1{STOP} \\2',
                  text)
    text = re.sub(ALPHABETIC + '[.]' + ALPHABETIC + '[.]' + ALPHABETIC + '[.]',
                  f'\\1{DOT}\\2{DOT}\\3' + DOT,
                  text
                 )
    text = re.sub(ALPHABETIC + '[.]' + ALPHABETIC + '[.]', f'\\1{DOT}\\2{DOT}', text)

    # Convert breaking punctuation to include STOP character
    # and convert special characters back to normal
    for stopper, replacement in SP_LOOKUP.items():
        text = text.replace(stopper, stopper + STOP)
        text = text.replace(replacement, stopper)

    sentences = text.split(STOP)
    sentences = [s.strip() for s in sentences]

    return list(filter(None, sentences))


@checker('.rst', '.md')
def check_sentence_count(file, lines, options=None):
    for paragraph_lno, paragraph in paragraphs(lines):
        for special_char in [DOT, STOP, QUESTION]:
            if special_char in paragraph:
                yield paragraph_lno, f'Contains the special character {special_char}'

        if paragraph.lstrip().startswith('.. '):
            continue

        for i, line in enumerate(paragraph.split('\n')):
            sentences = split_into_sentences(line)
            if len(sentences) <= 1:
                continue

            sentence0_words = ' '.join(sentences[0].split(' ')[-3:])
            sentence1_words = ' '.join(sentences[1].split(' ')[:3])
            yield paragraph_lno + i, f'Each sentence must start on a new line. Break between "{sentence0_words}" and "{sentence1_words}"'
