# Copyright 2025 Sony Group Corporation.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import argparse
import re
import sys

def is_one_sentence_per_line(line) -> bool:
    """
    Check if a line contains only one complete sentence.
    
    :param line: The line of file to check.
    """
    # this assumes a sentence ends with a period, question mark, or exclamation mark.
    sentence_endings = re.findall(r'[.!?]', line)
    # allow empty lines or lines with only one sentence
    return len(sentence_endings) <= 1

def check_changed_lines(filename) -> None:
    """
    Check only changed lines for violations.

    .. warning:: It checks only added / modified lines for the viaolation in the file,
       and ignores everything else including removed lines.
    
    :param filename: The name of the file to check.
    """
    with open(filename, 'r', encoding='utf-8') as file:
        lines = file.readlines()

    violations = []
    for lineno, line in enumerate(lines, start=1):
        line = line.strip()
        # check only added lines, ignore everything else
        if line.startswith("+") and not is_one_sentence_per_line(line[1:]):
            violations.append((lineno, line[1:]))

    if violations:
        print(f"\n⚠️ Found {len(violations)} violations: One sentence per line is required.")
        for lineno, line in violations:
            print(f"  ❌ Line {lineno}: {line}")
        # exit with non-zero status code to fail github actions
        sys.exit(1)
    else:
        print("✅ No violations found.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Check if modified contents contain only one complete sentence per line.")
    parser.add_argument(
        'filename', type=str, help='The name of the file to check.')
    args = parser.parse_args()

    check_changed_lines(args.filename)
