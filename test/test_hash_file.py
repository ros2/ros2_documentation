# Copyright 2025 Open Source Robotics Foundation, Inc.
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

import hashlib
import sys
import pathlib

# Workaround to be able to import conf without it being a proper module
sys.path.append('..')

from conf import _hash_file

def test_hash_file(tmp_path: pathlib.Path) -> None:
    non_existent = tmp_path / "does_not_exist.txt"
    assert _hash_file(non_existent) == ""

    empty_file = tmp_path / "empty.txt"
    empty_file.write_text("")
    expected_empty_hash = hashlib.sha256(b"").hexdigest()
    assert _hash_file(empty_file) == expected_empty_hash

    content = b"Hello, World!"
    known_file = tmp_path / "known.txt"
    known_file.write_bytes(content)
    expected_known_hash = hashlib.sha256(content).hexdigest()
    assert _hash_file(known_file) == expected_known_hash
