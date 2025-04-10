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

import textwrap

import sys

# Workaround to be able to import conf without it being a proper module
sys.path.append('..')

from conf import expand_interface_macros
from conf import expand_package_macros
from conf import expand_text_macros


def test_text_macros() -> None:
    macros = {
        'DISTRO': 'rolling',
        'some_macro_name': 'some_value',
    }
    text = textwrap.dedent("""
        The current distro is: {DISTRO}.

        This line does not use macros.
        The value of some_macro_name is: {some_macro_name}.
    """)
    expected = textwrap.dedent("""
        The current distro is: rolling.

        This line does not use macros.
        The value of some_macro_name is: some_value.
    """)
    assert expected == expand_text_macros(text, macros)


def test_interface_macros() -> None:
    macros = {
        'DISTRO': 'rolling',
    }
    text = textwrap.dedent("""
        Publish a {interface(pkg_msgs/msg/Msg)}.

        For more information, refer to `its definition <{interface_link(pkg_msgs/msg/Msg)}>`_.
    """)
    expected = textwrap.dedent("""
        Publish a `pkg_msgs/msg/Msg <https://docs.ros.org/en/rolling/p/pkg_msgs/msg/Msg.html>`_.

        For more information, refer to `its definition <https://docs.ros.org/en/rolling/p/pkg_msgs/msg/Msg.html>`_.
    """)
    # Expanded interface macros use the DISTRO text macro, so those need to be expanded too
    assert expected == expand_text_macros(expand_interface_macros(text), macros)


def test_package_macros() -> None:
    macros = {
        'DISTRO': 'rolling',
    }
    text = textwrap.dedent("""
        Add a dependency on {package(my_pkg_name)} to your package's ``package.xml`` file.

        For more information, refer to the `API documentation <{package_link(my_pkg_name)}>`_.
    """)
    expected = textwrap.dedent("""
        Add a dependency on `my_pkg_name <https://docs.ros.org/en/rolling/p/my_pkg_name/>`_ to your package's ``package.xml`` file.

        For more information, refer to the `API documentation <https://docs.ros.org/en/rolling/p/my_pkg_name/>`_.
    """)
    # Expanded package macros use the DISTRO text macro, so those need to be expanded too
    assert expected == expand_text_macros(expand_package_macros(text), macros)
