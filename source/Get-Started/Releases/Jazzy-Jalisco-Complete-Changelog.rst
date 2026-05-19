Jazzy Jalisco changelog
=======================

This page is a list of the complete changes in all ROS 2 core packages since the previous release.

.. contents:: Table of Contents
   :local:

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`action_tutorials_cpp <https://github.com/ros2/demos/tree/jazzy/action_tutorials/action_tutorials_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`__)
* Fix format-security warning with clang. (`#663 <https://github.com/ros2/demos/issues/663>`__)
* Migrate std::bind calls to lambda expressions (`#659 <https://github.com/ros2/demos/issues/659>`__)
* Contributors: Chris Lalancette, Felipe Gomes de Melo, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`action_tutorials_interfaces <https://github.com/ros2/demos/tree/jazzy/action_tutorials/action_tutorials_interfaces/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`action_tutorials_py <https://github.com/ros2/demos/tree/jazzy/action_tutorials/action_tutorials_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`__)
* Add tests to action_tutorials_py. (`#664 <https://github.com/ros2/demos/issues/664>`__)
* Contributors: Chris Lalancette, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`actionlib_msgs <https://github.com/ros2/common_interfaces/tree/jazzy/actionlib_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Clarify the license. (`#241 <https://github.com/ros2/common_interfaces/issues/241>`__) In particular, every package in this repository is Apache 2.0 licensed except for sensor_msgs_py.  So move the CONTRIBUTING.md and LICENSE files down into the individual packages, and make sure that sensor_msgs_py has the correct CONTRIBUTING.md file (it already had the correct LICENSE file).
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_clang_format <https://github.com/ament/ament_lint/tree/jazzy/ament_clang_format/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_clang_tidy <https://github.com/ament/ament_lint/tree/jazzy/ament_clang_tidy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Fix a warning from newer versions of flake8. (`#469 <https://github.com/ament/ament_lint/issues/469>`__)
* remove AMENT_IGNORE check in clang-tidy when looking for compilation db (`#441 <https://github.com/ament/ament_lint/issues/441>`__)
* Contributors: Alberto Soragna, Chris Lalancette, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake <https://github.com/ament/ament_cmake/tree/jazzy/ament_cmake/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#503 <https://github.com/ament/ament_cmake/issues/503>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_auto <https://github.com/ament/ament_cmake/tree/jazzy/ament_cmake_auto/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#503 <https://github.com/ament/ament_cmake/issues/503>`__)
* Add ament_auto_add_gmock to ament_cmake_auto (`#482 <https://github.com/ament/ament_cmake/issues/482>`__)
* Contributors: Jordan Palacios, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_clang_format <https://github.com/ament/ament_lint/tree/jazzy/ament_cmake_clang_format/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_clang_tidy <https://github.com/ament/ament_lint/tree/jazzy/ament_cmake_clang_tidy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Provide --header-filter and --jobs to CMake. (`#450 <https://github.com/ament/ament_lint/issues/450>`__)
* Contributors: Michael Jeronimo, Roderick Taylor


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_copyright <https://github.com/ament/ament_lint/tree/jazzy/ament_cmake_copyright/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_core <https://github.com/ament/ament_cmake/tree/jazzy/ament_cmake_core/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Set hints to find the python version we actually want. (`#508 <https://github.com/ament/ament_cmake/issues/508>`__)
* Update maintainer list in package.xml files (`#503 <https://github.com/ament/ament_cmake/issues/503>`__)
* Use CMAKE_CURRENT_BINARY_DIR instead of CMAKE_BINARY_DIR  in ament_generate_environment (`#485 <https://github.com/ament/ament_cmake/issues/485>`__)
* Fix CMake error when entire ament projects are added via add_subdirectory (`#484 <https://github.com/ament/ament_cmake/issues/484>`__)
* Contributors: Chris Lalancette, Michael Jeronimo, Silvio Traversaro


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_cppcheck <https://github.com/ament/ament_lint/tree/jazzy/ament_cmake_cppcheck/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_cpplint <https://github.com/ament/ament_lint/tree/jazzy/ament_cmake_cpplint/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Increased cpplint timeout by default on Windows (`#486 <https://github.com/ament/ament_lint/issues/486>`__)
* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Contributors: Alejandro Hernández Cordero, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_export_definitions <https://github.com/ament/ament_cmake/tree/jazzy/ament_cmake_export_definitions/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#503 <https://github.com/ament/ament_cmake/issues/503>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_export_dependencies <https://github.com/ament/ament_cmake/tree/jazzy/ament_cmake_export_dependencies/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#503 <https://github.com/ament/ament_cmake/issues/503>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_export_include_directories <https://github.com/ament/ament_cmake/tree/jazzy/ament_cmake_export_include_directories/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#503 <https://github.com/ament/ament_cmake/issues/503>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_export_interfaces <https://github.com/ament/ament_cmake/tree/jazzy/ament_cmake_export_interfaces/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#503 <https://github.com/ament/ament_cmake/issues/503>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_export_libraries <https://github.com/ament/ament_cmake/tree/jazzy/ament_cmake_export_libraries/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#503 <https://github.com/ament/ament_cmake/issues/503>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_export_link_flags <https://github.com/ament/ament_cmake/tree/jazzy/ament_cmake_export_link_flags/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#503 <https://github.com/ament/ament_cmake/issues/503>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_export_targets <https://github.com/ament/ament_cmake/tree/jazzy/ament_cmake_export_targets/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#503 <https://github.com/ament/ament_cmake/issues/503>`__)
* Add NAMESPACE support to ament_export_targets (`#498 <https://github.com/ament/ament_cmake/issues/498>`__)
* Contributors: Michael Jeronimo, Ryan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_flake8 <https://github.com/ament/ament_lint/tree/jazzy/ament_cmake_flake8/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_gen_version_h <https://github.com/ament/ament_cmake/tree/jazzy/ament_cmake_gen_version_h/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#503 <https://github.com/ament/ament_cmake/issues/503>`__)
* Update to C++17 (`#488 <https://github.com/ament/ament_cmake/issues/488>`__)
* Contributors: Chris Lalancette, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_gmock <https://github.com/ament/ament_cmake/tree/jazzy/ament_cmake_gmock/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#503 <https://github.com/ament/ament_cmake/issues/503>`__)
* Split ament_add_gmock into _executable and _test. (`#497 <https://github.com/ament/ament_cmake/issues/497>`__)
* Contributors: Chris Lalancette, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_google_benchmark <https://github.com/ament/ament_cmake/tree/jazzy/ament_cmake_google_benchmark/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#503 <https://github.com/ament/ament_cmake/issues/503>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_gtest <https://github.com/ament/ament_cmake/tree/jazzy/ament_cmake_gtest/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#503 <https://github.com/ament/ament_cmake/issues/503>`__)
* Split ament_add_gmock into _executable and _test. (`#497 <https://github.com/ament/ament_cmake/issues/497>`__)
* ament_add_gtest_test: add TEST_NAME parameter (`#492 <https://github.com/ament/ament_cmake/issues/492>`__)
* Contributors: Chris Lalancette, Christopher Wecht, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_include_directories <https://github.com/ament/ament_cmake/tree/jazzy/ament_cmake_include_directories/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#503 <https://github.com/ament/ament_cmake/issues/503>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_libraries <https://github.com/ament/ament_cmake/tree/jazzy/ament_cmake_libraries/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* perf: faster ament_libraries_deduplicate implementation (`#448 <https://github.com/ament/ament_cmake/issues/448>`__) Co-authored-by: Scott K Logan <logans@cottsay.net>
* Subtle fix for ament_libraries_deduplicate tests (`#516 <https://github.com/ament/ament_cmake/issues/516>`__)
* Add some basic tests to ament_cmake_libraries (`#512 <https://github.com/ament/ament_cmake/issues/512>`__)
* Update maintainer list in package.xml files (`#503 <https://github.com/ament/ament_cmake/issues/503>`__)
* Contributors: Michael Jeronimo, Scott K Logan, Vincent Richard


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_lint_cmake <https://github.com/ament/ament_lint/tree/jazzy/ament_cmake_lint_cmake/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_mypy <https://github.com/ament/ament_lint/tree/jazzy/ament_cmake_mypy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_pclint <https://github.com/ament/ament_lint/tree/jazzy/ament_cmake_pclint/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_pep257 <https://github.com/ament/ament_lint/tree/jazzy/ament_cmake_pep257/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_pycodestyle <https://github.com/ament/ament_lint/tree/jazzy/ament_cmake_pycodestyle/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_pyflakes <https://github.com/ament/ament_lint/tree/jazzy/ament_cmake_pyflakes/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_pytest <https://github.com/ament/ament_cmake/tree/jazzy/ament_cmake_pytest/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#503 <https://github.com/ament/ament_cmake/issues/503>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_python <https://github.com/ament/ament_cmake/tree/jazzy/ament_cmake_python/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add in a comment explaining where Python3::Interpreter comes from. (`#510 <https://github.com/ament/ament_cmake/issues/510>`__)
* Update maintainer list in package.xml files (`#503 <https://github.com/ament/ament_cmake/issues/503>`__)
* Contributors: Chris Lalancette, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_target_dependencies <https://github.com/ament/ament_cmake/tree/jazzy/ament_cmake_target_dependencies/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#503 <https://github.com/ament/ament_cmake/issues/503>`__)
* Fix ``ament_target_dependencies`` (`#452 <https://github.com/ament/ament_cmake/issues/452>`__)
* Contributors: Michael Jeronimo, Vincent Richard


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_test <https://github.com/ament/ament_cmake/tree/jazzy/ament_cmake_test/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#503 <https://github.com/ament/ament_cmake/issues/503>`__)
* Recursively check for errors/failures in produced JUnit result XMLs (`#446 <https://github.com/ament/ament_cmake/issues/446>`__)
* Contributors: Michael Jeronimo, Nick Morales


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_uncrustify <https://github.com/ament/ament_lint/tree/jazzy/ament_cmake_uncrustify/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Added Timeout to ament_uncrustify (`#485 <https://github.com/ament/ament_lint/issues/485>`__)
* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Contributors: Alejandro Hernández Cordero, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_vendor_package <https://github.com/ament/ament_cmake/tree/jazzy/ament_cmake_vendor_package/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add more CMake variables to pass to vendor projects (`#519 <https://github.com/ament/ament_cmake/issues/519>`__)
* Fix patch file dependencies in ament_cmake_vendor_package (`#520 <https://github.com/ament/ament_cmake/issues/520>`__)
* Update maintainer list in package.xml files (`#503 <https://github.com/ament/ament_cmake/issues/503>`__)
* Always set CMAKE_C[XX]_COMPILER for vendor packages if needed (`#476 <https://github.com/ament/ament_cmake/issues/476>`__)
* Switch to CMake 'braket arguments' (`#461 <https://github.com/ament/ament_cmake/issues/461>`__)
* Replace 'git' dep with 'vcstool' (`#462 <https://github.com/ament/ament_cmake/issues/462>`__)
* Add support for specifying a patch directory in ament_vendor (`#449 <https://github.com/ament/ament_cmake/issues/449>`__)
* Contributors: Christophe Bedard, Michael Jeronimo, Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_version <https://github.com/ament/ament_cmake/tree/jazzy/ament_cmake_version/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#503 <https://github.com/ament/ament_cmake/issues/503>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_xmllint <https://github.com/ament/ament_lint/tree/jazzy/ament_cmake_xmllint/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_copyright <https://github.com/ament/ament_lint/tree/jazzy/ament_copyright/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Small fixes for modern flake8. (`#484 <https://github.com/ament/ament_lint/issues/484>`__)
* Fix add-copyright year function (`#466 <https://github.com/ament/ament_lint/issues/466>`__)
* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Contributors: Chris Lalancette, Lloyd Pearson, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cppcheck <https://github.com/ament/ament_lint/tree/jazzy/ament_cppcheck/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Add in checks to ament_cppcheck code. (`#472 <https://github.com/ament/ament_lint/issues/472>`__)
* Contributors: Chris Lalancette, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cpplint <https://github.com/ament/ament_lint/tree/jazzy/ament_cpplint/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Pass --output argument to cpplint (`#453 <https://github.com/ament/ament_lint/issues/453>`__)
* Contributors: Michael Jeronimo, Vladimir Ivan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_flake8 <https://github.com/ament/ament_lint/tree/jazzy/ament_flake8/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Changes to make ament_flake8 work with v6+ (`#459 <https://github.com/ament/ament_lint/issues/459>`__)
* Add additional dependencies to ament_flake8. (`#454 <https://github.com/ament/ament_lint/issues/454>`__)
* Fix compatibility with flake8 version 5 (`#410 <https://github.com/ament/ament_lint/issues/410>`__)
* Contributors: Chris Lalancette, Michael Carroll, Michael Jeronimo, Timo Röhling


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_index_cpp <https://github.com/ament/ament_index/tree/jazzy/ament_index_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update quality declaration documents (`#94 <https://github.com/ament/ament_index/issues/94>`__)
* only append search paths on first PackageNotFound (`#91 <https://github.com/ament/ament_index/issues/91>`__)
* Update to C++17 (`#90 <https://github.com/ament/ament_index/issues/90>`__)
* Contributors: Chris Lalancette, Christophe Bedard, Lucas Walter


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_index_python <https://github.com/ament/ament_index/tree/jazzy/ament_index_python/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update quality declaration documents (`#94 <https://github.com/ament/ament_index/issues/94>`__)
* Add type annotations to python files. (`#93 <https://github.com/ament/ament_index/issues/93>`__)
* Contributors: Christophe Bedard, Michael Carlstrom


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_lint <https://github.com/ament/ament_lint/tree/jazzy/ament_lint/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Add an ament_lint test dependency on python3-pytest. (`#473 <https://github.com/ament/ament_lint/issues/473>`__)
* Contributors: Chris Lalancette, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_lint_auto <https://github.com/ament/ament_lint/tree/jazzy/ament_lint_auto/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_lint_cmake <https://github.com/ament/ament_lint/tree/jazzy/ament_lint_cmake/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_lint_common <https://github.com/ament/ament_lint/tree/jazzy/ament_lint_common/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_mypy <https://github.com/ament/ament_lint/tree/jazzy/ament_mypy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Fix a flake8 warning in ament_mypy. (`#470 <https://github.com/ament/ament_lint/issues/470>`__) No need for parentheses around an assert.
* Contributors: Chris Lalancette, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_package <https://github.com/ament/ament_package/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Migrate from legacy importlib-resources (`#143 <https://github.com/ament/ament_package/issues/143>`__)
* Add setuptools dependency back in. (`#141 <https://github.com/ament/ament_package/issues/141>`__)
* Make python dependencies exec_depend. (`#140 <https://github.com/ament/ament_package/issues/140>`__)
* Contributors: Chris Lalancette, Isabel Paredes


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_pclint <https://github.com/ament/ament_lint/tree/jazzy/ament_pclint/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_pep257 <https://github.com/ament/ament_lint/tree/jazzy/ament_pep257/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Convert linenumber to string when printing errors (`#443 <https://github.com/ament/ament_lint/issues/443>`__)
* Contributors: Michael Jeronimo, Robert Brothers


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_pycodestyle <https://github.com/ament/ament_lint/tree/jazzy/ament_pycodestyle/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_pyflakes <https://github.com/ament/ament_lint/tree/jazzy/ament_pyflakes/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_uncrustify <https://github.com/ament/ament_lint/tree/jazzy/ament_uncrustify/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Adds uncrustify 0.78.1 config (`#475 <https://github.com/ament/ament_lint/issues/475>`__)
* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* Fix a flake8 warning in ament_uncrustify. (`#471 <https://github.com/ament/ament_lint/issues/471>`__)
* Contributors: Chris Lalancette, Marco A. Gutierrez, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_xmllint <https://github.com/ament/ament_lint/tree/jazzy/ament_xmllint/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#474 <https://github.com/ament/ament_lint/issues/474>`__)
* (ament_xmllint) add extensions argument (`#456 <https://github.com/ament/ament_lint/issues/456>`__)
* Contributors: Matthijs van der Burgh, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`camera_calibration_parsers <https://github.com/ros-perception/image_common/tree/jazzy/camera_calibration_parsers/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update to yaml-cpp 0.8.0. (`#305 <https://github.com/ros-perception/image_common/issues/305>`__)
* Switch from rcpputils::fs to std::filesystem (`#300 <https://github.com/ros-perception/image_common/issues/300>`__)
* Removed C headers: camera_info_manager camera_calibration_parsers (`#290 <https://github.com/ros-perception/image_common/issues/290>`__)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Christophe Bedard


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`camera_info_manager <https://github.com/ros-perception/image_common/tree/jazzy/camera_info_manager/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Switch from rcpputils::fs to std::filesystem (`#300 <https://github.com/ros-perception/image_common/issues/300>`__)
* Removed C headers: camera_info_manager camera_calibration_parsers (`#290 <https://github.com/ros-perception/image_common/issues/290>`__)
* Contributors: Alejandro Hernández Cordero, Christophe Bedard


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`class_loader <https://github.com/ros/class_loader/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Remove all uses of ament_target_dependencies. (`#210 <https://github.com/ros/class_loader/issues/210>`__)
* Update to C++17 (`#209 <https://github.com/ros/class_loader/issues/209>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`common_interfaces <https://github.com/ros2/common_interfaces/tree/jazzy/common_interfaces/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Clarify the license. (`#241 <https://github.com/ros2/common_interfaces/issues/241>`__) In particular, every package in this repository is Apache 2.0 licensed except for sensor_msgs_py.  So move the CONTRIBUTING.md and LICENSE files down into the individual packages, and make sure that sensor_msgs_py has the correct CONTRIBUTING.md file (it already had the correct LICENSE file).
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`composition <https://github.com/ros2/demos/tree/jazzy/composition/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [composition] add launch action console output in the verify section (`#677 <https://github.com/ros2/demos/issues/677>`__) (`#681 <https://github.com/ros2/demos/issues/681>`__) (cherry picked from commit 34d29db73e78a84a174ad8699a2d646b0eeb1cdf) Co-authored-by: Mikael Arguedas <mikael.arguedas@gmail.com>
* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`__)
* Migrate std::bind calls to lambda expressions (`#659 <https://github.com/ros2/demos/issues/659>`__)
* Contributors: Felipe Gomes de Melo, Michael Jeronimo, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`demo_nodes_cpp <https://github.com/ros2/demos/tree/jazzy/demo_nodes_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [demo_nodes_cpp] some readme and executable name fixups (`#678 <https://github.com/ros2/demos/issues/678>`__) (`#688 <https://github.com/ros2/demos/issues/688>`__) (cherry picked from commit aa8df8904b864d063e31fd5b953ffe561c7a9fe0) Co-authored-by: Mikael Arguedas <mikael.arguedas@gmail.com>
* Fix gcc warnings when building with optimizations. (`#672 <https://github.com/ros2/demos/issues/672>`__) (`#673 <https://github.com/ros2/demos/issues/673>`__) * Fix gcc warnings when building with optimizations. When building the allocator_tutorial_pmr demo with -O2, gcc is throwing an error saying that new and delete are mismatched.  This is something of a misnomer, however; the real problem is that the global new override we have in that demo is actually implemented incorrectly. In particular, the documentation at https://en.cppreference.com/w/cpp/memory/new/operator_new very clearly specifies that operator new either has to return a valid pointer, or throw an exception on error. Our version wasn't throwing the exception, so change it to throw std::bad_alloc if std::malloc fails. While we are in here, also fix another small possible is where std::malloc could return nullptr on a zero-sized object, thus throwing an exception it shouldn't. * Always inline the new and delete operators. That's because gcc 13 has a bug where it can sometimes inline one or the other, and then it detects that they mismatch.  For gcc and clang, just force them to always be inline in this demo. * Switch to NOINLINE instead. Both clang and MSVC don't like inlining these, so instead ensure that they are *not* inlined.  This also works because the problem is when new is inlined but not delete (or vice-versa).  As long as they are both not inlined, this should fix the warning. (cherry picked from commit 957ddbb9f04f55cabd8496e8d74eb35ee4d29105) Co-authored-by: Chris Lalancette <clalancette@gmail.com>
* A few uncrustify fixes for 0.78. (`#667 <https://github.com/ros2/demos/issues/667>`__)
* Allow users to configure the executor for executables in ``demo_nodes_cpp`` (`#666 <https://github.com/ros2/demos/issues/666>`__)
* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`__)
* Added extra documentation and clarifications. (`#651 <https://github.com/ros2/demos/issues/651>`__)
* Add in support for both the PMR and custom allocator tutorials. (`#655 <https://github.com/ros2/demos/issues/655>`__)
* Replacing old-style C++ allocator with a polymorphic memory resource (PMR) (`#653 <https://github.com/ros2/demos/issues/653>`__)
* Remove unnecessary captures in the various demos. (`#647 <https://github.com/ros2/demos/issues/647>`__)
* Dramatically speed up the demo_nodes_cpp tests (`#641 <https://github.com/ros2/demos/issues/641>`__)
* Switch to using RCLCPP logging macros in the lifecycle package. (`#644 <https://github.com/ros2/demos/issues/644>`__)
* failed to call introspection_client (`#643 <https://github.com/ros2/demos/issues/643>`__)
* Small cleanups to the demos when running through them. (`#639 <https://github.com/ros2/demos/issues/639>`__)
* Cleanup demo_nodes_cpp CMake and dependencies (`#638 <https://github.com/ros2/demos/issues/638>`__)
* Change the service introspection parameter off value to 'disabled' (`#634 <https://github.com/ros2/demos/issues/634>`__)
* Add demos for using logger service (`#611 <https://github.com/ros2/demos/issues/611>`__)
* Contributors: Ali Ashkani Nia, Barry Xu, Chen Lihui, Chris Lalancette, Michael Jeronimo, Yadu, jrutgeer, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`demo_nodes_cpp_native <https://github.com/ros2/demos/tree/jazzy/demo_nodes_cpp_native/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`demo_nodes_py <https://github.com/ros2/demos/tree/jazzy/demo_nodes_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`__)
* Change the service introspection parameter off value to 'disabled' (`#634 <https://github.com/ros2/demos/issues/634>`__) With this we can avoid the tricky bits around YAML interpretation of 'off' as a boolean.
* Add demos for using logger service (`#611 <https://github.com/ros2/demos/issues/611>`__)
* Contributors: Barry Xu, Chris Lalancette, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`diagnostic_msgs <https://github.com/ros2/common_interfaces/tree/jazzy/diagnostic_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Clarify the license. (`#241 <https://github.com/ros2/common_interfaces/issues/241>`__) In particular, every package in this repository is Apache 2.0 licensed except for sensor_msgs_py.  So move the CONTRIBUTING.md and LICENSE files down into the individual packages, and make sure that sensor_msgs_py has the correct CONTRIBUTING.md file (it already had the correct LICENSE file).
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`dummy_map_server <https://github.com/ros2/demos/tree/jazzy/dummy_robot/dummy_map_server/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`dummy_robot_bringup <https://github.com/ros2/demos/tree/jazzy/dummy_robot/dummy_robot_bringup/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`__)
* Switch to file-content launch substitution (`#627 <https://github.com/ros2/demos/issues/627>`__)
* Contributors: Michael Jeronimo, Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`dummy_sensors <https://github.com/ros2/demos/tree/jazzy/dummy_robot/dummy_sensors/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update dummy_sensors readme to echo the correct topic (`#675 <https://github.com/ros2/demos/issues/675>`__) (`#684 <https://github.com/ros2/demos/issues/684>`__) (cherry picked from commit eec5c12ea95dfaaa230f9f1a8e9cff9b09dde5d5) Co-authored-by: jmackay2 <1.732mackay@gmail.com>
* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`__)
* Fix unstable LaserScan status for rviz2 (`#614 <https://github.com/ros2/demos/issues/614>`__)
* Contributors: Chen Lihui, Michael Jeronimo, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`example_interfaces <https://github.com/ros2/example_interfaces/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update to C++17. (`#18 <https://github.com/ros2/example_interfaces/issues/18>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_minimal_subscriber <https://github.com/ros2/examples/tree/jazzy/rclcpp/topics/minimal_subscriber/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix: Fixed compilation after API change of TimerBase::execute (`#375 <https://github.com/ros2/examples/issues/375>`__) Co-authored-by: Janosch Machowinski <J.Machowinski@cellumation.com>
* Split lambda and subscriber def in minimal example (`#363 <https://github.com/ros2/examples/issues/363>`__)
* Contributors: Felipe Gomes de Melo, jmachowinski


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_wait_set <https://github.com/ros2/examples/tree/jazzy/rclcpp/wait_set/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix: Fixed compilation after API change of TimerBase::execute (`#375 <https://github.com/ros2/examples/issues/375>`__) Co-authored-by: Janosch Machowinski <J.Machowinski@cellumation.com>
* Contributors: jmachowinski


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`foonathan_memory_vendor <https://github.com/eProsima/foonathan_memory_vendor/tree/master/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Improve mechanism to find an installation of foonathan_memory (#67)
* Added support for QNX 7.1 build (#65)


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`geometry_msgs <https://github.com/ros2/common_interfaces/tree/jazzy/geometry_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Remove references to index.ros.org. (`#244 <https://github.com/ros2/common_interfaces/issues/244>`__)
* Create new messages with all fields needed to define a velocity and transform it  (`#240 <https://github.com/ros2/common_interfaces/issues/240>`__) Co-authored-by: Dr. Denis <denis@stoglrobotics.de> Co-authored-by: Addisu Z. Taddese <addisuzt@intrinsic.ai> Co-authored-by: Tully Foote <tullyfoote@intrinsic.ai>
* Clarify the license. (`#241 <https://github.com/ros2/common_interfaces/issues/241>`__) In particular, every package in this repository is Apache 2.0 licensed except for sensor_msgs_py.  So move the CONTRIBUTING.md and LICENSE files down into the individual packages, and make sure that sensor_msgs_py has the correct CONTRIBUTING.md file (it already had the correct LICENSE file).
* adding IDs to geometry_msgs/Polygon, PolygonStamped (`#232 <https://github.com/ros2/common_interfaces/issues/232>`__)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Steve Macenski


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`google_benchmark_vendor <https://github.com/ament/google_benchmark_vendor/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update to 1.8.3. (`#29 <https://github.com/ament/google_benchmark_vendor/issues/29>`__)
* Contributors: Marco A. Gutierrez


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`gz_cmake_vendor <https://github.com/gazebo-release/gz_cmake_vendor/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update vendored version to 3.5.3
* Use an alias target for root library
* Add support for the ``<pkg>::<pkg>`` and ``<pkg>::all`` targets, fix sourcing of dsv files
* Update vendored version to 3.5.2
* Update vendored package version
* Patch the pkg-config directory in the gz-cmake code. (`#4 <https://github.com/gazebo-release/gz_cmake_vendor/issues/4>`__) * Patch the pkg-config directory in the gz-cmake code. When building on the ROS 2 buildfarm, we aren't setting some of the CMAKE_PREFIX variables.  This means that using CMAKE_INSTALL_FULL_LIBDIR actually creates a path like /opt/ros/rolling/... , which makes debuild upset. However, we actually need the FULL_LIBDIR in order to calculate the relative path between it and the INSTALL_PREFIX. Work around this by having two variables; the pkgconfig_install_dir (relative), used to install the files, and pkgconfig_abs_install_dir (absolute), used to calculate the relative path between them. This should fix the build on the buildfarm.  I'll note that we are doing it here and not in gz-cmake proper because of knock-on effects to downstream gazebo.  If this is successful we may end up merging it there, at which point we can drop this patch. * Update GzPackage as well. ---------
* Require calling find_package on the underlying package (`#3 <https://github.com/gazebo-release/gz_cmake_vendor/issues/3>`__) This also changes the version of the vendor package to 0.0.1 and adds the version of the vendored package in the description
* Fix linter (`#2 <https://github.com/gazebo-release/gz_cmake_vendor/issues/2>`__)
* Use ``<depend>`` on upstream package so that dependency is exported
* Update maintainer
* Add package.xml and CMakeLists (`#1 <https://github.com/gazebo-release/gz_cmake_vendor/issues/1>`__)
* Initial import
* Contributors: Addisu Z. Taddese, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`gz_math_vendor <https://github.com/gazebo-release/gz_math_vendor/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use an alias target for root library
* Add support for the ``<pkg>::<pkg>`` and ``<pkg>::all`` targets, fix sourcing of dsv files
* Disable SWIG to fix CMake warning
* Disable pybind11 for now
* Require calling find_package on the underlying package (`#2 <https://github.com/gazebo-release/gz_math_vendor/issues/2>`__)
* Fix linter (`#1 <https://github.com/gazebo-release/gz_math_vendor/issues/1>`__)
* Remove Nate
* Update maintainers
* Initial import
* Contributors: Addisu Z. Taddese


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`gz_utils_vendor <https://github.com/gazebo-release/gz_utils_vendor/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use an alias target for root library
* Add support for the ``<pkg>::<pkg>`` and ``<pkg>::all`` targets, fix sourcing of dsv files
* Require calling find_package on the underlying package (`#2 <https://github.com/gazebo-release/gz_utils_vendor/issues/2>`__)
* Fix linter (`#1 <https://github.com/gazebo-release/gz_utils_vendor/issues/1>`__)
* Initial import
* Contributors: Addisu Z. Taddese


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`image_tools <https://github.com/ros2/demos/tree/jazzy/image_tools/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* A few uncrustify fixes for 0.78. (`#667 <https://github.com/ros2/demos/issues/667>`__)
* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`__)
* Migrate std::bind calls to lambda expressions (`#659 <https://github.com/ros2/demos/issues/659>`__)
* Contributors: Chris Lalancette, Felipe Gomes de Melo, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`image_transport <https://github.com/ros-perception/image_common/tree/jazzy/image_transport/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Added rclcpp component to Republish (`#275 <https://github.com/ros-perception/image_common/issues/275>`__)
* Add QoS option reliability to republisher qos params (`#296 <https://github.com/ros-perception/image_common/issues/296>`__)
* implement CameraSubscriber::getNumPublishers (`#297 <https://github.com/ros-perception/image_common/issues/297>`__)
* Add missing definition for CameraPublisher::publish overload (`#278 <https://github.com/ros-perception/image_common/issues/278>`__)
* Advertize and subscribe with custom qos (`#288 <https://github.com/ros-perception/image_common/issues/288>`__)
* Removed C headers (`#289 <https://github.com/ros-perception/image_common/issues/289>`__)
* Switch to using the override keyword for simple_publisher_plugin. (`#285 <https://github.com/ros-perception/image_common/issues/285>`__)
* feat: enable plugin allowlist (`#264 <https://github.com/ros-perception/image_common/issues/264>`__)
* Expose option to set callback groups (`#274 <https://github.com/ros-perception/image_common/issues/274>`__)
* add support for lazy subscribers (`#272 <https://github.com/ros-perception/image_common/issues/272>`__)
* Contributors: Aditya Pande, Alejandro Hernández Cordero, Carlos Andrés Álvarez Restrepo, Chris Lalancette, Daisuke Nishimatsu, Michael Ferguson, s-hall


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`interactive_markers <https://github.com/ros-visualization/interactive_markers/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Shorten the length of a lambda. (`#106 <https://github.com/ros-visualization/interactive_markers/issues/106>`__)
* Fixed C++20 warning that ‘++’ expression of ‘volatile’-qualified type is deprecated (`#102 <https://github.com/ros-visualization/interactive_markers/issues/102>`__)
* Cleanup of interactive markers (`#101 <https://github.com/ros-visualization/interactive_markers/issues/101>`__)
* Contributors: AiVerisimilitude, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`intra_process_demo <https://github.com/ros2/demos/tree/jazzy/intra_process_demo/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`__)
* Migrate std::bind calls to lambda expressions (`#659 <https://github.com/ros2/demos/issues/659>`__)
* Fix executable name in README (`#618 <https://github.com/ros2/demos/issues/618>`__)
* Contributors: Felipe Gomes de Melo, Michael Jeronimo, Yadunund


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`kdl_parser <https://github.com/ros/kdl_parser/tree/jazzy/kdl_parser/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update to C++17. (`#82 <https://github.com/ros/kdl_parser/issues/82>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`keyboard_handler <https://github.com/ros-tooling/keyboard_handler/tree/jazzy/keyboard_handler/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Shorten lambdas so newer uncrustify is happier. (`#42 <https://github.com/ros-tooling/keyboard_handler/issues/42>`__)
* Fixed C++20 warning implicit capture of this in lambda (`#41 <https://github.com/ros-tooling/keyboard_handler/issues/41>`__)
* Update to C++17. (`#37 <https://github.com/ros-tooling/keyboard_handler/issues/37>`__)
* Contributors: AiVerisimilitude, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`laser_geometry <https://github.com/ros-perception/laser_geometry/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Switch to target_link_libraries. (`#92 <https://github.com/ros-perception/laser_geometry/issues/92>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch <https://github.com/ros2/launch/tree/jazzy/launch/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* (launch) fix describe of PathJoinSubstitution (`#771 <https://github.com/ros2/launch/issues/771>`__)
* Small fixes for modern flake8. (`#772 <https://github.com/ros2/launch/issues/772>`__)
* Cleanup some type annotations.
* Rework task exceptions loop. (`#755 <https://github.com/ros2/launch/issues/755>`__)
* add format overriding by environment variables (`#722 <https://github.com/ros2/launch/issues/722>`__)
* Add exception type to error output (`#753 <https://github.com/ros2/launch/issues/753>`__)
* Let XML executables/nodes be "required" (like in ROS 1) (`#751 <https://github.com/ros2/launch/issues/751>`__)
* Add conditional substitution (`#734 <https://github.com/ros2/launch/issues/734>`__)
* Add maximum times for a process to respawn (`#696 <https://github.com/ros2/launch/issues/696>`__)
* Add in a timeout for launch pytests. (`#725 <https://github.com/ros2/launch/issues/725>`__)
* Fix remaining occurrences of "There is no current event loop" (`#723 <https://github.com/ros2/launch/issues/723>`__)
* Update the launch code for newer flake8 and mypy. (`#719 <https://github.com/ros2/launch/issues/719>`__)
* Remove the deprecated some_actions_type.py (`#718 <https://github.com/ros2/launch/issues/718>`__)
* Improve launch file parsing error messages (`#626 <https://github.com/ros2/launch/issues/626>`__)
* Add file-content launch substitution (`#708 <https://github.com/ros2/launch/issues/708>`__)
* Contributors: Chris Lalancette, David Yackzan, Marc Bestmann, Matthew Elwin, Matthijs van der Burgh, Nick Lamprianidis, Santti4go, Scott K Logan, Timon Engelke


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch_pytest <https://github.com/ros2/launch/tree/jazzy/launch_pytest/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Switch tryfirst/trylast to hookimpl.
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch_ros <https://github.com/ros2/launch_ros/tree/jazzy/launch_ros/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix: typing. Iterable doesn't have __getitem_\_ (`#393 <https://github.com/ros2/launch_ros/issues/393>`__)
* Cleanup some type annotations. (`#392 <https://github.com/ros2/launch_ros/issues/392>`__)
* Create py.typed to mark this library as typed (`#379 <https://github.com/ros2/launch_ros/issues/379>`__)
* Remove create_future implementation. (`#372 <https://github.com/ros2/launch_ros/issues/372>`__)
* cache lookup of importlib metadata in Node action (`#365 <https://github.com/ros2/launch_ros/issues/365>`__)
* Get rid of unnecessary checks in composable_node_container. (`#364 <https://github.com/ros2/launch_ros/issues/364>`__)
* Contributors: Chris Lalancette, Jonas Otto, Matthijs van der Burgh, William Woodall


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch_testing <https://github.com/ros2/launch/tree/jazzy/launch_testing/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix a warning in modern unittest. (`#773 <https://github.com/ros2/launch/issues/773>`__) Newer versions of unittest no longer store an errors list; instead, they store a result, which then stores an error list.  Update the code here to be able to deal with either version.
* Add consider_namespace_packages=False (`#766 <https://github.com/ros2/launch/issues/766>`__)
* to open expected outpout file with an encoding parameter (`#717 <https://github.com/ros2/launch/issues/717>`__)
* Contributors: Chen Lihui, Chris Lalancette, Tony Najjar


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch_testing_examples <https://github.com/ros2/examples/tree/jazzy/launch_testing/launch_testing_examples/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Cleanup the launch_testing_examples. (`#374 <https://github.com/ros2/examples/issues/374>`__)
* Refactor WaitForNodes class. (`#373 <https://github.com/ros2/examples/issues/373>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch_testing_ros <https://github.com/ros2/launch_ros/tree/jazzy/launch_testing_ros/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Make launch_testing_ros examples more robust. (`#394 <https://github.com/ros2/launch_ros/issues/394>`__)
* added type hinting to launch_testing_ros/test/examples (`#386 <https://github.com/ros2/launch_ros/issues/386>`__)
* Handle spin() ExternalShutdownException. (`#378 <https://github.com/ros2/launch_ros/issues/378>`__)
* Increase the timeout in wait_for_topic_launch_test. (`#377 <https://github.com/ros2/launch_ros/issues/377>`__)
* ``WaitForTopics``: get content of messages for each topic (`#353 <https://github.com/ros2/launch_ros/issues/353>`__)
* Contributors: Chris Lalancette, Giorgio Pintaudi, Yaswanth


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch_xml <https://github.com/ros2/launch/tree/jazzy/launch_xml/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* launch_xml: fix xml syntax in README (`#770 <https://github.com/ros2/launch/issues/770>`__)
* Let XML executables/nodes be "required" (like in ROS 1) (`#751 <https://github.com/ros2/launch/issues/751>`__) * Let XML nodes be "required" Essentially on_exit="shutdown" is equivalent to ROS 1 required="true". This feature is implemented using the python launchfile on_exit mechanism. Right now "shutdown" is the only action accepted by on_exit, but theoretically more "on_exit" actions could be added later. Example: <executable cmd="ls" on_exit="shutdown"/> * Added tests for yaml
* Improve launch file parsing error messages (`#626 <https://github.com/ros2/launch/issues/626>`__)
* Contributors: Matthew Elwin, Steve Peters, Timon Engelke


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch_yaml <https://github.com/ros2/launch/tree/jazzy/launch_yaml/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix flake8 warnings in launch_yaml. (`#756 <https://github.com/ros2/launch/issues/756>`__)
* Let XML executables/nodes be "required" (like in ROS 1) (`#751 <https://github.com/ros2/launch/issues/751>`__) * Let XML nodes be "required" Essentially on_exit="shutdown" is equivalent to ROS 1 required="true". This feature is implemented using the python launchfile on_exit mechanism. Right now "shutdown" is the only action accepted by on_exit, but theoretically more "on_exit" actions could be added later. Example: <executable cmd="ls" on_exit="shutdown"/> * Added tests for yaml
* Improve launch file parsing error messages (`#626 <https://github.com/ros2/launch/issues/626>`__)
* Contributors: Chris Lalancette, Matthew Elwin, Timon Engelke


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`libcurl_vendor <https://github.com/ros/resource_retriever/tree/jazzy/libcurl_vendor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add "lib" to the Windows curl search path. (`#96 <https://github.com/ros/resource_retriever/issues/96>`__) (`#97 <https://github.com/ros/resource_retriever/issues/97>`__) In CMake 3.3, a commit made it so that the find_package module in CMake had a compatibility mode where it would automatically search for packages in a <prefix>/lib subdirectory. In CMake 3.6, this compatibility mode was reverted for all platforms *except* Windows. That means that since CMake 3.3, we haven't actually been using the path as specified in ``curl_DIR``, but we have instead been inadvertently relying on that fallback behavior. In CMake 3.28, that compatibilty mode was also removed for Windows, meaning that we are now failing to find_package(curl) in downstream packages (like resource_retriever). Fix this by adding in the "lib" directory that always should have been there.  I'll note that this *only* affects our Windows builds, because this code is in a if(WIN32) block. (cherry picked from commit 1839d583190eb9dcf339eaaf6bebe632d94664a6) Co-authored-by: Chris Lalancette <clalancette@gmail.com>
* Switch to ament_cmake_vendor_package (`#86 <https://github.com/ros/resource_retriever/issues/86>`__)
* Contributors: Scott K Logan, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`liblz4_vendor <https://github.com/ros2/rosbag2/tree/jazzy/liblz4_vendor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Make sure to build_export_depend liblz4-dev. (`#1614 <https://github.com/ros2/rosbag2/issues/1614>`__)
* Switch to using ament_vendor_package for lz4. (`#1583 <https://github.com/ros2/rosbag2/issues/1583>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`libstatistics_collector <https://github.com/ros-tooling/libstatistics_collector/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Bump pascalgn/automerge-action from 0.16.2 to 0.16.3
* Bump codecov/codecov-action from 4.1.1 to 4.2.0
* Fixes for newer uncrustify. (`#186 <https://github.com/ros-tooling/libstatistics_collector/issues/186>`__)
* Bump actions/upload-artifact from 3 to 4
* Switch to using target_link_libraries everywhere. (`#174 <https://github.com/ros-tooling/libstatistics_collector/issues/174>`__)
* Bump rolling to 1.6.3 (`#173 <https://github.com/ros-tooling/libstatistics_collector/issues/173>`__)
* Bump actions/checkout from 3 to 4 (`#169 <https://github.com/ros-tooling/libstatistics_collector/issues/169>`__)
* Add API to use message_info instead unserialized message (`#170 <https://github.com/ros-tooling/libstatistics_collector/issues/170>`__)
* Bump codecov/codecov-action from 3.1.3 to 3.1.4
* Bump actions/checkout from 3 to 4 (`#169 <https://github.com/ros-tooling/libstatistics_collector/issues/169>`__)
* Add API to use message_info instead unserialized message (`#170 <https://github.com/ros-tooling/libstatistics_collector/issues/170>`__)
* Bump codecov/codecov-action from 3.1.3 to 3.1.4
* Add in missing cstdint include. (`#165 <https://github.com/ros-tooling/libstatistics_collector/issues/165>`__)
* Bump codecov/codecov-action from 3.1.2 to 3.1.3
* Contributors: Chris Lalancette, Lucas Wendland, Michael Orlov, dependabot[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`libyaml_vendor <https://github.com/ros2/libyaml_vendor/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update quality declaration documents (`#62 <https://github.com/ros2/libyaml_vendor/issues/62>`__)
* remove rcpputils and rcutils dependency (`#61 <https://github.com/ros2/libyaml_vendor/issues/61>`__)
* Set to C++17. (`#59 <https://github.com/ros2/libyaml_vendor/issues/59>`__)
* Switch to ament_cmake_vendor_package (`#58 <https://github.com/ros2/libyaml_vendor/issues/58>`__)
* Contributors: Chris Lalancette, Christophe Bedard, Kenta Yonekura, Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`lifecycle <https://github.com/ros2/demos/tree/jazzy/lifecycle/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* A few uncrustify fixes for 0.78. (`#667 <https://github.com/ros2/demos/issues/667>`__)
* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`__)
* Migrate std::bind calls to lambda expressions (`#659 <https://github.com/ros2/demos/issues/659>`__)
* Switch to using RCLCPP logging macros in the lifecycle package. (`#644 <https://github.com/ros2/demos/issues/644>`__)
* Contributors: Chris Lalancette, Felipe Gomes de Melo, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`lifecycle_py <https://github.com/ros2/demos/tree/jazzy/lifecycle_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`logging_demo <https://github.com/ros2/demos/tree/jazzy/logging_demo/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`__)
* Migrate std::bind calls to lambda expressions (`#659 <https://github.com/ros2/demos/issues/659>`__)
* Contributors: Felipe Gomes de Melo, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`lttngpy <https://github.com/ros2/ros2_tracing/tree/jazzy/lttngpy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Replace all occurences of index.ros.org (`#114 <https://github.com/ros2/ros2_tracing/issues/114>`__)
* Fixes for newer uncrustify (`#101 <https://github.com/ros2/ros2_tracing/issues/101>`__)
* Fix Python not being found for lttngpy in Windows debug mode (`#87 <https://github.com/ros2/ros2_tracing/issues/87>`__)
* Switch to custom lttng-ctl Python bindings (`#81 <https://github.com/ros2/ros2_tracing/issues/81>`__)
* Contributors: Chris Lalancette, Christophe Bedard


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`map_msgs <https://github.com/ros-planning/navigation_msgs/tree/jazzy/map_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files
* Update to C++17
* Contributors: Chris Lalancette, Michael Jeronimo, Steve Macenski


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`mcap_vendor <https://github.com/ros2/rosbag2/tree/jazzy/mcap_vendor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Switch to using ament_vendor_package for lz4. (`#1583 <https://github.com/ros2/rosbag2/issues/1583>`__)
* Switch to target_link_libraries everywhere. (`#1504 <https://github.com/ros2/rosbag2/issues/1504>`__)
* Update mcap to v1.1.0 (`#1361 <https://github.com/ros2/rosbag2/issues/1361>`__)
* Contributors: Chris Lalancette, Emerson Knapp


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`message_filters <https://github.com/ros2/message_filters/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update TimeSynchronizer usage example. (`#115 <https://github.com/ros2/message_filters/issues/115>`__)
* Remove 'using' keyword in message_filters (`#106 <https://github.com/ros2/message_filters/issues/106>`__)
* Remove the use of ament_target_dependencies. (`#105 <https://github.com/ros2/message_filters/issues/105>`__)
* Fixes pointed out by clang (`#104 <https://github.com/ros2/message_filters/issues/104>`__)
* Mark subscription cb parameter const (`#103 <https://github.com/ros2/message_filters/issues/103>`__)
* Update the HasHeader check to be more specific. (`#101 <https://github.com/ros2/message_filters/issues/101>`__)
* TypeAdapters support (`#95 <https://github.com/ros2/message_filters/issues/95>`__) (`#96 <https://github.com/ros2/message_filters/issues/96>`__)
* Cleanup a few minor things in the filters. (`#100 <https://github.com/ros2/message_filters/issues/100>`__)
* Fix python examples (`#99 <https://github.com/ros2/message_filters/issues/99>`__)
* feat: add signal time functions to ExactTime policy (`#94 <https://github.com/ros2/message_filters/issues/94>`__)
* Contributors: Chris Lalancette, Patrick Roncagliolo, Ricardo de Azambuja, Russ, rkeating-planted


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`mimick_vendor <https://github.com/ros2/mimick_vendor/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Bump vendored mimick version for `ros2/Mimick#32 <https://github.com/ros2/Mimick/issues/32>`__ (`#35 <https://github.com/ros2/mimick_vendor/issues/35>`__)
* Update to the commit that fixes mmk_noreturn. (`#34 <https://github.com/ros2/mimick_vendor/issues/34>`__)
* Update to the commit the fixes exe stack on macOS. (`#33 <https://github.com/ros2/mimick_vendor/issues/33>`__)
* Update to the comment that fixes the executable stack. (`#32 <https://github.com/ros2/mimick_vendor/issues/32>`__)
* Update to take advantage of TARGET_ARCH (`#28 <https://github.com/ros2/mimick_vendor/issues/28>`__)
* Switch to ament_cmake_vendor_package (`#31 <https://github.com/ros2/mimick_vendor/issues/31>`__)
* Contributors: Chris Lalancette, Michael Carroll, Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`nav_msgs <https://github.com/ros2/common_interfaces/tree/jazzy/nav_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Removed TODO (`#243 <https://github.com/ros2/common_interfaces/issues/243>`__)
* Clarify the license. (`#241 <https://github.com/ros2/common_interfaces/issues/241>`__) In particular, every package in this repository is Apache 2.0 licensed except for sensor_msgs_py.  So move the CONTRIBUTING.md and LICENSE files down into the individual packages, and make sure that sensor_msgs_py has the correct CONTRIBUTING.md file (it already had the correct LICENSE file).
* Contributors: Alejandro Hernández Cordero, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`orocos_kdl_vendor <https://github.com/ros2/orocos_kdl_vendor/tree/jazzy/orocos_kdl_vendor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Ensure that orocos_kdl_vendor doesn't accidentally find itself. (`#27 <https://github.com/ros2/orocos_kdl_vendor/issues/27>`__) (`#28 <https://github.com/ros2/orocos_kdl_vendor/issues/28>`__) When initially building the orocos_kdl_vendor package (on platforms where it actually builds), it turns out that it places a valid cmake configuration in the build directory.  In turn, that means that a subsequent rebuild will find this configuration in the build directory, and throw the rest of the logic off. This only seems to be a problem with CMake 3.29 and later, though I can't say exactly why at the moment. Workaround this problem by writing the configuration out to a temporary file, and then moving it into the final place with the final name. (cherry picked from commit 7aad6d1ad9fa54f3a48f1f194a85127e362c8ade) Co-authored-by: Chris Lalancette <clalancette@gmail.com>
* Update to the latest orocos_kdl_kinematics commit. (`#25 <https://github.com/ros2/orocos_kdl_vendor/issues/25>`__)
* Switch to ament_cmake_vendor_package (`#20 <https://github.com/ros2/orocos_kdl_vendor/issues/20>`__)
* Contributors: Chris Lalancette, Scott K Logan, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`osrf_pycommon <https://github.com/osrf/osrf_pycommon/tree/master/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Catch all of the spurious warnings from get_event_loop. (`#94 <https://github.com/osrf/osrf_pycommon/issues/94>`__)
* Add bookworm as a python3 target (`#91 <https://github.com/osrf/osrf_pycommon/issues/91>`__)
* Suppress warning for specifically handled behavior (`#87 <https://github.com/osrf/osrf_pycommon/issues/87>`__)
* Update supported platforms (`#93 <https://github.com/osrf/osrf_pycommon/issues/93>`__)
* Add GitHub Actions CI workflow (`#88 <https://github.com/osrf/osrf_pycommon/issues/88>`__)
* Contributors: Chris Lalancette, Scott K Logan, Tully Foote


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`osrf_testing_tools_cpp <https://github.com/osrf/osrf_testing_tools_cpp/tree/jazzy/osrf_testing_tools_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Upgrade to Google test 1.14.0 (`#84 <https://github.com/osrf/osrf_testing_tools_cpp/issues/84>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`pendulum_control <https://github.com/ros2/demos/tree/jazzy/pendulum_control/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`__)
* [pendulum_control Install targets to project lib (`#624 <https://github.com/ros2/demos/issues/624>`__)
* Contributors: Michael Jeronimo, Yadu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`pendulum_msgs <https://github.com/ros2/demos/tree/jazzy/pendulum_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`pluginlib <https://github.com/ros/pluginlib/tree/jazzy/pluginlib/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Switch from rcpputils::fs to std::filesystem (`#254 <https://github.com/ros/pluginlib/issues/254>`__)
* Remove redundant throw of a std::runtime_error (`#232 <https://github.com/ros/pluginlib/issues/232>`__)
* Update to C++17 (`#251 <https://github.com/ros/pluginlib/issues/251>`__)
* Fix wShadow compile warning (`#250 <https://github.com/ros/pluginlib/issues/250>`__)
* Contributors: Chris Lalancette, Christophe Bedard, Hunter L. Allen, Steve Macenski


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`pybind11_vendor <https://github.com/ros2/pybind11_vendor/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update to pybind11 2.11.1 (`#28 <https://github.com/ros2/pybind11_vendor/issues/28>`__)
* Add Apache 2.0 LICENSE file (`#27 <https://github.com/ros2/pybind11_vendor/issues/27>`__)
* Switch to ament_cmake_vendor_package (`#24 <https://github.com/ros2/pybind11_vendor/issues/24>`__)
* Contributors: Chris Lalancette, Michael Carroll, Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`python_cmake_module <https://github.com/ros2/python_cmake_module/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use FindPython3 instead of FindPythonInterp (`#7 <https://github.com/ros2/python_cmake_module/issues/7>`__)
* Contributors: Shane Loretz


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`python_orocos_kdl_vendor <https://github.com/ros2/orocos_kdl_vendor/tree/jazzy/python_orocos_kdl_vendor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update to the latest orocos_kdl_kinematics commit. (`#25 <https://github.com/ros2/orocos_kdl_vendor/issues/25>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`python_qt_binding <https://github.com/ros-visualization/python_qt_binding/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Suppress warning from Shiboken2. (backport `#137 <https://github.com/ros-visualization/python_qt_binding/issues/137>`__) (`#138 <https://github.com/ros-visualization/python_qt_binding/issues/138>`__) Co-authored-by: Chris Lalancette <clalancette@gmail.com> Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Switch to C++17 for SIP and Shiboken (`#135 <https://github.com/ros-visualization/python_qt_binding/issues/135>`__)
* Set hints to find the python version we actually want. (`#134 <https://github.com/ros-visualization/python_qt_binding/issues/134>`__)
* Remove unnecessary parentheses around assert. (`#133 <https://github.com/ros-visualization/python_qt_binding/issues/133>`__)
* Switch to FindPython3 in the shiboken_helper.cmake. (`#132 <https://github.com/ros-visualization/python_qt_binding/issues/132>`__)
* Cleanup of the sip_configure.py file. (`#131 <https://github.com/ros-visualization/python_qt_binding/issues/131>`__)
* Update the SIP support so we can deal with a broken RHEL-9. (`#129 <https://github.com/ros-visualization/python_qt_binding/issues/129>`__)
* Contributors: Chris Lalancette, Christophe Bedard, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`qt_dotgraph <https://github.com/ros-visualization/qt_gui_core/tree/jazzy/qt_dotgraph/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Handle empty dotcode nodes. (`#290 <https://github.com/ros-visualization/qt_gui_core/issues/290>`__)
* Small fix for modern flake8. (`#289 <https://github.com/ros-visualization/qt_gui_core/issues/289>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`qt_gui <https://github.com/ros-visualization/qt_gui_core/tree/jazzy/qt_gui/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Remove unnecessary parentheses for assert. (`#286 <https://github.com/ros-visualization/qt_gui_core/issues/286>`__)
* (qt_gui) extended theme logic to get icons (`#279 <https://github.com/ros-visualization/qt_gui_core/issues/279>`__)
* Contributors: Chris Lalancette, Matthijs van der Burgh


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`qt_gui_cpp <https://github.com/ros-visualization/qt_gui_core/tree/jazzy/qt_gui_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Switch from rcpputils::fs to std::filesystem (`#288 <https://github.com/ros-visualization/qt_gui_core/issues/288>`__)
* Set hints to find the python version we actually want. (`#287 <https://github.com/ros-visualization/qt_gui_core/issues/287>`__)
* Update to C++17 (`#278 <https://github.com/ros-visualization/qt_gui_core/issues/278>`__)
* fix unload warning (`#274 <https://github.com/ros-visualization/qt_gui_core/issues/274>`__)
* Contributors: Chen Lihui, Chris Lalancette, Christophe Bedard


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`quality_of_service_demo_cpp <https://github.com/ros2/demos/tree/jazzy/quality_of_service_demo/rclcpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`__)
* Explicit time conversion (`#657 <https://github.com/ros2/demos/issues/657>`__)
* Cleanup the interactive quality of service demos. (`#637 <https://github.com/ros2/demos/issues/637>`__)
* More quality of service demo cleanup (`#632 <https://github.com/ros2/demos/issues/632>`__)
* Fix small typos in the incompatible_qos demos. (`#629 <https://github.com/ros2/demos/issues/629>`__)
* Contributors: AiVerisimilitude, Chris Lalancette, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`quality_of_service_demo_py <https://github.com/ros2/demos/tree/jazzy/quality_of_service_demo/rclpy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`__)
* More quality of service demo cleanup (`#632 <https://github.com/ros2/demos/issues/632>`__)
* Fix small typos in the incompatible_qos demos. (`#629 <https://github.com/ros2/demos/issues/629>`__)
* Fix the quality_of_service_demo_py output to look like the C++ one. (`#626 <https://github.com/ros2/demos/issues/626>`__)
* Use non-deprecated rclpy import. (`#615 <https://github.com/ros2/demos/issues/615>`__)
* Contributors: Chris Lalancette, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl <https://github.com/ros2/rcl/tree/jazzy/rcl/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix up rmw_cyclonedds timestamp testing. (`#1156 <https://github.com/ros2/rcl/issues/1156>`__) (`#1157 <https://github.com/ros2/rcl/issues/1157>`__) We are about to fix it so that rmw_cyclonedds has receive_timestamp support, so we also need to enable that support here in rcl.  We actually rewrite the logic a bit because now the only combination that doesn't work is rmw_connextdds on Windows. (cherry picked from commit 6d53d24a863c3e9e4a41e9fe5f550271210d9d9d) Co-authored-by: Chris Lalancette <clalancette@gmail.com>
* Fixed warnings - strict-prototypes (`#1148 <https://github.com/ros2/rcl/issues/1148>`__) (`#1150 <https://github.com/ros2/rcl/issues/1150>`__)
* chore: Minor style improvements (`#1147 <https://github.com/ros2/rcl/issues/1147>`__) Co-authored-by: Janosch Machowinski <J.Machowinski@cellumation.com>
* improved rcl_wait in the area of timeout computation and spurious wakeups (`#1146 <https://github.com/ros2/rcl/issues/1146>`__) Added special handling for timers with a clock that has time override enabled. For these timer we should not compute a timeout, as the waitset is waken up by the associated guard condition. Before this change, the waitset could wait up, because of an expected ready timer, that was acutally not ready, as the time update to the ROS_TIME had not yet arrived.
* Add tracepoint for publish_serialized_publish (`#1136 <https://github.com/ros2/rcl/issues/1136>`__) * Add tracepoint for publish_serialized_publish * Add: tracepoint for rcl_take_serialized_message ---------
* Revert "improved rcl_wait in the area of timeout computation and spurious wakeups (`#1135 <https://github.com/ros2/rcl/issues/1135>`__)" (`#1142 <https://github.com/ros2/rcl/issues/1142>`__) This reverts commit 3c6c5dc47dac23d70722a60b2c0a387d2e71b71d.
* improved rcl_wait in the area of timeout computation and spurious wakeups (`#1135 <https://github.com/ros2/rcl/issues/1135>`__) * feat: Allow usage of rcl_timer_clock with const rcl_timer_t* * fix: Fixed purious wake-ups on ROS_TIME timers with ROS_TIME enabled Added special handling for timers with a clock that has time override enabled. For theses timer we should not compute a timeout, as the waitset is waken up by the associated guard condition. Before this change, the waitset could wait up, because of an expected ready timer, that was acutally not ready, as the time update to the ROS_TIME had not yet arrived. * feat: Added rcl_timer_get_next_call_time * fix(rcl_wait): Improved timeout computation in case of many timers This commit changes the computation of the timer timeout, to be more precise, in the case, of many registered timers. --------- Co-authored-by: Janosch Machowinski <j.machowinski@nospam.org>
* Generate version header using ament_generate_version_header(..) (`#1141 <https://github.com/ros2/rcl/issues/1141>`__)
* Add rcl_timer_call_with_info function that retrieves the expected and the actual timer trigger times (`#1113 <https://github.com/ros2/rcl/issues/1113>`__) Co-authored-by: Alexis Tsogias <a.tsogias@cellumation.com> Co-authored-by: Michael Carroll <carroll.michael@gmail.com> Co-authored-by: Tomoya Fujita <Tomoya.Fujita@sony.com>
* document out parameters for rcl_get_node_names and rcl_get_node_names_with_enclaves (`#1137 <https://github.com/ros2/rcl/issues/1137>`__) * document out params for rcl_get_node_names Co-authored-by: Chris Lalancette <clalancette@gmail.com>
* Cleanups for uncrustify 0.78. (`#1134 <https://github.com/ros2/rcl/issues/1134>`__) Mostly this is expanding macros, as this is just easier to read anyway.  But we also mark one section as INDENT-OFF.
* Re-order rcl_logging_interface include (`#1133 <https://github.com/ros2/rcl/issues/1133>`__)
* Remove unnecessary macros. (`#1132 <https://github.com/ros2/rcl/issues/1132>`__) These really don't add anything, and allows us to avoid some changes in macro formatting between Ubuntu 22.04 and Ubuntu 24.04.
* Update quality declaration documents (`#1131 <https://github.com/ros2/rcl/issues/1131>`__)
* add unit tests for --log-file-name argument. (`#1130 <https://github.com/ros2/rcl/issues/1130>`__)
* support ``--log-file-name`` to ros args. (`#1127 <https://github.com/ros2/rcl/issues/1127>`__)
* Make sure to disable a test_node test on RHEL. (`#1124 <https://github.com/ros2/rcl/issues/1124>`__)
* remove static function rcl_ret_from_rcutils_ret(). (`#1122 <https://github.com/ros2/rcl/issues/1122>`__)
* Remove AMENT_DEPENDENCIES from rcl_add_custom_gtest. (`#1119 <https://github.com/ros2/rcl/issues/1119>`__)
* Remove unncecessary dependencies in tests (`#1114 <https://github.com/ros2/rcl/issues/1114>`__)
* a rosout publisher of a node might not exist (`#1115 <https://github.com/ros2/rcl/issues/1115>`__)
* Set disable loan to on by default. (`#1110 <https://github.com/ros2/rcl/issues/1110>`__)
* Return service from node_type_description_service_init (`#1112 <https://github.com/ros2/rcl/issues/1112>`__)
* next_call_time will always be greater than now after calling rcl_timer_call. (`#1089 <https://github.com/ros2/rcl/issues/1089>`__)
* Add rcl count clients, servicec & tests (`#1011 <https://github.com/ros2/rcl/issues/1011>`__)
* Improve the reliability of test_get_type_description_service. (`#1107 <https://github.com/ros2/rcl/issues/1107>`__)
* Remove most remaining uses of ament_target_dependencies. (`#1102 <https://github.com/ros2/rcl/issues/1102>`__)
* Just remove rcpputils::fs dependency (`#1105 <https://github.com/ros2/rcl/issues/1105>`__)
* Decouple rosout publisher init from node init. (`#1065 <https://github.com/ros2/rcl/issues/1065>`__)
* Cleanup the error handling in rcl_node_init. (`#1099 <https://github.com/ros2/rcl/issues/1099>`__)
* Fix a clang warning for suspicious string concatentation. (`#1101 <https://github.com/ros2/rcl/issues/1101>`__)
* add the link to the topic name rules. (`#1100 <https://github.com/ros2/rcl/issues/1100>`__)
* Cut down the amount of time for test_logging_rosout. (`#1098 <https://github.com/ros2/rcl/issues/1098>`__)
* Simplify local_namespace handling in rcl_node_init. (`#1097 <https://github.com/ros2/rcl/issues/1097>`__)
* Reduce the number of tests we run (`#1096 <https://github.com/ros2/rcl/issues/1096>`__)
* Adding duplicate node information (`#1088 <https://github.com/ros2/rcl/issues/1088>`__)
* Revamp the test_get_type_description_service. (`#1095 <https://github.com/ros2/rcl/issues/1095>`__)
* Cleanup network flow endpoints test. (`#1094 <https://github.com/ros2/rcl/issues/1094>`__)
* Reduce the failure timeout time for namespaces. (`#1093 <https://github.com/ros2/rcl/issues/1093>`__)
* Shorten wait time for a subscription not being ready. (`#1092 <https://github.com/ros2/rcl/issues/1092>`__)
* rcl_send_response returns RCL_RET_TIMEOUT. (`#1048 <https://github.com/ros2/rcl/issues/1048>`__)
* Move test_namespace into the correct directory. (`#1087 <https://github.com/ros2/rcl/issues/1087>`__)
* Reset errors in tests to reduce warnings (`#1085 <https://github.com/ros2/rcl/issues/1085>`__)
* Cleanup error reporting in the type hash code. (`#1084 <https://github.com/ros2/rcl/issues/1084>`__)
* Instrument loaned message publication code path (`#1083 <https://github.com/ros2/rcl/issues/1083>`__)
* Add ``~/get_type_description`` service (rep2011) (`#1052 <https://github.com/ros2/rcl/issues/1052>`__)
* Modifies timers API to select autostart state (`#1004 <https://github.com/ros2/rcl/issues/1004>`__)
* test publisher/subscription with the c/cpp typesupport for test_msgs::msg::array (`#1074 <https://github.com/ros2/rcl/issues/1074>`__)
* validation result should be used to print the error message. (`#1077 <https://github.com/ros2/rcl/issues/1077>`__)
* improve error msg of ``rcl_expand_topic_name`` (`#1076 <https://github.com/ros2/rcl/issues/1076>`__)
* Use TRACETOOLS\_ prefix for tracepoint-related macros (`#1058 <https://github.com/ros2/rcl/issues/1058>`__)
* fix comment (`#1073 <https://github.com/ros2/rcl/issues/1073>`__)
* localhost_only prevails auto discovery options if enabled. (`#1069 <https://github.com/ros2/rcl/issues/1069>`__)
* Avoid dynamic allocation of message before sending over rosout (`#1067 <https://github.com/ros2/rcl/issues/1067>`__)
* clarify ``rcl_node_init`` return code (`#1066 <https://github.com/ros2/rcl/issues/1066>`__)
* Fix a format-security warning when building with clang. (`#1064 <https://github.com/ros2/rcl/issues/1064>`__)
* Contributors: Chen Lihui, Chris Lalancette, Christophe Bedard, Christopher Wecht, Eloy Briceno, Eric W, Felix Penzlin, G.A. vd. Hoorn, Hans-Joachim Krauch, Kenta Yonekura, Lee, Lucas Wendland, Michael Carroll, Minju, Thiemo Kohrt, Tomoya Fujita, h-suzuki-isp, jmachowinski, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl_action <https://github.com/ros2/rcl/tree/jazzy/rcl_action/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Generate version header using ament_generate_version_header(..) (`#1141 <https://github.com/ros2/rcl/issues/1141>`__)
* add RCL_RET_TIMEOUT to action service response. (`#1138 <https://github.com/ros2/rcl/issues/1138>`__) * add RCL_RET_TIMEOUT to action service response. * address review comment. ---------
* Update quality declaration documents (`#1131 <https://github.com/ros2/rcl/issues/1131>`__)
* Remove most remaining uses of ament_target_dependencies. (`#1102 <https://github.com/ros2/rcl/issues/1102>`__)
* Add ``~/get_type_description`` service (rep2011) (`#1052 <https://github.com/ros2/rcl/issues/1052>`__)
* Modifies timers API to select autostart state (`#1004 <https://github.com/ros2/rcl/issues/1004>`__)
* Contributors: Chris Lalancette, Christophe Bedard, Eloy Briceno, G.A. vd. Hoorn, Hans-Joachim Krauch, Tomoya Fujita


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl_interfaces <https://github.com/ros2/rcl_interfaces/tree/jazzy/rcl_interfaces/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the Log.msg constant types. (`#161 <https://github.com/ros2/rcl_interfaces/issues/161>`__)
* Update the comments for SetParametersResult to reflect reality. (`#159 <https://github.com/ros2/rcl_interfaces/issues/159>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl_lifecycle <https://github.com/ros2/rcl/tree/jazzy/rcl_lifecycle/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fixed warnings - strict-prototypes (`#1148 <https://github.com/ros2/rcl/issues/1148>`__) (`#1150 <https://github.com/ros2/rcl/issues/1150>`__)
* Generate version header using ament_generate_version_header(..) (`#1141 <https://github.com/ros2/rcl/issues/1141>`__)
* Update quality declaration documents (`#1131 <https://github.com/ros2/rcl/issues/1131>`__)
* Remove most remaining uses of ament_target_dependencies. (`#1102 <https://github.com/ros2/rcl/issues/1102>`__)
* Use TRACETOOLS\_ prefix for tracepoint-related macros (`#1058 <https://github.com/ros2/rcl/issues/1058>`__)
* Contributors: Chris Lalancette, Christophe Bedard, G.A. vd. Hoorn, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl_logging_interface <https://github.com/ros2/rcl_logging/tree/jazzy/rcl_logging_interface/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Check allocator validity in some rcl_logging functions (`#116 <https://github.com/ros2/rcl_logging/issues/116>`__) If the allocator is zero-initialized, it may cause a segfault when it is used later in the functions.
* Use (void) in declaration of param-less function (`#114 <https://github.com/ros2/rcl_logging/issues/114>`__)
* add file_name_prefix parameter to external log configuration. (`#109 <https://github.com/ros2/rcl_logging/issues/109>`__)
* Migrate to std::filesystem (`#104 <https://github.com/ros2/rcl_logging/issues/104>`__)
* Remove the last uses of ament_target_dependencies in this repo. (`#102 <https://github.com/ros2/rcl_logging/issues/102>`__)
* Contributors: Chris Lalancette, Christophe Bedard, Kenta Yonekura, Scott K Logan, Tomoya Fujita


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl_logging_noop <https://github.com/ros2/rcl_logging/tree/jazzy/rcl_logging_noop/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* add file_name_prefix parameter to external log configuration. (`#109 <https://github.com/ros2/rcl_logging/issues/109>`__)
* Remove the last uses of ament_target_dependencies in this repo. (`#102 <https://github.com/ros2/rcl_logging/issues/102>`__)
* Contributors: Chris Lalancette, Tomoya Fujita


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl_logging_spdlog <https://github.com/ros2/rcl_logging/tree/jazzy/rcl_logging_spdlog/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Check allocator validity in some rcl_logging functions (`#116 <https://github.com/ros2/rcl_logging/issues/116>`__) If the allocator is zero-initialized, it may cause a segfault when it is used later in the functions.
* Cleanup the tests. (`#115 <https://github.com/ros2/rcl_logging/issues/115>`__) * Cleanup the tests. There are a few different fixes in here: 1.  Move away from using "popen" to get the list of files in a directory.  Instead, switch to using the C++ std::filesystem directory iterator and doing the work ourselves, which is portable and much less error-prone. 2.  Set the ROS_LOG_DIR for all of the tests in here.  This should make the test resistant to being run in parallel with other tests. 3.  Consistently use rcpputils::set_env_var, rather than a mix of rcpputils and rcutils.
* Update quality declaration document (`#112 <https://github.com/ros2/rcl_logging/issues/112>`__)
* Re-order rcl_logging_interface include (`#111 <https://github.com/ros2/rcl_logging/issues/111>`__)
* add file_name_prefix parameter to external log configuration. (`#109 <https://github.com/ros2/rcl_logging/issues/109>`__)
* Migrate to std::filesystem (`#104 <https://github.com/ros2/rcl_logging/issues/104>`__)
* Remove the last uses of ament_target_dependencies in this repo. (`#102 <https://github.com/ros2/rcl_logging/issues/102>`__)
* Contributors: Chris Lalancette, Christophe Bedard, Kenta Yonekura, Scott K Logan, Tomoya Fujita


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl_yaml_param_parser <https://github.com/ros2/rcl/tree/jazzy/rcl_yaml_param_parser/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Generate version header using ament_generate_version_header(..) (`#1141 <https://github.com/ros2/rcl/issues/1141>`__)
* Update quality declaration documents (`#1131 <https://github.com/ros2/rcl/issues/1131>`__)
* Fix for incorrect integer value conversion on Windows (`#1126 <https://github.com/ros2/rcl/issues/1126>`__)
* Just remove rcpputils::fs dependency (`#1105 <https://github.com/ros2/rcl/issues/1105>`__)
* Contributors: Christophe Bedard, G.A. vd. Hoorn, Kenta Yonekura, Michael Orlov


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rclcpp <https://github.com/ros2/rclcpp/tree/jazzy/rclcpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* add impl pointer for ExecutorOptions (`#2523 <https://github.com/ros2/rclcpp/issues/2523>`__) (`#2525 <https://github.com/ros2/rclcpp/issues/2525>`__) * add impl pointer for ExecutorOptions (cherry picked from commit 343b29b617b163ad72b9fe3f6441dd4ed3d3af09) Co-authored-by: William Woodall <william@osrfoundation.org>
* Fixup Executor::spin_all() regression fix (`#2517 <https://github.com/ros2/rclcpp/issues/2517>`__) (`#2521 <https://github.com/ros2/rclcpp/issues/2521>`__) * test(Executors): Added tests for busy waiting Checks if executors are busy waiting while they should block in spin_some or spin_all. * fix: Reworked spinAll test This test was strange. It looked like, it assumed that spin_all did not return instantly. Also it was racy, as the thread could terminate instantly. * fix(Executor): Fixed spin_all not returning instantly is no work was available * Update rclcpp/test/rclcpp/executors/test_executors.cpp * test(executors): Added test for busy waiting while calling spin * fix(executor): Reset wait_result on every call to spin_some_impl Before, the method would not recollect available work in case of spin_some, spin_all. This would lead to the method behaving differently than to what the documentation states. * restore previous test logic for now * refactor spin_some_impl's logic and improve busy wait tests * added some more comments about the implementation --------- Co-authored-by: Janosch Machowinski <J.Machowinski@cellumation.com> Co-authored-by: jmachowinski <jmachowinski@users.noreply.github.com> Co-authored-by: Tomoya Fujita <Tomoya.Fujita@sony.com> Co-authored-by: William Woodall <william@osrfoundation.org>
* Revise the description of service configure_introspection() (`#2511 <https://github.com/ros2/rclcpp/issues/2511>`__) (`#2513 <https://github.com/ros2/rclcpp/issues/2513>`__)
* Remove references to index.ros.org. (`#2504 <https://github.com/ros2/rclcpp/issues/2504>`__)
* Reduce overhead for inheriting from rclcpp::Executor when base functionality is not reused (`#2506 <https://github.com/ros2/rclcpp/issues/2506>`__)
* [wjwwood] Updated "Data race fixes" (`#2500 <https://github.com/ros2/rclcpp/issues/2500>`__) * Fix callback group logic in executor * fix: Fixed unnecessary copy of wait_set * fix(executor): Fixed race conditions with rebuild of wait_sets Before this change, the rebuild of wait set would be triggered after the wait set was waken up. With bad timing, this could lead to the rebuild not happening with multi threaded executor. * fix(Executor): Fixed lost of entities rebuild request * chore: Added assert for not set callback_group in execute_any_executable * Add test for cbg getting reset Co-authored-by: Janosch Machowinski <j.machowinski@nospam.org> * chore: renamed test cases to snake_case * style * fixup test to avoid polling and short timeouts * fix: Use correct notify_waitable\_ instance * fix(StaticSingleThreadedExecutor): Added missing special case handling for current_notify_waitable\_ * fix(TestCallbackGroup): Fixed test after change to timers --------- Co-authored-by: Janosch Machowinski <j.machowinski@cellumation.com> Co-authored-by: Michael Carroll <mjcarroll@intrinsic.ai> Co-authored-by: Janosch Machowinski <j.machowinski@nospam.org>
* fixup var names to snake case (`#2501 <https://github.com/ros2/rclcpp/issues/2501>`__)
* Added optional TimerInfo to timer callback (`#2343 <https://github.com/ros2/rclcpp/issues/2343>`__) Co-authored-by: Alexis Tsogias <a.tsogias@cellumation.com> Co-authored-by: Janosch Machowinski <J.Machowinski@cellumation.com>
* Fix uninitialized memory in test (`#2498 <https://github.com/ros2/rclcpp/issues/2498>`__) When I added in the tests for large messages, I made a mistake and reserved space in the strings, but didn't actually expand it.  Thus, we were writing into uninitialized memory.  Fix this by just using the correct constructor for string, which will allocate and initialize the memory properly.
* Ensure waitables handle guard condition retriggering (`#2483 <https://github.com/ros2/rclcpp/issues/2483>`__) Co-authored-by: Michael Carroll <mjcarroll@intrinsic.ai>
* fix: init concatenated_vector with begin() & end() (`#2492 <https://github.com/ros2/rclcpp/issues/2492>`__) * this commit will fix the warning [-Wstringop-overflow=] `#2461 <https://github.com/ros2/rclcpp/issues/2461>`__
* Use the same context for the specified node in rclcpp::spin functions (`#2433 <https://github.com/ros2/rclcpp/issues/2433>`__) * Use the same conext for the specified node in rclcpp::spin_xx functions * Add test for spinning with non-default-context * Format code ---------
* Disable compare-function-pointers in test_utilities (`#2489 <https://github.com/ros2/rclcpp/issues/2489>`__)
* address ambiguous auto variable. (`#2481 <https://github.com/ros2/rclcpp/issues/2481>`__)
* Increase the cppcheck timeout to 1200 seconds (`#2484 <https://github.com/ros2/rclcpp/issues/2484>`__)
* Removed test_timers_manager clang warning (`#2479 <https://github.com/ros2/rclcpp/issues/2479>`__)
* Flaky timer test fix (`#2469 <https://github.com/ros2/rclcpp/issues/2469>`__) * fix(time_source): Fixed possible race condition * fix(test_executors_time_cancel_behaviour): Fixed multiple race conditions --------- Co-authored-by: Janosch Machowinski <j.machowinski@nospam.org>
* Add tracepoint for generic publisher/subscriber (`#2448 <https://github.com/ros2/rclcpp/issues/2448>`__)
* update rclcpp::Waitable API to use references and const (`#2467 <https://github.com/ros2/rclcpp/issues/2467>`__)
* Utilize rclcpp::WaitSet as part of the executors (`#2142 <https://github.com/ros2/rclcpp/issues/2142>`__) * Deprecate callback_group call taking context * Add base executor objects that can be used by implementors * Template common operations * Address reviewer feedback: * Add callback to EntitiesCollector constructor * Make function to check automatically added callback groups take a list * Lint * Address reviewer feedback and fix templates * Lint and docs * Make executor own the notify waitable * Add pending queue to collector, remove from waitable Also change node's get_guard_condition to return shared_ptr * Change interrupt guard condition to shared_ptr Check if guard condition is valid before adding it to the waitable * Lint and docs * Utilize rclcpp::WaitSet as part of the executors * Don't exchange atomic twice * Fix add_node and add more tests * Make get_notify_guard_condition follow API tick-tock * Improve callback group tick-tocking * Don't lock twice * Address reviewer feedback * Add thread safety annotations and make locks consistent * @wip * Reset callback groups for multithreaded executor * Avoid many small function calls when building executables * Re-trigger guard condition if buffer has data * Address reviewer feedback * Trace points * Remove tracepoints * Reducing diff * Reduce diff * Uncrustify * Restore tests * Back to weak_ptr and reduce test time * reduce diff and lint * Restore static single threaded tests that weren't working before * Restore more tests * Fix multithreaded test * Fix assert * Fix constructor test * Change ready_executables signature back * Don't enforce removing callback groups before nodes * Remove the "add_valid_node" API * Only notify if the trigger condition is valid * Only trigger if valid and needed * Fix spin_some/spin_all implementation * Restore single threaded executor * Picking ABI-incompatible executor changes * Add PIMPL * Additional waitset prune * Fix bad merge * Expand test timeout * Introduce method to clear expired entities from a collection * Make sure to call remove_expired_entities(). * Prune queued work when callback group is removed * Prune subscriptions from dynamic storage * Styles fixes. * Re-trigger guard conditions * Condense to just use watiable.take_data * Lint * Address reviewer comments (nits) * Lock mutex when copying * Refactors to static single threaded based on reviewers * More small refactoring * Lint * Lint * Add ready executable accessors to WaitResult * Make use of accessors from wait_set * Fix tests * Fix more tests * Tidy up single threaded executor implementation * Don't null out timer, rely on call * change how timers are checked from wait result in executors * peak -> peek * fix bug in next_waitable logic * fix bug in StaticSTE that broke the add callback groups to executor tests * style --------- Co-authored-by: Chris Lalancette <clalancette@gmail.com> Co-authored-by: William Woodall <william@osrfoundation.org>
* fix flakiness in TestTimersManager unit-test (`#2468 <https://github.com/ros2/rclcpp/issues/2468>`__) the previous version of the test was relying on the assumption that a timer with 1ms period gets called at least 6 times if the main thread waits 15ms. this is true most of the times, but it's not guaranteed, especially when running the test on windows CI servers. the new version of the test makes no assumptions on how much time it takes for the timers manager to invoke the timers, but rather focuses on ensuring that they are called the right amount of times, which is what's important for the purpose of the test
* fix spin_some_max_duration unit-test for events-executor (`#2465 <https://github.com/ros2/rclcpp/issues/2465>`__)
* refactor and improve the parameterized spin_some tests for executors (`#2460 <https://github.com/ros2/rclcpp/issues/2460>`__) * refactor and improve the spin_some parameterized tests for executors * disable spin_some_max_duration for the StaticSingleThreadedExecutor and EventsExecutor * fixup and clarify the docstring for Executor::spin_some() * style * review comments ---------
* enable simulation clock for timer canceling test. (`#2458 <https://github.com/ros2/rclcpp/issues/2458>`__) * enable simulation clock for timer canceling test. * move MainExecutorTypes to test_executors_timer_cancel_behavior.cpp. ---------
* Revert "relax the test simulation rate for timer canceling tests. (`#2453 <https://github.com/ros2/rclcpp/issues/2453>`__)" (`#2456 <https://github.com/ros2/rclcpp/issues/2456>`__) This reverts commit 1c350d0d7fb9c7158e0a39057112486ddbd38e9a.
* relax the test simulation rate for timer canceling tests. (`#2453 <https://github.com/ros2/rclcpp/issues/2453>`__)
* Fix TypeAdapted publishing with large messages. (`#2443 <https://github.com/ros2/rclcpp/issues/2443>`__) Mostly by ensuring we aren't attempting to store large messages on the stack.  Also add in tests. I verified that before these changes, the tests failed, while after them they succeed.
* Implement generic client (`#2358 <https://github.com/ros2/rclcpp/issues/2358>`__) * Implement generic client * Fix the incorrect parameter declaration * Deleted copy constructor and assignment for FutureAndRequestId * Update codes after rebase * Address review comments * Address review comments from iuhilnehc-ynos * Correct an error in a description * Fix window build errors * Address review comments from William * Add doc strings to create_generic_client ---------
* Rule of five: implement move operators (`#2425 <https://github.com/ros2/rclcpp/issues/2425>`__)
* Various cleanups to deal with uncrustify 0.78. (`#2439 <https://github.com/ros2/rclcpp/issues/2439>`__) These should also work with uncrustify 0.72.
* Remove the set_deprecated signatures in any_subscription_callback. (`#2431 <https://github.com/ros2/rclcpp/issues/2431>`__) These have been deprecated since April 2021, so it is safe to remove them now.
* fix doxygen syntax for NodeInterfaces (`#2428 <https://github.com/ros2/rclcpp/issues/2428>`__)
* Set hints to find the python version we actually want. (`#2426 <https://github.com/ros2/rclcpp/issues/2426>`__) The comment in the commit explains the reasoning behind it.
* Update quality declaration documents (`#2427 <https://github.com/ros2/rclcpp/issues/2427>`__)
* feat: add/minus for msg::Time and rclcpp::Duration (`#2419 <https://github.com/ros2/rclcpp/issues/2419>`__) * feat: add/minus for msg::Time and rclcpp::Duration
* Split test_executors up into smaller chunks. (`#2421 <https://github.com/ros2/rclcpp/issues/2421>`__)
* [events executor] - Fix Behavior with Timer Cancel (`#2375 <https://github.com/ros2/rclcpp/issues/2375>`__)
* Removed deprecated header (`#2413 <https://github.com/ros2/rclcpp/issues/2413>`__)
* Make sure to mark RingBuffer methods as 'override'. (`#2410 <https://github.com/ros2/rclcpp/issues/2410>`__)
* Increase the cppcheck timeout to 600 seconds. (`#2409 <https://github.com/ros2/rclcpp/issues/2409>`__)
* Add transient local durability support to publisher and subscriptions when using intra-process communication (`#2303 <https://github.com/ros2/rclcpp/issues/2303>`__)
* Stop storing the context in the guard condition. (`#2400 <https://github.com/ros2/rclcpp/issues/2400>`__)
* Updated GenericSubscription to AnySubscriptionCallback (`#1928 <https://github.com/ros2/rclcpp/issues/1928>`__)
* make type support helper supported for service (`#2209 <https://github.com/ros2/rclcpp/issues/2209>`__)
* Adding QoS to subscription options (`#2323 <https://github.com/ros2/rclcpp/issues/2323>`__)
* Switch to target_link_libraries. (`#2374 <https://github.com/ros2/rclcpp/issues/2374>`__)
* aligh with rcl that a rosout publisher of a node might not exist (`#2357 <https://github.com/ros2/rclcpp/issues/2357>`__)
* Fix data race in EventHandlerBase (`#2349 <https://github.com/ros2/rclcpp/issues/2349>`__)
* Support users holding onto shared pointers in the message memory pool (`#2336 <https://github.com/ros2/rclcpp/issues/2336>`__)
* fix (signal_handler.hpp): spelling (`#2356 <https://github.com/ros2/rclcpp/issues/2356>`__)
* Updates to not use std::move in some places. (`#2353 <https://github.com/ros2/rclcpp/issues/2353>`__)
* rclcpp::Time::max() clock type support. (`#2352 <https://github.com/ros2/rclcpp/issues/2352>`__)
* Serialized Messages with Topic Statistics (`#2274 <https://github.com/ros2/rclcpp/issues/2274>`__)
* Add a custom deleter when constructing rcl_service_t (`#2351 <https://github.com/ros2/rclcpp/issues/2351>`__)
* Disable the loaned messages inside the executor. (`#2335 <https://github.com/ros2/rclcpp/issues/2335>`__)
* Use message_info in SubscriptionTopicStatistics instead of typed message (`#2337 <https://github.com/ros2/rclcpp/issues/2337>`__)
* Add missing 'enable_rosout' comments (`#2345 <https://github.com/ros2/rclcpp/issues/2345>`__)
* Adjust rclcpp usage of type description service (`#2344 <https://github.com/ros2/rclcpp/issues/2344>`__)
* address rate related flaky tests. (`#2329 <https://github.com/ros2/rclcpp/issues/2329>`__)
* Fixes pointed out by the clang analyzer. (`#2339 <https://github.com/ros2/rclcpp/issues/2339>`__)
* Remove useless ROSRate class (`#2326 <https://github.com/ros2/rclcpp/issues/2326>`__)
* add clients & services count (`#2072 <https://github.com/ros2/rclcpp/issues/2072>`__)
* remove invalid sized allocation test for SerializedMessage. (`#2330 <https://github.com/ros2/rclcpp/issues/2330>`__)
* Adding API to copy all parameters from one node to another (`#2304 <https://github.com/ros2/rclcpp/issues/2304>`__)
* Add locking to protect the TimeSource::NodeState::node_base\_ (`#2320 <https://github.com/ros2/rclcpp/issues/2320>`__)
* Update SignalHandler get_global_signal_handler to avoid complex types in static memory (`#2316 <https://github.com/ros2/rclcpp/issues/2316>`__)
* Removing Old Connext Tests (`#2313 <https://github.com/ros2/rclcpp/issues/2313>`__)
* Documentation for list_parameters  (`#2315 <https://github.com/ros2/rclcpp/issues/2315>`__)
* Decouple rosout publisher init from node init. (`#2174 <https://github.com/ros2/rclcpp/issues/2174>`__)
* fix the depth to relative in list_parameters (`#2300 <https://github.com/ros2/rclcpp/issues/2300>`__)
* Fix the return type of Rate::period. (`#2301 <https://github.com/ros2/rclcpp/issues/2301>`__)
* Update API docs links in package READMEs (`#2302 <https://github.com/ros2/rclcpp/issues/2302>`__)
* Cleanup flaky timers_manager tests. (`#2299 <https://github.com/ros2/rclcpp/issues/2299>`__)
* Topic correct typeadapter deduction (`#2294 <https://github.com/ros2/rclcpp/issues/2294>`__)
* Fix C++20 allocator construct deprecation (`#2292 <https://github.com/ros2/rclcpp/issues/2292>`__)
* Make Rate to select the clock to work with (`#2123 <https://github.com/ros2/rclcpp/issues/2123>`__)
* Correct the position of a comment. (`#2290 <https://github.com/ros2/rclcpp/issues/2290>`__)
* Remove unnecessary lambda captures in the tests. (`#2289 <https://github.com/ros2/rclcpp/issues/2289>`__)
* Add rcl_logging_interface as an explicit dependency. (`#2284 <https://github.com/ros2/rclcpp/issues/2284>`__)
* Revamp list_parameters to be more efficient and easier to read. (`#2282 <https://github.com/ros2/rclcpp/issues/2282>`__)
* Do not crash Executor when send_response fails due to client failure. (`#2276 <https://github.com/ros2/rclcpp/issues/2276>`__)
* Adding Custom Unknown Type Error (`#2272 <https://github.com/ros2/rclcpp/issues/2272>`__)
* Add a pimpl inside rclcpp::Node for future distro backports (`#2228 <https://github.com/ros2/rclcpp/issues/2228>`__)
* Remove an unused variable from the events executor tests. (`#2270 <https://github.com/ros2/rclcpp/issues/2270>`__)
* Add spin_all shortcut (`#2246 <https://github.com/ros2/rclcpp/issues/2246>`__)
* Adding Missing Group Exceptions (`#2256 <https://github.com/ros2/rclcpp/issues/2256>`__)
* Change associated clocks storage to unordered_set (`#2257 <https://github.com/ros2/rclcpp/issues/2257>`__)
* associated clocks should be protected by mutex. (`#2255 <https://github.com/ros2/rclcpp/issues/2255>`__)
* Instrument loaned message publication code path (`#2240 <https://github.com/ros2/rclcpp/issues/2240>`__)
* Implement get_node_type_descriptions_interface for lifecyclenode and add smoke test for it (`#2237 <https://github.com/ros2/rclcpp/issues/2237>`__)
* Add new node interface TypeDescriptionsInterface to provide GetTypeDescription service (`#2224 <https://github.com/ros2/rclcpp/issues/2224>`__)
* Move always_false_v to detail namespace (`#2232 <https://github.com/ros2/rclcpp/issues/2232>`__)
* Revamp the test_subscription.cpp tests. (`#2227 <https://github.com/ros2/rclcpp/issues/2227>`__)
* warning: comparison of integer expressions of different signedness (`#2219 <https://github.com/ros2/rclcpp/issues/2219>`__)
* Modifies timers API to select autostart state (`#2005 <https://github.com/ros2/rclcpp/issues/2005>`__)
* Enable callback group tests for connextdds (`#2182 <https://github.com/ros2/rclcpp/issues/2182>`__)
* Fix up misspellings of "receive". (`#2208 <https://github.com/ros2/rclcpp/issues/2208>`__)
* Remove flaky stressAddRemoveNode test (`#2206 <https://github.com/ros2/rclcpp/issues/2206>`__)
* Use TRACETOOLS\_ prefix for tracepoint-related macros (`#2162 <https://github.com/ros2/rclcpp/issues/2162>`__)
* remove nolint since ament_cpplint updated for the c++17 header (`#2198 <https://github.com/ros2/rclcpp/issues/2198>`__)
* Feature/available capacity of ipm (`#2173 <https://github.com/ros2/rclcpp/issues/2173>`__)
* add mutex to protect events_executor current entity collection (`#2187 <https://github.com/ros2/rclcpp/issues/2187>`__)
* Declare rclcpp callbacks before the rcl entities (`#2024 <https://github.com/ros2/rclcpp/issues/2024>`__)
* Fix race condition in events-executor (`#2177 <https://github.com/ros2/rclcpp/issues/2177>`__)
* Add missing stdexcept include (`#2186 <https://github.com/ros2/rclcpp/issues/2186>`__)
* Fix a format-security warning when building with clang (`#2171 <https://github.com/ros2/rclcpp/issues/2171>`__)
* Fix delivered message kind (`#2175 <https://github.com/ros2/rclcpp/issues/2175>`__)
* Contributors: AiVerisimilitude, Alberto Soragna, Alejandro Hernández Cordero, Alexey Merzlyakov, Barry Xu, Chen Lihui, Chris Lalancette, Christophe Bedard, Christopher Wecht, DensoADAS, Eloy Briceno, Emerson Knapp, Homalozoa X, HuaTsai, Jeffery Hsu, Jiaqi Li, Jonas Otto, Kotaro Yoshimoto, Lee, Luca Della Vedova, Lucas Wendland, Matt Condino, Michael Carroll, Michael Orlov, Minju, Nathan Wiebe Neufeldt, Steve Macenski, Tim Clephas, Tomoya Fujita, Tony Najjar, Tully Foote, William Woodall, Zard-C, h-suzuki-isp, jmachowinski, mauropasse, mergify[bot], methylDragon, Øystein Sture


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rclcpp_action <https://github.com/ros2/rclcpp/tree/jazzy/rclcpp_action/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Remove references to index.ros.org. (`#2504 <https://github.com/ros2/rclcpp/issues/2504>`__)
* Callback after cancel (`#2281 <https://github.com/ros2/rclcpp/issues/2281>`__) * feat(Client): Added function to stop callbacks of a goal handle This function allows us to drop the handle in a locked context. If we do not do this within a lock, there will be a race condition between the deletion of the shared_ptr of the handle and the result / feedback callbacks. * fix: make Client goal handle recursive This fixes deadlocks due to release of goal handles in callbacks etc. * fix(ActionGoalClient): Fixed memory leak for nominal case This fixes a memory leak due to a self reference in the ClientGoalHandle. Note, this fix will only work, if the ClientGoalHandle ever receives a result callback. * doc: Updated documentation of rclcpp_action::Client::async_send_goal * docs: Made the async_send_goal documentation more explicit Co-authored-by: Janosch Machowinski <J.Machowinski@cellumation.com>
* Remake of "fix: Fixed race condition in action server between is_ready and take" (`#2495 <https://github.com/ros2/rclcpp/issues/2495>`__) Some background information: is_ready, take_data and execute data may be called from different threads in any order. The code in the old state expected them to be called in series, without interruption. This lead to multiple race conditions, as the state of the pimpl objects was altered by the three functions in a non thread safe way. Co-authored-by: Janosch Machowinski <j.machowinski@nospam.org>
* update rclcpp::Waitable API to use references and const (`#2467 <https://github.com/ros2/rclcpp/issues/2467>`__)
* Do not generate the exception when action service response timeout. (`#2464 <https://github.com/ros2/rclcpp/issues/2464>`__) * Do not generate the exception when action service response timeout. * address review comment. ---------
* Modify rclcpp_action::GoalUUID hashing algorithm (`#2441 <https://github.com/ros2/rclcpp/issues/2441>`__) * Add unit tests for hashing rclcpp_action::GoalUUID's * Use the FNV-1a hash algorithm for Goal UUID
* Various cleanups to deal with uncrustify 0.78. (`#2439 <https://github.com/ros2/rclcpp/issues/2439>`__) These should also work with uncrustify 0.72.
* Update quality declaration documents (`#2427 <https://github.com/ros2/rclcpp/issues/2427>`__)
* Switch to target_link_libraries. (`#2374 <https://github.com/ros2/rclcpp/issues/2374>`__)
* Update API docs links in package READMEs (`#2302 <https://github.com/ros2/rclcpp/issues/2302>`__)
* fix(ClientGoalHandle): Made mutex recursive to prevent deadlocks (`#2267 <https://github.com/ros2/rclcpp/issues/2267>`__)
* Correct the position of a comment. (`#2290 <https://github.com/ros2/rclcpp/issues/2290>`__)
* Fix a typo in a comment. (`#2283 <https://github.com/ros2/rclcpp/issues/2283>`__)
* doc fix: call ``canceled`` only after goal state is in canceling. (`#2266 <https://github.com/ros2/rclcpp/issues/2266>`__)
* Contributors: Chris Lalancette, Christophe Bedard, Jiaqi Li, Tomoya Fujita, William Woodall, jmachowinski, mauropasse


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rclcpp_components <https://github.com/ros2/rclcpp/tree/jazzy/rclcpp_components/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Remove references to index.ros.org. (`#2504 <https://github.com/ros2/rclcpp/issues/2504>`__)
* Add EXECUTOR docs (`#2440 <https://github.com/ros2/rclcpp/issues/2440>`__)
* Update quality declaration documents (`#2427 <https://github.com/ros2/rclcpp/issues/2427>`__)
* crash on no class found (`#2415 <https://github.com/ros2/rclcpp/issues/2415>`__) * crash on no class found * error on no class found instead of no callback groups Co-authored-by: Chris Lalancette <clalancette@gmail.com>
* Switch to target_link_libraries. (`#2374 <https://github.com/ros2/rclcpp/issues/2374>`__)
* feat(rclcpp_components): support events executor in node main template (`#2366 <https://github.com/ros2/rclcpp/issues/2366>`__)
* fix(rclcpp_components): increase the service queue sizes in component_container (`#2363 <https://github.com/ros2/rclcpp/issues/2363>`__)
* Add missing header required by the rclcpp::NodeOptions type (`#2324 <https://github.com/ros2/rclcpp/issues/2324>`__)
* Update API docs links in package READMEs (`#2302 <https://github.com/ros2/rclcpp/issues/2302>`__)
* Contributors: Adam Aposhian, Chris Lalancette, Christophe Bedard, Daisuke Nishimatsu, Ignacio Vizzo, M. Fatih Cırıt, Ruddick Lawrence


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rclcpp_lifecycle <https://github.com/ros2/rclcpp/tree/jazzy/rclcpp_lifecycle/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Revert "call shutdown in LifecycleNode dtor to avoid leaving the device in un… (`#2450 <https://github.com/ros2/rclcpp/issues/2450>`__)" (`#2522 <https://github.com/ros2/rclcpp/issues/2522>`__) (`#2524 <https://github.com/ros2/rclcpp/issues/2524>`__) This reverts commit 04ea0bb00293387791522590b7347a2282cda290. (cherry picked from commit 42b0b5775b4e68718c5949308c9e1a059930ded7) Co-authored-by: Chris Lalancette <clalancette@gmail.com>
* Remove references to index.ros.org. (`#2504 <https://github.com/ros2/rclcpp/issues/2504>`__)
* call shutdown in LifecycleNode dtor to avoid leaving the device in un… (`#2450 <https://github.com/ros2/rclcpp/issues/2450>`__) * call shutdown in LifecycleNode dtor to avoid leaving the device in unknown state. * add test to verify LifecycleNode::shutdown is called on destructor. ---------
* Update quality declaration documents (`#2427 <https://github.com/ros2/rclcpp/issues/2427>`__)
* Increase timeout for rclcpp_lifecycle to 360 (`#2395 <https://github.com/ros2/rclcpp/issues/2395>`__)
* Fix rclcpp_lifecycle inclusion on Windows. (`#2331 <https://github.com/ros2/rclcpp/issues/2331>`__)
* add clients & services count (`#2072 <https://github.com/ros2/rclcpp/issues/2072>`__)
* Update API docs links in package READMEs (`#2302 <https://github.com/ros2/rclcpp/issues/2302>`__)
* add logger level service to lifecycle node. (`#2277 <https://github.com/ros2/rclcpp/issues/2277>`__)
* Stop using constref signature of benchmark DoNotOptimize. (`#2238 <https://github.com/ros2/rclcpp/issues/2238>`__)
* Implement get_node_type_descriptions_interface for lifecyclenode and add smoke test for it (`#2237 <https://github.com/ros2/rclcpp/issues/2237>`__)
* Switch lifecycle to use the RCLCPP macros. (`#2233 <https://github.com/ros2/rclcpp/issues/2233>`__)
* Add new node interface TypeDescriptionsInterface to provide GetTypeDescription service (`#2224 <https://github.com/ros2/rclcpp/issues/2224>`__)
* Contributors: Chris Lalancette, Christophe Bedard, Emerson Knapp, Jorge Perez, Lee, Minju, Tomoya Fujita, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rclpy <https://github.com/ros2/rclpy/tree/jazzy/rclpy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Clock.py types. (`#1244 <https://github.com/ros2/rclpy/issues/1244>`__) * Start typing time.py * Testing out Enum wrapper for ClockType * convert to rcl_clock_type_t * Update create_time_point * add types to logging_service * Add types to duration.py * Add newlines for class definintions * update type alias name * Update to use Protocols * Add types to time.py * Add types * Fix import order * Started typing clock.py * Move typealias import
* pybind11 definition doc typo fixes. (`#1270 <https://github.com/ros2/rclpy/issues/1270>`__)
* Fix small flake8 error in rclpy. (`#1267 <https://github.com/ros2/rclpy/issues/1267>`__) Newer versions of flake8 complain that using 'str' as a variable shadows a builtin.  Just make it 's'.
* Allow specifying qos (`#1225 <https://github.com/ros2/rclpy/issues/1225>`__)
* update RCL_RET_TIMEOUT error handling with action service response. (`#1258 <https://github.com/ros2/rclpy/issues/1258>`__)
* Add types to time_source.py (`#1259 <https://github.com/ros2/rclpy/issues/1259>`__)
* Small fixes for modern flake8. (`#1264 <https://github.com/ros2/rclpy/issues/1264>`__)
* Add types to qos_overriding_options.py (`#1248 <https://github.com/ros2/rclpy/issues/1248>`__)
* Add types to context.py (`#1240 <https://github.com/ros2/rclpy/issues/1240>`__)
* Add back Type hash __slots_\_ and add test cases. (`#1245 <https://github.com/ros2/rclpy/issues/1245>`__)
* Revert "Add types to TypeHash and moved away from __slots_\_ usage (`#1232 <https://github.com/ros2/rclpy/issues/1232>`__)" (`#1243 <https://github.com/ros2/rclpy/issues/1243>`__)
* Time.py Types (`#1237 <https://github.com/ros2/rclpy/issues/1237>`__)
* Add types to TypeHash and moved away from __slots_\_ usage (`#1232 <https://github.com/ros2/rclpy/issues/1232>`__)
* Add Static Typing to Validate files (`#1230 <https://github.com/ros2/rclpy/issues/1230>`__)
* Add types to duration.py (`#1233 <https://github.com/ros2/rclpy/issues/1233>`__)
* added python3-yaml (`#1242 <https://github.com/ros2/rclpy/issues/1242>`__)
* Add types to exceptions.py (`#1241 <https://github.com/ros2/rclpy/issues/1241>`__)
* Add types (`#1231 <https://github.com/ros2/rclpy/issues/1231>`__)
* Creates Enum wrapper for ClockType and ClockChange (`#1235 <https://github.com/ros2/rclpy/issues/1235>`__)
* Add types to expand_topic_name (`#1238 <https://github.com/ros2/rclpy/issues/1238>`__)
* Add types to logging_service.py (`#1227 <https://github.com/ros2/rclpy/issues/1227>`__)
* Add types to logging.py (`#1226 <https://github.com/ros2/rclpy/issues/1226>`__)
* forbid parameter to be declared statically without initialization. (`#1216 <https://github.com/ros2/rclpy/issues/1216>`__)
* Remove parentheses from assert statements. (`#1213 <https://github.com/ros2/rclpy/issues/1213>`__)
* Add doc-string warnings for destroy methods for services. (`#1205 <https://github.com/ros2/rclpy/issues/1205>`__)
* Add doc-string warnings for destroy() methods (`#1204 <https://github.com/ros2/rclpy/issues/1204>`__)
* Add an optional timeout_sec input to Client.call() to fix issue https://github.com/ros2/rclpy/issues/1181 (`#1188 <https://github.com/ros2/rclpy/issues/1188>`__)
* aligh with rcl that a rosout publisher of a node might not exist (`#1196 <https://github.com/ros2/rclpy/issues/1196>`__)
* call ok() to see if rclpy and context is initialized. (`#1198 <https://github.com/ros2/rclpy/issues/1198>`__)
* Adjust python usage of the type_description service API (`#1192 <https://github.com/ros2/rclpy/issues/1192>`__)
* Document that spin_once() should not be called from multiple threads (`#1079 <https://github.com/ros2/rclpy/issues/1079>`__)
* making optional things Optional (`#1182 <https://github.com/ros2/rclpy/issues/1182>`__)
* Use timeout object to avoid callback losing in wait_for_ready_callbacks (`#1165 <https://github.com/ros2/rclpy/issues/1165>`__)
* Fix to issue https://github.com/ros2/rclpy/issues/1179 (`#1180 <https://github.com/ros2/rclpy/issues/1180>`__)
* Add count services, clients & test (`#1024 <https://github.com/ros2/rclpy/issues/1024>`__)
* 1105 parameter event handler (`#1135 <https://github.com/ros2/rclpy/issues/1135>`__)
* unregister_sigterm_signal_handler should be called. (`#1170 <https://github.com/ros2/rclpy/issues/1170>`__)
* Handle take failure in wait_for_message (`#1172 <https://github.com/ros2/rclpy/issues/1172>`__)
* Decouple rosout publisher init from node init. (`#1121 <https://github.com/ros2/rclpy/issues/1121>`__)
* Fix _list_parameters_callback & test (`#1161 <https://github.com/ros2/rclpy/issues/1161>`__)
* add list_parameters & test (`#1124 <https://github.com/ros2/rclpy/issues/1124>`__)
* Support to get remapped service name (`#1156 <https://github.com/ros2/rclpy/issues/1156>`__)
* a couple of typo fixes. (`#1158 <https://github.com/ros2/rclpy/issues/1158>`__)
* Fix get_type_description service bug and add a unit test (`#1155 <https://github.com/ros2/rclpy/issues/1155>`__)
* Fix an inherent race in execution vs. destruction. (`#1150 <https://github.com/ros2/rclpy/issues/1150>`__)
* Cleanup of test_node.py. (`#1153 <https://github.com/ros2/rclpy/issues/1153>`__)
* Avoid generating the exception when rcl_send_response times out. (`#1136 <https://github.com/ros2/rclpy/issues/1136>`__)
* Store time source clocks in a set (`#1146 <https://github.com/ros2/rclpy/issues/1146>`__)
* Fix spin_once_until_future_complete to quit when the future finishes. (`#1143 <https://github.com/ros2/rclpy/issues/1143>`__)
* get_type_description service (`#1139 <https://github.com/ros2/rclpy/issues/1139>`__)
* Add in the ability to start timers paused. (`#1138 <https://github.com/ros2/rclpy/issues/1138>`__)
* Modifies ros_timer_init for ros_timer_init2 (`#999 <https://github.com/ros2/rclpy/issues/999>`__)
* Fix/param namespace association 894 (`#1132 <https://github.com/ros2/rclpy/issues/1132>`__)
* Include type hash in topic endpoint info (`#1104 <https://github.com/ros2/rclpy/issues/1104>`__)
* Fix iteration over modified list (`#1129 <https://github.com/ros2/rclpy/issues/1129>`__)
* making optional things Optional (`#974 <https://github.com/ros2/rclpy/issues/974>`__)
* Fix type signature of Client.wait_for_service (`#1128 <https://github.com/ros2/rclpy/issues/1128>`__)
* Fix action server crash when the client goes away. (`#1114 <https://github.com/ros2/rclpy/issues/1114>`__)
* Turn Executor into a ContextManager (`#1118 <https://github.com/ros2/rclpy/issues/1118>`__)
* Turn Context into a ContextManager (`#1117 <https://github.com/ros2/rclpy/issues/1117>`__)
* Fix type in Node init args (`#1115 <https://github.com/ros2/rclpy/issues/1115>`__)
* Contributors: AndyZe, Anton Kesy, Barry Xu, Brian, Chen Lihui, Chris Lalancette, Eloy Briceno, Emerson Knapp, EsipovPA, Felix Divo, Hans-Joachim Krauch, KKSTB, Lee, Luca Della Vedova, M. Hofstätter, Michael Carlstrom, Michael Carroll, Minju, Russ, SnIcK, Steve Peters, Tim Clephas, Tomoya Fujita, mhidalgo-bdai


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcpputils <https://github.com/ros2/rcpputils/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Generate version header with ament_generate_version_header function (`#190 <https://github.com/ros2/rcpputils/issues/190>`__)
* Update docs for rcpputils::split functions (`#188 <https://github.com/ros2/rcpputils/issues/188>`__)
* Included tl_expected (`#185 <https://github.com/ros2/rcpputils/issues/185>`__)
* Switch to using target_link_libraries. (`#183 <https://github.com/ros2/rcpputils/issues/183>`__)
* Add a missing header due to missing PATH_MAX variable (`#181 <https://github.com/ros2/rcpputils/issues/181>`__)
* Add unique_lock implementation with clang thread safety annotations (`#180 <https://github.com/ros2/rcpputils/issues/180>`__)
* Add in a missing cstdint. (`#178 <https://github.com/ros2/rcpputils/issues/178>`__)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Christophe Bedard, Emerson Knapp, Sai Kishor Kothakota, wojciechmadry


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcutils <https://github.com/ros2/rcutils/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Removed warnings - strict-prototypes (`#461 <https://github.com/ros2/rcutils/issues/461>`__) (`#465 <https://github.com/ros2/rcutils/issues/465>`__)
* Increase timeout repl_str test (`#463 <https://github.com/ros2/rcutils/issues/463>`__) (`#464 <https://github.com/ros2/rcutils/issues/464>`__)
* validate the allocator before use. (`#455 <https://github.com/ros2/rcutils/issues/455>`__) * validate the allocator before use. * address review comments. - validate allocator only if the function specifically uses. - argument null check comes before validation of value. ---------
* feat: Add human readable date to logging formats (`#441 <https://github.com/ros2/rcutils/issues/441>`__)
* Updates for uncrustify 0.78. (`#454 <https://github.com/ros2/rcutils/issues/454>`__)
* Set hints to find the python version we actually want. (`#451 <https://github.com/ros2/rcutils/issues/451>`__)
* Bring ament_add_gtest/target_link_libraries back together (`#452 <https://github.com/ros2/rcutils/issues/452>`__)
* Change 'ROS2' to 'ROS 2' in quality declaration (`#453 <https://github.com/ros2/rcutils/issues/453>`__)
* Allow parsing of escape sequence in log format (`#443 <https://github.com/ros2/rcutils/issues/443>`__)
* Clean up unused references to mimick/mocking in tests (`#450 <https://github.com/ros2/rcutils/issues/450>`__)
* Fix if(TARGET ...) condition for test (`#447 <https://github.com/ros2/rcutils/issues/447>`__)
* Zero-initialize rcutils_string_array_t in test_string_array (`#446 <https://github.com/ros2/rcutils/issues/446>`__)
* Use rcutils_string_array_init in rcutils_split & handle alloc fail (`#445 <https://github.com/ros2/rcutils/issues/445>`__)
* Make rcutils_split() return RCUTILS_RET_BAD_ALLOC if alloc fails (`#444 <https://github.com/ros2/rcutils/issues/444>`__)
* Remove two last uses of ament_target_dependencies. (`#440 <https://github.com/ros2/rcutils/issues/440>`__)
* time_win32: Update dead link (`#438 <https://github.com/ros2/rcutils/issues/438>`__)
* memmove for overlaping memory (`#434 <https://github.com/ros2/rcutils/issues/434>`__)
* make escape characters work (`#426 <https://github.com/ros2/rcutils/issues/426>`__)
* Remove unused 'max' functions from sha256.c (`#429 <https://github.com/ros2/rcutils/issues/429>`__)
* Contributors: Chen Lihui, Chris Lalancette, Christophe Bedard, Kaju-Bubanja, Marc Bestmann, Silvio Traversaro, Tomoya Fujita, Tyler Weaver, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`resource_retriever <https://github.com/ros/resource_retriever/tree/jazzy/resource_retriever/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update resource retreiver to use rule of five (`#95 <https://github.com/ros/resource_retriever/issues/95>`__)
* Use default ament_lint_auto (`#92 <https://github.com/ros/resource_retriever/issues/92>`__)
* Switch to target_link_libraries. (`#89 <https://github.com/ros/resource_retriever/issues/89>`__)
* Update to C++17 (`#88 <https://github.com/ros/resource_retriever/issues/88>`__)
* Contributors: Chris Lalancette, Michael Carroll


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw <https://github.com/ros2/rmw/tree/jazzy/rmw/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Removed warnings - strict-prototypes (`#365 <https://github.com/ros2/rmw/issues/365>`__) (`#366 <https://github.com/ros2/rmw/issues/366>`__)
* Switch to target_link_libraries. (`#361 <https://github.com/ros2/rmw/issues/361>`__)
* Remove unnecessary c++14 flag. (`#360 <https://github.com/ros2/rmw/issues/360>`__)
* definition of local means being in the same context. (`#359 <https://github.com/ros2/rmw/issues/359>`__)
* typo fix. (`#355 <https://github.com/ros2/rmw/issues/355>`__)
* Contributors: Chris Lalancette, Tomoya Fujita, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_connextdds <https://github.com/ros2/rmw_connextdds/tree/jazzy/rmw_connextdds/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add rmw count clients services impl (`#93 <https://github.com/ros2/rmw_connextdds/issues/93>`__)
* Cleanup context implementation (`#131 <https://github.com/ros2/rmw_connextdds/issues/131>`__)
* Update to C++17 (`#125 <https://github.com/ros2/rmw_connextdds/issues/125>`__)
* Contributors: Chris Lalancette, Lee, Minju


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_connextdds_common <https://github.com/ros2/rmw_connextdds/tree/jazzy/rmw_connextdds_common/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Revert "Mitigate discovery race condition between clients and services (`#132 <https://github.com/ros2/rmw_connextdds/issues/132>`__)" (`#146 <https://github.com/ros2/rmw_connextdds/issues/146>`__) This reverts commit 7c95abbfc4559b293ebf5e94e20250bdd99d3ac6.
* Mitigate discovery race condition between clients and services (`#132 <https://github.com/ros2/rmw_connextdds/issues/132>`__) * Mitigate discovery race condition between clients and services.
* Add: tracepoint for subscribe serialized_message (`#145 <https://github.com/ros2/rmw_connextdds/issues/145>`__) * Add: tracepoint for take_serialized_message * Fix: TRACETOOLS_TRACEPOINT args * Update rmw_connextdds_common/src/common/rmw_subscription.cpp Co-authored-by: Christophe Bedard <bedard.christophe@gmail.com>
* Support Fast CDR v2 (`#141 <https://github.com/ros2/rmw_connextdds/issues/141>`__)
* Fix the rmw_connextdds_common build with gcc 13.2. (`#142 <https://github.com/ros2/rmw_connextdds/issues/142>`__) The most important fix here is to #include <cstdint>, but also make sure we #include for all used STL functions.
* Fix basic request reply mapping for ConnextDDS Pro (`#139 <https://github.com/ros2/rmw_connextdds/issues/139>`__)
* Add ros2_tracing tracepoints (`#120 <https://github.com/ros2/rmw_connextdds/issues/120>`__)
* avoid using dds common public mutex directly (`#134 <https://github.com/ros2/rmw_connextdds/issues/134>`__)
* Fix a couple of warnings pointed out by clang. (`#133 <https://github.com/ros2/rmw_connextdds/issues/133>`__)
* Add rmw count clients services impl (`#93 <https://github.com/ros2/rmw_connextdds/issues/93>`__)
* Conditional internal API access to support Connext 7+ (`#121 <https://github.com/ros2/rmw_connextdds/issues/121>`__)
* Cleanup context implementation (`#131 <https://github.com/ros2/rmw_connextdds/issues/131>`__)
* Fix RMW_Connext_Client::is_service_available for micro (`#130 <https://github.com/ros2/rmw_connextdds/issues/130>`__)
* Update to C++17 (`#125 <https://github.com/ros2/rmw_connextdds/issues/125>`__)
* Pass parameters in the correct order to DDS_DataReader_read in rmw_connextdds_count_unread_samples for micro (`#129 <https://github.com/ros2/rmw_connextdds/issues/129>`__)
* Optimize QoS to improve responsiveness of reliable endpoints (`#26 <https://github.com/ros2/rmw_connextdds/issues/26>`__)
* Clear out errors once we have handled them. (`#126 <https://github.com/ros2/rmw_connextdds/issues/126>`__)
* Add support for listener callbacks (`#76 <https://github.com/ros2/rmw_connextdds/issues/76>`__)
* Contributors: Andrea Sorbini, Chen Lihui, Chris Lalancette, Christopher Wecht, Lee, Miguel Company, Minju, h-suzuki-isp


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_connextddsmicro <https://github.com/ros2/rmw_connextdds/tree/jazzy/rmw_connextddsmicro/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add rmw count clients services impl (`#93 <https://github.com/ros2/rmw_connextdds/issues/93>`__)
* Cleanup context implementation (`#131 <https://github.com/ros2/rmw_connextdds/issues/131>`__)
* Update to C++17 (`#125 <https://github.com/ros2/rmw_connextdds/issues/125>`__)
* Contributors: Chris Lalancette, Lee, Minju


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_cyclonedds_cpp <https://github.com/ros2/rmw_cyclonedds/tree/jazzy/rmw_cyclonedds_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Set received_timestamp to system_clock::now() in message_info (`#491 <https://github.com/ros2/rmw_cyclonedds/issues/491>`__) (`#493 <https://github.com/ros2/rmw_cyclonedds/issues/493>`__) * Set received_timestamp to steady_clock::now() in message_info * Use 'system_clock' instead of 'steady_clock' * Also update receive_timestamp for services. (cherry picked from commit 76c9d8f38a03d160b258902af6d1d06f6ed9391e) Co-authored-by: Michael Orlov <morlovmr@gmail.com>
* Add tracepoint for publish/subscribe serialized message (`#485 <https://github.com/ros2/rmw_cyclonedds/issues/485>`__) Co-authored-by: eboasson <eb@ilities.com>
* Remove a bunch of unnecessary macros. (`#482 <https://github.com/ros2/rmw_cyclonedds/issues/482>`__)
* compare string contents but string pointer addresses. (`#481 <https://github.com/ros2/rmw_cyclonedds/issues/481>`__)
* Add timestamp to rmw_publish tracepoint (`#454 <https://github.com/ros2/rmw_cyclonedds/issues/454>`__)
* avoid using dds common public mutex directly (`#474 <https://github.com/ros2/rmw_cyclonedds/issues/474>`__)
* Add rmw count clients,services impl (`#427 <https://github.com/ros2/rmw_cyclonedds/issues/427>`__)
* Minor revamp of the CMakeLists.txt. (`#468 <https://github.com/ros2/rmw_cyclonedds/issues/468>`__)
* Clear out errors once we have handled them. (`#464 <https://github.com/ros2/rmw_cyclonedds/issues/464>`__)
* Instrument loaned message publication code path
* Use TRACETOOLS\_ prefix for tracepoint-related macros (`#450 <https://github.com/ros2/rmw_cyclonedds/issues/450>`__)
* Contributors: Chen Lihui, Chris Lalancette, Christophe Bedard, Christopher Wecht, Lee, Minju, Tomoya Fujita, h-suzuki-isp, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_dds_common <https://github.com/ros2/rmw_dds_common/tree/jazzy/rmw_dds_common/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add pkcs11 support to get_security_files (`#66 <https://github.com/ros2/rmw_dds_common/issues/66>`__)
* make a new private mutex and add updating graph methods (`#73 <https://github.com/ros2/rmw_dds_common/issues/73>`__)
* Just remove rcpputils::fs dependency (`#72 <https://github.com/ros2/rmw_dds_common/issues/72>`__)
* Contributors: Chen Lihui, Kenta Yonekura, Miguel Company


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_fastrtps_cpp <https://github.com/ros2/rmw_fastrtps/tree/jazzy/rmw_fastrtps_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Support Fast CDR v2 (`#746 <https://github.com/ros2/rmw_fastrtps/issues/746>`__) * Require fastcdr version 2 * Changes to build rmw_fastrtps_shared_cpp * Changes to build rmw_fastrtps_cpp * Changes to build rmw_fastrtps_dynamic_cpp
* Capture ``std::bad_alloc`` on deserializeROSmessage. (`#665 <https://github.com/ros2/rmw_fastrtps/issues/665>`__)
* Switch to target_link_libraries for linking. (`#734 <https://github.com/ros2/rmw_fastrtps/issues/734>`__)
* avoid using dds common public mutex directly (`#725 <https://github.com/ros2/rmw_fastrtps/issues/725>`__)
* Add rmw_count clients,services impl (`#641 <https://github.com/ros2/rmw_fastrtps/issues/641>`__)
* Improve node graph delivery by using a unique listening port (`#711 <https://github.com/ros2/rmw_fastrtps/issues/711>`__)
* Use TRACETOOLS\_ prefix for tracepoint-related macros (`#686 <https://github.com/ros2/rmw_fastrtps/issues/686>`__)
* Contributors: Chen Lihui, Chris Lalancette, Christophe Bedard, Lee, Miguel Company, Minju


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_fastrtps_dynamic_cpp <https://github.com/ros2/rmw_fastrtps/tree/jazzy/rmw_fastrtps_dynamic_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Support Fast CDR v2 (`#746 <https://github.com/ros2/rmw_fastrtps/issues/746>`__) * Require fastcdr version 2 * Changes to build rmw_fastrtps_shared_cpp * Changes to build rmw_fastrtps_cpp * Changes to build rmw_fastrtps_dynamic_cpp
* compare string contents but string pointer addresses. (`#744 <https://github.com/ros2/rmw_fastrtps/issues/744>`__)
* Improve wide string (de)serialization in rwm_dynamic_fastrtps_cpp (`#740 <https://github.com/ros2/rmw_fastrtps/issues/740>`__) * Move type support headers to src * Fix references to moved headers * move macros.hpp to src/serialization_helpers.hpp * Move other non-api headers * Move common code into serialize_wide_string. * Move common code into deserialize_wide_string. * Move serialization into serialization_helpers.hpp * Move deserialization into serialization_helpers.hpp * Fix header guards * Linters * Do not account for extra character on serialized size calculation * Remove dependency on rosidl_typesupport_fastrtps_c(pp) ---------
* Capture ``std::bad_alloc`` on deserializeROSmessage. (`#665 <https://github.com/ros2/rmw_fastrtps/issues/665>`__)
* Switch to target_link_libraries for linking. (`#734 <https://github.com/ros2/rmw_fastrtps/issues/734>`__)
* avoid using dds common public mutex directly (`#725 <https://github.com/ros2/rmw_fastrtps/issues/725>`__)
* Account for alignment on is_plain calculations. (`#716 <https://github.com/ros2/rmw_fastrtps/issues/716>`__)
* Add rmw_count clients,services impl (`#641 <https://github.com/ros2/rmw_fastrtps/issues/641>`__)
* Improve node graph delivery by using a unique listening port (`#711 <https://github.com/ros2/rmw_fastrtps/issues/711>`__)
* Contributors: Chen Lihui, Chris Lalancette, Lee, Miguel Company, Minju, Tomoya Fujita


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_fastrtps_shared_cpp <https://github.com/ros2/rmw_fastrtps/tree/jazzy/rmw_fastrtps_shared_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Allow pkcs11 when calling rmw_dds_common::get_security_files. (`#565 <https://github.com/ros2/rmw_fastrtps/issues/565>`__) Co-authored-by: Miguel Company <MiguelCompany@eprosima.com>
* Add tracepoint for publish/subscribe serialized_message (`#748 <https://github.com/ros2/rmw_fastrtps/issues/748>`__) * Add: tracepoint for generic pub/sub * Fix: correspond to PR 454 * Fix: change write to write_to_timestamp ---------
* Support Fast CDR v2 (`#746 <https://github.com/ros2/rmw_fastrtps/issues/746>`__) * Require fastcdr version 2 * Changes to build rmw_fastrtps_shared_cpp * Changes to build rmw_fastrtps_cpp * Changes to build rmw_fastrtps_dynamic_cpp
* Remove an unnecessary constructor. (`#743 <https://github.com/ros2/rmw_fastrtps/issues/743>`__) We can just use brace initialization here, and this allows us to side-step an uncrustify issue with the constructor.
* Add timestamp to rmw_publish tracepoint (`#694 <https://github.com/ros2/rmw_fastrtps/issues/694>`__)
* Switch to Unix line endings. (`#736 <https://github.com/ros2/rmw_fastrtps/issues/736>`__)
* Switch to target_link_libraries for linking. (`#734 <https://github.com/ros2/rmw_fastrtps/issues/734>`__)
* Quiet compiler warning in Release mode. (`#730 <https://github.com/ros2/rmw_fastrtps/issues/730>`__)
* avoid using dds common public mutex directly (`#725 <https://github.com/ros2/rmw_fastrtps/issues/725>`__)
* Add rmw_count clients,services impl (`#641 <https://github.com/ros2/rmw_fastrtps/issues/641>`__)
* Switch to using rclcpp::unique_lock. (`#712 <https://github.com/ros2/rmw_fastrtps/issues/712>`__)
* Use DataWriter Qos to configure max_blocking_time on rmw_send_response (`#704 <https://github.com/ros2/rmw_fastrtps/issues/704>`__)
* Clear out errors once we have handled them. (`#701 <https://github.com/ros2/rmw_fastrtps/issues/701>`__)
* Instrument loaned message publication code path (`#698 <https://github.com/ros2/rmw_fastrtps/issues/698>`__)
* Add in a missing data_reader check when creating subscription. (`#697 <https://github.com/ros2/rmw_fastrtps/issues/697>`__)
* Use TRACETOOLS\_ prefix for tracepoint-related macros (`#686 <https://github.com/ros2/rmw_fastrtps/issues/686>`__)
* typo fix. (`#693 <https://github.com/ros2/rmw_fastrtps/issues/693>`__)
* address clang nightly build error. (`#689 <https://github.com/ros2/rmw_fastrtps/issues/689>`__)
* Check for errors while doing an rmw_discovery_options_copy. (`#690 <https://github.com/ros2/rmw_fastrtps/issues/690>`__)
* Contributors: Chen Lihui, Chris Lalancette, Christophe Bedard, Christopher Wecht, IkerLuengo, Lee, Miguel Company, Minju, Tomoya Fujita, h-suzuki-isp


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_implementation <https://github.com/ros2/rmw_implementation/tree/jazzy/rmw_implementation/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update quality declaration document (`#225 <https://github.com/ros2/rmw_implementation/issues/225>`__) (`#226 <https://github.com/ros2/rmw_implementation/issues/226>`__)
* Switch to using target_link_libraries everywhere. (`#222 <https://github.com/ros2/rmw_implementation/issues/222>`__)
* Add rmw_count_clients,services & test (`#208 <https://github.com/ros2/rmw_implementation/issues/208>`__)
* Contributors: Chris Lalancette, Lee, Minju, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`robot_state_publisher <https://github.com/ros/robot_state_publisher/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix reload after a description with a mimic joint (`#212 <https://github.com/ros/robot_state_publisher/issues/212>`__)
* Remove ament_target_dependencies. (`#209 <https://github.com/ros/robot_state_publisher/issues/209>`__)
* Improve log messages (`#206 <https://github.com/ros/robot_state_publisher/issues/206>`__)
* Contributors: Chris Lalancette, Guillaume Doisy, Nick Lamprianidis


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2action <https://github.com/ros2/ros2cli/tree/jazzy/ros2action/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* call get_action_interfaces() properly. (`#898 <https://github.com/ros2/ros2cli/issues/898>`__) (`#900 <https://github.com/ros2/ros2cli/issues/900>`__) (cherry picked from commit 305ef763b83e42ebddc4802ac788869d178b6e93) Co-authored-by: Tomoya Fujita <Tomoya.Fujita@sony.com>
* support ``ros2 action type <action name>``. (`#894 <https://github.com/ros2/ros2cli/issues/894>`__) * support ``ros2 action type <action name>``. * add review comments. ---------
* Load a message/request/goal from standard input (`#844 <https://github.com/ros2/ros2cli/issues/844>`__)
* Contributors: Tomoya Fujita, mergify[bot], ymd-stella


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2bag <https://github.com/ros2/rosbag2/tree/jazzy/ros2bag/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add option to disable recorder keyboard controls (`#1607 <https://github.com/ros2/rosbag2/issues/1607>`__)
* Support service 2/2 --- rosbag2 service play (`#1481 <https://github.com/ros2/rosbag2/issues/1481>`__)
* Added exclude-topic-types to record (`#1582 <https://github.com/ros2/rosbag2/issues/1582>`__)
* Overhaul in the rosbag2_transport::TopicFilter class and relevant tests (`#1585 <https://github.com/ros2/rosbag2/issues/1585>`__)
* Filter topic by type  (`#1577 <https://github.com/ros2/rosbag2/issues/1577>`__)
* Implement service recording and display info about recorded services (`#1480 <https://github.com/ros2/rosbag2/issues/1480>`__)
* Add python3-yaml as a dependency (`#1490 <https://github.com/ros2/rosbag2/issues/1490>`__)
* Fix the description of paramter '--topics' for play (`#1426 <https://github.com/ros2/rosbag2/issues/1426>`__)
* When using sim time, wait for /clock before beginning recording (`#1378 <https://github.com/ros2/rosbag2/issues/1378>`__)
* Revert "Don't record sim-time messages before first /clock (`#1354 <https://github.com/ros2/rosbag2/issues/1354>`__)" (`#1377 <https://github.com/ros2/rosbag2/issues/1377>`__)
* Don't record sim-time messages before first /clock (`#1354 <https://github.com/ros2/rosbag2/issues/1354>`__)
* Fix wrong descritpion for '--ignore-leaf-topics' (`#1344 <https://github.com/ros2/rosbag2/issues/1344>`__)
* Cleanup the help text for ros2 bag record. (`#1329 <https://github.com/ros2/rosbag2/issues/1329>`__)
* Contributors: Alejandro Hernández Cordero, Barry Xu, Bernd Pfrommer, Chris Lalancette, Emerson Knapp, Michael Orlov, Michal Sojka


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2cli <https://github.com/ros2/ros2cli/tree/jazzy/ros2cli/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* ros2cli.node.daemon : try getting fdsize from /proc for open fd limit (`#888 <https://github.com/ros2/ros2cli/issues/888>`__)
* Fix the SIGTERM handling in the ros2 daemon. (`#887 <https://github.com/ros2/ros2cli/issues/887>`__)
* Replace unmaintained ``netifaces`` library to avoid local wheel builds (`#875 <https://github.com/ros2/ros2cli/issues/875>`__)
* make handles not inheritable to prevent from blocking durning tab-completion (`#852 <https://github.com/ros2/ros2cli/issues/852>`__)
* Add ros2 service info (`#771 <https://github.com/ros2/ros2cli/issues/771>`__)
* catch ExternalShutdownException ros2cli main. (`#854 <https://github.com/ros2/ros2cli/issues/854>`__)
* Load a message/request/goal from standard input (`#844 <https://github.com/ros2/ros2cli/issues/844>`__)
* Fix tests with get_type_description service and param present (`#838 <https://github.com/ros2/ros2cli/issues/838>`__)
* Add marshalling functions for rclpy.type_hash.TypeHash (rep2011) (`#816 <https://github.com/ros2/ros2cli/issues/816>`__)
* [service introspection] ros2 service echo (`#745 <https://github.com/ros2/ros2cli/issues/745>`__)
* Contributors: Brian, Chen Lihui, Chris Lalancette, Emerson Knapp, Hans-Joachim Krauch, Laurenz, Lee, Minju, Tomoya Fujita, akssri-sony, ymd-stella


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2cli_test_interfaces <https://github.com/ros2/ros2cli/tree/jazzy/ros2cli_test_interfaces/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update to C++17 (`#848 <https://github.com/ros2/ros2cli/issues/848>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2component <https://github.com/ros2/ros2cli/tree/jazzy/ros2component/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Warning: get_parameter_value() is deprecated. (`#866 <https://github.com/ros2/ros2cli/issues/866>`__)
* Contributors: Tomoya Fujita


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2doctor <https://github.com/ros2/ros2cli/tree/jazzy/ros2doctor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Remove references to https://index.ros.org (`#897 <https://github.com/ros2/ros2cli/issues/897>`__)
* (ros2doctor) fix PackageCheck (`#860 <https://github.com/ros2/ros2cli/issues/860>`__) * (ros2doctor)(package) improve result string generation
* Shutdown ros2doctor hello when ctrl-c is received (`#826 <https://github.com/ros2/ros2cli/issues/826>`__)
* Contributors: Chris Lalancette, Matthijs van der Burgh, Michael Carroll


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2interface <https://github.com/ros2/ros2cli/tree/jazzy/ros2interface/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add interface type filters to ros2 interface package (`#765 <https://github.com/ros2/ros2cli/issues/765>`__)
* Contributors: David V. Lu!!


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2param <https://github.com/ros2/ros2cli/tree/jazzy/ros2param/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* ros2 param dump should handle empty list as exception. (`#881 <https://github.com/ros2/ros2cli/issues/881>`__)
* Warning: get_parameter_value() is deprecated. (`#866 <https://github.com/ros2/ros2cli/issues/866>`__)
* Fix tests with get_type_description service and param present (`#838 <https://github.com/ros2/ros2cli/issues/838>`__)
* Update ros2 param dump dosctring. (`#837 <https://github.com/ros2/ros2cli/issues/837>`__)
* Contributors: Emerson Knapp, Murilo M Marinho, Tomoya Fujita


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2pkg <https://github.com/ros2/ros2cli/tree/jazzy/ros2pkg/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the package template for our new include directories. (`#847 <https://github.com/ros2/ros2cli/issues/847>`__)
* Fix typo in ros2pkg warning message. (`#827 <https://github.com/ros2/ros2cli/issues/827>`__)
* Contributors: Chris Lalancette, Tomoya Fujita


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2service <https://github.com/ros2/ros2cli/tree/jazzy/ros2service/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add ros2 service info (`#771 <https://github.com/ros2/ros2cli/issues/771>`__)
* Load a message/request/goal from standard input (`#844 <https://github.com/ros2/ros2cli/issues/844>`__)
* Fix tests with get_type_description service and param present (`#838 <https://github.com/ros2/ros2cli/issues/838>`__)
* [service introspection] ros2 service echo (`#745 <https://github.com/ros2/ros2cli/issues/745>`__)
* Contributors: Brian, Emerson Knapp, Lee, Minju, ymd-stella


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2topic <https://github.com/ros2/ros2cli/tree/jazzy/ros2topic/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Remove parentheses from assert statement. (`#878 <https://github.com/ros2/ros2cli/issues/878>`__)
* Load a message/request/goal from standard input (`#844 <https://github.com/ros2/ros2cli/issues/844>`__)
* Add marshalling functions for rclpy.type_hash.TypeHash (rep2011) (`#816 <https://github.com/ros2/ros2cli/issues/816>`__)
* [service introspection] ros2 service echo (`#745 <https://github.com/ros2/ros2cli/issues/745>`__)
* Contributors: Brian, Chris Lalancette, Hans-Joachim Krauch, ymd-stella


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2trace <https://github.com/ros2/ros2_tracing/tree/jazzy/ros2trace/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Replace all occurences of index.ros.org (`#114 <https://github.com/ros2/ros2_tracing/issues/114>`__)
* Create start/pause/resume/stop sub-commands for 'ros2 trace' (`#70 <https://github.com/ros2/ros2_tracing/issues/70>`__)
* Switch <depend> to <exec_depend> in pure Python packages (`#67 <https://github.com/ros2/ros2_tracing/issues/67>`__)
* Contributors: Chris Lalancette, Christophe Bedard


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_compression <https://github.com/ros2/rosbag2/tree/jazzy/rosbag2_compression/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use middleware send and receive timestamps from message_info during recording (`#1531 <https://github.com/ros2/rosbag2/issues/1531>`__)
* Use std::filesystem instead of rcpputils::fs (`#1576 <https://github.com/ros2/rosbag2/issues/1576>`__)
* Make some changes for newer versions of uncrustify. (`#1578 <https://github.com/ros2/rosbag2/issues/1578>`__)
* Add topic_id returned by storage to the TopicMetadata (`#1538 <https://github.com/ros2/rosbag2/issues/1538>`__)
* Add default initialization for CompressionOptions (`#1539 <https://github.com/ros2/rosbag2/issues/1539>`__)
* Add option to set compression threads priority (`#1457 <https://github.com/ros2/rosbag2/issues/1457>`__)
* Fixes pointed out by clang. (`#1493 <https://github.com/ros2/rosbag2/issues/1493>`__)
* Use enum values for offered_qos_profiles in code and string names in serialized metadata (`#1476 <https://github.com/ros2/rosbag2/issues/1476>`__)
* Add in a missing cstdint include. (`#1321 <https://github.com/ros2/rosbag2/issues/1321>`__)
* Fix warning from ClassLoader in sequential compression reader and writer (`#1299 <https://github.com/ros2/rosbag2/issues/1299>`__)
* Contributors: Arne B, Chris Lalancette, Michael Orlov, Patrick Roncagliolo, Roman Sokolkov, jmachowinski


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_compression_zstd <https://github.com/ros2/rosbag2/tree/jazzy/rosbag2_compression_zstd/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use std::filesystem instead of rcpputils::fs (`#1576 <https://github.com/ros2/rosbag2/issues/1576>`__)
* Make some changes for newer versions of uncrustify. (`#1578 <https://github.com/ros2/rosbag2/issues/1578>`__)
* Contributors: Chris Lalancette, Roman Sokolkov


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_cpp <https://github.com/ros2/rosbag2/tree/jazzy/rosbag2_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Support service 2/2 --- rosbag2 service play (`#1481 <https://github.com/ros2/rosbag2/issues/1481>`__)
* Use middleware send and receive timestamps from message_info during recording (`#1531 <https://github.com/ros2/rosbag2/issues/1531>`__)
* Update to use yaml-cpp version 0.8.0. (`#1605 <https://github.com/ros2/rosbag2/issues/1605>`__)
* Use std::filesystem instead of rcpputils::fs (`#1576 <https://github.com/ros2/rosbag2/issues/1576>`__)
* Make some changes for newer versions of uncrustify. (`#1578 <https://github.com/ros2/rosbag2/issues/1578>`__)
* Add topic_id returned by storage to the TopicMetadata (`#1538 <https://github.com/ros2/rosbag2/issues/1538>`__)
* call cv.wait_until only if necessary. (`#1521 <https://github.com/ros2/rosbag2/issues/1521>`__)
* Implement service recording and display info about recorded services (`#1480 <https://github.com/ros2/rosbag2/issues/1480>`__)
* Switch to target_link_libraries everywhere. (`#1504 <https://github.com/ros2/rosbag2/issues/1504>`__)
* Use enum values for offered_qos_profiles in code and string names in serialized metadata (`#1476 <https://github.com/ros2/rosbag2/issues/1476>`__)
* ros2 bag convert now excludes messages not in [start_time;end_time] (`#1455 <https://github.com/ros2/rosbag2/issues/1455>`__)
* Replace TSAUniqueLock implementation with rcpputils::unique_lock (`#1454 <https://github.com/ros2/rosbag2/issues/1454>`__)
* Add BagSplitInfo service call on bag close (`#1422 <https://github.com/ros2/rosbag2/issues/1422>`__)
* Rewrite TimeControllerClockTest.unpaused_sleep_returns_true to be correct (`#1384 <https://github.com/ros2/rosbag2/issues/1384>`__)
* Implement storing and loading ROS_DISTRO from metadata.yaml and mcap files (`#1241 <https://github.com/ros2/rosbag2/issues/1241>`__)
* Don't crash when type definition cannot be found (`#1350 <https://github.com/ros2/rosbag2/issues/1350>`__)
* Add recorder stop() API (`#1300 <https://github.com/ros2/rosbag2/issues/1300>`__)
* Contributors: Barry Xu, Chris Lalancette, Emerson Knapp, Michael Orlov, Patrick Roncagliolo, Peter Favrholdt, Roman Sokolkov, Tomoya Fujita, jmachowinski


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_examples_cpp <https://github.com/ros2/rosbag2/tree/jazzy/rosbag2_examples/rosbag2_examples_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add topic_id returned by storage to the TopicMetadata (`#1538 <https://github.com/ros2/rosbag2/issues/1538>`__)
* Use enum values for offered_qos_profiles in code and string names in serialized metadata (`#1476 <https://github.com/ros2/rosbag2/issues/1476>`__)
* Contributors: Michael Orlov, Patrick Roncagliolo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_examples_py <https://github.com/ros2/rosbag2/tree/jazzy/rosbag2_examples/rosbag2_examples_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add topic_id returned by storage to the TopicMetadata (`#1538 <https://github.com/ros2/rosbag2/issues/1538>`__)
* Fix a warning from python setuptools. (`#1312 <https://github.com/ros2/rosbag2/issues/1312>`__)
* Contributors: Chris Lalancette, Michael Orlov


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_interfaces <https://github.com/ros2/rosbag2/tree/jazzy/rosbag2_interfaces/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add node name to the Read(Write)SplitEvent message (`#1609 <https://github.com/ros2/rosbag2/issues/1609>`__)
* Contributors: Michael Orlov


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_performance_benchmarking <https://github.com/ros2/rosbag2/tree/jazzy/rosbag2_performance/rosbag2_performance_benchmarking/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use middleware send and receive timestamps from message_info during recording (`#1531 <https://github.com/ros2/rosbag2/issues/1531>`__)
* Update to use yaml-cpp version 0.8.0. (`#1605 <https://github.com/ros2/rosbag2/issues/1605>`__)
* Add option to set compression threads priority (`#1457 <https://github.com/ros2/rosbag2/issues/1457>`__)
* Add per group statistics for rosbag2_performance_benchmarking report (`#1306 <https://github.com/ros2/rosbag2/issues/1306>`__)
* Set CPU affinity for producers and recorder from benchmark parameters (`#1305 <https://github.com/ros2/rosbag2/issues/1305>`__)
* Add CPU usage to rosbag2_performance_benchmarking results report (`#1304 <https://github.com/ros2/rosbag2/issues/1304>`__)
* Add config option to use storage_id parameter in benchmark_launch.py (`#1303 <https://github.com/ros2/rosbag2/issues/1303>`__)
* Contributors: Chris Lalancette, Michael Orlov, jmachowinski


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_py <https://github.com/ros2/rosbag2/tree/jazzy/rosbag2_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add option to disable recorder keyboard controls (`#1607 <https://github.com/ros2/rosbag2/issues/1607>`__)
* Support service 2/2 --- rosbag2 service play (`#1481 <https://github.com/ros2/rosbag2/issues/1481>`__)
* Use middleware send and receive timestamps from message_info during recording (`#1531 <https://github.com/ros2/rosbag2/issues/1531>`__)
* Switch rclpy to be an exec_depend here. (`#1606 <https://github.com/ros2/rosbag2/issues/1606>`__)
* Gracefully handle SIGINT and SIGTERM signals for play and burst CLI (`#1557 <https://github.com/ros2/rosbag2/issues/1557>`__)
* Added exclude-topic-types to record (`#1582 <https://github.com/ros2/rosbag2/issues/1582>`__)
* Fix for false negative tests in rosbag2_py (`#1592 <https://github.com/ros2/rosbag2/issues/1592>`__)
* Update rosbag2_py stubs (`#1593 <https://github.com/ros2/rosbag2/issues/1593>`__)
* Add Python stubs for rosbag2_py (`#1459 <https://github.com/ros2/rosbag2/issues/1459>`__) (`#1569 <https://github.com/ros2/rosbag2/issues/1569>`__)
* Filter topic by type  (`#1577 <https://github.com/ros2/rosbag2/issues/1577>`__)
* Add topic_id returned by storage to the TopicMetadata (`#1538 <https://github.com/ros2/rosbag2/issues/1538>`__)
* Install signal handlers in recorder only inside record method (`#1464 <https://github.com/ros2/rosbag2/issues/1464>`__)
* add missing import otherwise it doesnt compile (`#1524 <https://github.com/ros2/rosbag2/issues/1524>`__)
* Implement service recording and display info about recorded services (`#1480 <https://github.com/ros2/rosbag2/issues/1480>`__)
* Make ``rosbag2_transport::Player::play()`` run in a separate thread (`#1503 <https://github.com/ros2/rosbag2/issues/1503>`__)
* Switch to target_link_libraries everywhere. (`#1504 <https://github.com/ros2/rosbag2/issues/1504>`__)
* Use enum values for offered_qos_profiles in code and string names in serialized metadata (`#1476 <https://github.com/ros2/rosbag2/issues/1476>`__)
* ros2 bag convert now excludes messages not in [start_time;end_time] (`#1455 <https://github.com/ros2/rosbag2/issues/1455>`__)
* Add support for compression to python API (`#1425 <https://github.com/ros2/rosbag2/issues/1425>`__)
* Gracefully handle SIGINT and SIGTERM in rosbag2 recorder (`#1301 <https://github.com/ros2/rosbag2/issues/1301>`__)
* Implement storing and loading ROS_DISTRO from metadata.yaml and mcap files (`#1241 <https://github.com/ros2/rosbag2/issues/1241>`__)
* Add binding to close the writer (`#1339 <https://github.com/ros2/rosbag2/issues/1339>`__)
* Contributors: Alejandro Hernández Cordero, Andrew Symington, Barry Xu, Bernd Pfrommer, Chris Lalancette, Emerson Knapp, Michael Orlov, Mikael Arguedas, Patrick Roncagliolo, Peter Favrholdt, Roman Sokolkov, Yadu, jmachowinski


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_storage <https://github.com/ros2/rosbag2/tree/jazzy/rosbag2_storage/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Support service 2/2 --- rosbag2 service play (`#1481 <https://github.com/ros2/rosbag2/issues/1481>`__)
* Use middleware send and receive timestamps from message_info during recording (`#1531 <https://github.com/ros2/rosbag2/issues/1531>`__)
* Update to use yaml-cpp version 0.8.0. (`#1605 <https://github.com/ros2/rosbag2/issues/1605>`__)
* Use std::filesystem instead of rcpputils::fs (`#1576 <https://github.com/ros2/rosbag2/issues/1576>`__)
* Make some changes for newer versions of uncrustify. (`#1578 <https://github.com/ros2/rosbag2/issues/1578>`__)
* Add topic_id returned by storage to the TopicMetadata (`#1538 <https://github.com/ros2/rosbag2/issues/1538>`__)
* Remove rcpputils::fs dependencies from rosbag2_storages (`#1558 <https://github.com/ros2/rosbag2/issues/1558>`__)
* Improve performance in SqliteStorage::get_bagfile_size() (`#1516 <https://github.com/ros2/rosbag2/issues/1516>`__)
* Make Player and Recorder Composable (`#902 <https://github.com/ros2/rosbag2/issues/902>`__) (`#1419 <https://github.com/ros2/rosbag2/issues/1419>`__)
* Use enum values for offered_qos_profiles in code and string names in serialized metadata (`#1476 <https://github.com/ros2/rosbag2/issues/1476>`__)
* ros2 bag convert now excludes messages not in [start_time;end_time] (`#1455 <https://github.com/ros2/rosbag2/issues/1455>`__)
* Fix missing cstdint include (`#1383 <https://github.com/ros2/rosbag2/issues/1383>`__)
* Implement storing and loading ROS_DISTRO from metadata.yaml and mcap files (`#1241 <https://github.com/ros2/rosbag2/issues/1241>`__)
* Contributors: Barry Xu, Chris Lalancette, Emerson Knapp, Michael Orlov, Patrick Roncagliolo, Peter Favrholdt, Roman Sokolkov, Zac Stanton, jmachowinski


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_storage_mcap <https://github.com/ros2/rosbag2/tree/jazzy/rosbag2_storage_mcap/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Support service 2/2 --- rosbag2 service play (`#1481 <https://github.com/ros2/rosbag2/issues/1481>`__)
* Use middleware send and receive timestamps from message_info during recording (`#1531 <https://github.com/ros2/rosbag2/issues/1531>`__)
* Update to use yaml-cpp version 0.8.0. (`#1605 <https://github.com/ros2/rosbag2/issues/1605>`__)
* Check existence of a file before passing it to the mcap reader (`#1594 <https://github.com/ros2/rosbag2/issues/1594>`__)
* Add topic_id returned by storage to the TopicMetadata (`#1538 <https://github.com/ros2/rosbag2/issues/1538>`__)
* Use rw_lock to protect mcap metadata lists. (`#1561 <https://github.com/ros2/rosbag2/issues/1561>`__)
* Remove rcpputils::fs dependencies from rosbag2_storages (`#1558 <https://github.com/ros2/rosbag2/issues/1558>`__)
* remove unused headers (`#1544 <https://github.com/ros2/rosbag2/issues/1544>`__)
* Link and compile against rosbag2_storage_mcap: Fixed issue 1492 (`#1496 <https://github.com/ros2/rosbag2/issues/1496>`__)
* Use enum values for offered_qos_profiles in code and string names in serialized metadata (`#1476 <https://github.com/ros2/rosbag2/issues/1476>`__)
* Store serialized metadata in MCAP file (`#1423 <https://github.com/ros2/rosbag2/issues/1423>`__)
* Implement storing and loading ROS_DISTRO from metadata.yaml and mcap files (`#1241 <https://github.com/ros2/rosbag2/issues/1241>`__)
* Contributors: Alejandro Hernández Cordero, Barry Xu, Chris Lalancette, Christopher Wecht, Emerson Knapp, Michael Orlov, Patrick Roncagliolo, Roman Sokolkov, Tomoya Fujita, jmachowinski, uupks


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_storage_sqlite3 <https://github.com/ros2/rosbag2/tree/jazzy/rosbag2_storage_sqlite3/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Support service 2/2 --- rosbag2 service play (`#1481 <https://github.com/ros2/rosbag2/issues/1481>`__)
* Use middleware send and receive timestamps from message_info during recording (`#1531 <https://github.com/ros2/rosbag2/issues/1531>`__)
* Update to use yaml-cpp version 0.8.0. (`#1605 <https://github.com/ros2/rosbag2/issues/1605>`__)
* Make some changes for newer versions of uncrustify. (`#1578 <https://github.com/ros2/rosbag2/issues/1578>`__)
* Add topic_id returned by storage to the TopicMetadata (`#1538 <https://github.com/ros2/rosbag2/issues/1538>`__)
* Remove rcpputils::fs dependencies from rosbag2_storages (`#1558 <https://github.com/ros2/rosbag2/issues/1558>`__)
* Change an incorrect TSA annotation. (`#1552 <https://github.com/ros2/rosbag2/issues/1552>`__)
* Improve performance in SqliteStorage::get_bagfile_size() (`#1516 <https://github.com/ros2/rosbag2/issues/1516>`__)
* Update rosbag2_storage_sqlite3 to C++17. (`#1501 <https://github.com/ros2/rosbag2/issues/1501>`__)
* Use enum values for offered_qos_profiles in code and string names in serialized metadata (`#1476 <https://github.com/ros2/rosbag2/issues/1476>`__)
* Stop inheriting from std::iterator. (`#1424 <https://github.com/ros2/rosbag2/issues/1424>`__)
* Implement storing and loading ROS_DISTRO from metadata.yaml and mcap files (`#1241 <https://github.com/ros2/rosbag2/issues/1241>`__)
* Store metadata in db3 file (`#1294 <https://github.com/ros2/rosbag2/issues/1294>`__)
* Contributors: Barry Xu, Chris Lalancette, Emerson Knapp, Michael Orlov, Patrick Roncagliolo, Roman Sokolkov, jmachowinski


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_test_common <https://github.com/ros2/rosbag2/tree/jazzy/rosbag2_test_common/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Support service 2/2 --- rosbag2 service play (`#1481 <https://github.com/ros2/rosbag2/issues/1481>`__)
* Make some changes for newer versions of uncrustify. (`#1578 <https://github.com/ros2/rosbag2/issues/1578>`__)
* Implement service recording and display info about recorded services (`#1480 <https://github.com/ros2/rosbag2/issues/1480>`__)
* Add extra checks in execute_and_wait_until_completion(..) (`#1346 <https://github.com/ros2/rosbag2/issues/1346>`__)
* Address flakiness in rosbag2_play_end_to_end tests (`#1297 <https://github.com/ros2/rosbag2/issues/1297>`__)
* Contributors: Barry Xu, Chris Lalancette, Michael Orlov


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_test_msgdefs <https://github.com/ros2/rosbag2/tree/jazzy/rosbag2_test_msgdefs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Implement service recording and display info about recorded services (`#1480 <https://github.com/ros2/rosbag2/issues/1480>`__)
* Don't crash when type definition cannot be found (`#1350 <https://github.com/ros2/rosbag2/issues/1350>`__) * Don't fail when type definition cannot be found
* Contributors: Barry Xu, Emerson Knapp


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_tests <https://github.com/ros2/rosbag2/tree/jazzy/rosbag2_tests/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use middleware send and receive timestamps from message_info during recording (`#1531 <https://github.com/ros2/rosbag2/issues/1531>`__)
* Added exclude-topic-types to record (`#1582 <https://github.com/ros2/rosbag2/issues/1582>`__)
* Use std::filesystem instead of rcpputils::fs (`#1576 <https://github.com/ros2/rosbag2/issues/1576>`__)
* Filter topic by type  (`#1577 <https://github.com/ros2/rosbag2/issues/1577>`__)
* Make some changes for newer versions of uncrustify. (`#1578 <https://github.com/ros2/rosbag2/issues/1578>`__)
* Add topic_id returned by storage to the TopicMetadata (`#1538 <https://github.com/ros2/rosbag2/issues/1538>`__)
* Improve performance in SqliteStorage::get_bagfile_size() (`#1516 <https://github.com/ros2/rosbag2/issues/1516>`__)
* Implement service recording and display info about recorded services (`#1480 <https://github.com/ros2/rosbag2/issues/1480>`__)
* Mark play_end_to_end test as xfail in Windows (`#1452 <https://github.com/ros2/rosbag2/issues/1452>`__)
* Implement storing and loading ROS_DISTRO from metadata.yaml and mcap files (`#1241 <https://github.com/ros2/rosbag2/issues/1241>`__)
* Address flakiness in rosbag2_play_end_to_end tests (`#1297 <https://github.com/ros2/rosbag2/issues/1297>`__)
* Contributors: Alejandro Hernández Cordero, Barry Xu, Chris Lalancette, Cristóbal Arroyo, Emerson Knapp, Michael Orlov, Roman Sokolkov, jmachowinski


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_transport <https://github.com/ros2/rosbag2/tree/jazzy/rosbag2_transport/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Removed warnings - unqualified-std-cast-call (`#1618 <https://github.com/ros2/rosbag2/issues/1618>`__) (`#1622 <https://github.com/ros2/rosbag2/issues/1622>`__)
* Add node name to the Read(Write)SplitEvent message (`#1609 <https://github.com/ros2/rosbag2/issues/1609>`__)
* Add option to disable recorder keyboard controls (`#1607 <https://github.com/ros2/rosbag2/issues/1607>`__)
* Support service 2/2 --- rosbag2 service play (`#1481 <https://github.com/ros2/rosbag2/issues/1481>`__)
* Use middleware send and receive timestamps from message_info during recording (`#1531 <https://github.com/ros2/rosbag2/issues/1531>`__)
* Update to use yaml-cpp version 0.8.0. (`#1605 <https://github.com/ros2/rosbag2/issues/1605>`__)
* Gracefully handle SIGINT and SIGTERM signals for play and burst CLI (`#1557 <https://github.com/ros2/rosbag2/issues/1557>`__)
* Added exclude-topic-types to record (`#1582 <https://github.com/ros2/rosbag2/issues/1582>`__)
* Use std::filesystem instead of rcpputils::fs (`#1576 <https://github.com/ros2/rosbag2/issues/1576>`__)
* Add transactional state mutex for RecorderImpl class. (`#1547 <https://github.com/ros2/rosbag2/issues/1547>`__)
* Overhaul in the rosbag2_transport::TopicFilter class and relevant tests (`#1585 <https://github.com/ros2/rosbag2/issues/1585>`__)
* Filter topic by type  (`#1577 <https://github.com/ros2/rosbag2/issues/1577>`__)
* fix: use size_t instead of uint64_t in play_options YAML converter (`#1575 <https://github.com/ros2/rosbag2/issues/1575>`__)
* Make some changes for newer versions of uncrustify. (`#1578 <https://github.com/ros2/rosbag2/issues/1578>`__)
* Add topic_id returned by storage to the TopicMetadata (`#1538 <https://github.com/ros2/rosbag2/issues/1538>`__)
* Workaround for flaky test_play_services running with fastrtps (`#1556 <https://github.com/ros2/rosbag2/issues/1556>`__)
* Add proper message for --start-paused (`#1537 <https://github.com/ros2/rosbag2/issues/1537>`__)
* ``Recording stopped`` prints only once. (`#1530 <https://github.com/ros2/rosbag2/issues/1530>`__)
* Cleanup the rosbag2_transport tests (`#1518 <https://github.com/ros2/rosbag2/issues/1518>`__)
* Implement service recording and display info about recorded services (`#1480 <https://github.com/ros2/rosbag2/issues/1480>`__)
* Add option to set compression threads priority (`#1457 <https://github.com/ros2/rosbag2/issues/1457>`__)
* Bugfix for incorrect playback rate changes when pressing buttons (`#1513 <https://github.com/ros2/rosbag2/issues/1513>`__)
* Make Player and Recorder Composable (`#902 <https://github.com/ros2/rosbag2/issues/902>`__) (`#1419 <https://github.com/ros2/rosbag2/issues/1419>`__)
* Clang fixes for the latest PlayerImpl code. (`#1507 <https://github.com/ros2/rosbag2/issues/1507>`__)
* Make ``rosbag2_transport::Player::play()`` run in a separate thread (`#1503 <https://github.com/ros2/rosbag2/issues/1503>`__)
* Switch to target_link_libraries everywhere. (`#1504 <https://github.com/ros2/rosbag2/issues/1504>`__)
* Use enum values for offered_qos_profiles in code and string names in serialized metadata (`#1476 <https://github.com/ros2/rosbag2/issues/1476>`__)
* Redesign Player class with PIMPL idiom (`#1447 <https://github.com/ros2/rosbag2/issues/1447>`__)
* Don't warn for unknown types if topics are not selected (`#1466 <https://github.com/ros2/rosbag2/issues/1466>`__)
* Remove unused concurrentqueue implementation. (`#1465 <https://github.com/ros2/rosbag2/issues/1465>`__)
* Fix uninitialized value pointed out by clang static analysis. (`#1440 <https://github.com/ros2/rosbag2/issues/1440>`__)
* Fix the build with rmw_fastrtps_dynamic. (`#1416 <https://github.com/ros2/rosbag2/issues/1416>`__)
* Fix for rosbag2_transport::Recorder failures due to the unhandled exceptions (`#1382 <https://github.com/ros2/rosbag2/issues/1382>`__)
* When using sim time, wait for /clock before beginning recording (`#1378 <https://github.com/ros2/rosbag2/issues/1378>`__)
* Fix for possible freeze in Recorder::stop() (`#1381 <https://github.com/ros2/rosbag2/issues/1381>`__)
* Revert "Don't record sim-time messages before first /clock (`#1354 <https://github.com/ros2/rosbag2/issues/1354>`__)" (`#1377 <https://github.com/ros2/rosbag2/issues/1377>`__)
* Don't record sim-time messages before first /clock (`#1354 <https://github.com/ros2/rosbag2/issues/1354>`__)
* Fix a clang warning about uninitialized variable. (`#1370 <https://github.com/ros2/rosbag2/issues/1370>`__)
* [bugfix] for parameters not passing to recorder's node from child component (`#1360 <https://github.com/ros2/rosbag2/issues/1360>`__)
* Change subscriptions from GenericSubscripton to SubscriptionBase (`#1337 <https://github.com/ros2/rosbag2/issues/1337>`__)
* Add recorder stop() API (`#1300 <https://github.com/ros2/rosbag2/issues/1300>`__)
* Contributors: Alejandro Hernández Cordero, Barry Xu, Bernd Pfrommer, Chris Lalancette, Christoph Fröhlich, Daisuke Nishimatsu, Emerson Knapp, Michael Orlov, Patrick Roncagliolo, Roman Sokolkov, Tomoya Fujita, jmachowinski, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_cmake <https://github.com/ros2/rosidl/tree/jazzy/rosidl_cmake/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Improve deprecation notice of rosidl_target_interface to give a hint on how to update the code (`#788 <https://github.com/ros2/rosidl/issues/788>`__)
* Add rosidl_find_package_idl helper function (`#754 <https://github.com/ros2/rosidl/issues/754>`__)
* Remove unused splitting of .srv files in CMake (`#753 <https://github.com/ros2/rosidl/issues/753>`__)
* Contributors: Alexis Paques, Mike Purvis, Shane Loretz


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_dynamic_typesupport <https://github.com/ros2/rosidl_dynamic_typesupport/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* uchar: fix conditional include/typedef (`#10 <https://github.com/ros2/rosidl_dynamic_typesupport/issues/10>`__)
* uchar: use __has_include(..) on separate line (`#8 <https://github.com/ros2/rosidl_dynamic_typesupport/issues/8>`__)
* Refactor the handling of nested types. (`#7 <https://github.com/ros2/rosidl_dynamic_typesupport/issues/7>`__)
* Add C++ version check to char16 definition (`#3 <https://github.com/ros2/rosidl_dynamic_typesupport/issues/3>`__)
* Contributors: Antonio Cuadros, Chris Lalancette, G.A. vd. Hoorn


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_generator_c <https://github.com/ros2/rosidl/tree/jazzy/rosidl_generator_c/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fixed warnings - strict-prototypes (`#800 <https://github.com/ros2/rosidl/issues/800>`__) (`#802 <https://github.com/ros2/rosidl/issues/802>`__)
* Set hints to find the python version we actually want. (`#785 <https://github.com/ros2/rosidl/issues/785>`__)
* Add rosidl_find_package_idl helper function (`#754 <https://github.com/ros2/rosidl/issues/754>`__)
* Fix IWYU for clangd in C and C++ (`#742 <https://github.com/ros2/rosidl/issues/742>`__)
* Contributors: Alexis Paques, Chris Lalancette, Mike Purvis, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_generator_cpp <https://github.com/ros2/rosidl/tree/jazzy/rosidl_generator_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Set hints to find the python version we actually want. (`#785 <https://github.com/ros2/rosidl/issues/785>`__)
* Fix constant generation for C++ floats (`#772 <https://github.com/ros2/rosidl/issues/772>`__)
* Add rosidl_find_package_idl helper function (`#754 <https://github.com/ros2/rosidl/issues/754>`__)
* Fixed visibility control file added to wrong header list variable. (`#755 <https://github.com/ros2/rosidl/issues/755>`__)
* Fix deprecation warnings for message constants (`#750 <https://github.com/ros2/rosidl/issues/750>`__)
* Generate typesupport declarations for actions, messages and services (`#703 <https://github.com/ros2/rosidl/issues/703>`__)
* Fix IWYU for clangd in C and C++ (`#742 <https://github.com/ros2/rosidl/issues/742>`__)
* Contributors: Alexis Paques, Chris Lalancette, Emerson Knapp, Mike Purvis, Stefan Fabian


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_generator_dds_idl <https://github.com/ros2/rosidl_dds/tree/jazzy/rosidl_generator_dds_idl/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Remove unnecessary parentheses. (`#61 <https://github.com/ros2/rosidl_dds/issues/61>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_generator_py <https://github.com/ros2/rosidl_python/tree/jazzy/rosidl_generator_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Revert install of .so files into python path (`#211 <https://github.com/ros2/rosidl_python/issues/211>`__) There seems that some regression might have happened after `#195 <https://github.com/ros2/rosidl_python/issues/195>`__. When removing those 2 lines, we avoid to install the .so files in lib *and* python path.
* Prototype code for seeing if FindPython3 is usable for rosidl_python (`#140 <https://github.com/ros2/rosidl_python/issues/140>`__)
* Add in a missing space. (`#203 <https://github.com/ros2/rosidl_python/issues/203>`__)
* Install compiled libraries only to 'lib' (`#195 <https://github.com/ros2/rosidl_python/issues/195>`__)
* Fix: Missing dependency that causes cmake error in downstream (resolves https://github.com/ros2/rosidl_python/issues/198) (`#199 <https://github.com/ros2/rosidl_python/issues/199>`__)
* Contributors: Chris Lalancette, Isaac Saito, Matthias Schoepfer, Scott K Logan, Shane Loretz


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_generator_tests <https://github.com/ros2/rosidl/tree/jazzy/rosidl_generator_tests/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fixed warnings - strict-prototypes (`#800 <https://github.com/ros2/rosidl/issues/800>`__) (`#802 <https://github.com/ros2/rosidl/issues/802>`__)
* Increased the cpplint timeout to 300 seconds (`#797 <https://github.com/ros2/rosidl/issues/797>`__)
* Fixes for modern uncrustify. (`#793 <https://github.com/ros2/rosidl/issues/793>`__)
* Fix constant generation for C++ floats (`#772 <https://github.com/ros2/rosidl/issues/772>`__)
* Fix same named types overriding typesources (`#759 <https://github.com/ros2/rosidl/issues/759>`__)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Emerson Knapp, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_generator_type_description <https://github.com/ros2/rosidl/tree/jazzy/rosidl_generator_type_description/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Set hints to find the python version we actually want. (`#785 <https://github.com/ros2/rosidl/issues/785>`__)
* Remove unnecessary parentheses. (`#783 <https://github.com/ros2/rosidl/issues/783>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_parser <https://github.com/ros2/rosidl/tree/jazzy/rosidl_parser/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Adding interfaces to support ``@key`` annotation (`#796 <https://github.com/ros2/rosidl/issues/796>`__) Co-authored-by: Mario Dominguez <mariodominguez@eprosima.com>
* Small fix for newer flake8 compatibility. (`#792 <https://github.com/ros2/rosidl/issues/792>`__)
* Remove unnecessary parentheses. (`#783 <https://github.com/ros2/rosidl/issues/783>`__)
* Contributors: Chris Lalancette, Miguel Company


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_pycommon <https://github.com/ros2/rosidl/tree/jazzy/rosidl_pycommon/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Remove unnecessary parentheses. (`#783 <https://github.com/ros2/rosidl/issues/783>`__)
* Fix same named types overriding typesources (`#759 <https://github.com/ros2/rosidl/issues/759>`__)
* Contributors: Chris Lalancette, Emerson Knapp


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_runtime_c <https://github.com/ros2/rosidl/tree/jazzy/rosidl_runtime_c/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Switch to target_link_libraries. (`#776 <https://github.com/ros2/rosidl/issues/776>`__)
* Set the C++ version to 17. (`#761 <https://github.com/ros2/rosidl/issues/761>`__)
* Mark _ in benchmark tests as unused. (`#741 <https://github.com/ros2/rosidl/issues/741>`__) This helps clang static analysis.
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_runtime_cpp <https://github.com/ros2/rosidl/tree/jazzy/rosidl_runtime_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Suppress a warning around BoundedVector. (`#803 <https://github.com/ros2/rosidl/issues/803>`__) (`#804 <https://github.com/ros2/rosidl/issues/804>`__) The comment has more explanation, but in short GCC 13 has false positives around some warnings, so we suppress it for BoundedVector. (cherry picked from commit 858e76adb03edba00469b91d50dd5fe0dcb34236) Co-authored-by: Chris Lalancette <clalancette@gmail.com>
* Contributors: mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_runtime_py <https://github.com/ros2/rosidl_runtime_py/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Some fixes for modern flake8. (`#25 <https://github.com/ros2/rosidl_runtime_py/issues/25>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_c <https://github.com/ros2/rosidl_typesupport/tree/jazzy/rosidl_typesupport_c/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fixed warnings - strict-prototypes (`#155 <https://github.com/ros2/rosidl_typesupport/issues/155>`__) (`#156 <https://github.com/ros2/rosidl_typesupport/issues/156>`__)
* compare string contents but string pointer addresses. (`#153 <https://github.com/ros2/rosidl_typesupport/issues/153>`__)
* Set hints to find the python version we actually want. (`#150 <https://github.com/ros2/rosidl_typesupport/issues/150>`__)
* Don't override user provided compile definitions (`#145 <https://github.com/ros2/rosidl_typesupport/issues/145>`__)
* Contributors: Chris Lalancette, Emerson Knapp, Tomoya Fujita, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_cpp <https://github.com/ros2/rosidl_typesupport/tree/jazzy/rosidl_typesupport_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* compare string contents but string pointer addresses. (`#153 <https://github.com/ros2/rosidl_typesupport/issues/153>`__)
* Set hints to find the python version we actually want. (`#150 <https://github.com/ros2/rosidl_typesupport/issues/150>`__)
* Don't override user provided compile definitions (`#145 <https://github.com/ros2/rosidl_typesupport/issues/145>`__)
* Added C interfaces to obtain service and action type support. (`#143 <https://github.com/ros2/rosidl_typesupport/issues/143>`__)
* Contributors: Chris Lalancette, Emerson Knapp, Stefan Fabian, Tomoya Fujita


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_fastrtps_c <https://github.com/ros2/rosidl_typesupport_fastrtps/tree/jazzy/rosidl_typesupport_fastrtps_c/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Adding interfaces to support ``@key`` annotation (`#116 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/116>`__) Co-authored-by: Mario Dominguez <mariodominguez@eprosima.com>
* Support Fast CDR v2 (`#114 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/114>`__)
* Improve wide string (de)serialization (`#113 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/113>`__)
* Set hints to find the python version we actually want. (`#112 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/112>`__) Co-authored-by: Michael Carroll <michael@openrobotics.org>
* Update to C++17 (`#111 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/111>`__)
* Account for alignment on ``is_plain`` calculations (`#108 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/108>`__)
* Contributors: Chris Lalancette, Miguel Company


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_fastrtps_cpp <https://github.com/ros2/rosidl_typesupport_fastrtps/tree/jazzy/rosidl_typesupport_fastrtps_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix how header template works to prevent double-inclusion (`#117 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/117>`__) Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Adding interfaces to support ``@key`` annotation (`#116 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/116>`__) Co-authored-by: Mario Dominguez <mariodominguez@eprosima.com>
* Support Fast CDR v2 (`#114 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/114>`__)
* Improve wide string (de)serialization (`#113 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/113>`__)
* Set hints to find the python version we actually want. (`#112 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/112>`__) Co-authored-by: Michael Carroll <michael@openrobotics.org>
* Update to C++17 (`#111 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/111>`__)
* Account for alignment on ``is_plain`` calculations (`#108 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/108>`__)
* Avoid redundant declarations in generated code for services and actions (`#102 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/102>`__)
* Contributors: Chris Lalancette, Emerson Knapp, Michael Carroll, Miguel Company


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_introspection_c <https://github.com/ros2/rosidl/tree/jazzy/rosidl_typesupport_introspection_c/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fixed warnings - strict-prototypes (`#800 <https://github.com/ros2/rosidl/issues/800>`__) (`#802 <https://github.com/ros2/rosidl/issues/802>`__)
* Adding interfaces to support ``@key`` annotation (`#796 <https://github.com/ros2/rosidl/issues/796>`__) Co-authored-by: Mario Dominguez <mariodominguez@eprosima.com>
* Set hints to find the python version we actually want. (`#785 <https://github.com/ros2/rosidl/issues/785>`__)
* Add rosidl_find_package_idl helper function (`#754 <https://github.com/ros2/rosidl/issues/754>`__)
* update comment (`#757 <https://github.com/ros2/rosidl/issues/757>`__)
* Contributors: Chen Lihui, Chris Lalancette, Miguel Company, Mike Purvis, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_introspection_cpp <https://github.com/ros2/rosidl/tree/jazzy/rosidl_typesupport_introspection_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Adding interfaces to support ``@key`` annotation (`#796 <https://github.com/ros2/rosidl/issues/796>`__) Co-authored-by: Mario Dominguez <mariodominguez@eprosima.com>
* Set hints to find the python version we actually want. (`#785 <https://github.com/ros2/rosidl/issues/785>`__)
* Switch to target_link_libraries. (`#776 <https://github.com/ros2/rosidl/issues/776>`__)
* Add rosidl_find_package_idl helper function (`#754 <https://github.com/ros2/rosidl/issues/754>`__)
* update comment (`#757 <https://github.com/ros2/rosidl/issues/757>`__)
* Fix deprecation warnings for message constants (`#750 <https://github.com/ros2/rosidl/issues/750>`__)
* Contributors: Chen Lihui, Chris Lalancette, Emerson Knapp, Miguel Company, Mike Purvis


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_introspection_tests <https://github.com/ros2/rosidl/tree/jazzy/rosidl_typesupport_introspection_tests/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* One last uncrustify fix for newer uncrustify. (`#795 <https://github.com/ros2/rosidl/issues/795>`__)
* Disable zero-variadic-macro-arguments warning when using clang. (`#768 <https://github.com/ros2/rosidl/issues/768>`__)
* Fixed C++20 warning implicit capture of this in lambda (`#766 <https://github.com/ros2/rosidl/issues/766>`__)
* Contributors: AiVerisimilitude, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_tests <https://github.com/ros2/rosidl_typesupport/tree/jazzy/rosidl_typesupport_tests/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Suppress uncrustify on long lines. (`#152 <https://github.com/ros2/rosidl_typesupport/issues/152>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rpyutils <https://github.com/ros2/rpyutils/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* correct the URL and f-strings format (`#11 <https://github.com/ros2/rpyutils/issues/11>`__)
* Contributors: Chen Lihui


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt <https://github.com/ros-visualization/rqt/tree/jazzy/rqt/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add a test dependency on pytest. (`#306 <https://github.com/ros-visualization/rqt/issues/306>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_bag <https://github.com/ros-visualization/rqt_bag/tree/jazzy/rqt_bag/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add in copyright tests to rqt_bag. (`#154 <https://github.com/ros-visualization/rqt_bag/issues/154>`__)
* Add a test dependency on pytest. (`#153 <https://github.com/ros-visualization/rqt_bag/issues/153>`__)
* Revert "Add a dependency on pytest to rqt_bag and rqt_bag_plugins. (#… (`#151 <https://github.com/ros-visualization/rqt_bag/issues/151>`__)
* Update maintainer to myself. (`#150 <https://github.com/ros-visualization/rqt_bag/issues/150>`__)
* Update maintainer list in package.xml files (`#149 <https://github.com/ros-visualization/rqt_bag/issues/149>`__)
* Add a dependency on pytest to rqt_bag and rqt_bag_plugins. (`#148 <https://github.com/ros-visualization/rqt_bag/issues/148>`__)
* [ros2] Enable Save (`#142 <https://github.com/ros-visualization/rqt_bag/issues/142>`__)
* Call close (`#141 <https://github.com/ros-visualization/rqt_bag/issues/141>`__)
* Use default storage id (`#139 <https://github.com/ros-visualization/rqt_bag/issues/139>`__)
* Contributors: Chris Lalancette, Michael Jeronimo, Yadu, Yadunund


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_bag_plugins <https://github.com/ros-visualization/rqt_bag/tree/jazzy/rqt_bag_plugins/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add a test dependency on pytest. (`#153 <https://github.com/ros-visualization/rqt_bag/issues/153>`__)
* Revert "Add a dependency on pytest to rqt_bag and rqt_bag_plugins. (#… (`#151 <https://github.com/ros-visualization/rqt_bag/issues/151>`__)
* Update maintainer to myself. (`#150 <https://github.com/ros-visualization/rqt_bag/issues/150>`__)
* Update maintainer list in package.xml files (`#149 <https://github.com/ros-visualization/rqt_bag/issues/149>`__)
* Add a dependency on pytest to rqt_bag and rqt_bag_plugins. (`#148 <https://github.com/ros-visualization/rqt_bag/issues/148>`__)
* Contributors: Chris Lalancette, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_console <https://github.com/ros-visualization/rqt_console/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add a test dependency on pytest. (`#45 <https://github.com/ros-visualization/rqt_console/issues/45>`__)
* Contributors: Arne Hitzmann, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_graph <https://github.com/ros-visualization/rqt_graph/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#92 <https://github.com/ros-visualization/rqt_graph/issues/92>`__)
* Add a test dependency on python3-pytest. (`#91 <https://github.com/ros-visualization/rqt_graph/issues/91>`__)
* Refresh rosgraph when params checkbox is clicked (`#86 <https://github.com/ros-visualization/rqt_graph/issues/86>`__)
* Contributors: Chris Lalancette, Michael Jeronimo, Yadunund


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_gui_cpp <https://github.com/ros-visualization/rqt/tree/jazzy/rqt_gui_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Switch to target_link_libraries. (`#297 <https://github.com/ros-visualization/rqt/issues/297>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_gui_py <https://github.com/ros-visualization/rqt/tree/jazzy/rqt_gui_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix an exception raised while press ctrl+c to exit (`#291 <https://github.com/ros-visualization/rqt/issues/291>`__)
* Contributors: Chen Lihui


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_msg <https://github.com/ros-visualization/rqt_msg/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add in python3-pytest test dependency. (`#19 <https://github.com/ros-visualization/rqt_msg/issues/19>`__)
* Small cleanups to rqt_msg (`#16 <https://github.com/ros-visualization/rqt_msg/issues/16>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_plot <https://github.com/ros-visualization/rqt_plot/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add in copyright tests to rqt_bag. (`#95 <https://github.com/ros-visualization/rqt_plot/issues/95>`__)
* Add a test dependency on pytest. (`#94 <https://github.com/ros-visualization/rqt_plot/issues/94>`__)
* Add in a pytest dependency for running tests. (`#92 <https://github.com/ros-visualization/rqt_plot/issues/92>`__)
* Fix regression from #87 (`#90 <https://github.com/ros-visualization/rqt_plot/issues/90>`__)
* Contributors: Chris Lalancette, Yadunund


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_publisher <https://github.com/ros-visualization/rqt_publisher/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use raw strings for regular expressions. (`#44 <https://github.com/ros-visualization/rqt_publisher/issues/44>`__)
* Switch maintainer to me. (`#43 <https://github.com/ros-visualization/rqt_publisher/issues/43>`__)
* Update maintainer list in package.xml files (`#42 <https://github.com/ros-visualization/rqt_publisher/issues/42>`__)
* Add in a test dependency on pytest. (`#41 <https://github.com/ros-visualization/rqt_publisher/issues/41>`__)
* Contributors: Chris Lalancette, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_py_common <https://github.com/ros-visualization/rqt/tree/jazzy/rqt_py_common/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Allow to autocomplete namespaced topics (`#299 <https://github.com/ros-visualization/rqt/issues/299>`__)
* Contributors: Alejandro Hernández Cordero


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_py_console <https://github.com/ros-visualization/rqt_py_console/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add in test dependency on pytest. (`#16 <https://github.com/ros-visualization/rqt_py_console/issues/16>`__)
* Fix a crash in the rqt_py_console dialog box. (`#15 <https://github.com/ros-visualization/rqt_py_console/issues/15>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_reconfigure <https://github.com/ros-visualization/rqt_reconfigure/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Explicitly add a pytest test dependency. (`#141 <https://github.com/ros-visualization/rqt_reconfigure/issues/141>`__)
* Remove unnecessary parentheses around if statements. (`#140 <https://github.com/ros-visualization/rqt_reconfigure/issues/140>`__)
* Fixed executor conflict (`#126 <https://github.com/ros-visualization/rqt_reconfigure/issues/126>`__)
* Add param filtering (`#128 <https://github.com/ros-visualization/rqt_reconfigure/issues/128>`__)
* Fix handling of namespaces in the node tree  (`#132 <https://github.com/ros-visualization/rqt_reconfigure/issues/132>`__)
* Contributors: Aleksander Szymański, Chris Lalancette, Devarsi Rawal, Nick Lamprianidis


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_service_caller <https://github.com/ros-visualization/rqt_service_caller/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add in a pytest test dependency. (`#28 <https://github.com/ros-visualization/rqt_service_caller/issues/28>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_shell <https://github.com/ros-visualization/rqt_shell/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Change maintainer to clalancette. (`#21 <https://github.com/ros-visualization/rqt_shell/issues/21>`__)
* Add in pytest test dependency. (`#19 <https://github.com/ros-visualization/rqt_shell/issues/19>`__)
* Contributors: Chris Lalancette, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_srv <https://github.com/ros-visualization/rqt_srv/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add explicit dependency to python3-pytest. (`#12 <https://github.com/ros-visualization/rqt_srv/issues/12>`__)
* Minor cleanups in rqt_srv for ROS 2 (`#9 <https://github.com/ros-visualization/rqt_srv/issues/9>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_topic <https://github.com/ros-visualization/rqt_topic/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Small fix for modern flake8. (`#50 <https://github.com/ros-visualization/rqt_topic/issues/50>`__)
* Add explicit python3-pytest dependency. (`#48 <https://github.com/ros-visualization/rqt_topic/issues/48>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rti_connext_dds_cmake_module <https://github.com/ros2/rmw_connextdds/tree/jazzy/rti_connext_dds_cmake_module/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use unified approach for checking the existence of environment variables (`#105 <https://github.com/ros2/rmw_connextdds/issues/105>`__)
* Contributors: Christopher Wecht


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rttest <https://github.com/ros2/realtime_support/tree/jazzy/rttest/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update to C++17 (`#124 <https://github.com/ros2/realtime_support/issues/124>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rviz2 <https://github.com/ros2/rviz/tree/jazzy/rviz2/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add "R" key as shortcut for resetTime (`#1088 <https://github.com/ros2/rviz/issues/1088>`__)
* Switch to target_link_libraries. (`#1098 <https://github.com/ros2/rviz/issues/1098>`__)
* Contributors: Chris Lalancette, Paul Erik Frivold


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rviz_assimp_vendor <https://github.com/ros2/rviz/tree/jazzy/rviz_assimp_vendor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Removed assimp warnings (`#1191 <https://github.com/ros2/rviz/issues/1191>`__) (`#1192 <https://github.com/ros2/rviz/issues/1192>`__) (cherry picked from commit e8dd485d19a35d3abba905020741973e613334e3) Co-authored-by: Alejandro Hernández Cordero <alejandro@openrobotics.org>
* Update the vendored package path. (`#1184 <https://github.com/ros2/rviz/issues/1184>`__) Since we just updated to assimp 5.3, we also need to update the path we look for it. This should fix the build with clang which is currently failing.
* Update assimp vendor to 5.3.1 (`#1182 <https://github.com/ros2/rviz/issues/1182>`__) This matches what is in Ubuntu 24.04.
* Update to assimp 5.2.2 (`#968 <https://github.com/ros2/rviz/issues/968>`__)
* Fix the vendoring flags for clang compilation. (`#1003 <https://github.com/ros2/rviz/issues/1003>`__)
* Switch to ament_cmake_vendor_package (`#995 <https://github.com/ros2/rviz/issues/995>`__)
* Contributors: Chris Lalancette, Scott K Logan, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rviz_common <https://github.com/ros2/rviz/tree/jazzy/rviz_common/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update to yaml-cpp 0.8.0 (`#1183 <https://github.com/ros2/rviz/issues/1183>`__) yaml-cpp 0.8.0 has a proper CMake target, i.e. yaml-cpp::yaml-cpp. Use that here.
* Remove regex_filter_property.hpp from the moc lines. (`#1172 <https://github.com/ros2/rviz/issues/1172>`__) Since it has no SLOTS or SIGNALS, we don't need to run MOC on it.  That will both speed up the compilation and remove a warning when building.
* Added regex filter field for TF display (`#1032 <https://github.com/ros2/rviz/issues/1032>`__)
* Fix camera display overlay (`#1151 <https://github.com/ros2/rviz/issues/1151>`__)
* Fixes for uncrustify 0.78. (`#1155 <https://github.com/ros2/rviz/issues/1155>`__) Mostly what we do here is to disable the indentation on certain constructs that are different between 0.72 and 0.78.  It isn't my preferred solution, but since it only affects a small amount of code (and most of that in macros), this seems acceptable to me.
* Append measured subscription frequency to topic status (`#1113 <https://github.com/ros2/rviz/issues/1113>`__)
* Implement reset time service (`#1109 <https://github.com/ros2/rviz/issues/1109>`__)
* Add "R" key as shortcut for resetTime (`#1088 <https://github.com/ros2/rviz/issues/1088>`__)
* Add fullscreen startup option (`#1097 <https://github.com/ros2/rviz/issues/1097>`__)
* Switch to target_link_libraries. (`#1098 <https://github.com/ros2/rviz/issues/1098>`__)
* Initialize more of the visualization_manager members. (`#1090 <https://github.com/ros2/rviz/issues/1090>`__)
* Explicit time conversions and comparisons (`#1087 <https://github.com/ros2/rviz/issues/1087>`__)
* Rolling namespace in title (`#1074 <https://github.com/ros2/rviz/issues/1074>`__)
* Removed unused code (`#1044 <https://github.com/ros2/rviz/issues/1044>`__)
* Remove unused LineEditWithButton::simulateReturnPressed() (`#1040 <https://github.com/ros2/rviz/issues/1040>`__)
* Remove warning in depth_cloud_mld.cpp (`#1021 <https://github.com/ros2/rviz/issues/1021>`__)
* Added DepthCloud default plugin (`#996 <https://github.com/ros2/rviz/issues/996>`__)
* Stop inheriting from std::iterator. (`#1013 <https://github.com/ros2/rviz/issues/1013>`__) In C++17, inheriting from std::iterator has been deprecated: https://www.fluentcpp.com/2018/05/08/std-iterator-deprecated/ Here, switch away from inheriting and just define the interface ourselves (which is the current recommended best practice). This removes some warnings when building with gcc 13.1.1
* use static QCoreApplication::processEvents() function without a QApplication instance (`#924 <https://github.com/ros2/rviz/issues/924>`__)
* Re-implemented setName for tools (`#989 <https://github.com/ros2/rviz/issues/989>`__)
* Add a libqt5-svg dependency to rviz_common. (`#992 <https://github.com/ros2/rviz/issues/992>`__)
* Remove onHelpWiki. (`#985 <https://github.com/ros2/rviz/issues/985>`__)
* Clean Code (`#975 <https://github.com/ros2/rviz/issues/975>`__)
* Contributors: AiVerisimilitude, Alejandro Hernández Cordero, Chris Lalancette, Felix Exner (fexner), Hyunseok, Markus Bader, Paul Erik Frivold, Yadu, Yannis Gerlach, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rviz_default_plugins <https://github.com/ros2/rviz/tree/jazzy/rviz_default_plugins/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Make sure to export all rviz_default_plugins dependencies. (`#1181 <https://github.com/ros2/rviz/issues/1181>`__)
* Increase the cpplint timeout to 180 seconds. (`#1179 <https://github.com/ros2/rviz/issues/1179>`__)
* Switch to gz_math_vendor. (`#1177 <https://github.com/ros2/rviz/issues/1177>`__)
* Fixed camera info warning (`#1175 <https://github.com/ros2/rviz/issues/1175>`__)
* Added CameraInfo display (`#1166 <https://github.com/ros2/rviz/issues/1166>`__)
* apply origin rotation to inertia box visualization (`#1171 <https://github.com/ros2/rviz/issues/1171>`__)
* Added regex filter field for TF display (`#1032 <https://github.com/ros2/rviz/issues/1032>`__)
* Added point_cloud_transport (`#1008 <https://github.com/ros2/rviz/issues/1008>`__)
* Select QoS reliability policy in DepthCloud Plugin (`#1159 <https://github.com/ros2/rviz/issues/1159>`__)
* Fixed crash on DepthCloud plugin (`#1161 <https://github.com/ros2/rviz/issues/1161>`__)
* Fixes for uncrustify 0.78. (`#1155 <https://github.com/ros2/rviz/issues/1155>`__) Mostly what we do here is to disable the indentation on certain constructs that are different between 0.72 and 0.78.  It isn't my preferred solution, but since it only affects a small amount of code (and most of that in macros), this seems acceptable to me.
* Fixed crash on DepthCloudPlugin (`#1133 <https://github.com/ros2/rviz/issues/1133>`__)
* Wrench accepth nan values fix (`#1141 <https://github.com/ros2/rviz/issues/1141>`__)
* DepthCloud plugin: Append measured subscription frequency to topic status (`#1137 <https://github.com/ros2/rviz/issues/1137>`__)
* Added Cache to camera display for TimeExact (`#1138 <https://github.com/ros2/rviz/issues/1138>`__)
* Fixed transport name in DepthCloud plugin (`#1134 <https://github.com/ros2/rviz/issues/1134>`__)
* Fix time-syncing message (`#1121 <https://github.com/ros2/rviz/issues/1121>`__)
* Switch from ROS_TIME to SYSTEM_TIME on rclcpp::Time construction (`#1117 <https://github.com/ros2/rviz/issues/1117>`__)
* Append measured subscription frequency to topic status (`#1113 <https://github.com/ros2/rviz/issues/1113>`__)
* Fix typo (`#1104 <https://github.com/ros2/rviz/issues/1104>`__)
* Fix potencial leak / seg fault (`#726 <https://github.com/ros2/rviz/issues/726>`__)
* Fixed screw display (`#1093 <https://github.com/ros2/rviz/issues/1093>`__)
* Explicit time conversions and comparisons (`#1087 <https://github.com/ros2/rviz/issues/1087>`__)
* Handle missing effort limit in URDF (`#1084 <https://github.com/ros2/rviz/issues/1084>`__)
* (robot) fix styling of log msg (`#1080 <https://github.com/ros2/rviz/issues/1080>`__)
* Fix image display wrapping (`#1038 <https://github.com/ros2/rviz/issues/1038>`__)
* removed enableInteraction reference (`#1075 <https://github.com/ros2/rviz/issues/1075>`__)
* Fix ODR violations in interactive_marker displays. (`#1068 <https://github.com/ros2/rviz/issues/1068>`__)
* Improve error handling in LaserScanDisplay (`#1035 <https://github.com/ros2/rviz/issues/1035>`__)
* Fix implicit capture of "this" warning in C++20 (`#1053 <https://github.com/ros2/rviz/issues/1053>`__)
* Removed unused code (`#1044 <https://github.com/ros2/rviz/issues/1044>`__)
* Fixed AccelStamped, TwistStamped and Wrench icons (`#1041 <https://github.com/ros2/rviz/issues/1041>`__)
* Fix the flakey rviz_rendering tests (`#1026 <https://github.com/ros2/rviz/issues/1026>`__)
* Don't pass screw_display.hpp to the moc generator. (`#1018 <https://github.com/ros2/rviz/issues/1018>`__) Since it isn't a Qt class, you get a warning from moc: Note: No relevant classes found. No output generated. Just skip adding it to the moc list here, which gets rid of the warning.
* Added DepthCloud default plugin (`#996 <https://github.com/ros2/rviz/issues/996>`__)
* Added TwistStamped and AccelStamped default plugins (`#991 <https://github.com/ros2/rviz/issues/991>`__)
* Added Effort plugin (`#990 <https://github.com/ros2/rviz/issues/990>`__)
* Improve the compilation time of rviz_default_plugins (`#1007 <https://github.com/ros2/rviz/issues/1007>`__)
* Switch to ament_cmake_vendor_package (`#995 <https://github.com/ros2/rviz/issues/995>`__)
* Modify access specifier to protected or public for the scope of processMessage() member function (`#984 <https://github.com/ros2/rviz/issues/984>`__)
* Contributors: AiVerisimilitude, Alejandro Hernández Cordero, Austin Moore, Chris Lalancette, Christoph Fröhlich, Hyunseok, Jonas Otto, Lewe Christiansen, Matthijs van der Burgh, Patrick Roncagliolo, Scott K Logan, Yadu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rviz_ogre_vendor <https://github.com/ros2/rviz/tree/jazzy/rviz_ogre_vendor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update zlib into CMakeLists.txt (`#1128 <https://github.com/ros2/rviz/issues/1128>`__) (`#1195 <https://github.com/ros2/rviz/issues/1195>`__) Changes in 1.3 (18 Aug 2023) - Remove K&R function definitions and zlib2ansi - Fix bug in deflateBound() for level 0 and memLevel 9 - Fix bug when gzungetc() is used immediately after gzopen() - Fix bug when using gzflush() with a very small buffer - Fix crash when gzsetparams() attempted for transparent write - Fix test/example.c to work with FORCE_STORED - Rewrite of zran in examples (see zran.c version history) - Fix minizip to allow it to open an empty zip file - Fix reading disk number start on zip64 files in minizip - Fix logic error in minizip argument processing - Add minizip testing to Makefile - Read multiple bytes instead of byte-by-byte in minizip unzip.c - Add memory sanitizer to configure (--memory) - Various portability improvements - Various documentation improvements - Various spelling and typo corrections Co-authored-by: Chris Lalancette <clalancette@gmail.com> (cherry picked from commit 32eb8b9404927883247e868ab0c7d62b80df2ed1) Co-authored-by: mosfet80 <realeandrea@yahoo.it>
* Change an rviz_ogre_vendor dependency to libfreetype-dev. (`#1167 <https://github.com/ros2/rviz/issues/1167>`__) The situation is complicated, but in versions of Ubuntu prior to Focal and versions of Debian prior to Bookworm, the name of the library was 'libfreetype6-dev'.  Since Focal and Bookworm, the name of the library is 'libfreetype-dev'. While 'libfreetype-dev' provides a "virtual package" for 'libfreetype6-dev', we should really use the new canonical name. Further, there is currently a bug on ros_buildfarm where it doesn't properly deal with "virtual packages" like this. This is currently preventing this package from building on Ubuntu Noble.  That bug is being worked on separately. Finally, I'll note that we already have a libfreetype-dev key in rosdep, so we just switch to using that here which should work around the bug on the buildfarm, and also use the correct canonical name going forward.
* fix: modify typo in cmake args for mac (`#1160 <https://github.com/ros2/rviz/issues/1160>`__)
* feat: support macos (`#1156 <https://github.com/ros2/rviz/issues/1156>`__)
* Suppress a couple more of clang warnings in rviz_ogre_vendor. (`#1102 <https://github.com/ros2/rviz/issues/1102>`__)
* Fix the vendoring flags for clang compilation. (`#1003 <https://github.com/ros2/rviz/issues/1003>`__) Several of the flags are not available on clang, so don't add them there.  This fixes the clang build for me locally.
* Switch to ament_cmake_vendor_package (`#995 <https://github.com/ros2/rviz/issues/995>`__)
* CMake: rename FeatureSummary.cmake to avoid name clashes (`#953 <https://github.com/ros2/rviz/issues/953>`__)
* FIX CVE in external libraries (`#961 <https://github.com/ros2/rviz/issues/961>`__)
* Contributors: Chris Lalancette, Daisuke Nishimatsu, Gökçe Aydos, Scott K Logan, mergify[bot], mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rviz_rendering <https://github.com/ros2/rviz/tree/jazzy/rviz_rendering/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Added CameraInfo display (`#1166 <https://github.com/ros2/rviz/issues/1166>`__)
* Fix camera display overlay (`#1151 <https://github.com/ros2/rviz/issues/1151>`__)
* Fixes for uncrustify 0.78. (`#1155 <https://github.com/ros2/rviz/issues/1155>`__) Mostly what we do here is to disable the indentation on certain constructs that are different between 0.72 and 0.78.  It isn't my preferred solution, but since it only affects a small amount of code (and most of that in macros), this seems acceptable to me.
* fixed MovableText::getWorldTransforms transform (`#1118 <https://github.com/ros2/rviz/issues/1118>`__)
* Switch to target_link_libraries. (`#1098 <https://github.com/ros2/rviz/issues/1098>`__)
* Update rviz_rendering and rviz_rendering_tests to C++17. (`#1096 <https://github.com/ros2/rviz/issues/1096>`__)
* Include MeshShape class (`#1064 <https://github.com/ros2/rviz/issues/1064>`__)
* Use assimp to load stl (`#1063 <https://github.com/ros2/rviz/issues/1063>`__)
* RVIZ_RENDERING_PUBLIC to export class RenderSystem (`#1072 <https://github.com/ros2/rviz/issues/1072>`__)
* Restore the maybe-uninitialized flag in covariance_visual.hpp (`#1071 <https://github.com/ros2/rviz/issues/1071>`__)
* Fix up warnings when building with clang. (`#1070 <https://github.com/ros2/rviz/issues/1070>`__)
* Use buildsystem info to get the ros_package_name (`#1062 <https://github.com/ros2/rviz/issues/1062>`__)
* make box-mode point cloud shader lighter on top than bottom (`#1058 <https://github.com/ros2/rviz/issues/1058>`__)
* Removed warning when building in release mode (`#1057 <https://github.com/ros2/rviz/issues/1057>`__)
* Fixed low FPS when sending point markers (`#1049 <https://github.com/ros2/rviz/issues/1049>`__)
* Removed unused code (`#1044 <https://github.com/ros2/rviz/issues/1044>`__)
* Fix the flakey rviz_rendering tests (`#1026 <https://github.com/ros2/rviz/issues/1026>`__)
* Added TwistStamped and AccelStamped default plugins (`#991 <https://github.com/ros2/rviz/issues/991>`__)
* Added Effort plugin (`#990 <https://github.com/ros2/rviz/issues/990>`__)
* load GLB meshes (`#1001 <https://github.com/ros2/rviz/issues/1001>`__)
* Fixed camera default plusin crash (`#999 <https://github.com/ros2/rviz/issues/999>`__)
* Clean Code (`#975 <https://github.com/ros2/rviz/issues/975>`__) * Clean Code
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Felix F Xu, Morgan Quigley, Yaswanth, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rviz_rendering_tests <https://github.com/ros2/rviz/tree/jazzy/rviz_rendering_tests/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Remove the loading_ascii_stl_files_fail (`#1125 <https://github.com/ros2/rviz/issues/1125>`__)
* Update rviz_rendering and rviz_rendering_tests to C++17. (`#1096 <https://github.com/ros2/rviz/issues/1096>`__)
* Use assimp to load stl (`#1063 <https://github.com/ros2/rviz/issues/1063>`__)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rviz_visual_testing_framework <https://github.com/ros2/rviz/tree/jazzy/rviz_visual_testing_framework/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Improve the compilation time of rviz_default_plugins (`#1007 <https://github.com/ros2/rviz/issues/1007>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`sensor_msgs <https://github.com/ros2/common_interfaces/tree/jazzy/sensor_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Clarify the license. (`#241 <https://github.com/ros2/common_interfaces/issues/241>`__) In particular, every package in this repository is Apache 2.0 licensed except for sensor_msgs_py.  So move the CONTRIBUTING.md and LICENSE files down into the individual packages, and make sure that sensor_msgs_py has the correct CONTRIBUTING.md file (it already had the correct LICENSE file).
* [J-Turtle] Fix uninitialized values in NavSatFix and add missing NavSatStatus UNKNOWN (`#220 <https://github.com/ros2/common_interfaces/issues/220>`__) * Fix unitialized values in NavSatFix and add missing UNKNOWN * Fixes `#196 <https://github.com/ros2/common_interfaces/issues/196>`__ * Fix default initialization instead of constants * Define SERVICE_UNKNOWN Co-authored-by: Tully Foote <tully.foote@gmail.com> Co-authored-by: Martin Pecka <peci1@seznam.cz>
* Use target qualifier for checking the cpp typesupport exists (`#238 <https://github.com/ros2/common_interfaces/issues/238>`__)
* sensor_msgs/CompressedImage: updated description of format field (`#231 <https://github.com/ros2/common_interfaces/issues/231>`__)
* Return true for isColor if format is YUYV or UYUV (`#229 <https://github.com/ros2/common_interfaces/issues/229>`__)
* Contributors: Chris Lalancette, Kenji Brameld, Ryan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`sensor_msgs_py <https://github.com/ros2/common_interfaces/tree/jazzy/sensor_msgs_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Clarify the license. (`#241 <https://github.com/ros2/common_interfaces/issues/241>`__) In particular, every package in this repository is Apache 2.0 licensed except for sensor_msgs_py.  So move the CONTRIBUTING.md and LICENSE files down into the individual packages, and make sure that sensor_msgs_py has the correct CONTRIBUTING.md file (it already had the correct LICENSE file).
* Allow pointcloud create_cloud function to set specific point_step (`#223 <https://github.com/ros2/common_interfaces/issues/223>`__)
* Fix read_points_numpy field_names parameter
* Contributors: Chris Lalancette, George Broughton


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`shape_msgs <https://github.com/ros2/common_interfaces/tree/jazzy/shape_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Clarify the license. (`#241 <https://github.com/ros2/common_interfaces/issues/241>`__) In particular, every package in this repository is Apache 2.0 licensed except for sensor_msgs_py.  So move the CONTRIBUTING.md and LICENSE files down into the individual packages, and make sure that sensor_msgs_py has the correct CONTRIBUTING.md file (it already had the correct LICENSE file).
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`shared_queues_vendor <https://github.com/ros2/rosbag2/tree/jazzy/shared_queues_vendor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Remove unused concurrentqueue implementation. (`#1465 <https://github.com/ros2/rosbag2/issues/1465>`__) rosbag2 only depends on the readerwriter queue.
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`spdlog_vendor <https://github.com/ros2/spdlog_vendor/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Removed spdlog_vendor warnings (`#36 <https://github.com/ros2/spdlog_vendor/issues/36>`__) (`#37 <https://github.com/ros2/spdlog_vendor/issues/37>`__) (cherry picked from commit 4510d9ab4389f84daac77210f3fdf8aab372b938) Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Upgrade to v1.12.0. (`#35 <https://github.com/ros2/spdlog_vendor/issues/35>`__)
* Switch to ament_cmake_vendor_package (`#34 <https://github.com/ros2/spdlog_vendor/issues/34>`__)
* Contributors: Marco A. Gutierrez, Scott K Logan, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`sqlite3_vendor <https://github.com/ros2/rosbag2/tree/jazzy/sqlite3_vendor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Switch to ament_cmake_vendor_package (`#1400 <https://github.com/ros2/rosbag2/issues/1400>`__)
* Contributors: Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`sros2 <https://github.com/ros2/sros2/tree/jazzy/sros2/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix linux tutorial: cloning example policies and set of default policies for a node (`#295 <https://github.com/ros2/sros2/issues/295>`__) (`#296 <https://github.com/ros2/sros2/issues/296>`__) * clone policies to temporary dir as subversion hack doesnt work anymore * add get_type_description service to policies * update MacOS similarly * update all permissions with new topics * dont rule out cycloneDDS * example of enclave override Co-authored-by: Chris Lalancette <clalancette@gmail.com> (cherry picked from commit ca6bb12cc650b73e7ccfc0fa789d8b49358d44ad) Co-authored-by: Mikael Arguedas <mikael.arguedas@gmail.com>
* Use modern PKCS7 to sign the certificate bytes. (`#290 <https://github.com/ros2/sros2/issues/290>`__)
* Fix a number of warnings on Ubuntu 24.04. (`#289 <https://github.com/ros2/sros2/issues/289>`__)
* Fix SSH commands in SROS2_Linux.md (`#286 <https://github.com/ros2/sros2/issues/286>`__)
* Contributors: Boris Boutillier, Chris Lalancette, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`std_msgs <https://github.com/ros2/common_interfaces/tree/jazzy/std_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Clarify the license. (`#241 <https://github.com/ros2/common_interfaces/issues/241>`__) In particular, every package in this repository is Apache 2.0 licensed except for sensor_msgs_py.  So move the CONTRIBUTING.md and LICENSE files down into the individual packages, and make sure that sensor_msgs_py has the correct CONTRIBUTING.md file (it already had the correct LICENSE file).
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`std_srvs <https://github.com/ros2/common_interfaces/tree/jazzy/std_srvs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Clarify the license. (`#241 <https://github.com/ros2/common_interfaces/issues/241>`__) In particular, every package in this repository is Apache 2.0 licensed except for sensor_msgs_py.  So move the CONTRIBUTING.md and LICENSE files down into the individual packages, and make sure that sensor_msgs_py has the correct CONTRIBUTING.md file (it already had the correct LICENSE file).
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`stereo_msgs <https://github.com/ros2/common_interfaces/tree/jazzy/stereo_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Clarify the license. (`#241 <https://github.com/ros2/common_interfaces/issues/241>`__) In particular, every package in this repository is Apache 2.0 licensed except for sensor_msgs_py.  So move the CONTRIBUTING.md and LICENSE files down into the individual packages, and make sure that sensor_msgs_py has the correct CONTRIBUTING.md file (it already had the correct LICENSE file).
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_cli <https://github.com/ros2/system_tests/tree/jazzy/test_cli/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Switch to target_link_libraries everywhere. (`#532 <https://github.com/ros2/system_tests/issues/532>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_cli_remapping <https://github.com/ros2/system_tests/tree/jazzy/test_cli_remapping/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Switch to target_link_libraries everywhere. (`#532 <https://github.com/ros2/system_tests/issues/532>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_communication <https://github.com/ros2/system_tests/tree/jazzy/test_communication/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Small fix for modern flake8. (`#539 <https://github.com/ros2/system_tests/issues/539>`__)
* Switch to target_link_libraries everywhere. (`#532 <https://github.com/ros2/system_tests/issues/532>`__)
* Add integration test for nested messages. (`#530 <https://github.com/ros2/system_tests/issues/530>`__)
* Adjust for new rclcpp::Rate API (`#516 <https://github.com/ros2/system_tests/issues/516>`__)
* Contributors: Alexey Merzlyakov, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_launch_ros <https://github.com/ros2/launch_ros/tree/jazzy/test_launch_ros/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Small fixes for modern flake8. (`#395 <https://github.com/ros2/launch_ros/issues/395>`__)
* add "--log-file-name" command line argument for test. (`#387 <https://github.com/ros2/launch_ros/issues/387>`__)
* Fix an assert in the test_launch_ros tests. (`#367 <https://github.com/ros2/launch_ros/issues/367>`__)
* Fix misspelled "receive". (`#362 <https://github.com/ros2/launch_ros/issues/362>`__)
* Contributors: Chris Lalancette, Tomoya Fujita


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_launch_testing <https://github.com/ros2/launch/tree/jazzy/test_launch_testing/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update to C++17 (`#742 <https://github.com/ros2/launch/issues/742>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_msgs <https://github.com/ros2/rcl_interfaces/tree/jazzy/test_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Increase the timeout for the test_msgs rosidl_generated_cpp cpplint. (`#163 <https://github.com/ros2/rcl_interfaces/issues/163>`__) This should make it much more likely to succeed on Windows.
* Fix for invalid conversion from const char8_t* to char for C++20 (`#160 <https://github.com/ros2/rcl_interfaces/issues/160>`__)
* Contributors: AiVerisimilitude, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_quality_of_service <https://github.com/ros2/system_tests/tree/jazzy/test_quality_of_service/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Cleanup header includes in test_quality_of_service. (`#533 <https://github.com/ros2/system_tests/issues/533>`__)
* Switch to target_link_libraries everywhere. (`#532 <https://github.com/ros2/system_tests/issues/532>`__)
* Fix test QoS on macOS by moving qos_utilities.cpp to the four tests; fixes `#517 <https://github.com/ros2/system_tests/issues/517>`__ (`#518 <https://github.com/ros2/system_tests/issues/518>`__)
* Contributors: Chris Lalancette, PhDittmann


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_rclcpp <https://github.com/ros2/system_tests/tree/jazzy/test_rclcpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Addressed TODO in test_local_parameters (`#545 <https://github.com/ros2/system_tests/issues/545>`__)
* Actually skip the gtest_subscription test on Connext. (`#544 <https://github.com/ros2/system_tests/issues/544>`__)
* Increased time in test_multithreaded (`#543 <https://github.com/ros2/system_tests/issues/543>`__)
* Improve the node_name test. (`#538 <https://github.com/ros2/system_tests/issues/538>`__)
* Change up the formatting in the test_rclcpp tests. (`#537 <https://github.com/ros2/system_tests/issues/537>`__)
* Revamp test_rclcpp to compile far few files. (`#535 <https://github.com/ros2/system_tests/issues/535>`__)
* Mark gtest_subscription__rmw_connextdds xfail. (`#531 <https://github.com/ros2/system_tests/issues/531>`__)
* refactor corrected depth check for prefix in parameter_fixtures.hpp (`#529 <https://github.com/ros2/system_tests/issues/529>`__)
* Remove an unnecessary capture in test_rclcpp. (`#527 <https://github.com/ros2/system_tests/issues/527>`__)
* Cleanup of the CMakeLists.txt for test_rclcpp. (`#526 <https://github.com/ros2/system_tests/issues/526>`__)
* Add a fix for the tests given the new type description parameter (`#520 <https://github.com/ros2/system_tests/issues/520>`__)
* Changes ros_timer_init for ros_timer_init2 (`#508 <https://github.com/ros2/system_tests/issues/508>`__)
* refactor the multi_access_publisher test to avoid dead locks (`#515 <https://github.com/ros2/system_tests/issues/515>`__)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Eloy Briceno, Emerson Knapp, Lee, Minju, William Woodall


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_rmw_implementation <https://github.com/ros2/rmw_implementation/tree/jazzy/test_rmw_implementation/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Compile the test_rmw_implementation tests fewer times. (`#224 <https://github.com/ros2/rmw_implementation/issues/224>`__)
* Switch to using target_link_libraries everywhere. (`#222 <https://github.com/ros2/rmw_implementation/issues/222>`__)
* Add rmw_count_clients,services & test (`#208 <https://github.com/ros2/rmw_implementation/issues/208>`__)
* Contributors: Chris Lalancette, Lee, Minju


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_ros2trace <https://github.com/ros2/ros2_tracing/tree/jazzy/test_ros2trace/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add explicit context fields test to test_ros2trace (`#107 <https://github.com/ros2/ros2_tracing/issues/107>`__)
* Allow tracing tests to be run in parallel with other tests (`#95 <https://github.com/ros2/ros2_tracing/issues/95>`__)
* Make test_ros2trace depend on test_tracetools_launch.
* Switch to custom lttng-ctl Python bindings (`#81 <https://github.com/ros2/ros2_tracing/issues/81>`__)
* Contributors: Chris Lalancette, Christophe Bedard


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_security <https://github.com/ros2/system_tests/tree/jazzy/test_security/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update to C++17 (`#528 <https://github.com/ros2/system_tests/issues/528>`__)
* Switch to target_link_libraries everywhere. (`#532 <https://github.com/ros2/system_tests/issues/532>`__)
* Adjust for new rclcpp::Rate API (`#516 <https://github.com/ros2/system_tests/issues/516>`__)
* Extract sros_artifacts fixture into a CMake script (`#525 <https://github.com/ros2/system_tests/issues/525>`__)
* Use test fixtures to create SROS artifacts (`#522 <https://github.com/ros2/system_tests/issues/522>`__)
* Contributors: Alexey Merzlyakov, Chris Lalancette, Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_tf2 <https://github.com/ros2/geometry2/tree/jazzy/test_tf2/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Compile fix for upcomming changes to rclcpp::Executor (`#668 <https://github.com/ros2/geometry2/issues/668>`__)
* Adding addition BUILD_TESTING requirement (`#660 <https://github.com/ros2/geometry2/issues/660>`__)
* normalize quaternions on tf2_eigen (`#644 <https://github.com/ros2/geometry2/issues/644>`__)
* Contributors: Lucas Wendland, Paul Gesel, jmachowinski


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_tracetools <https://github.com/ros2/ros2_tracing/tree/jazzy/test_tracetools/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Improve tracetools_test and simplify test_tracetools code (`#109 <https://github.com/ros2/ros2_tracing/issues/109>`__)
* Install test_tracetools_mark_process (`#113 <https://github.com/ros2/ros2_tracing/issues/113>`__)
* Remove unnecessary <string> include (`#111 <https://github.com/ros2/ros2_tracing/issues/111>`__)
* Include <string> in mark_process.cpp (`#110 <https://github.com/ros2/ros2_tracing/issues/110>`__)
* Remove unnecessary print in test (`#108 <https://github.com/ros2/ros2_tracing/issues/108>`__)
* Add test for GenericPublisher/Subscriber (`#97 <https://github.com/ros2/ros2_tracing/issues/97>`__)
* Use lttng_ust_tracef instead of lttng_ust__tracef (`#103 <https://github.com/ros2/ros2_tracing/issues/103>`__)
* Use a memcmp for the expected symbol name. (`#100 <https://github.com/ros2/ros2_tracing/issues/100>`__)
* Fix the build on RHEL-9. (`#98 <https://github.com/ros2/ros2_tracing/issues/98>`__)
* Allow tracing tests to be run in parallel with other tests (`#95 <https://github.com/ros2/ros2_tracing/issues/95>`__)
* Fix interference between test_tracetools and ros2lifecycle tests (`#96 <https://github.com/ros2/ros2_tracing/issues/96>`__)
* Make tracing test assert messages more descriptive (`#93 <https://github.com/ros2/ros2_tracing/issues/93>`__)
* Update tests and docs after new rmw_publish timestamp field (`#90 <https://github.com/ros2/ros2_tracing/issues/90>`__)
* Switch to target_link_libraries in test_tracetools. (`#83 <https://github.com/ros2/ros2_tracing/issues/83>`__)
* Improve test coverage of rclcpp_callback_register in test_tracetools (`#69 <https://github.com/ros2/ros2_tracing/issues/69>`__)
* Disable tracing on Android (`#71 <https://github.com/ros2/ros2_tracing/issues/71>`__)
* Contributors: Chris Lalancette, Christophe Bedard, Przemysław Dąbrowski, h-suzuki-isp


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_tracetools_launch <https://github.com/ros2/ros2_tracing/tree/jazzy/test_tracetools_launch/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Improve tracing configuration error reporting (`#85 <https://github.com/ros2/ros2_tracing/issues/85>`__)
* Contributors: Christophe Bedard


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2 <https://github.com/ros2/geometry2/tree/jazzy/tf2/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Enable Twist interpolator (`#646 <https://github.com/ros2/geometry2/issues/646>`__) Co-authored-by: Tully Foote <tullyfoote@intrinsic.ai>
* Warning Message Intervals for canTransform (`#663 <https://github.com/ros2/geometry2/issues/663>`__)
* Nacho/minor fixes tf2 cache (`#658 <https://github.com/ros2/geometry2/issues/658>`__)
* Removing console_bridge (`#655 <https://github.com/ros2/geometry2/issues/655>`__)
* Fix constantly increasing memory in std::list (`#636 <https://github.com/ros2/geometry2/issues/636>`__)
* Update the tf2 documentation (`#638 <https://github.com/ros2/geometry2/issues/638>`__)
* Fix error code returned in BufferCore::walkToTopParent (`#601 <https://github.com/ros2/geometry2/issues/601>`__)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Ignacio Vizzo, Lucas Wendland, Patrick Roncagliolo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_bullet <https://github.com/ros2/geometry2/tree/jazzy/tf2_bullet/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Removed obsolete headers (`#645 <https://github.com/ros2/geometry2/issues/645>`__)
* Contributors: Alejandro Hernández Cordero


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_eigen <https://github.com/ros2/geometry2/tree/jazzy/tf2_eigen/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Removed obsolete headers (`#645 <https://github.com/ros2/geometry2/issues/645>`__)
* normalize quaternions on tf2_eigen (`#644 <https://github.com/ros2/geometry2/issues/644>`__)
* Fix clang build warnings. (`#628 <https://github.com/ros2/geometry2/issues/628>`__)
* Add another reference for twist transformation. Comment correction. (`#620 <https://github.com/ros2/geometry2/issues/620>`__)
* Contributors: Alejandro Hernández Cordero, AndyZe, Chris Lalancette, Paul Gesel


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_eigen_kdl <https://github.com/ros2/geometry2/tree/jazzy/tf2_eigen_kdl/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix installation directory of .dll files in tf2_eigen_kdl (`#657 <https://github.com/ros2/geometry2/issues/657>`__)
* Remove unnecessary use of ament_target_dependencies. (`#637 <https://github.com/ros2/geometry2/issues/637>`__) We can just use target_link_libraries instead.
* Fix clang build warnings. (`#628 <https://github.com/ros2/geometry2/issues/628>`__)
* Contributors: Chris Lalancette, Silvio Traversaro


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_geometry_msgs <https://github.com/ros2/geometry2/tree/jazzy/tf2_geometry_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Enable Twist interpolator (`#646 <https://github.com/ros2/geometry2/issues/646>`__) Co-authored-by: Tully Foote <tullyfoote@intrinsic.ai>
* Removed obsolete headers (`#645 <https://github.com/ros2/geometry2/issues/645>`__)
* Add doTransform support for Point32, Polygon and PolygonStamped (backport `#616 <https://github.com/ros2/geometry2/issues/616>`__) (`#619 <https://github.com/ros2/geometry2/issues/619>`__)
* Contributors: Alejandro Hernández Cordero, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_kdl <https://github.com/ros2/geometry2/tree/jazzy/tf2_kdl/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Removed obsolete headers (`#645 <https://github.com/ros2/geometry2/issues/645>`__)
* Contributors: Alejandro Hernández Cordero


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_py <https://github.com/ros2/geometry2/tree/jazzy/tf2_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Enable Twist interpolator (`#646 <https://github.com/ros2/geometry2/issues/646>`__) Co-authored-by: Tully Foote <tullyfoote@intrinsic.ai>
* Contributors: Alejandro Hernández Cordero


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_ros <https://github.com/ros2/geometry2/tree/jazzy/tf2_ros/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Compile fix for upcomming changes to rclcpp::Executor (`#668 <https://github.com/ros2/geometry2/issues/668>`__)
* Enable Twist interpolator (`#646 <https://github.com/ros2/geometry2/issues/646>`__) Co-authored-by: Tully Foote <tullyfoote@intrinsic.ai>
* Adding NodeInterfaces to Buffer (`#656 <https://github.com/ros2/geometry2/issues/656>`__)
* Reformat some code to make uncrustify happier. (`#654 <https://github.com/ros2/geometry2/issues/654>`__)
* Enable intra-process (`#649 <https://github.com/ros2/geometry2/issues/649>`__) (`#642 <https://github.com/ros2/geometry2/issues/642>`__)
* Avoid unecessary time conversions. (`#635 <https://github.com/ros2/geometry2/issues/635>`__)
* Expose TF2 listener CB (`#632 <https://github.com/ros2/geometry2/issues/632>`__)
* Fix invalid timer handle exception (`#474 <https://github.com/ros2/geometry2/issues/474>`__)
* Fix for `#589 <https://github.com/ros2/geometry2/issues/589>`__ - Should be able to transform with default timeout (`#593 <https://github.com/ros2/geometry2/issues/593>`__)
* Enable StaticTransformBroadcaster in Intra-process enabled components (`#607 <https://github.com/ros2/geometry2/issues/607>`__)
* Contributors: AiVerisimilitude, Alejandro Hernández Cordero, Chris Lalancette, Cliff Wu, Lucas Wendland, Patrick Roncagliolo, Steve Macenski, jmachowinski, vineet131


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_ros_py <https://github.com/ros2/geometry2/tree/jazzy/tf2_ros_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Transform Data Callback Python (`#664 <https://github.com/ros2/geometry2/issues/664>`__)
* Make sure to cache transforms in tf2_ros_py. (`#634 <https://github.com/ros2/geometry2/issues/634>`__)
* Remove 'efficient copy' prints (`#625 <https://github.com/ros2/geometry2/issues/625>`__)
* Add time jump callback (`#608 <https://github.com/ros2/geometry2/issues/608>`__)
* Contributors: Chris Lalancette, Erich L Foster, Lucas Wendland, Matthijs van der Burgh


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_sensor_msgs <https://github.com/ros2/geometry2/tree/jazzy/tf2_sensor_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Removed obsolete headers (`#645 <https://github.com/ros2/geometry2/issues/645>`__)
* Fix clang build warnings. (`#628 <https://github.com/ros2/geometry2/issues/628>`__)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_tools <https://github.com/ros2/geometry2/tree/jazzy/tf2_tools/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [view_frames] log filenames after it's been determined (`#674 <https://github.com/ros2/geometry2/issues/674>`__) (`#675 <https://github.com/ros2/geometry2/issues/675>`__) (cherry picked from commit 24643fce510d8cc836fe6e5277a1d3f86a21af04) Co-authored-by: Mikael Arguedas <mikael.arguedas@gmail.com>
* Add in tests for tf2_tools. (`#647 <https://github.com/ros2/geometry2/issues/647>`__)
* Contributors: Chris Lalancette, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`topic_monitor <https://github.com/ros2/demos/tree/jazzy/topic_monitor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`__)
* fix readme for topic_monitor. (`#630 <https://github.com/ros2/demos/issues/630>`__)
* Contributors: Michael Jeronimo, Tomoya Fujita


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`topic_statistics_demo <https://github.com/ros2/demos/tree/jazzy/topic_statistics_demo/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`__)
* Contributors: Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tracetools <https://github.com/ros2/ros2_tracing/tree/jazzy/tracetools/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Replace all occurences of index.ros.org (`#114 <https://github.com/ros2/ros2_tracing/issues/114>`__)
* Switch to ament_generate_version_header for tracetools (`#112 <https://github.com/ros2/ros2_tracing/issues/112>`__)
* Fixes for newer uncrustify (`#101 <https://github.com/ros2/ros2_tracing/issues/101>`__)
* Update tests and docs after new rmw_publish timestamp field (`#90 <https://github.com/ros2/ros2_tracing/issues/90>`__)
* Add timestamp to rmw_publish tracepoint (`#74 <https://github.com/ros2/ros2_tracing/issues/74>`__)
* Add TRACETOOLS\_ prefix to tracepoint-related public macros (`#56 <https://github.com/ros2/ros2_tracing/issues/56>`__)
* Disable tracing on Android (`#71 <https://github.com/ros2/ros2_tracing/issues/71>`__)
* Add new rclcpp_subscription_init tracepoint to support new intra-process comms
* Contributors: Chris Lalancette, Christophe Bedard, Christopher Wecht, Przemysław Dąbrowski


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tracetools_launch <https://github.com/ros2/ros2_tracing/tree/jazzy/tracetools_launch/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Replace all occurences of index.ros.org (`#114 <https://github.com/ros2/ros2_tracing/issues/114>`__)
* Use single underscore for private vars in Python (`#92 <https://github.com/ros2/ros2_tracing/issues/92>`__)
* Improve tracing configuration error reporting (`#85 <https://github.com/ros2/ros2_tracing/issues/85>`__)
* Fix warnings when using mypy 1.8.0. (`#89 <https://github.com/ros2/ros2_tracing/issues/89>`__)
* Remove extra single quote in LdPreload debug log (`#79 <https://github.com/ros2/ros2_tracing/issues/79>`__)
* Contributors: Chris Lalancette, Christophe Bedard


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tracetools_read <https://github.com/ros2/ros2_tracing/tree/jazzy/tracetools_read/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Replace all occurences of index.ros.org (`#114 <https://github.com/ros2/ros2_tracing/issues/114>`__)
* Improve tracetools_test and simplify test_tracetools code (`#109 <https://github.com/ros2/ros2_tracing/issues/109>`__)
* Allow tracing tests to be run in parallel with other tests (`#95 <https://github.com/ros2/ros2_tracing/issues/95>`__)
* Contributors: Chris Lalancette, Christophe Bedard


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tracetools_test <https://github.com/ros2/ros2_tracing/tree/jazzy/tracetools_test/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Replace all occurences of index.ros.org (`#114 <https://github.com/ros2/ros2_tracing/issues/114>`__)
* Improve tracetools_test and simplify test_tracetools code (`#109 <https://github.com/ros2/ros2_tracing/issues/109>`__)
* Improve assertEventOrder failure output (`#106 <https://github.com/ros2/ros2_tracing/issues/106>`__)
* Allow tracing tests to be run in parallel with other tests (`#95 <https://github.com/ros2/ros2_tracing/issues/95>`__)
* Fix interference between test_tracetools and ros2lifecycle tests (`#96 <https://github.com/ros2/ros2_tracing/issues/96>`__)
* Make tracing test assert messages more descriptive (`#93 <https://github.com/ros2/ros2_tracing/issues/93>`__)
* Fix use of mutable default arg in tracetools_test (`#84 <https://github.com/ros2/ros2_tracing/issues/84>`__)
* Switch <depend> to <exec_depend> in pure Python packages (`#67 <https://github.com/ros2/ros2_tracing/issues/67>`__)
* Contributors: Chris Lalancette, Christophe Bedard


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tracetools_trace <https://github.com/ros2/ros2_tracing/tree/jazzy/tracetools_trace/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Replace all occurences of index.ros.org (`#114 <https://github.com/ros2/ros2_tracing/issues/114>`__)
* Improve tracing configuration error reporting (`#85 <https://github.com/ros2/ros2_tracing/issues/85>`__)
* Add a space in between not and parentheses. (`#88 <https://github.com/ros2/ros2_tracing/issues/88>`__)
* Switch to custom lttng-ctl Python bindings (`#81 <https://github.com/ros2/ros2_tracing/issues/81>`__)
* Create start/pause/resume/stop sub-commands for 'ros2 trace' (`#70 <https://github.com/ros2/ros2_tracing/issues/70>`__)
* Detect issue with LTTng and Docker and report error when tracing (`#66 <https://github.com/ros2/ros2_tracing/issues/66>`__)
* Contributors: Chris Lalancette, Christophe Bedard


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`trajectory_msgs <https://github.com/ros2/common_interfaces/tree/jazzy/trajectory_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Clarify the license. (`#241 <https://github.com/ros2/common_interfaces/issues/241>`__) In particular, every package in this repository is Apache 2.0 licensed except for sensor_msgs_py.  So move the CONTRIBUTING.md and LICENSE files down into the individual packages, and make sure that sensor_msgs_py has the correct CONTRIBUTING.md file (it already had the correct LICENSE file).
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`turtlesim <https://github.com/ros/ros_tutorials/tree/jazzy/turtlesim/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add icon for Jazzy. (`#167 <https://github.com/ros/ros_tutorials/issues/167>`__) (`#168 <https://github.com/ros/ros_tutorials/issues/168>`__) (cherry picked from commit 014955e15a6ac3b1649cbf21e11c8547ebd47af7) Co-authored-by: Marco A. Gutierrez <marcogg@marcogg.com>
* [teleop_turtle_key] update usage string to match keys captured by keyboard (`#165 <https://github.com/ros/ros_tutorials/issues/165>`__) (`#166 <https://github.com/ros/ros_tutorials/issues/166>`__) On windows it will stay uppercase but shouldn't impact users compared to current situation (cherry picked from commit e2853cac87f0c62db6294e5bc351e5b52fcd1ae1) Co-authored-by: Mikael Arguedas <mikael.arguedas@gmail.com>
* Shorten the callback definition for uncrustify. (`#163 <https://github.com/ros/ros_tutorials/issues/163>`__)
* Use same QoS for all topic pub/subs (`#161 <https://github.com/ros/ros_tutorials/issues/161>`__)
* Remove all uses of ament_target_dependencies. (`#159 <https://github.com/ros/ros_tutorials/issues/159>`__)
* Crop galactic.png and rolling.png to 45x45. (`#158 <https://github.com/ros/ros_tutorials/issues/158>`__)
* Remove the unused member variable last_state\_ (`#156 <https://github.com/ros/ros_tutorials/issues/156>`__)
* Added common tests (`#154 <https://github.com/ros/ros_tutorials/issues/154>`__)
* Heavy cleanup of the draw_square tutorial. (`#152 <https://github.com/ros/ros_tutorials/issues/152>`__) * Heavy cleanup of the draw_square tutorial. In particular: 1. Make it conform to the current ROS 2 style. 2. Add in copyright information. 3. Refactor the entire code into a class, which tidies it up quite a bit and removes a bunch of globals. 4. Make sure to wait for the reset to complete before trying to move the turtle.
* Remove the range constraints from the holonomic parameter. (`#150 <https://github.com/ros/ros_tutorials/issues/150>`__)
* Add icon (`#148 <https://github.com/ros/ros_tutorials/issues/148>`__)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Jason O'Kane, Yadu, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`uncrustify_vendor <https://github.com/ament/uncrustify_vendor/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update to uncrustify 0.78.1 (`#37 <https://github.com/ament/uncrustify_vendor/issues/37>`__) * Update to uncrustify 0.78.1 * Fix the uncrustify version detection logic. And make sure we are at least 0.78.
* Switch to ament_cmake_vendor_package (`#34 <https://github.com/ament/uncrustify_vendor/issues/34>`__)
* Contributors: Chris Lalancette, Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`unique_identifier_msgs <https://github.com/ros2/unique_identifier_msgs/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update to C++17 (`#27 <https://github.com/ros2/unique_identifier_msgs/issues/27>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`urdf <https://github.com/ros2/urdf/tree/jazzy/urdf/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Switch to target_link_libraries (`#36 <https://github.com/ros2/urdf/issues/36>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`urdf_parser_plugin <https://github.com/ros2/urdf/tree/jazzy/urdf_parser_plugin/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Switch to target_link_libraries (`#36 <https://github.com/ros2/urdf/issues/36>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`visualization_msgs <https://github.com/ros2/common_interfaces/tree/jazzy/visualization_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Remove references to index.ros.org. (`#244 <https://github.com/ros2/common_interfaces/issues/244>`__)
* Adds ARROW_STRIP to Marker.msg (`#242 <https://github.com/ros2/common_interfaces/issues/242>`__)
* Clarify the license. (`#241 <https://github.com/ros2/common_interfaces/issues/241>`__) In particular, every package in this repository is Apache 2.0 licensed except for sensor_msgs_py.  So move the CONTRIBUTING.md and LICENSE files down into the individual packages, and make sure that sensor_msgs_py has the correct CONTRIBUTING.md file (it already had the correct LICENSE file).
* Contributors: Chris Lalancette, Tom Noble


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`yaml_cpp_vendor <https://github.com/ros2/yaml_cpp_vendor/tree/jazzy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Removed warnigns (`#49 <https://github.com/ros2/yaml_cpp_vendor/issues/49>`__) (`#50 <https://github.com/ros2/yaml_cpp_vendor/issues/50>`__) (cherry picked from commit 4b6808fd0f9b0b5e05928c0c8e44fd976a043d33) Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Upgrade to yaml-cpp 0.8.0 (`#48 <https://github.com/ros2/yaml_cpp_vendor/issues/48>`__) Co-authored-by: Chris Lalancette <clalancette@gmail.com>
* Support yaml-cpp >= 0.8.0 (`#46 <https://github.com/ros2/yaml_cpp_vendor/issues/46>`__)
* Disable the -Wshadow warning when building under clang. (`#45 <https://github.com/ros2/yaml_cpp_vendor/issues/45>`__)
* Switch to ament_cmake_vendor_package (`#43 <https://github.com/ros2/yaml_cpp_vendor/issues/43>`__)
* Revamp the extras file to find the correct version. (`#42 <https://github.com/ros2/yaml_cpp_vendor/issues/42>`__)
* Contributors: Chris Lalancette, Marco A. Gutierrez, Scott K Logan, Silvio Traversaro, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`zstd_vendor <https://github.com/ros2/rosbag2/tree/jazzy/zstd_vendor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Updated zstd to 1.5.5 (`#1617 <https://github.com/ros2/rosbag2/issues/1617>`__) (`#1624 <https://github.com/ros2/rosbag2/issues/1624>`__)
* Switch to ament_cmake_vendor_package (`#1400 <https://github.com/ros2/rosbag2/issues/1400>`__)
* Contributors: Scott K Logan, mergify[bot]
