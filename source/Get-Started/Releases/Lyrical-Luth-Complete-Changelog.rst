ROS 2 Lyrical Luth Complete Changelog
=====================================

This page is a list of the complete changes in all ROS 2 core packages since the previous release.

.. contents:: Table of Contents
   :local:

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`action_msgs <https://github.com/ros2/rcl_interfaces/tree/lyrical/action_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix cmake deprecation (`#180 <https://github.com/ros2/rcl_interfaces/issues/180>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`action_tutorials_cpp <https://github.com/ros2/demos/tree/lyrical/action_tutorials/action_tutorials_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#781 <https://github.com/ros2/demos//issues/781>`__)
* Uniform CMAKE min VERSION (`#714 <https://github.com/ros2/demos/issues/714>`__) demo_nodes_cpp/CMakeLists.txt require cmake min version 3.12 other modules cmake 3.5. It is proposed to standardize with version 3.12. This also fixes cmake <3.10 deprecation warnings
* Update action cpp demos to support setting introspection (`#709 <https://github.com/ros2/demos/issues/709>`__) * Update action cpp demos to support setting introspection * Add the missing header file declaration ---------
* Contributors: Barry Xu, Emerson Knapp, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`action_tutorials_py <https://github.com/ros2/demos/tree/lyrical/action_tutorials/action_tutorials_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* action_tutorials_py: add ament_mypy support (`#775 <https://github.com/ros2/demos//issues/775>`__)
* fix setuptools deprecations (`#733 <https://github.com/ros2/demos/issues/733>`__)
* support cancel handler in action_tutorials_py action server. (`#727 <https://github.com/ros2/demos/issues/727>`__)
* Update action python demos to support setting introspection (`#708 <https://github.com/ros2/demos/issues/708>`__) * Update action python demos to support setting introspection * Correct the errors in the document ---------
* Contributors: Barry Xu, Tomoya Fujita, mohit, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_clang_format <https://github.com/ament/ament_lint/tree/lyrical/ament_clang_format/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [ament_mypy] Fix config for ament_cmake packages and type entrypoints (`#574 <https://github.com/ament/ament_lint/issues/574>`__)
* Drop setuptools from install_requires (`#566 <https://github.com/ament/ament_lint/issues/566>`__)
* Export typing information for ament linters (`#553 <https://github.com/ament/ament_lint/issues/553>`__)
* fix setuptools deprecations (`#547 <https://github.com/ament/ament_lint/issues/547>`__)
* Contributors: Jochen Sprickerhof, Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_clang_tidy <https://github.com/ament/ament_lint/tree/lyrical/ament_clang_tidy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [ament_mypy] Fix config for ament_cmake packages and type entrypoints (`#574 <https://github.com/ament/ament_lint/issues/574>`__)
* Drop setuptools from install_requires (`#566 <https://github.com/ament/ament_lint/issues/566>`__)
* Export typing information for ament linters (`#553 <https://github.com/ament/ament_lint/issues/553>`__)
* fix setuptools deprecations (`#547 <https://github.com/ament/ament_lint/issues/547>`__)
* Contributors: Jochen Sprickerhof, Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake <https://github.com/ament/ament_cmake/tree/lyrical/ament_cmake/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Removed deprecated ament_cmake_export_interfaces package (`#581 <https://github.com/ament/ament_cmake/issues/581>`__)
* Contributors: Alejandro Hernández Cordero


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_auto <https://github.com/ament/ament_cmake/tree/lyrical/ament_cmake_auto/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Do not error on USE_SCOPED_HEADER_INSTALL_DIR (`#596 <https://github.com/ament/ament_cmake/issues/596>`__)
* Contributors: Tim Clephas


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_clang_format <https://github.com/ament/ament_lint/tree/lyrical/ament_cmake_clang_format/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix cmake deprecation (`#539 <https://github.com/ament/ament_lint/issues/539>`__)
* Allow overriding clang-format version via CMake (`#536 <https://github.com/ament/ament_lint/issues/536>`__)
* Contributors: Nathan Wiebe Neufeldt, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_clang_tidy <https://github.com/ament/ament_lint/tree/lyrical/ament_cmake_clang_tidy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix cmake deprecation (`#539 <https://github.com/ament/ament_lint/issues/539>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_copyright <https://github.com/ament/ament_lint/tree/lyrical/ament_cmake_copyright/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix cmake deprecation (`#539 <https://github.com/ament/ament_lint/issues/539>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_core <https://github.com/ament/ament_cmake/tree/lyrical/ament_cmake_core/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Remove unused AMENT_CMAKE_ENVIRONMENT_GENERATION option (`#354 <https://github.com/ament/ament_cmake/issues/354>`__)
* Address ament_lint_cmake regressions (`#604 <https://github.com/ament/ament_cmake/issues/604>`__)
* Respect find_package(QUIET) in chains from ament_cmake_core (`#603 <https://github.com/ament/ament_cmake/issues/603>`__)
* perf: faster normalize_path implementation using cmake_path (`#586 <https://github.com/ament/ament_cmake/issues/586>`__)
* Contributors: Nathan Boisard, Scott K Logan, Shane Loretz


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_cppcheck <https://github.com/ament/ament_lint/tree/lyrical/ament_cmake_cppcheck/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix cmake deprecation (`#539 <https://github.com/ament/ament_lint/issues/539>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_cpplint <https://github.com/ament/ament_lint/tree/lyrical/ament_cmake_cpplint/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fixing EXCLUDE consistency (`#481 <https://github.com/ament/ament_lint/issues/481>`__)
* cpplint: update link to upstream cpplint repo (`#538 <https://github.com/ament/ament_lint/issues/538>`__)
* Fix cmake deprecation (`#539 <https://github.com/ament/ament_lint/issues/539>`__)
* Contributors: Romain Reignier, Tom Moore, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_export_targets <https://github.com/ament/ament_cmake/tree/lyrical/ament_cmake_export_targets/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Address ament_lint_cmake regressions (`#604 <https://github.com/ament/ament_cmake/issues/604>`__)
* Contributors: Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_flake8 <https://github.com/ament/ament_lint/tree/lyrical/ament_cmake_flake8/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fixing EXCLUDE consistency (`#481 <https://github.com/ament/ament_lint/issues/481>`__)
* Fix cmake deprecation (`#539 <https://github.com/ament/ament_lint/issues/539>`__)
* Contributors: Tom Moore, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_gen_version_h <https://github.com/ament/ament_cmake/tree/lyrical/ament_cmake_gen_version_h/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Address ament_lint_cmake regressions (`#604 <https://github.com/ament/ament_cmake/issues/604>`__)
* Update CMake requirement (`#589 <https://github.com/ament/ament_cmake/issues/589>`__)
* Removed deprecated function ament_cmake_gen_version_h (`#582 <https://github.com/ament/ament_cmake/issues/582>`__)
* Contributors: Alejandro Hernández Cordero, Scott K Logan, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_gmock <https://github.com/ament/ament_cmake/tree/lyrical/ament_cmake_gmock/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use libgtest-dev and libgmock-dev (`#622 <https://github.com/ament/ament_cmake//issues/622>`__)
* Contributors: Shane Loretz


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_gtest <https://github.com/ament/ament_cmake/tree/lyrical/ament_cmake_gtest/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use libgtest-dev and libgmock-dev (`#622 <https://github.com/ament/ament_cmake//issues/622>`__)
* Contributors: Shane Loretz


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_libraries <https://github.com/ament/ament_cmake/tree/lyrical/ament_cmake_libraries/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Address ament_lint_cmake regressions (`#604 <https://github.com/ament/ament_cmake/issues/604>`__)
* Contributors: Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_lint_cmake <https://github.com/ament/ament_lint/tree/lyrical/ament_cmake_lint_cmake/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix cmake deprecation (`#539 <https://github.com/ament/ament_lint/issues/539>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_mypy <https://github.com/ament/ament_lint/tree/lyrical/ament_cmake_mypy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [ament_mypy] Add ``--ament-strict`` flag for more strict type checking. (`#573 <https://github.com/ament/ament_lint/issues/573>`__)
* Fix cmake deprecation (`#539 <https://github.com/ament/ament_lint/issues/539>`__)
* Contributors: Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_pclint <https://github.com/ament/ament_lint/tree/lyrical/ament_cmake_pclint/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix cmake deprecation (`#539 <https://github.com/ament/ament_lint/issues/539>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_pep257 <https://github.com/ament/ament_lint/tree/lyrical/ament_cmake_pep257/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix cmake deprecation (`#539 <https://github.com/ament/ament_lint/issues/539>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_pycodestyle <https://github.com/ament/ament_lint/tree/lyrical/ament_cmake_pycodestyle/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix cmake deprecation (`#539 <https://github.com/ament/ament_lint/issues/539>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_pyflakes <https://github.com/ament/ament_lint/tree/lyrical/ament_cmake_pyflakes/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix cmake deprecation (`#539 <https://github.com/ament/ament_lint/issues/539>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_python <https://github.com/ament/ament_cmake/tree/lyrical/ament_cmake_python/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* feature: allow extending a python package in ``ament_python_install_package`` (`#587 <https://github.com/ament/ament_cmake//issues/587>`__)
* Add missing dependency (`#617 <https://github.com/ament/ament_cmake/issues/617>`__)
* Contributors: Nadav Elkabets, Robert Haschke


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_python_test <https://github.com/ament/ament_cmake/tree/lyrical/ament_cmake_python_test/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* feature: allow extending a python package in ``ament_python_install_package`` (`#587 <https://github.com/ament/ament_cmake//issues/587>`__)
* Contributors: Nadav Elkabets


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_ros <https://github.com/ros2/ament_cmake_ros/tree/lyrical/ament_cmake_ros/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix cmake deprecation (`#47 <https://github.com/ros2/ament_cmake_ros/issues/47>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_ros_core <https://github.com/ros2/ament_cmake_ros/tree/lyrical/ament_cmake_ros_core/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add ``ament_ros_defaults`` target (`#62 <https://github.com/ros2/ament_cmake_ros/issues/62>`__)
* fix cmake deprecation (`#47 <https://github.com/ros2/ament_cmake_ros/issues/47>`__)
* Contributors: Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_target_dependencies <https://github.com/ament/ament_cmake/tree/lyrical/ament_cmake_target_dependencies/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Revert "Revert "Removed deprecated function ament_cmake_target_dependencies (…" (`#614 <https://github.com/ament/ament_cmake/issues/614>`__)
* Revert "Removed deprecated function ament_cmake_target_dependencies" (`#585 <https://github.com/ament/ament_cmake/issues/585>`__)
* Removed deprecated function ament_cmake_target_dependencies (`#583 <https://github.com/ament/ament_cmake/issues/583>`__)
* Contributors: Alejandro Hernández Cordero, Shane Loretz


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_uncrustify <https://github.com/ament/ament_lint/tree/lyrical/ament_cmake_uncrustify/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [ament_cmake_uncrustify] Add ament_cmake_uncrustify_LANGUAGE variable (`#384 <https://github.com/ament/ament_lint/issues/384>`__)
* Fix cmake deprecation (`#539 <https://github.com/ament/ament_lint/issues/539>`__)
* Contributors: Abrar Rahman Protyasha, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_vendor_package <https://github.com/ament/ament_cmake/tree/lyrical/ament_cmake_vendor_package/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* ament_vendor: Propagate additional variables to ExternalProject (`#593 <https://github.com/ament/ament_cmake/issues/593>`__)
* Contributors: Silvio Traversaro


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_xmllint <https://github.com/ament/ament_lint/tree/lyrical/ament_cmake_xmllint/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix cmake deprecation (`#539 <https://github.com/ament/ament_lint/issues/539>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_copyright <https://github.com/ament/ament_lint/tree/lyrical/ament_copyright/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [ament_mypy] Fix config for ament_cmake packages and type entrypoints (`#574 <https://github.com/ament/ament_lint/issues/574>`__)
* Drop setuptools from install_requires (`#566 <https://github.com/ament/ament_lint/issues/566>`__)
* Remove ``importlib_metadata`` (`#564 <https://github.com/ament/ament_lint/issues/564>`__)
* Remove invalid license template. (`#209 <https://github.com/ament/ament_lint/issues/209>`__)
* Export typing information for ament linters (`#553 <https://github.com/ament/ament_lint/issues/553>`__)
* fix setuptools deprecations (`#547 <https://github.com/ament/ament_lint/issues/547>`__)
* Contributors: Jochen Sprickerhof, Michael Carlstrom, Tully Foote, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cppcheck <https://github.com/ament/ament_lint/tree/lyrical/ament_cppcheck/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [ament_mypy] Fix config for ament_cmake packages and type entrypoints (`#574 <https://github.com/ament/ament_lint/issues/574>`__)
* Drop setuptools from install_requires (`#566 <https://github.com/ament/ament_lint/issues/566>`__)
* Export typing information for ament linters (`#553 <https://github.com/ament/ament_lint/issues/553>`__)
* fix setuptools deprecations (`#547 <https://github.com/ament/ament_lint/issues/547>`__)
* Contributors: Jochen Sprickerhof, Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cpplint <https://github.com/ament/ament_lint/tree/lyrical/ament_cpplint/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [ament_mypy] Fix config for ament_cmake packages and type entrypoints (`#574 <https://github.com/ament/ament_lint/issues/574>`__)
* Drop setuptools from install_requires (`#566 <https://github.com/ament/ament_lint/issues/566>`__)
* Export typing information for ament linters (`#553 <https://github.com/ament/ament_lint/issues/553>`__)
* fix setuptools deprecations (`#547 <https://github.com/ament/ament_lint/issues/547>`__)
* cpplint: update link to upstream cpplint repo (`#538 <https://github.com/ament/ament_lint/issues/538>`__)
* Contributors: Jochen Sprickerhof, Michael Carlstrom, Romain Reignier, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_flake8 <https://github.com/ament/ament_lint/tree/lyrical/ament_flake8/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [ament_mypy] Add ``--ament-strict`` flag for more strict type checking. (`#573 <https://github.com/ament/ament_lint/issues/573>`__)
* Drop setuptools from install_requires (`#566 <https://github.com/ament/ament_lint/issues/566>`__)
* Drop dependency on python3-flake8-docstrings (`#513 <https://github.com/ament/ament_lint/issues/513>`__)
* Export typing information for ament linters (`#553 <https://github.com/ament/ament_lint/issues/553>`__)
* fix setuptools deprecations (`#547 <https://github.com/ament/ament_lint/issues/547>`__)
* Contributors: Jochen Sprickerhof, Michael Carlstrom, Scott K Logan, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_index_cpp <https://github.com/ament/ament_index/tree/lyrical/ament_index_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Cleanups (`#114 <https://github.com/ament/ament_index/issues/114>`__)
* Use get_package_share_path just as python (`#112 <https://github.com/ament/ament_index//issues/112>`__)
* Add autogenerated version header (`#105 <https://github.com/ament/ament_index/issues/105>`__)
* Extend API to use std::filesystem (`#104 <https://github.com/ament/ament_index/issues/104>`__)
* Fix CMake deprecation (`#102 <https://github.com/ament/ament_index/issues/102>`__)
* Contributors: Alejandro Hernández Cordero, Eric Lujan, Tim Clephas, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_index_python <https://github.com/ament/ament_index/tree/lyrical/ament_index_python/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Cleanups ament_index_python (`#115 <https://github.com/ament/ament_index/issues/115>`__)
* fix setuptools deprecations (`#101 <https://github.com/ament/ament_index/issues/101>`__)
* Contributors: Alejandro Hernández Cordero, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_lint <https://github.com/ament/ament_lint/tree/lyrical/ament_lint/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Drop setuptools from install_requires (`#566 <https://github.com/ament/ament_lint/issues/566>`__)
* Export typing information for ament linters (`#553 <https://github.com/ament/ament_lint/issues/553>`__)
* fix setuptools deprecations (`#547 <https://github.com/ament/ament_lint/issues/547>`__)
* Contributors: Jochen Sprickerhof, Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_lint_auto <https://github.com/ament/ament_lint/tree/lyrical/ament_lint_auto/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix cmake deprecation (`#539 <https://github.com/ament/ament_lint/issues/539>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_lint_cmake <https://github.com/ament/ament_lint/tree/lyrical/ament_lint_cmake/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Drop setuptools from install_requires (`#566 <https://github.com/ament/ament_lint/issues/566>`__)
* Export typing information for ament linters (`#553 <https://github.com/ament/ament_lint/issues/553>`__)
* fix setuptools deprecations (`#547 <https://github.com/ament/ament_lint/issues/547>`__)
* Contributors: Jochen Sprickerhof, Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_lint_common <https://github.com/ament/ament_lint/tree/lyrical/ament_lint_common/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix cmake deprecation (`#539 <https://github.com/ament/ament_lint/issues/539>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_mypy <https://github.com/ament/ament_lint/tree/lyrical/ament_mypy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Allow unused ignores (`#575 <https://github.com/ament/ament_lint/issues/575>`__)
* [ament_mypy] Fix config for ament_cmake packages and type entrypoints (`#574 <https://github.com/ament/ament_lint/issues/574>`__)
* [ament_mypy] Add ``--ament-strict`` flag for more strict type checking. (`#573 <https://github.com/ament/ament_lint/issues/573>`__)
* Drop setuptools from install_requires (`#566 <https://github.com/ament/ament_lint/issues/566>`__)
* Export typing information for ament linters (`#553 <https://github.com/ament/ament_lint/issues/553>`__)
* fix setuptools deprecations (`#547 <https://github.com/ament/ament_lint/issues/547>`__)
* Contributors: Jochen Sprickerhof, Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_package <https://github.com/ament/ament_package/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* feat: add support for fish (`#164 <https://github.com/ament/ament_package/issues/164>`__)
* Fix flake8 (`#163 <https://github.com/ament/ament_package/issues/163>`__)
* Remove unneeded deps (`#161 <https://github.com/ament/ament_package/issues/161>`__)
* fix setuptools deprecations (`#156 <https://github.com/ament/ament_package/issues/156>`__)
* Contributors: Michael Carlstrom, SPeak, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_pclint <https://github.com/ament/ament_lint/tree/lyrical/ament_pclint/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [ament_mypy] Fix config for ament_cmake packages and type entrypoints (`#574 <https://github.com/ament/ament_lint/issues/574>`__)
* Drop setuptools from install_requires (`#566 <https://github.com/ament/ament_lint/issues/566>`__)
* Export typing information for ament linters (`#553 <https://github.com/ament/ament_lint/issues/553>`__)
* clean setup.py (`#552 <https://github.com/ament/ament_lint/issues/552>`__)
* fix setuptools deprecation (`#551 <https://github.com/ament/ament_lint/issues/551>`__)
* fix setuptools deprecations (`#547 <https://github.com/ament/ament_lint/issues/547>`__)
* Contributors: Jochen Sprickerhof, Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_pep257 <https://github.com/ament/ament_lint/tree/lyrical/ament_pep257/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Skip pydocstyle tests if it can't be imported (`#579 <https://github.com/ament/ament_lint/issues/579>`__)
* [ament_mypy] Fix config for ament_cmake packages and type entrypoints (`#574 <https://github.com/ament/ament_lint/issues/574>`__)
* Drop setuptools from install_requires (`#566 <https://github.com/ament/ament_lint/issues/566>`__)
* Export typing information for ament linters (`#553 <https://github.com/ament/ament_lint/issues/553>`__)
* fix setuptools deprecations (`#547 <https://github.com/ament/ament_lint/issues/547>`__)
* Contributors: Jochen Sprickerhof, Michael Carlstrom, Scott K Logan, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_pycodestyle <https://github.com/ament/ament_lint/tree/lyrical/ament_pycodestyle/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [ament_mypy] Fix config for ament_cmake packages and type entrypoints (`#574 <https://github.com/ament/ament_lint/issues/574>`__)
* Drop setuptools from install_requires (`#566 <https://github.com/ament/ament_lint/issues/566>`__)
* Export typing information for ament linters (`#553 <https://github.com/ament/ament_lint/issues/553>`__)
* fix setuptools deprecations (`#547 <https://github.com/ament/ament_lint/issues/547>`__)
* Contributors: Jochen Sprickerhof, Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_pyflakes <https://github.com/ament/ament_lint/tree/lyrical/ament_pyflakes/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [ament_mypy] Fix config for ament_cmake packages and type entrypoints (`#574 <https://github.com/ament/ament_lint/issues/574>`__)
* Drop setuptools from install_requires (`#566 <https://github.com/ament/ament_lint/issues/566>`__)
* Export typing information for ament linters (`#553 <https://github.com/ament/ament_lint/issues/553>`__)
* fix setuptools deprecations (`#547 <https://github.com/ament/ament_lint/issues/547>`__)
* Contributors: Jochen Sprickerhof, Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_uncrustify <https://github.com/ament/ament_lint/tree/lyrical/ament_uncrustify/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [ament_mypy] Fix config for ament_cmake packages and type entrypoints (`#574 <https://github.com/ament/ament_lint/issues/574>`__)
* Drop setuptools from install_requires (`#566 <https://github.com/ament/ament_lint/issues/566>`__)
* Revert "Removed uncrustify_vendor (`#556 <https://github.com/ament/ament_lint/issues/556>`__)" (`#561 <https://github.com/ament/ament_lint/issues/561>`__)
* Removed uncrustify_vendor (`#556 <https://github.com/ament/ament_lint/issues/556>`__)
* Export typing information for ament linters (`#553 <https://github.com/ament/ament_lint/issues/553>`__)
* fix setuptools deprecations (`#547 <https://github.com/ament/ament_lint/issues/547>`__)
* Contributors: Alejandro Hernández Cordero, Jochen Sprickerhof, Michael Carlstrom, Michael Orlov, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_xmllint <https://github.com/ament/ament_lint/tree/lyrical/ament_xmllint/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [ament_mypy] Add ``--ament-strict`` flag for more strict type checking. (`#573 <https://github.com/ament/ament_lint/issues/573>`__)
* xmllint: fetch external schemas via Python (`#570 <https://github.com/ament/ament_lint/issues/570>`__)
* Drop setuptools from install_requires (`#566 <https://github.com/ament/ament_lint/issues/566>`__)
* Export typing information for ament linters (`#553 <https://github.com/ament/ament_lint/issues/553>`__)
* fix setuptools deprecations (`#547 <https://github.com/ament/ament_lint/issues/547>`__)
* Contributors: Jochen Sprickerhof, Michael Carlstrom, Michael Carroll, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`builtin_interfaces <https://github.com/ros2/rcl_interfaces/tree/lyrical/builtin_interfaces/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix cmake deprecation (`#180 <https://github.com/ros2/rcl_interfaces/issues/180>`__)
* Add info to duration message and time message comments (`#176 <https://github.com/ros2/rcl_interfaces/issues/176>`__)
* Contributors: Jimmy McElwain, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`camera_calibration_parsers <https://github.com/ros-perception/image_common/tree/lyrical/camera_calibration_parsers/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#396 <https://github.com/ros-perception/image_common/issues/396>`__)
* Delete camera_calibration_parsers/setup.py (`#393 <https://github.com/ros-perception/image_common/issues/393>`__)
* Update BSD licenses to SPDX identifier (`#389 <https://github.com/ros-perception/image_common/issues/389>`__) Co-authored-by: Alejandro Hernandez Cordero <ahcorde@gmail.com>
* Fix cmake deprecation (`#367 <https://github.com/ros-perception/image_common/issues/367>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#345 <https://github.com/ros-perception/image_common/issues/345>`__)
* Contributors: Emerson Knapp, Garrett Brown, Michael Carlstrom, Shane Loretz, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`camera_info_manager <https://github.com/ros-perception/image_common/tree/lyrical/camera_info_manager/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#396 <https://github.com/ros-perception/image_common/issues/396>`__)
* Added camera info manager unit test (`#358 <https://github.com/ros-perception/image_common/issues/358>`__)
* Use get_package_share_path (`#391 <https://github.com/ros-perception/image_common/issues/391>`__)
* Update BSD licenses to SPDX identifier (`#389 <https://github.com/ros-perception/image_common/issues/389>`__) Co-authored-by: Alejandro Hernandez Cordero <ahcorde@gmail.com>
* Updated deprecated ament_index_cpp API (`#388 <https://github.com/ros-perception/image_common/issues/388>`__)
* Fix compilation error with clang (`#372 <https://github.com/ros-perception/image_common/issues/372>`__)
* Support lifecycle node - NodeInterfaces (`#352 <https://github.com/ros-perception/image_common/issues/352>`__)
* Deprecated rmw_qos_profile_t in favour of rclcpp::QoS (`#364 <https://github.com/ros-perception/image_common/issues/364>`__)
* Fix cmake deprecation (`#367 <https://github.com/ros-perception/image_common/issues/367>`__)
* Contributors: Alejandro Hernández Cordero, Emerson Knapp, Garrett Brown, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`camera_info_manager_py <https://github.com/ros-perception/image_common/tree/lyrical/camera_info_manager_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update BSD licenses to SPDX identifier (`#389 <https://github.com/ros-perception/image_common/issues/389>`__) Co-authored-by: Alejandro Hernandez Cordero <ahcorde@gmail.com>
* Cleanup mislabeled BSD license (`#382 <https://github.com/ros-perception/image_common/issues/382>`__)
* fix setuptools deprecation (`#366 <https://github.com/ros-perception/image_common/issues/366>`__)
* Fix CameraInfo distortion coefficients and logger (`#360 <https://github.com/ros-perception/image_common/issues/360>`__)
* Contributors: Alejandro Hernández Cordero, Garrett Brown, Rick-v-E, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`class_loader <https://github.com/ros/class_loader/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix compiler error with clang (`#227 <https://github.com/ros/class_loader/issues/227>`__)
* Remove ament_cmake_ros dependency (`#226 <https://github.com/ros/class_loader/issues/226>`__) The dependent ament_cmake_ros package transitively pulls in RMW-layer packages which is unnecessarily heavy to class_loader that is supposed to be an independent plugin loading library. This commit removes the ament_cmake_ros dependency and replaces with a plain ament_cmake with an explicit SHARED library type to keep the dependency minimal.
* Improvements (`#225 <https://github.com/ros/class_loader/issues/225>`__)
* Clean up tests (`#224 <https://github.com/ros/class_loader/issues/224>`__)
* Add support for passing arguments to constructors (`#223 <https://github.com/ros/class_loader//issues/223>`__)
* Thread and Address Sanitizer CI (`#198 <https://github.com/ros/class_loader/issues/198>`__)
* Update cmake requirement
* Remove CODEOWNERS and mirror-rolling-to-main workflow (`#215 <https://github.com/ros/class_loader/issues/215>`__)
* Contributors: Alejandro Hernández Cordero, CY Chen, Tyler Weaver, mosfet80, pum1k


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`common_interfaces <https://github.com/ros2/common_interfaces/tree/lyrical/common_interfaces/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix CMAKE deprecation (`#288 <https://github.com/ros2/common_interfaces/issues/288>`__)
* Removed deprecated actionlib_msgs (`#280 <https://github.com/ros2/common_interfaces/issues/280>`__)
* Contributors: Alejandro Hernández Cordero, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`composition <https://github.com/ros2/demos/tree/lyrical/composition/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#781 <https://github.com/ros2/demos//issues/781>`__)
* Add tests isolation in test_dlopen_composition.py.in and test_linktime_composition.py.in (`#764 <https://github.com/ros2/demos//issues/764>`__)
* Switching to example_interfaces (`#674 <https://github.com/ros2/demos/issues/674>`__)
* Log message for linktime composition on Windows (`#640 <https://github.com/ros2/demos/issues/640>`__)
* correct name of shared libraries and their location (`#722 <https://github.com/ros2/demos/issues/722>`__) (`#726 <https://github.com/ros2/demos/issues/726>`__)
* Use EnableRmwIsolation in launch tests (`#724 <https://github.com/ros2/demos/issues/724>`__)
* Uniform CMAKE min VERSION (`#714 <https://github.com/ros2/demos/issues/714>`__)
* Set envars to run tests with rmw_zenoh_cpp with multicast discovery (`#711 <https://github.com/ros2/demos/issues/711>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#707 <https://github.com/ros2/demos/issues/707>`__)
* Contributors: Alejandro Hernández Cordero, Emerson Knapp, Julien Enoch, Lucas Wendland, Scott K Logan, Shane Loretz, mergify[bot], mosfet80, yadunund


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`composition_interfaces <https://github.com/ros2/rcl_interfaces/tree/lyrical/composition_interfaces/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix cmake deprecation (`#180 <https://github.com/ros2/rcl_interfaces/issues/180>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`console_bridge_vendor <https://github.com/ros2/console_bridge_vendor/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update CMake version here and console_bridge (`#44 <https://github.com/ros2/console_bridge_vendor/issues/44>`__)
* Remove CODEOWNERS and mirror-rolling-to-master workflow. (`#42 <https://github.com/ros2/console_bridge_vendor/issues/42>`__)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`demo_nodes_cpp <https://github.com/ros2/demos/tree/lyrical/demo_nodes_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#781 <https://github.com/ros2/demos//issues/781>`__)
* add child logger under parent node, with different log levels. (`#772 <https://github.com/ros2/demos//issues/772>`__)
* Switching to example_interfaces (`#674 <https://github.com/ros2/demos/issues/674>`__)
* Update subscription callback signatures (`#754 <https://github.com/ros2/demos/issues/754>`__)
* Use EnableRmwIsolation in launch tests (`#724 <https://github.com/ros2/demos/issues/724>`__)
* fix typo in docs demo_nodes_cpp (`#715 <https://github.com/ros2/demos/issues/715>`__)
* Uniform CMAKE min VERSION (`#714 <https://github.com/ros2/demos/issues/714>`__)
* Set envars to run tests with rmw_zenoh_cpp with multicast discovery (`#711 <https://github.com/ros2/demos/issues/711>`__)
* Contributors: Alejandro Hernández Cordero, Emerson Knapp, Khaled Gabr, Lucas Wendland, Scott K Logan, Tomoya Fujita, mini-1235, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`demo_nodes_cpp_native <https://github.com/ros2/demos/tree/lyrical/demo_nodes_cpp_native/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#781 <https://github.com/ros2/demos//issues/781>`__)
* Switching to example_interfaces (`#674 <https://github.com/ros2/demos/issues/674>`__)
* Removed outdated TODO (`#723 <https://github.com/ros2/demos/issues/723>`__)
* Uniform CMAKE min VERSION (`#714 <https://github.com/ros2/demos/issues/714>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#707 <https://github.com/ros2/demos/issues/707>`__)
* Contributors: Alejandro Hernández Cordero, Emerson Knapp, Lucas Wendland, Shane Loretz, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`demo_nodes_py <https://github.com/ros2/demos/tree/lyrical/demo_nodes_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* add child logger under parent node, with different log levels. (`#772 <https://github.com/ros2/demos//issues/772>`__)
* Fix deprecated RcutilsLogger::warn() usage in LoggerServiceNode (`#773 <https://github.com/ros2/demos//issues/773>`__)
* Ignore A005 (`#771 <https://github.com/ros2/demos//issues/771>`__)
* Switching to example_interfaces (`#674 <https://github.com/ros2/demos/issues/674>`__)
* fix setuptools deprecations (`#733 <https://github.com/ros2/demos/issues/733>`__)
* Revert "Revert "fix loading parameter behavior from yaml file. (`#656 <https://github.com/ros2/demos/issues/656>`__)" (`#660 <https://github.com/ros2/demos/issues/660>`__)" (`#661 <https://github.com/ros2/demos/issues/661>`__)
* Contributors: Barry Xu, Lucas Wendland, Michael Carlstrom, Tomoya Fujita, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`diagnostic_msgs <https://github.com/ros2/common_interfaces/tree/lyrical/diagnostic_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix CMAKE deprecation (`#288 <https://github.com/ros2/common_interfaces/issues/288>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`domain_coordinator <https://github.com/ros2/ament_cmake_ros/tree/lyrical/domain_coordinator/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix setuptools deprecations (`#49 <https://github.com/ros2/ament_cmake_ros/issues/49>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`dummy_map_server <https://github.com/ros2/demos/tree/lyrical/dummy_robot/dummy_map_server/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#781 <https://github.com/ros2/demos//issues/781>`__)
* get rid of deprecated rclcpp::spin_some(). (`#734 <https://github.com/ros2/demos/issues/734>`__)
* Uniform CMAKE min VERSION (`#714 <https://github.com/ros2/demos/issues/714>`__) demo_nodes_cpp/CMakeLists.txt require cmake min version 3.12 other modules cmake 3.5. It is proposed to standardize with version 3.12. This also fixes cmake <3.10 deprecation warnings
* Use target_link_libraries instead of ament_target_dependencies (`#707 <https://github.com/ros2/demos/issues/707>`__)
* Contributors: Emerson Knapp, Shane Loretz, Tomoya Fujita, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`dummy_robot_bringup <https://github.com/ros2/demos/tree/lyrical/dummy_robot/dummy_robot_bringup/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fixed launch file (`#759 <https://github.com/ros2/demos/issues/759>`__)
* Added README.md for dummy_robot_bringup. (`#574 <https://github.com/ros2/demos/issues/574>`__)
* Uniform CMAKE min VERSION (`#714 <https://github.com/ros2/demos/issues/714>`__) demo_nodes_cpp/CMakeLists.txt require cmake min version 3.12 other modules cmake 3.5. It is proposed to standardize with version 3.12. This also fixes cmake <3.10 deprecation warnings
* Contributors: Alejandro Hernández Cordero, Gary Bey, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`dummy_sensors <https://github.com/ros2/demos/tree/lyrical/dummy_robot/dummy_sensors/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#781 <https://github.com/ros2/demos//issues/781>`__)
* get rid of deprecated rclcpp::spin_some(). (`#734 <https://github.com/ros2/demos/issues/734>`__)
* Uniform CMAKE min VERSION (`#714 <https://github.com/ros2/demos/issues/714>`__) demo_nodes_cpp/CMakeLists.txt require cmake min version 3.12 other modules cmake 3.5. It is proposed to standardize with version 3.12. This also fixes cmake <3.10 deprecation warnings
* Use target_link_libraries instead of ament_target_dependencies (`#707 <https://github.com/ros2/demos/issues/707>`__)
* Contributors: Emerson Knapp, Shane Loretz, Tomoya Fujita, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`eigen3_cmake_module <https://github.com/ros2/eigen3_cmake_module/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix cmake deprecation (`#10 <https://github.com/ros2/eigen3_cmake_module/issues/10>`__)
* Remove CODEOWNERS and mirror-rolling-to-master workflow. (`#8 <https://github.com/ros2/eigen3_cmake_module/issues/8>`__)
* Contributors: Chris Lalancette, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`example_interfaces <https://github.com/ros2/example_interfaces/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix cmake deprecation (`#23 <https://github.com/ros2/example_interfaces/issues/23>`__)
* Remove .github/ISSUE_TEMPLATE.md (old version of templates) (`#21 <https://github.com/ros2/example_interfaces/issues/21>`__)
* Remove CODEOWNERS and mirror-rolling-to-master workflow. (`#19 <https://github.com/ros2/example_interfaces/issues/19>`__)
* Contributors: Chris Lalancette, Tomoya Fujita, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_async_client <https://github.com/ros2/examples/tree/lyrical/rclcpp/services/async_client/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#444 <https://github.com/ros2/examples/issues/444>`__)
* Fix CMAKE deprecation (`#419 <https://github.com/ros2/examples/issues/419>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#404 <https://github.com/ros2/examples/issues/404>`__)
* Contributors: Emerson Knapp, Shane Loretz, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_cbg_executor <https://github.com/ros2/examples/tree/lyrical/rclcpp/executors/cbg_executor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#444 <https://github.com/ros2/examples/issues/444>`__)
* Fix CMAKE deprecation (`#419 <https://github.com/ros2/examples/issues/419>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#404 <https://github.com/ros2/examples/issues/404>`__)
* Contributors: Emerson Knapp, Shane Loretz, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_minimal_action_client <https://github.com/ros2/examples/tree/lyrical/rclcpp/actions/minimal_action_client/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#444 <https://github.com/ros2/examples/issues/444>`__)
* get rid of deprecated rclcpp::spin_some(). (`#422 <https://github.com/ros2/examples//issues/422>`__)
* Fix CMAKE deprecation (`#419 <https://github.com/ros2/examples/issues/419>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#404 <https://github.com/ros2/examples/issues/404>`__)
* Contributors: Emerson Knapp, Shane Loretz, Tomoya Fujita, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_minimal_action_server <https://github.com/ros2/examples/tree/lyrical/rclcpp/actions/minimal_action_server/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#444 <https://github.com/ros2/examples/issues/444>`__)
* Add rclcpp single goal action server example (`#429 <https://github.com/ros2/examples/issues/429>`__)
* Fix CMAKE deprecation (`#419 <https://github.com/ros2/examples/issues/419>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#404 <https://github.com/ros2/examples/issues/404>`__)
* Contributors: Emerson Knapp, Shane Loretz, Taiga Arai, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_minimal_client <https://github.com/ros2/examples/tree/lyrical/rclcpp/services/minimal_client/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#444 <https://github.com/ros2/examples/issues/444>`__)
* Fix CMAKE deprecation (`#419 <https://github.com/ros2/examples/issues/419>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#404 <https://github.com/ros2/examples/issues/404>`__)
* Contributors: Emerson Knapp, Shane Loretz, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_minimal_composition <https://github.com/ros2/examples/tree/lyrical/rclcpp/composition/minimal_composition/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#444 <https://github.com/ros2/examples/issues/444>`__)
* Fix CMAKE deprecation (`#419 <https://github.com/ros2/examples/issues/419>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#404 <https://github.com/ros2/examples/issues/404>`__)
* Contributors: Emerson Knapp, Shane Loretz, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_minimal_publisher <https://github.com/ros2/examples/tree/lyrical/rclcpp/topics/minimal_publisher/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#444 <https://github.com/ros2/examples/issues/444>`__)
* Improve minimal_publisher README with clearer structure and usage guidance (`#434 <https://github.com/ros2/examples/issues/434>`__)
* get rid of deprecated rclcpp::spin_some(). (`#422 <https://github.com/ros2/examples//issues/422>`__)
* Fix CMAKE deprecation (`#419 <https://github.com/ros2/examples/issues/419>`__)
* wait 5 secs until all subscriptions acknowledge the messages. (`#414 <https://github.com/ros2/examples/issues/414>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#404 <https://github.com/ros2/examples/issues/404>`__)
* Contributors: Emerson Knapp, Shane Loretz, Tomoya Fujita, Yadnyeshwar Amol Sakhare, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_minimal_service <https://github.com/ros2/examples/tree/lyrical/rclcpp/services/minimal_service/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#444 <https://github.com/ros2/examples/issues/444>`__)
* Fix CMAKE deprecation (`#419 <https://github.com/ros2/examples/issues/419>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#404 <https://github.com/ros2/examples/issues/404>`__)
* Contributors: Emerson Knapp, Shane Loretz, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_minimal_subscriber <https://github.com/ros2/examples/tree/lyrical/rclcpp/topics/minimal_subscriber/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#444 <https://github.com/ros2/examples/issues/444>`__)
* Fix CMAKE deprecation (`#419 <https://github.com/ros2/examples/issues/419>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#404 <https://github.com/ros2/examples/issues/404>`__)
* Contributors: Emerson Knapp, Shane Loretz, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_minimal_timer <https://github.com/ros2/examples/tree/lyrical/rclcpp/timers/minimal_timer/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix CMAKE deprecation (`#419 <https://github.com/ros2/examples/issues/419>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#404 <https://github.com/ros2/examples/issues/404>`__)
* Contributors: Shane Loretz, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_multithreaded_executor <https://github.com/ros2/examples/tree/lyrical/rclcpp/executors/multithreaded_executor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#444 <https://github.com/ros2/examples/issues/444>`__)
* Fix CMAKE deprecation (`#419 <https://github.com/ros2/examples/issues/419>`__)
* Improve readibility of reported thread ids in the multithreaded executor example (`#415 <https://github.com/ros2/examples/issues/415>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#404 <https://github.com/ros2/examples/issues/404>`__)
* Contributors: Emerson Knapp, José Faria, Shane Loretz, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_wait_set <https://github.com/ros2/examples/tree/lyrical/rclcpp/wait_set/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#444 <https://github.com/ros2/examples/issues/444>`__)
* Fix CMAKE deprecation (`#419 <https://github.com/ros2/examples/issues/419>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#404 <https://github.com/ros2/examples/issues/404>`__)
* Contributors: Emerson Knapp, Shane Loretz, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclpy_executors <https://github.com/ros2/examples/tree/lyrical/rclpy/executors/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix setuptools deprecations (`#421 <https://github.com/ros2/examples/issues/421>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclpy_guard_conditions <https://github.com/ros2/examples/tree/lyrical/rclpy/guard_conditions/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix setuptools deprecations (`#421 <https://github.com/ros2/examples/issues/421>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclpy_minimal_action_client <https://github.com/ros2/examples/tree/lyrical/rclpy/actions/minimal_action_client/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix setuptools deprecations (`#421 <https://github.com/ros2/examples/issues/421>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclpy_minimal_action_server <https://github.com/ros2/examples/tree/lyrical/rclpy/actions/minimal_action_server/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix setuptools deprecations (`#421 <https://github.com/ros2/examples/issues/421>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclpy_minimal_client <https://github.com/ros2/examples/tree/lyrical/rclpy/services/minimal_client/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* flake8 fixes (`#445 <https://github.com/ros2/examples/issues/445>`__)
* Fix setuptools deprecations (`#421 <https://github.com/ros2/examples/issues/421>`__)
* Contributors: Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclpy_minimal_publisher <https://github.com/ros2/examples/tree/lyrical/rclpy/topics/minimal_publisher/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix setuptools deprecations (`#421 <https://github.com/ros2/examples/issues/421>`__)
* Address flake8 errors for examples_rclpy_minimal_publisher (`#410 <https://github.com/ros2/examples/issues/410>`__)
* Add publisher_member_function_with_wait_for_all_acked.py (`#407 <https://github.com/ros2/examples/issues/407>`__)
* Contributors: Tomoya Fujita, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclpy_minimal_service <https://github.com/ros2/examples/tree/lyrical/rclpy/services/minimal_service/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* flake8 fixes (`#445 <https://github.com/ros2/examples/issues/445>`__)
* Fix setuptools deprecations (`#421 <https://github.com/ros2/examples/issues/421>`__)
* Contributors: Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclpy_minimal_subscriber <https://github.com/ros2/examples/tree/lyrical/rclpy/topics/minimal_subscriber/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* flake8 fixes (`#445 <https://github.com/ros2/examples/issues/445>`__)
* Fix setuptools deprecations (`#421 <https://github.com/ros2/examples/issues/421>`__)
* Contributors: Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclpy_pointcloud_publisher <https://github.com/ros2/examples/tree/lyrical/rclpy/topics/pointcloud_publisher/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix setuptools deprecations (`#421 <https://github.com/ros2/examples/issues/421>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_tf2_py <https://github.com/ros2/geometry2/tree/lyrical/examples_tf2_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix typos (`#921 <https://github.com/ros2/geometry2/issues/921>`__)
* Modernize conf.py files to only include modified Copyright, eliminati… (`#865 <https://github.com/ros2/geometry2/issues/865>`__)
* Fix Setuptools deprecations (`#809 <https://github.com/ros2/geometry2/issues/809>`__)
* Contributors: Auguste Lalande, R Kent James, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`foonathan_memory_vendor <https://github.com/eProsima/foonathan_memory_vendor/tree/master/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Change upstream to fix build with clang (#80)
* Change upstream to eProsima fork to avoid patch command (#80)
* Update upstream to release 0.7-4 (#75)
* Remove installer CMake patches (#75)
* Improve mechanism to find an installation of foonathan_memory (#67)
* Fix ament_lint_cmake errors (#68)
* Add FORCE_BUILD option to cmake (#69)
* Shorten new option description (#70)


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`geometry2 <https://github.com/ros2/geometry2/tree/lyrical/geometry2/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Uniform cmake min version (`#764 <https://github.com/ros2/geometry2/issues/764>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`geometry_msgs <https://github.com/ros2/common_interfaces/tree/lyrical/geometry_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Clarify ``Inertia.msg`` expresses inertia about the center of mass (`#313 <https://github.com/ros2/common_interfaces/issues/313>`__)
* Fix CMAKE deprecation (`#288 <https://github.com/ros2/common_interfaces/issues/288>`__)
* Removed deprecated geometry_msgs/Pose2d (`#283 <https://github.com/ros2/common_interfaces/issues/283>`__)
* Contributors: Alejandro Hernández Cordero, Andrew Symington, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`gmock_vendor <https://github.com/ament/googletest/tree/lyrical/googlemock/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Deprecate gtest_vendor and gmock_vendor (`#41 <https://github.com/ament/googletest/issues/41>`__)
* Contributors: Shane Loretz


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`gtest_vendor <https://github.com/ament/googletest/tree/lyrical/googletest/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Deprecate gtest_vendor and gmock_vendor (`#41 <https://github.com/ament/googletest/issues/41>`__)
* Contributors: Shane Loretz


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`gz_cmake_vendor <https://github.com/gazebo-release/gz_cmake_vendor/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Bump version to 5.1.0 (`#24 <https://github.com/gazebo-release/gz_cmake_vendor/issues/24>`__)
* Merge pull request `#23 <https://github.com/gazebo-release/gz_cmake_vendor/issues/23>`__ Bump version to 5.0.2  ---------
* Bump version to 5.0.1 (`#20 <https://github.com/gazebo-release/gz_cmake_vendor/issues/20>`__)
* Bump version to 5.0.0 (`#19 <https://github.com/gazebo-release/gz_cmake_vendor/issues/19>`__)
* Jetty support: bump to 5.0.0, fix package names (`#16 <https://github.com/gazebo-release/gz_cmake_vendor/issues/16>`__) * Jetty support: bump to 5.0.0, fix package names Major version numbers have been removed from package names in Gazebo Jetty, so extra cmake config files are no longer needed. * Add option VENDOR_FROM_LIB_VCS_REF This allows vendoring from a specified vcs ref instead of the hard-coded tag. When this option is set to true, a branch, tag, or commit can be specified in the LIB_VCS_REF variable. If LIB_VCS_REF is unspecified, vendoring will use main. * remove unused cmake config template * use lowercase to fix linter complaint * 5.0.0~pre1 ---------
* Bump version to 4.2.0 (`#15 <https://github.com/gazebo-release/gz_cmake_vendor/issues/15>`__)
* Contributors: Addisu Z. Taddese, Jose Luis Rivero, Steve Peters


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`gz_math_vendor <https://github.com/gazebo-release/gz_math_vendor/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Bump version to 9.1.0 (`#20 <https://github.com/gazebo-release/gz_math_vendor/issues/20>`__)
* Bump version to 9.0.0 (`#17 <https://github.com/gazebo-release/gz_math_vendor/issues/17>`__)
* Set PYTHONPATH for Jetty packages (`#14 <https://github.com/gazebo-release/gz_math_vendor/issues/14>`__) * Set PYTHONPATH for unversioned packages * Bump to 9.0.0-pre2 * Set PYTHONPATH in separate dsv file ---------
* Bump to 9.0.0-pre2 (`#16 <https://github.com/gazebo-release/gz_math_vendor/issues/16>`__)
* Jetty support: bump to 9.0.0, fix package names (`#12 <https://github.com/gazebo-release/gz_math_vendor/issues/12>`__) * Jetty support: bump to 9.0.0, fix package names Major version numbers have been removed from package names in Gazebo Jetty, so extra cmake config files are no longer needed. * Add option VENDOR_FROM_LIB_VCS_REF This allows vendoring from a specified vcs ref instead of the hard-coded tag. When this option is set to true, a branch, tag, or commit can be specified in the LIB_VCS_REF variable. If LIB_VCS_REF is unspecified, vendoring will use main. * remove unused cmake config file * use lowercase to fix linter complaint * build python bindings * 9.0.0~pre1 ---------
* Bump version to 8.2.0 (`#11 <https://github.com/gazebo-release/gz_math_vendor/issues/11>`__)
* Contributors: Addisu Z. Taddese, Ian Chen, Jose Luis Rivero, Steve Peters


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`gz_utils_vendor <https://github.com/gazebo-release/gz_utils_vendor/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Bump version to 4.0.0 (`#12 <https://github.com/gazebo-release/gz_utils_vendor/issues/12>`__)
* Add dsv for PYTHONPATH for Jetty packages (`#13 <https://github.com/gazebo-release/gz_utils_vendor/issues/13>`__)
* Jetty support: bump to 4.0.0, fix package names (`#11 <https://github.com/gazebo-release/gz_utils_vendor/issues/11>`__) * Jetty support: bump to 4.0.0, fix package names Major version numbers have been removed from package names in Gazebo Jetty, so extra cmake config files are no longer needed. * Add option VENDOR_FROM_LIB_VCS_REF This allows vendoring from a specified vcs ref instead of the hard-coded tag. When this option is set to true, a branch, tag, or commit can be specified in the LIB_VCS_REF variable. If LIB_VCS_REF is unspecified, vendoring will use main. * remove unused cmake config file * use lowercase to fix linter complaint * Add dependency on cli11 * 4.0.0~pre1 * Use vendored version of CLI11 --------- Co-authored-by: Addisu Z. Taddese <addisu@openrobotics.org>
* Contributors: Addisu Z. Taddese, Steve Peters


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`image_common <https://github.com/ros-perception/image_common/tree/lyrical/image_common/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update BSD licenses to SPDX identifier (`#389 <https://github.com/ros-perception/image_common/issues/389>`__) Co-authored-by: Alejandro Hernandez Cordero <ahcorde@gmail.com>
* Fix cmake deprecation (`#367 <https://github.com/ros-perception/image_common/issues/367>`__)
* Contributors: Garrett Brown, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`image_tools <https://github.com/ros2/demos/tree/lyrical/image_tools/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#781 <https://github.com/ros2/demos//issues/781>`__)
* Don't use ``libopencv-dev`` for exec (`#760 <https://github.com/ros2/demos//issues/760>`__)
* Switching to example_interfaces (`#674 <https://github.com/ros2/demos/issues/674>`__)
* Use EnableRmwIsolation in launch tests (`#724 <https://github.com/ros2/demos/issues/724>`__)
* Uniform CMAKE min VERSION (`#714 <https://github.com/ros2/demos/issues/714>`__)
* Lint image_tools/CMakeLists.txt (`#712 <https://github.com/ros2/demos/issues/712>`__)
* Set envars to run tests with rmw_zenoh_cpp with multicast discovery (`#711 <https://github.com/ros2/demos/issues/711>`__)
* Contributors: Alejandro Hernández Cordero, Emerson Knapp, Lucas Wendland, Michael Carlstrom, Scott K Logan, mosfet80, yadunund


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`image_transport <https://github.com/ros-perception/image_common/tree/lyrical/image_transport/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Removed clang warning (`#399 <https://github.com/ros-perception/image_common/issues/399>`__)
* Include message type (`#394 <https://github.com/ros-perception/image_common/issues/394>`__)
* Use new ROSIDL aggregate CMake target (`#396 <https://github.com/ros-perception/image_common/issues/396>`__)
* Update BSD licenses to SPDX identifier (`#389 <https://github.com/ros-perception/image_common/issues/389>`__) Co-authored-by: Alejandro Hernandez Cordero <ahcorde@gmail.com>
* properly shut down rclcpp after all tests complete. (`#384 <https://github.com/ros-perception/image_common/issues/384>`__)
* Fix QoS override tests (`#376 <https://github.com/ros-perception/image_common/issues/376>`__)
* Fix rclcpp_lifecycle dependency (`#373 <https://github.com/ros-perception/image_common/issues/373>`__)
* Fix compilation error with clang (`#372 <https://github.com/ros-perception/image_common/issues/372>`__)
* Support lifecycle node - NodeInterfaces (`#352 <https://github.com/ros-perception/image_common/issues/352>`__)
* Fixed clang build (`#371 <https://github.com/ros-perception/image_common/issues/371>`__)
* fixed build (`#369 <https://github.com/ros-perception/image_common/issues/369>`__)
* Deprecated rmw_qos_profile_t in favour of rclcpp::QoS (`#364 <https://github.com/ros-perception/image_common/issues/364>`__)
* Removed deprecated code (`#356 <https://github.com/ros-perception/image_common/issues/356>`__)
* Fix cmake deprecation (`#367 <https://github.com/ros-perception/image_common/issues/367>`__)
* Fix topic resolution for plugins (`#365 <https://github.com/ros-perception/image_common/issues/365>`__)
* Remove windows warnings (`#350 <https://github.com/ros-perception/image_common/issues/350>`__)
* Add ``rclcpp::shutdown`` (`#347 <https://github.com/ros-perception/image_common/issues/347>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#345 <https://github.com/ros-perception/image_common/issues/345>`__)
* Contributors: Alejandro Hernández Cordero, Alex Tyshka, Emerson Knapp, Garrett Brown, Shane Loretz, Tomoya Fujita, Yuyuan Yuan, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`image_transport_py <https://github.com/ros-perception/image_common/tree/lyrical/image_transport_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#396 <https://github.com/ros-perception/image_common/issues/396>`__)
* Update BSD licenses to SPDX identifier (`#389 <https://github.com/ros-perception/image_common/issues/389>`__) Co-authored-by: Alejandro Hernandez Cordero <ahcorde@gmail.com>
* Use pybind11 from deb or pixi (`#374 <https://github.com/ros-perception/image_common/issues/374>`__)
* Support lifecycle node - NodeInterfaces (`#352 <https://github.com/ros-perception/image_common/issues/352>`__)
* Contributors: Alejandro Hernández Cordero, Emerson Knapp, Garrett Brown


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`interactive_markers <https://github.com/ros-visualization/interactive_markers/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix: Fix compilation on MSVC 2022 (`#120 <https://github.com/ros-visualization/interactive_markers/issues/120>`__)
* Use new ROSIDL aggregate CMake target (`#119 <https://github.com/ros-visualization/interactive_markers/issues/119>`__)
* Cleanup mislabeled BSD license (`#118 <https://github.com/ros-visualization/interactive_markers/issues/118>`__)
* Explicit Time comparissons (`#105 <https://github.com/ros-visualization/interactive_markers/issues/105>`__)
* fix cmake deprecation (`#113 <https://github.com/ros-visualization/interactive_markers/issues/113>`__)
* Contributors: AiVerisimilitude, Alejandro Hernández Cordero, Emerson Knapp, Janosch Machowinski, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`intra_process_demo <https://github.com/ros2/demos/tree/lyrical/intra_process_demo/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#781 <https://github.com/ros2/demos//issues/781>`__)
* Don't use ``libopencv-dev`` for exec (`#760 <https://github.com/ros2/demos//issues/760>`__)
* Switching to example_interfaces (`#674 <https://github.com/ros2/demos/issues/674>`__)
* fixup image_pipeline_demo (`#755 <https://github.com/ros2/demos/issues/755>`__)
* Use EnableRmwIsolation in launch tests (`#724 <https://github.com/ros2/demos/issues/724>`__)
* Uniform CMAKE min VERSION (`#714 <https://github.com/ros2/demos/issues/714>`__)
* Set envars to run tests with rmw_zenoh_cpp with multicast discovery (`#711 <https://github.com/ros2/demos/issues/711>`__)
* Contributors: Alejandro Hernández Cordero, Emerson Knapp, Lucas Wendland, Michael Carlstrom, Scott K Logan, William Woodall, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`kdl_parser <https://github.com/ros/kdl_parser/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Removed kdl vendor dependency (`#90 <https://github.com/ros/kdl_parser//issues/90>`__)
* Cmake requirement (`#88 <https://github.com/ros/kdl_parser/issues/88>`__)
* Remove kdl_parser_py. (`#89 <https://github.com/ros/kdl_parser/issues/89>`__)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`keyboard_handler <https://github.com/ros-tooling/keyboard_handler/tree/lyrical/keyboard_handler/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix cmake deprecation (`#55 <https://github.com/ros-tooling/keyboard_handler/issues/55>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`laser_geometry <https://github.com/ros-perception/laser_geometry/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#115 <https://github.com/ros-perception/laser_geometry/issues/115>`__)
* Use seconds in sensor_msgs::msg::LaserScan msg inside the test (`#107 <https://github.com/ros-perception/laser_geometry/issues/107>`__)
* Use constructor of rclcpp::Time instead of conversion. (`#91 <https://github.com/ros-perception/laser_geometry/issues/91>`__)
* fix cmake deprecation (`#105 <https://github.com/ros-perception/laser_geometry/issues/105>`__)
* Remove hard-coded eigen3 header path for linux hosts (`#95 <https://github.com/ros-perception/laser_geometry/issues/95>`__) Co-authored-by: Alejandro Hernandez Cordero <ahcorde@gmail.com>
* Contributors: AiVerisimilitude, Alejandro Hernández Cordero, Emerson Knapp, Lukas Schäper, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch <https://github.com/ros2/launch/tree/lyrical/launch/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Correct typos (`#961 <https://github.com/ros2/launch/issues/961>`__)
* hotfix (`#950 <https://github.com/ros2/launch/issues/950>`__)
* Declare Boolean Launch Argument (`#944 <https://github.com/ros2/launch/issues/944>`__)
* Support frontends for PathJoinSubstitution (`#943 <https://github.com/ros2/launch/issues/943>`__)
* test python substitution with submodules (`#688 <https://github.com/ros2/launch/issues/688>`__)
* Scope launch file dir/path locals to included launch file (`#862 <https://github.com/ros2/launch/issues/862>`__)
* Capture the environment variables in TimerAction (`#728 <https://github.com/ros2/launch/issues/728>`__)
* Remove importlib metadata (`#932 <https://github.com/ros2/launch/issues/932>`__)
* Fix intersphinx_mapping format (`#921 <https://github.com/ros2/launch/issues/921>`__)
* Make the directory-finding substitutions into a PathSubstitution for / operator (`#914 <https://github.com/ros2/launch//issues/914>`__)
* Expose StringJoinSubstitution to frontend (`#857 <https://github.com/ros2/launch//issues/857>`__)
* Shared logic for substitutions (`#769 <https://github.com/ros2/launch/issues/769>`__)
* Use yaml types (`#781 <https://github.com/ros2/launch/issues/781>`__)
* Switch osrf_pycommon dependency to system package (`#817 <https://github.com/ros2/launch/issues/817>`__)
* Fix all/any in xml and yaml launch files (`#906 <https://github.com/ros2/launch/issues/906>`__)
* Allow providing launch args to include using let in frontends (`#848 <https://github.com/ros2/launch//issues/848>`__)
* Fix Setuptoolsdeprecations (`#898 <https://github.com/ros2/launch/issues/898>`__)
* Remove LaunchDescriptionArgument (`#891 <https://github.com/ros2/launch/issues/891>`__)
* Make sure to install py.typed files (`#886 <https://github.com/ros2/launch/issues/886>`__)
* use custom log_file name as per the user setting (`#861 <https://github.com/ros2/launch/issues/861>`__)
* Using ``TimerAction`` with ``SetParameter`` from launch_ros causes crash (`#879 <https://github.com/ros2/launch/issues/879>`__)
* Fix ``log\_*`` warnings (`#883 <https://github.com/ros2/launch/issues/883>`__)
* Updated ``launch`` typings (`#831 <https://github.com/ros2/launch/issues/831>`__)
* Allow Path in substitutions, instead of requiring cast to str (`#873 <https://github.com/ros2/launch/issues/873>`__)
* Add a ``/`` path join operator for ``PathJoinSubstitution`` (`#868 <https://github.com/ros2/launch/issues/868>`__)
* Other Logging Implementations with ``getLevelNamesMapping`` fix (`#866 <https://github.com/ros2/launch/issues/866>`__)
* Revert "Add Other Logging Implementations (`#858 <https://github.com/ros2/launch/issues/858>`__)" (`#865 <https://github.com/ros2/launch/issues/865>`__) This reverts commit b7b31c45b0eb350deedd282b88398d1ca0d5faf4.
* Add Other Logging Implementations (`#858 <https://github.com/ros2/launch/issues/858>`__)
* Contributors: Auguste Lalande, Christian Ruf, Christophe Bedard, David V. Lu!!, Emerson Knapp, Harrison Chen, Jonas Otto, Kenji Brameld (TRACLabs), Matthijs van der Burgh, Michael Carlstrom, Scott K Logan, Sebastian Javier D'Alessandro Szymanowski, Tanishq Chaudhary, Will, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch_pytest <https://github.com/ros2/launch/tree/lyrical/launch_pytest/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix regressions (`#959 <https://github.com/ros2/launch/issues/959>`__)
* fix: add get_launch_test_fixture_scope for pytest compatibility (`#949 <https://github.com/ros2/launch/issues/949>`__)
* Switch osrf_pycommon dependency to system package (`#817 <https://github.com/ros2/launch/issues/817>`__)
* Fix Setuptoolsdeprecations (`#898 <https://github.com/ros2/launch/issues/898>`__)
* Make sure to install py.typed files (`#886 <https://github.com/ros2/launch/issues/886>`__)
* Add remaining ``py.typed`` (`#884 <https://github.com/ros2/launch/issues/884>`__)
* Allow Path in substitutions, instead of requiring cast to str (`#873 <https://github.com/ros2/launch/issues/873>`__)
* fix(launch_pytest): prevent re-wrapping test funtions on re-run (`#855 <https://github.com/ros2/launch/issues/855>`__)
* Contributors: Christophe Bedard, Daisuke Nishimatsu, David Revay, Emerson Knapp, Michael Carlstrom, Scott K Logan, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch_ros <https://github.com/ros2/launch_ros/tree/lyrical/launch_ros/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix flake8 (`#529 <https://github.com/ros2/launch_ros//issues/529>`__)
* correct typos (`#524 <https://github.com/ros2/launch_ros//issues/524>`__)
* Fix regression (`#521 <https://github.com/ros2/launch_ros//issues/521>`__)
* Fix rhel10 flake8 error (`#515 <https://github.com/ros2/launch_ros//issues/515>`__)
* Compatiblity with 'Populate Transitions' `ros2/rcl#1269 <https://github.com/ros2/rcl/issues/1269>`__ (`#495 <https://github.com/ros2/launch_ros/issues/495>`__)
* remove importlib (`#508 <https://github.com/ros2/launch_ros/issues/508>`__)
* Make FindPackage substitutions a Path to get operator / (`#494 <https://github.com/ros2/launch_ros/issues/494>`__)
* Expose lifecycle_node (`#327 <https://github.com/ros2/launch_ros/issues/327>`__) (with test) (`#482 <https://github.com/ros2/launch_ros/issues/482>`__)
* Expose composable_lifecycle_node in front-end (`#480 <https://github.com/ros2/launch_ros/issues/480>`__)
* Switch osrf_pycommon dependency to system package (`#431 <https://github.com/ros2/launch_ros/issues/431>`__)
* Fix SetUseSimTime for launch frontends (`#488 <https://github.com/ros2/launch_ros/issues/488>`__)
* fix setuptools deprecations (`#475 <https://github.com/ros2/launch_ros/issues/475>`__)
* improve type readability in errors (`#469 <https://github.com/ros2/launch_ros/issues/469>`__)
* Fix: LoadComposableNodes fails to parse wildcard param files correctly (`#460 <https://github.com/ros2/launch_ros/issues/460>`__) (`#465 <https://github.com/ros2/launch_ros/issues/465>`__)
* Contributors: Auguste Lalande, Christophe Bedard, Emerson Knapp, Emre Kuru, Jasper van Brakel, Kenji Brameld, Michael Carlstrom, Scott K Logan, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch_testing <https://github.com/ros2/launch/tree/lyrical/launch_testing/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Correct typos (`#961 <https://github.com/ros2/launch/issues/961>`__)
* Fix test_io_tests for Ubuntu26 (`#960 <https://github.com/ros2/launch/issues/960>`__)
* Fix flake8 (`#952 <https://github.com/ros2/launch/issues/952>`__)
* Switch osrf_pycommon dependency to system package (`#817 <https://github.com/ros2/launch/issues/817>`__)
* Fix Setuptoolsdeprecations (`#898 <https://github.com/ros2/launch/issues/898>`__)
* Make sure to install py.typed files (`#886 <https://github.com/ros2/launch/issues/886>`__)
* Add remaining ``py.typed`` (`#884 <https://github.com/ros2/launch/issues/884>`__)
* Updated ``launch`` typings (`#831 <https://github.com/ros2/launch/issues/831>`__)
* Contributors: Auguste Lalande, Christophe Bedard, Michael Carlstrom, Scott K Logan, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch_testing_ament_cmake <https://github.com/ros2/launch/tree/lyrical/launch_testing_ament_cmake/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix CMake deprecation (`#899 <https://github.com/ros2/launch/issues/899>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch_testing_examples <https://github.com/ros2/examples/tree/lyrical/launch_testing/launch_testing_examples/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* improve test integrity with rmw_cyclonedds_cpp. (`#440 <https://github.com/ros2/examples/issues/440>`__)
* Fix setuptools deprecations (`#421 <https://github.com/ros2/examples/issues/421>`__)
* Contributors: Tomoya Fujita, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch_testing_ros <https://github.com/ros2/launch_ros/tree/lyrical/launch_testing_ros/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add tests isolation in launch_testing_ros (`#528 <https://github.com/ros2/launch_ros//issues/528>`__)
* Surpressing multi-threaded process warning from flake8. (`#520 <https://github.com/ros2/launch_ros//issues/520>`__)
* correct typos (`#524 <https://github.com/ros2/launch_ros//issues/524>`__)
* Fix launch_ros_testing shutdown race in WaitForTopics (`#511 <https://github.com/ros2/launch_ros/issues/511>`__)
* Give the option to inject a quality of service profile (`#493 <https://github.com/ros2/launch_ros/issues/493>`__)
* fix setuptools deprecations (`#475 <https://github.com/ros2/launch_ros/issues/475>`__)
* ``WaitForTopics``: wait for publisher-subscriber connection to be established (`#474 <https://github.com/ros2/launch_ros/issues/474>`__)
* Contributors: Auguste Lalande, Giorgio Pintaudi, Julien Enoch, Michael Carroll, Tomoya Fujita, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch_xml <https://github.com/ros2/launch/tree/lyrical/launch_xml/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Correct typos (`#961 <https://github.com/ros2/launch/issues/961>`__)
* Support frontends for PathJoinSubstitution (`#943 <https://github.com/ros2/launch/issues/943>`__)
* Capture the environment variables in TimerAction (`#728 <https://github.com/ros2/launch/issues/728>`__)
* Expose StringJoinSubstitution to frontend (`#857 <https://github.com/ros2/launch//issues/857>`__)
* Fix all/any in xml and yaml launch files (`#906 <https://github.com/ros2/launch/issues/906>`__)
* Allow providing launch args to include using let in frontends (`#848 <https://github.com/ros2/launch//issues/848>`__)
* Fix Setuptoolsdeprecations (`#898 <https://github.com/ros2/launch/issues/898>`__)
* Make sure to install py.typed files (`#886 <https://github.com/ros2/launch/issues/886>`__)
* Add remaining ``py.typed`` (`#884 <https://github.com/ros2/launch/issues/884>`__)
* Fix ``log\_*`` warnings (`#883 <https://github.com/ros2/launch/issues/883>`__)
* Allow Path in substitutions, instead of requiring cast to str (`#873 <https://github.com/ros2/launch/issues/873>`__)
* Other Logging Implementations with ``getLevelNamesMapping`` fix (`#866 <https://github.com/ros2/launch/issues/866>`__)
* Revert "Add Other Logging Implementations (`#858 <https://github.com/ros2/launch/issues/858>`__)" (`#865 <https://github.com/ros2/launch/issues/865>`__) This reverts commit b7b31c45b0eb350deedd282b88398d1ca0d5faf4.
* Add Other Logging Implementations (`#858 <https://github.com/ros2/launch/issues/858>`__)
* Contributors: Auguste Lalande, Christian Ruf, Christophe Bedard, Emerson Knapp, Matthijs van der Burgh, Michael Carlstrom, Sebastian Javier D'Alessandro Szymanowski, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch_yaml <https://github.com/ros2/launch/tree/lyrical/launch_yaml/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Correct typos (`#961 <https://github.com/ros2/launch/issues/961>`__)
* Support frontends for PathJoinSubstitution (`#943 <https://github.com/ros2/launch/issues/943>`__)
* Capture the environment variables in TimerAction (`#728 <https://github.com/ros2/launch/issues/728>`__)
* Expose StringJoinSubstitution to frontend (`#857 <https://github.com/ros2/launch//issues/857>`__)
* Fix all/any in xml and yaml launch files (`#906 <https://github.com/ros2/launch/issues/906>`__)
* Allow providing launch args to include using let in frontends (`#848 <https://github.com/ros2/launch//issues/848>`__)
* Fix Setuptoolsdeprecations (`#898 <https://github.com/ros2/launch/issues/898>`__)
* Make sure to install py.typed files (`#886 <https://github.com/ros2/launch/issues/886>`__)
* Add remaining ``py.typed`` (`#884 <https://github.com/ros2/launch/issues/884>`__)
* Fix ``log\_*`` warnings (`#883 <https://github.com/ros2/launch/issues/883>`__)
* Other Logging Implementations with ``getLevelNamesMapping`` fix (`#866 <https://github.com/ros2/launch/issues/866>`__)
* Revert "Add Other Logging Implementations (`#858 <https://github.com/ros2/launch/issues/858>`__)" (`#865 <https://github.com/ros2/launch/issues/865>`__) This reverts commit b7b31c45b0eb350deedd282b88398d1ca0d5faf4.
* Add Other Logging Implementations (`#858 <https://github.com/ros2/launch/issues/858>`__)
* Contributors: Auguste Lalande, Christian Ruf, Christophe Bedard, Matthijs van der Burgh, Michael Carlstrom, Sebastian Javier D'Alessandro Szymanowski, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`libstatistics_collector <https://github.com/ros-tooling/libstatistics_collector/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new aggregate rosidl target instead of _TARGETS (`#222 <https://github.com/ros-tooling/libstatistics_collector/issues/222>`__)
* fix cmake deprecation (`#214 <https://github.com/ros-tooling/libstatistics_collector/issues/214>`__)
* Bump ros-tooling/action-ros-ci from 0.3 to 0.4
* Bump codecov/codecov-action from 5.3.1 to 5.4.0
* Bump codecov/codecov-action from 5.1.2 to 5.3.1
* Bump codecov/codecov-action from 5.0.7 to 5.1.2
* Bump codecov/codecov-action from 4.6.0 to 5.0.7
* Contributors: Alexis Tsogias, dependabot[bot], mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`libyaml_vendor <https://github.com/ros2/libyaml_vendor/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Replace ament_vendor with cmake module (`#67 <https://github.com/ros2/libyaml_vendor/issues/67>`__)
* Remove CODEOWNERS and mirror-rolling-to-master workflow. (`#65 <https://github.com/ros2/libyaml_vendor/issues/65>`__)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`lifecycle <https://github.com/ros2/demos/tree/lyrical/lifecycle/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#781 <https://github.com/ros2/demos//issues/781>`__)
* Switching to example_interfaces (`#674 <https://github.com/ros2/demos/issues/674>`__)
* r-simonelli/demos-lifecycle (`#750 <https://github.com/ros2/demos/issues/750>`__)
* Uniform CMAKE min VERSION (`#714 <https://github.com/ros2/demos/issues/714>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#707 <https://github.com/ros2/demos/issues/707>`__)
* Contributors: Emerson Knapp, Lucas Wendland, Shane Loretz, mosfet80, r-simonelli


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`lifecycle_msgs <https://github.com/ros2/rcl_interfaces/tree/lyrical/lifecycle_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use builtin_interfaces/Time for TransitionEvent stamp (`#185 <https://github.com/ros2/rcl_interfaces/issues/185>`__)
* Fix cmake deprecation (`#180 <https://github.com/ros2/rcl_interfaces/issues/180>`__)
* Contributors: Jasper van Brakel, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`lifecycle_py <https://github.com/ros2/demos/tree/lyrical/lifecycle_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add ``ament_mypy`` support and type hints to ``lifecycle_py`` (`#778 <https://github.com/ros2/demos/issues/778>`__)
* Revert lifecycle_py accidental merge - ament_mypy (`#777 <https://github.com/ros2/demos//issues/777>`__)
* action_tutorials_py: add ament_mypy support (`#775 <https://github.com/ros2/demos//issues/775>`__)
* Switching to example_interfaces (`#674 <https://github.com/ros2/demos/issues/674>`__)
* fix setuptools deprecations (`#733 <https://github.com/ros2/demos/issues/733>`__)
* Contributors: Lucas Wendland, Mohit Kumaresan, mohit, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`logging_demo <https://github.com/ros2/demos/tree/lyrical/logging_demo/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#781 <https://github.com/ros2/demos//issues/781>`__)
* Switching to example_interfaces (`#674 <https://github.com/ros2/demos/issues/674>`__)
* Use EnableRmwIsolation in launch tests (`#724 <https://github.com/ros2/demos/issues/724>`__)
* Uniform CMAKE min VERSION (`#714 <https://github.com/ros2/demos/issues/714>`__)
* Set envars to run tests with rmw_zenoh_cpp with multicast discovery (`#711 <https://github.com/ros2/demos/issues/711>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#707 <https://github.com/ros2/demos/issues/707>`__)
* Contributors: Alejandro Hernández Cordero, Emerson Knapp, Lucas Wendland, Scott K Logan, Shane Loretz, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`lttngpy <https://github.com/ros2/ros2_tracing/tree/lyrical/lttngpy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use <lttng/lttng.h> in lttngpy and clean up includes (`#222 <https://github.com/ros2/ros2_tracing/issues/222>`__)
* Allow creating snapshot sessions (`#195 <https://github.com/ros2/ros2_tracing/issues/195>`__)
* [Fix] compile fail (`#194 <https://github.com/ros2/ros2_tracing/issues/194>`__)
* Use pybind11 from deb or pixi (`#197 <https://github.com/ros2/ros2_tracing/issues/197>`__)
* Add support for starting tracing at runtime (`#191 <https://github.com/ros2/ros2_tracing/issues/191>`__)
* Contributors: Alejandro Hernández Cordero, RHolland, Shravan Deva, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`map_msgs <https://github.com/ros-planning/navigation_msgs/tree/lyrical/map_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Change email address associated with maintainer
* fix cmake deprecation
* Contributors: David V. Lu, Steve Macenski, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`mcap_vendor <https://github.com/ros2/rosbag2/tree/lyrical/mcap_vendor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update mcap dependency to version 2.1.3 (`#2355 <https://github.com/ros2/rosbag2/issues/2355>`__)
* Remove lz4 vendor package (`#2165 <https://github.com/ros2/rosbag2/issues/2165>`__)
* Replace ``zstd_vendor`` with ``zstd_cmake_module`` (`#2166 <https://github.com/ros2/rosbag2/issues/2166>`__)
* Fix CMAKE deprecation (`#2067 <https://github.com/ros2/rosbag2/issues/2067>`__)
* Backport missing ``cstdint`` include (`#2008 <https://github.com/ros2/rosbag2/issues/2008>`__)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, David Anthony, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`message_filters <https://github.com/ros2/message_filters/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Avoid vector assignment in message_filters signal callback (`#292 <https://github.com/ros2/message_filters/issues/292>`__) (`#293 <https://github.com/ros2/message_filters/issues/293>`__)
* Cleanup headers and removed deadcode (`#284 <https://github.com/ros2/message_filters/issues/284>`__) (`#291 <https://github.com/ros2/message_filters/issues/291>`__)
* feat(python): add python implementation of InputAligner  (backport `#283 <https://github.com/ros2/message_filters/issues/283>`__) (`#286 <https://github.com/ros2/message_filters/issues/286>`__)
* C++20 style (`#272 <https://github.com/ros2/message_filters/issues/272>`__)
* (`#221 <https://github.com/ros2/message_filters/issues/221>`__) Tutorials: Add DeltaFilter Python tutorial (`#277 <https://github.com/ros2/message_filters/issues/277>`__)
* DeltaFilter(C++): Add DeltaFilter class. Add tests (`#273 <https://github.com/ros2/message_filters/issues/273>`__) (`#273 <https://github.com/ros2/message_filters/issues/273>`__)
* Removed dead code
* Improvements and more test coverage
* Use new ROSIDL aggregate CMake target
* Tutorials minor fixers: Replace the TODOs with the actual links to other tutorials as required. Rename Approximate-Tyme tutorial to Approximate-Time (`#266 <https://github.com/ros2/message_filters/issues/266>`__)
* Tutorials: Add LatestTime synchronization policy tutorial (`#266 <https://github.com/ros2/message_filters/issues/266>`__)
* Tutorials: Approximate-Synchronizer: Label CMake code blocks with the right language markings
* Tutorials: Add C++ tutorial for Approximate Epsilon Time Sync policy
* DeltaFilter(Python): Add DeltaFilter for Python. Add tests. Add docstring to filters and comparison handlers (`#252 <https://github.com/ros2/message_filters/issues/252>`__)
* remove setup.py (`#257 <https://github.com/ros2/message_filters/issues/257>`__)
* (`#246 <https://github.com/ros2/message_filters/issues/246>`__, `#186 <https://github.com/ros2/message_filters/issues/186>`__) Subscriber(Python): Add callback_group, event_callbacks, qos_overriding_options, raw and content_filter_options arguments to __init_\_. (`#251 <https://github.com/ros2/message_filters/issues/251>`__)
* Add kwargs passing from Subscriber to node.create_subscription (`#247 <https://github.com/ros2/message_filters/issues/247>`__) Fixes callers that use callback_group
* Get topic name from base class to propagate remaps (`#68 <https://github.com/ros2/message_filters/issues/68>`__)
* `#130 <https://github.com/ros2/message_filters/issues/130>`__ add simple filter tutorial for cpp (`#239 <https://github.com/ros2/message_filters/issues/239>`__)
* `#200 <https://github.com/ros2/message_filters/issues/200>`__ fix inconsistensy between cpp and python exact time synchronizer impl (`#238 <https://github.com/ros2/message_filters/issues/238>`__)
* Add simple filter tutorials (`#226 <https://github.com/ros2/message_filters/issues/226>`__)
* Update subscription callback signatures (`#222 <https://github.com/ros2/message_filters/issues/222>`__)
* Add chain tutorial python (`#219 <https://github.com/ros2/message_filters/issues/219>`__)
* Change function signature for Python Subscriber class (`#220 <https://github.com/ros2/message_filters/issues/220>`__)
* Add Python implementation for a Chain filter (`#213 <https://github.com/ros2/message_filters/issues/213>`__)
* Fix comparison of different time sources in C++ TimeSequencer (`#202 <https://github.com/ros2/message_filters/issues/202>`__)
* Some fixes to documentation (`#208 <https://github.com/ros2/message_filters/issues/208>`__)
* Create a Chain class tutorial for C++ (`#203 <https://github.com/ros2/message_filters/issues/203>`__)
* get rid of deprecated rclcpp::spin_some(). (`#201 <https://github.com/ros2/message_filters/issues/201>`__)
* Add 'Cache (C++)' tutorial (`#196 <https://github.com/ros2/message_filters/issues/196>`__)
* cache.hpp: Add allow_headerless (`#195 <https://github.com/ros2/message_filters/issues/195>`__)
* Simplify method call (`#194 <https://github.com/ros2/message_filters/issues/194>`__)
* Fix cache tutorial: added tab extension (`#190 <https://github.com/ros2/message_filters/issues/190>`__)
* Add tutorial for Cache filter for Python (`#185 <https://github.com/ros2/message_filters/issues/185>`__)
* fix cmake deprecation (`#182 <https://github.com/ros2/message_filters/issues/182>`__)
* update documentation (`#180 <https://github.com/ros2/message_filters/issues/180>`__)
* Removed missing pragma (`#179 <https://github.com/ros2/message_filters/issues/179>`__)
* Removed Subscriber deprecation (`#177 <https://github.com/ros2/message_filters/issues/177>`__)
* Removed deprecated headers (`#176 <https://github.com/ros2/message_filters/issues/176>`__)
* Use warning instead of warn (`#178 <https://github.com/ros2/message_filters/issues/178>`__)
* Docs - Remove C++ implementation limit of 9 channels (`#174 <https://github.com/ros2/message_filters/issues/174>`__)
* Contributors: Alejandro Hernandez Cordero, Alejandro Hernández Cordero, Alex Spitzer, Emerson Knapp, Erwin L., EsipovPA, Johannes Böhm, Michael Carlstrom, Patrick Roncagliolo, Pavel Esipov, Samuel Foo Enze, Tomoya Fujita, mergify[bot], mini-1235, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`mimick_vendor <https://github.com/ros2/mimick_vendor/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Remove CODEOWNERS and mirror-rolling-to-master workflow. (`#40 <https://github.com/ros2/mimick_vendor/issues/40>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`nav_msgs <https://github.com/ros2/common_interfaces/tree/lyrical/nav_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Adding the Trajectory and trajectoryPoint messages (`#296 <https://github.com/ros2/common_interfaces/issues/296>`__)
* Fix CMAKE deprecation (`#288 <https://github.com/ros2/common_interfaces/issues/288>`__)
* Contributors: Steve Macenski, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`osrf_testing_tools_cpp <https://github.com/osrf/osrf_testing_tools_cpp/tree/lyrical/osrf_testing_tools_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix cmake min version (`#96 <https://github.com/osrf/osrf_testing_tools_cpp/issues/96>`__)
* fix cmake deprecation  (`#94 <https://github.com/osrf/osrf_testing_tools_cpp/issues/94>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`pendulum_control <https://github.com/ros2/demos/tree/lyrical/pendulum_control/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#781 <https://github.com/ros2/demos//issues/781>`__)
* Update subscription callback signatures (`#754 <https://github.com/ros2/demos/issues/754>`__)
* get rid of deprecated rclcpp::spin_some(). (`#734 <https://github.com/ros2/demos/issues/734>`__)
* Use EnableRmwIsolation in launch tests (`#724 <https://github.com/ros2/demos/issues/724>`__)
* Uniform CMAKE min VERSION (`#714 <https://github.com/ros2/demos/issues/714>`__)
* Set envars to run tests with rmw_zenoh_cpp with multicast discovery (`#711 <https://github.com/ros2/demos/issues/711>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#707 <https://github.com/ros2/demos/issues/707>`__)
* Contributors: Alejandro Hernández Cordero, Emerson Knapp, Scott K Logan, Shane Loretz, Tomoya Fujita, mini-1235, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`pendulum_msgs <https://github.com/ros2/demos/tree/lyrical/pendulum_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Uniform CMAKE min VERSION (`#714 <https://github.com/ros2/demos/issues/714>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`performance_test_fixture <https://github.com/ros2/performance_test_fixture/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix cmake deprecation (`#31 <https://github.com/ros2/performance_test_fixture/issues/31>`__)
* Remove CODEOWNERS and mirror-rolling-to-main workflow. (`#28 <https://github.com/ros2/performance_test_fixture/issues/28>`__)
* Contributors: Chris Lalancette, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`pluginlib <https://github.com/ros/pluginlib/tree/lyrical/pluginlib/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix some minor issues (`#292 <https://github.com/ros/pluginlib/issues/292>`__)
* Add support for passing arguments to constructors (`#291 <https://github.com/ros/pluginlib/issues/291>`__)
* Export includes (`#290 <https://github.com/ros/pluginlib/issues/290>`__)
* Updated deprecated ament_index_cpp API (`#289 <https://github.com/ros/pluginlib/issues/289>`__)
* refactor: replace regex with find_last_of to split plugin name (`#271 <https://github.com/ros/pluginlib/issues/271>`__)
* Removed tinyxml2_vendor dependency (`#274 <https://github.com/ros/pluginlib/issues/274>`__)
* Add ros2plugin (`#165 <https://github.com/ros/pluginlib/issues/165>`__)
* Contributors: Alejandro Hernández Cordero, Jeremie Deray, ipa-fez, pum1k


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`point_cloud_transport <https://github.com/ros-perception/point_cloud_transport/tree/lyrical/point_cloud_transport/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix exit crash on aarch64 by using leaky singleton for global loader (`#157 <https://github.com/ros-perception/point_cloud_transport/issues/157>`__)
* Include message type (`#152 <https://github.com/ros-perception/point_cloud_transport/issues/152>`__)
* Use new aggregate rosidl target instead of _TARGETS (`#153 <https://github.com/ros-perception/point_cloud_transport/issues/153>`__) Co-authored-by: Alexis Tsogias <a.tsogias@cellumation.com>
* Improvements (`#150 <https://github.com/ros-perception/point_cloud_transport/issues/150>`__)
* Expose original ROS Publishers and Subscription (`#146 <https://github.com/ros-perception/point_cloud_transport/issues/146>`__) (`#148 <https://github.com/ros-perception/point_cloud_transport/issues/148>`__)
* Fix duplicate component registration for Republisher (`#142 <https://github.com/ros-perception/point_cloud_transport/issues/142>`__)
* Removed outdated comment (`#138 <https://github.com/ros-perception/point_cloud_transport/issues/138>`__)
* Use standard unsigned int in place of uint for Windows compatibility (`#134 <https://github.com/ros-perception/point_cloud_transport/issues/134>`__)
* Update subscriber filter (`#126 <https://github.com/ros-perception/point_cloud_transport/issues/126>`__)
* Simplify NodeInterface API mehotd call (`#129 <https://github.com/ros-perception/point_cloud_transport/issues/129>`__)
* Fixed QOS override tests (`#128 <https://github.com/ros-perception/point_cloud_transport/issues/128>`__)
* Deprecated rmw_qos_profile_t (`#125 <https://github.com/ros-perception/point_cloud_transport/issues/125>`__)
* Feat/Add LifecycleNode Support (`#109 <https://github.com/ros-perception/point_cloud_transport/issues/109>`__)
* Add ``rclcpp::shutdown`` (`#110 <https://github.com/ros-perception/point_cloud_transport/issues/110>`__)
* Contributors: Alejandro Hernández Cordero, Alexis Tsogias, ElSayed ElSheikh, Michael Carroll, Silvio Traversaro, Yuyuan Yuan, mergify[bot], mini-1235


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`point_cloud_transport_py <https://github.com/ros-perception/point_cloud_transport/tree/lyrical/point_cloud_transport_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new aggregate rosidl target instead of _TARGETS (`#153 <https://github.com/ros-perception/point_cloud_transport/issues/153>`__)
* Python improvements (`#151 <https://github.com/ros-perception/point_cloud_transport/issues/151>`__)
* Use pybind11 from deb or pixi (`#131 <https://github.com/ros-perception/point_cloud_transport/issues/131>`__)
* Simplify NodeInterface API mehotd call (`#129 <https://github.com/ros-perception/point_cloud_transport/issues/129>`__)
* Feat/Add LifecycleNode Support (`#109 <https://github.com/ros-perception/point_cloud_transport/issues/109>`__)
* Contributors: Alejandro Hernández Cordero, Alexis Tsogias, ElSayed ElSheikh


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`python_qt_binding <https://github.com/ros-visualization/python_qt_binding/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Pick Qt version at build time, not install time (`#161 <https://github.com/ros-visualization/python_qt_binding/issues/161>`__)
* Re-add exec depend on python3 qt bindings rosdep key (`#160 <https://github.com/ros-visualization/python_qt_binding/issues/160>`__)
* Remove qt6-base-dev from package.xml (`#159 <https://github.com/ros-visualization/python_qt_binding/issues/159>`__)
* Depend on python3-dev (`#158 <https://github.com/ros-visualization/python_qt_binding/issues/158>`__)
* Use sip-build and python3_add_library for Qt5/Qt6 (`#157 <https://github.com/ros-visualization/python_qt_binding/issues/157>`__)
* fix setuptools deprecation (`#151 <https://github.com/ros-visualization/python_qt_binding/issues/151>`__)
* fix cmake deprecation (`#150 <https://github.com/ros-visualization/python_qt_binding/issues/150>`__)
* Remove the mirror-rolling-to-main workflow (`#145 <https://github.com/ros-visualization/python_qt_binding/issues/145>`__)
* Remove CODEOWNERS (`#144 <https://github.com/ros-visualization/python_qt_binding/issues/144>`__)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Shane Loretz, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`qt_dotgraph <https://github.com/ros-visualization/qt_gui_core/tree/lyrical/qt_dotgraph/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* More qt6 fixes (`#334 <https://github.com/ros-visualization/qt_gui_core/issues/334>`__) (`#335 <https://github.com/ros-visualization/qt_gui_core/issues/335>`__) (cherry picked from commit 62f29544c4061006f9c09c3dfa4bf2895e8126e0) Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Support qt6 (`#293 <https://github.com/ros-visualization/qt_gui_core/issues/293>`__)
* Ignore case when asserting snippet presence in tests (`#314 <https://github.com/ros-visualization/qt_gui_core/issues/314>`__)
* Fix setupTools deprecations (`#308 <https://github.com/ros-visualization/qt_gui_core/issues/308>`__)
* Contributors: Alejandro Hernández Cordero, Scott K Logan, mergify[bot], mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`qt_gui <https://github.com/ros-visualization/qt_gui_core/tree/lyrical/qt_gui/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* More qt6 fixes (`#334 <https://github.com/ros-visualization/qt_gui_core/issues/334>`__) (`#335 <https://github.com/ros-visualization/qt_gui_core/issues/335>`__) (cherry picked from commit 62f29544c4061006f9c09c3dfa4bf2895e8126e0) Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Support qt6 (`#293 <https://github.com/ros-visualization/qt_gui_core/issues/293>`__)
* fix(qt_gui): __builtin_\_ -> builtins (`#315 <https://github.com/ros-visualization/qt_gui_core/issues/315>`__)
* Fix cmake deprecations (`#307 <https://github.com/ros-visualization/qt_gui_core/issues/307>`__)
* Contributors: Alejandro Hernández Cordero, Matthijs van der Burgh, mergify[bot], mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`qt_gui_app <https://github.com/ros-visualization/qt_gui_core/tree/lyrical/qt_gui_app/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix cmake deprecations (`#307 <https://github.com/ros-visualization/qt_gui_core/issues/307>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`qt_gui_core <https://github.com/ros-visualization/qt_gui_core/tree/lyrical/qt_gui_core/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update qt_gui_core to package.xml version 2. (`#319 <https://github.com/ros-visualization/qt_gui_core/issues/319>`__)
* Fix cmake deprecations (`#307 <https://github.com/ros-visualization/qt_gui_core/issues/307>`__)
* Contributors: Chris Lalancette, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`qt_gui_cpp <https://github.com/ros-visualization/qt_gui_core/tree/lyrical/qt_gui_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* More qt6 fixes (`#334 <https://github.com/ros-visualization/qt_gui_core/issues/334>`__) (`#335 <https://github.com/ros-visualization/qt_gui_core/issues/335>`__) (cherry picked from commit 62f29544c4061006f9c09c3dfa4bf2895e8126e0) Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* find_package(Qt...) in downstream packages (`#332 <https://github.com/ros-visualization/qt_gui_core/issues/332>`__)
* Export qt dependencies in package.xml (`#331 <https://github.com/ros-visualization/qt_gui_core/issues/331>`__)
* Use qt-base-dev / libqtwidgets (`#330 <https://github.com/ros-visualization/qt_gui_core/issues/330>`__)
* Support qt6 (`#293 <https://github.com/ros-visualization/qt_gui_core/issues/293>`__)
* Use new aggregate rosidl target instead of _TARGETS (`#325 <https://github.com/ros-visualization/qt_gui_core/issues/325>`__)
* remove unsued setup.py (`#323 <https://github.com/ros-visualization/qt_gui_core/issues/323>`__)
* Removed tinyxml2_vendor dependency (`#309 <https://github.com/ros-visualization/qt_gui_core/issues/309>`__)
* Fix cmake deprecations (`#307 <https://github.com/ros-visualization/qt_gui_core/issues/307>`__)
* Removed deprecated headers (`#305 <https://github.com/ros-visualization/qt_gui_core/issues/305>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#302 <https://github.com/ros-visualization/qt_gui_core/issues/302>`__)
* Contributors: Alejandro Hernández Cordero, Alexis Tsogias, Michael Carlstrom, Shane Loretz, mergify[bot], mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`qt_gui_py_common <https://github.com/ros-visualization/qt_gui_core/tree/lyrical/qt_gui_py_common/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* More qt6 fixes (`#334 <https://github.com/ros-visualization/qt_gui_core/issues/334>`__) (`#335 <https://github.com/ros-visualization/qt_gui_core/issues/335>`__)
* Support qt6 (`#293 <https://github.com/ros-visualization/qt_gui_core/issues/293>`__)
* remove unsued setup.py (`#323 <https://github.com/ros-visualization/qt_gui_core/issues/323>`__)
* Fix cmake deprecations (`#307 <https://github.com/ros-visualization/qt_gui_core/issues/307>`__)
* Contributors: Alejandro Hernández Cordero, Michael Carlstrom, mergify[bot], mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`quality_of_service_demo_cpp <https://github.com/ros2/demos/tree/lyrical/quality_of_service_demo/rclcpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#781 <https://github.com/ros2/demos//issues/781>`__)
* Switching to example_interfaces (`#674 <https://github.com/ros2/demos/issues/674>`__)
* Uniform CMAKE min VERSION (`#714 <https://github.com/ros2/demos/issues/714>`__) demo_nodes_cpp/CMakeLists.txt require cmake min version 3.12 other modules cmake 3.5. It is proposed to standardize with version 3.12. This also fixes cmake <3.10 deprecation warnings
* Contributors: Emerson Knapp, Lucas Wendland, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`quality_of_service_demo_py <https://github.com/ros2/demos/tree/lyrical/quality_of_service_demo/rclpy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Switching to example_interfaces (`#674 <https://github.com/ros2/demos/issues/674>`__)
* fix setuptools deprecations (`#731 <https://github.com/ros2/demos/issues/731>`__)
* Contributors: Lucas Wendland, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl <https://github.com/ros2/rcl/tree/lyrical/rcl/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* feat: Added check for double usage of entities in rcl_waitset (`#1206 <https://github.com/ros2/rcl/issues/1206>`__)
* Preserve ``rmw_create_node`` error state in ``rcl_node_init`` by using ``RCL_EXPECT_ERROR_IS_SET`` (`#1313 <https://github.com/ros2/rcl/issues/1313>`__)
* Remove clang warnings (`#1315 <https://github.com/ros2/rcl/issues/1315>`__)
* Add RCL_EXPECT_ERROR_IS_SET macro (`#1312 <https://github.com/ros2/rcl/issues/1312>`__)
* Improved documentation of rcl_XYZ_set_on_new_XYZ_callback (`#1289 <https://github.com/ros2/rcl/issues/1289>`__)
* Add rcl_subscription_options_set_acceptable_buffer_backends with proper lifetime management (`#1308 <https://github.com/ros2/rcl/issues/1308>`__)
* Added tracepoint to rcl_take_loaned_message (`#1300 <https://github.com/ros2/rcl/issues/1300>`__)
* Apply change from "Use new aggregate rosidl target instead of _TARGETS (`#1302 <https://github.com/ros2/rcl/issues/1302>`__)" on some leftovers (`#1309 <https://github.com/ros2/rcl/issues/1309>`__)
* Remove the check for content filter support at the RCL layer (`#1304 <https://github.com/ros2/rcl/issues/1304>`__)
* Use new aggregate rosidl target instead of _TARGETS (`#1302 <https://github.com/ros2/rcl/issues/1302>`__)
* Add API for client libraries to set action server goal expiration callbacks (`#1295 <https://github.com/ros2/rcl/issues/1295>`__)
* Fujitatomoya/improve rcl test graph (`#1296 <https://github.com/ros2/rcl/issues/1296>`__)
* Add content filtering support check for subscriptions (`#1293 <https://github.com/ros2/rcl/issues/1293>`__)
* rcl_logging_implementation package support. (`#1276 <https://github.com/ros2/rcl/issues/1276>`__)
* Remove default from switch with enum, so that compiler warns. (`#1278 <https://github.com/ros2/rcl/issues/1278>`__)
* Add clients servers info (`#1161 <https://github.com/ros2/rcl/issues/1161>`__)
* Fix REP url locations (`#1271 <https://github.com/ros2/rcl/issues/1271>`__)
* rcl_logging_allocator_initialize() support. (`#1049 <https://github.com/ros2/rcl/issues/1049>`__)
* Fix typos: occurrs->occurs, successfull->successful (`#1259 <https://github.com/ros2/rcl/issues/1259>`__)
* Refer to 'the middleware' and not 'the DDS implementation' in doc (`#1260 <https://github.com/ros2/rcl/issues/1260>`__)
* Switch to isolated testing via rmw_test_fixture (`#1251 <https://github.com/ros2/rcl/issues/1251>`__)
* Fix Cmake deprecation (`#1249 <https://github.com/ros2/rcl/issues/1249>`__)
* Assert HistoryQoS in test_info_by_topic (`#1242 <https://github.com/ros2/rcl//issues/1242>`__)
* Add a test for the subscription option 'ignore_local_publications' (`#1239 <https://github.com/ros2/rcl//issues/1239>`__)
* remove unnecessary test_with_localhost_only. (`#1238 <https://github.com/ros2/rcl/issues/1238>`__)
* Address memory leaks in rcl test_timer_init_state (`#1236 <https://github.com/ros2/rcl/issues/1236>`__)
* Removed unused nondefault_qos_profile (`#1233 <https://github.com/ros2/rcl/issues/1233>`__)
* Removed unused functions (`#1230 <https://github.com/ros2/rcl/issues/1230>`__)
* remove rcl_qos_profile_rosout_default. (`#1225 <https://github.com/ros2/rcl/issues/1225>`__)
* remove rmw_connext from test. (`#1226 <https://github.com/ros2/rcl/issues/1226>`__)
* Fix a dangling pointer discovered by a fresh Clang (`#1222 <https://github.com/ros2/rcl/issues/1222>`__)
* Contributors: Akihiko Komada, Alejandro Hernández Cordero, Alexander Kornienko, Alexis Tsogias, Barry Xu, CY Chen, Christophe Bedard, Emerson Knapp, Janosch Machowinski, Lee, Mario Domínguez López, Michael Orlov, Minju, Oren Bell PhD, Rushhaank Sahay, Sai Kishor Kothakota, Shane Loretz, Skyler Medeiros, Tim Clephas, Tomoya Fujita, mosfet80, yadunund


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl_action <https://github.com/ros2/rcl/tree/lyrical/rcl_action/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix(rcl_action): use RMW isolation for cross-node tests (`#1311 <https://github.com/ros2/rcl/issues/1311>`__)
* Add 2 interfaces for configuring action client feedback subscription contents filter (`#1287 <https://github.com/ros2/rcl/issues/1287>`__)
* Apply change from "Use new aggregate rosidl target instead of _TARGETS (`#1302 <https://github.com/ros2/rcl/issues/1302>`__)" on some leftovers (`#1309 <https://github.com/ros2/rcl/issues/1309>`__)
* simplify error logging for timer cancellation (`#1307 <https://github.com/ros2/rcl/issues/1307>`__)
* fix: Prevent short time endless loop in expire_timer (`#1303 <https://github.com/ros2/rcl/issues/1303>`__)
* Add API for client libraries to set action server goal expiration callbacks (`#1295 <https://github.com/ros2/rcl/issues/1295>`__)
* support rcl_action_count_clients and rcl_action_count_servers. (`#1294 <https://github.com/ros2/rcl/issues/1294>`__)
* Fix REP url locations (`#1271 <https://github.com/ros2/rcl/issues/1271>`__)
* add rcl_action_goal_handle_is_abortable(). (`#1257 <https://github.com/ros2/rcl/issues/1257>`__)
* Fix Cmake deprecation (`#1249 <https://github.com/ros2/rcl/issues/1249>`__)
* Contributors: Alexis Tsogias, Barry Xu, Janosch Machowinski, Skyler Medeiros, Tim Clephas, Tomoya Fujita, William Woodall, Yuyuan Yuan, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl_interfaces <https://github.com/ros2/rcl_interfaces/tree/lyrical/rcl_interfaces/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix cmake deprecation (`#180 <https://github.com/ros2/rcl_interfaces/issues/180>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl_lifecycle <https://github.com/ros2/rcl/tree/lyrical/rcl_lifecycle/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Apply change from "Use new aggregate rosidl target instead of _TARGETS (`#1302 <https://github.com/ros2/rcl/issues/1302>`__)" on some leftovers (`#1309 <https://github.com/ros2/rcl/issues/1309>`__)
* Populate Transitions in Transition Events (continuation) (`#1269 <https://github.com/ros2/rcl/issues/1269>`__)
* Fix REP url locations (`#1271 <https://github.com/ros2/rcl/issues/1271>`__)
* Fix Cmake deprecation (`#1249 <https://github.com/ros2/rcl/issues/1249>`__)
* introduce rcl_lifecycle_get_transition_label_by_id(). (`#1229 <https://github.com/ros2/rcl/issues/1229>`__)
* Contributors: Alexis Tsogias, Jasper van Brakel, Tim Clephas, Tomoya Fujita, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl_logging_implementation <https://github.com/ros2/rcl_logging/tree/lyrical/rcl_logging_implementation/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* update rcl_logging_implementation architecture diagram. (`#137 <https://github.com/ros2/rcl_logging/issues/137>`__)
* rcl logging implementation (`#135 <https://github.com/ros2/rcl_logging/issues/135>`__) * 1st draft bring-up for rcl_logging_implementation package. * add test_logging_implementation to check dynamic loading. * address Copilot review comments. * fix: correct visibility macro for DLL export in CMakeLists.txt * add visibility control with RCL_LOGGING_IMPLEMENTATION_DEFAULT_VISIBILITY. * load the all symbols at the initialization. * Use goto pattern to eliminate the cleanup duplication. * Add basic design doc of rmw_logging_implementation. * use RCPPUTILS_SCOPE_EXIT instead of goto statement. * logging visibility macro was incorrect. * logging symbols stay until the peocess actually exits. --------- Co-authored-by: Barry Xu <barry.xu@sony.com>
* Contributors: Tomoya Fujita


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl_logging_interface <https://github.com/ros2/rcl_logging/tree/lyrical/rcl_logging_interface/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix cmake deprecation (`#133 <https://github.com/ros2/rcl_logging/issues/133>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl_logging_noop <https://github.com/ros2/rcl_logging/tree/lyrical/rcl_logging_noop/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix cmake deprecation (`#133 <https://github.com/ros2/rcl_logging/issues/133>`__)
* Cleanup rcl_logging_noop dependencies. (`#132 <https://github.com/ros2/rcl_logging/issues/132>`__) It shouldn't build_export_depend anything (as nothing downstream should link against it), and all of its dependencies can be private.
* Contributors: Chris Lalancette, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl_logging_spdlog <https://github.com/ros2/rcl_logging/tree/lyrical/rcl_logging_spdlog/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* feat: add env variable to configure flushing interval (`#139 <https://github.com/ros2/rcl_logging/issues/139>`__)
* Fix cmake deprecation (`#133 <https://github.com/ros2/rcl_logging/issues/133>`__)
* Cleanup overwritten warning messages on error. (`#128 <https://github.com/ros2/rcl_logging/issues/128>`__)
* Contributors: Achille Verheye, Chris Lalancette, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl_yaml_param_parser <https://github.com/ros2/rcl/tree/lyrical/rcl_yaml_param_parser/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Remove clang warnings (`#1315 <https://github.com/ros2/rcl/issues/1315>`__)
* fix (`#1310 <https://github.com/ros2/rcl/issues/1310>`__)
* Use the POSIX locale to parse YAML double (`#1292 <https://github.com/ros2/rcl/issues/1292>`__)
* rcl_yaml_node_struct_print print loop interation fix. (`#1290 <https://github.com/ros2/rcl//issues/1290>`__)
* rcl_yaml_param_parser: add support for binary tag to load byte arrays parameters (`#1256 <https://github.com/ros2/rcl//issues/1256>`__)
* Validate name input in add_name_to_ns function (`#1281 <https://github.com/ros2/rcl/issues/1281>`__)
* parse_key() should use yaml_map_lvl_t instead of uint_32. (`#1279 <https://github.com/ros2/rcl/issues/1279>`__)
* Remove default from switch with enum, so that compiler warns. (`#1278 <https://github.com/ros2/rcl/issues/1278>`__)
* Add yaml tags support (`#1275 <https://github.com/ros2/rcl/issues/1275>`__) Co-authored-by: Lei Liu <Lei.Liu.AP@sony.com>
* Fix REP url locations (`#1271 <https://github.com/ros2/rcl/issues/1271>`__)
* Fix param file parsing failure with wildcards due to ordering (`#1253 <https://github.com/ros2/rcl/issues/1253>`__)
* Fix Cmake deprecation (`#1249 <https://github.com/ros2/rcl/issues/1249>`__)
* Contributors: Alejandro Hernández Cordero, Barry Xu, Hugal31, Michael Carlstrom, Romain Reignier, Tim Clephas, Tomoya Fujita, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rclcpp <https://github.com/ros2/rclcpp/tree/lyrical/rclcpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Include EventsCBGExecutor (`#3137 <https://github.com/ros2/rclcpp/issues/3137>`__)
* Fix topic statistics for IPC subscriptions (`#3130 <https://github.com/ros2/rclcpp/issues/3130>`__)
* fix: Fixed MSVC compile errors (`#3135 <https://github.com/ros2/rclcpp/issues/3135>`__)
* feat: Added callback group events executor (`#3097 <https://github.com/ros2/rclcpp/issues/3097>`__)
* Fix wrong dependency (`#3133 <https://github.com/ros2/rclcpp/issues/3133>`__)
* feat: Switch to c++20 and remove resulting compile warnings (`#3124 <https://github.com/ros2/rclcpp/issues/3124>`__)
* fix: Compile fix for MSVC 2022 (`#3131 <https://github.com/ros2/rclcpp/issues/3131>`__)
* Remove warnings on tests (`#3125 <https://github.com/ros2/rclcpp/issues/3125>`__)
* feat: Add per-node log level support via NodeOptions (`#3092 <https://github.com/ros2/rclcpp/issues/3092>`__)
* Improve error message when parameter value is missing (`#3093 <https://github.com/ros2/rclcpp/issues/3093>`__)
* Fix incorrect internal clear inside ``RingBufferImplementation`` (`#3116 <https://github.com/ros2/rclcpp/issues/3116>`__)
* Add acceptable_buffer_backends field in SubscriptionOptionsBase (`#3098 <https://github.com/ros2/rclcpp/issues/3098>`__)
* Remove comment about removed StaticSingleThreadedExecutor (`#3121 <https://github.com/ros2/rclcpp/issues/3121>`__)
* Added tracepoint (`#3103 <https://github.com/ros2/rclcpp/issues/3103>`__)
* Add ConstRefCallback in take_shared_method (`#3066 <https://github.com/ros2/rclcpp/issues/3066>`__)
* Replace mispelled "${rcl_interfaces_TARGES}" by rcl_interfaces::rcl_interfaces (`#3112 <https://github.com/ros2/rclcpp/issues/3112>`__)
* Use new ROSIDL aggregate CMake target (`#3105 <https://github.com/ros2/rclcpp/issues/3105>`__)
* remove duplicate test cases in TestAnySubscriptionCallback::is_serialized_message_callback (`#3104 <https://github.com/ros2/rclcpp/issues/3104>`__)
* keep the event alive throught the assertion, preveiting the race. (`#3099 <https://github.com/ros2/rclcpp/issues/3099>`__)
* Add support check for content filter feature in subscription (`#3089 <https://github.com/ros2/rclcpp/issues/3089>`__)
* Expose ServiceType in Service public API (`#3088 <https://github.com/ros2/rclcpp/issues/3088>`__)
* perf: Optimized out shared_ptr copies (`#3079 <https://github.com/ros2/rclcpp/issues/3079>`__)
* avoid stale parameter events in content filter tests. (`#3085 <https://github.com/ros2/rclcpp/issues/3085>`__)
* improve lookup time for matches_any_publishers() (`#3084 <https://github.com/ros2/rclcpp/issues/3084>`__)
* Add tests isolation (`#3081 <https://github.com/ros2/rclcpp/issues/3081>`__)
* Revert "improve lookup time for matches_any_publishers(). (`#3068 <https://github.com/ros2/rclcpp/issues/3068>`__)" (`#3077 <https://github.com/ros2/rclcpp/issues/3077>`__)
* improve lookup time for matches_any_publishers(). (`#3068 <https://github.com/ros2/rclcpp/issues/3068>`__)
* fix: Use default rcl allocator if allocator is std::allocator (`#3058 <https://github.com/ros2/rclcpp/issues/3058>`__)
* fix: Various data races in test cases (`#3057 <https://github.com/ros2/rclcpp/issues/3057>`__)
* fix: Fix data race in CallbackGroup::size() (`#3056 <https://github.com/ros2/rclcpp/issues/3056>`__)
* remove default: so that compiler can detect the missing case. (`#3048 <https://github.com/ros2/rclcpp/issues/3048>`__)
* use weak_ptr for rcl entities in the memory strategy. (`#2988 <https://github.com/ros2/rclcpp/issues/2988>`__)
* remove test_static_executor_entities_collector.cpp (`#3041 <https://github.com/ros2/rclcpp/issues/3041>`__)
* include the 1st spin that might throw the exception. (`#3042 <https://github.com/ros2/rclcpp/issues/3042>`__)
* print warning message on owner node if the parameter operation fails. (`#3037 <https://github.com/ros2/rclcpp/issues/3037>`__)
* fix context in wait for message wait set (`#3030 <https://github.com/ros2/rclcpp/issues/3030>`__)
* Revert "construct wait set with passed in context (`#3021 <https://github.com/ros2/rclcpp/issues/3021>`__)" (`#3028 <https://github.com/ros2/rclcpp/issues/3028>`__)
* construct wait set with passed in context (`#3021 <https://github.com/ros2/rclcpp/issues/3021>`__)
* Improve the robustness of the TopicEndpointInfo constructor (`#3013 <https://github.com/ros2/rclcpp/issues/3013>`__)
* Deprecate the shared_ptr<MessageT> subscription callback signatures (`#2975 <https://github.com/ros2/rclcpp/issues/2975>`__)
* Updated deprecated ament_index_cpp API (`#3011 <https://github.com/ros2/rclcpp/issues/3011>`__)
* Unified Node Interfaces: Add const version of get_node_x_interface() (`#3006 <https://github.com/ros2/rclcpp/issues/3006>`__)
* Parameter Descriptor Simplification  (`#2179 <https://github.com/ros2/rclcpp/issues/2179>`__)
* ParameterEventHandler support ContentFiltering (`#2971 <https://github.com/ros2/rclcpp/issues/2971>`__)
* update policy_name_from_kind && test_qos (`#2156 <https://github.com/ros2/rclcpp/issues/2156>`__)
* Add ability to disable and enable subscription's callbacks (`#2985 <https://github.com/ros2/rclcpp/issues/2985>`__)
* Switch to isolated testing via rmw_test_fixture (`#2929 <https://github.com/ros2/rclcpp/issues/2929>`__)
* remove I/O from signal handler. (`#2986 <https://github.com/ros2/rclcpp/issues/2986>`__)
* correct test function descriptions (`#2970 <https://github.com/ros2/rclcpp/issues/2970>`__)
* add : get clients, servers info (`#2569 <https://github.com/ros2/rclcpp/issues/2569>`__)
* Fix REP url locations (`#2987 <https://github.com/ros2/rclcpp/issues/2987>`__)
* clear handles before node destruction in test_memory_strategy. (`#2969 <https://github.com/ros2/rclcpp/issues/2969>`__)
* Added static assert asserting custom types have no overloaded operator new (`#2954 <https://github.com/ros2/rclcpp/issues/2954>`__)
* Store graph listener inside the context instead of the node graph (`#2952 <https://github.com/ros2/rclcpp/issues/2952>`__)
* Reapply "Catch the exception from rate.sleep() if the context is invalid. (`#2956 <https://github.com/ros2/rclcpp/issues/2956>`__)" (`#2963 <https://github.com/ros2/rclcpp/issues/2963>`__) (`#2964 <https://github.com/ros2/rclcpp/issues/2964>`__)
* Revert "Catch the exception from rate.sleep() if the context is invalid. (`#2956 <https://github.com/ros2/rclcpp/issues/2956>`__)" (`#2963 <https://github.com/ros2/rclcpp/issues/2963>`__)
* Catch the exception from rate.sleep() if the context is invalid. (`#2956 <https://github.com/ros2/rclcpp/issues/2956>`__)
* update Time documentation (`#2955 <https://github.com/ros2/rclcpp/issues/2955>`__)
* Removed warning (`#2949 <https://github.com/ros2/rclcpp/issues/2949>`__)
* add note about problems with spin_until_future_complete (`#2849 <https://github.com/ros2/rclcpp/issues/2849>`__)
* deprecate rclcpp::spin_some and rclcpp::spin_all (`#2848 <https://github.com/ros2/rclcpp/issues/2848>`__)
* Improve the function extract_type_identifier (`#2923 <https://github.com/ros2/rclcpp/issues/2923>`__)
* Allow for implicitly convertable loggers as well (`#2922 <https://github.com/ros2/rclcpp/issues/2922>`__)
* Fix: improve exception context for parameter_value_from (`#2917 <https://github.com/ros2/rclcpp/issues/2917>`__)
* Fix ``start_type_description_service`` param handling (`#2897 <https://github.com/ros2/rclcpp/issues/2897>`__)
* Add qos parameter for wait_for_message function (`#2903 <https://github.com/ros2/rclcpp/issues/2903>`__)
* Fujitatomoya/test append parameter override (`#2896 <https://github.com/ros2/rclcpp/issues/2896>`__)
* Expose ``typesupport_helpers`` API needed for the Rosbag2 (`#2858 <https://github.com/ros2/rclcpp/issues/2858>`__)
* Remove comment about now-removed StaticSingleThreadedExecutor (`#2893 <https://github.com/ros2/rclcpp/issues/2893>`__)
* Add overload of ``append_parameter_override`` (`#2891 <https://github.com/ros2/rclcpp/issues/2891>`__)
* fix: Don't deadlock if removing shutdown callbacks in a shutdown callback (`#2886 <https://github.com/ros2/rclcpp/issues/2886>`__)
* Hand-code logging.hpp (`#2870 <https://github.com/ros2/rclcpp/issues/2870>`__)
* Adressed TODO in node_graph (`#2877 <https://github.com/ros2/rclcpp/issues/2877>`__)
* fix test_publisher_with_system_default_qos. (`#2881 <https://github.com/ros2/rclcpp/issues/2881>`__)
* Fix for memory leaks in rclcpp::SerializedMessage (`#2861 <https://github.com/ros2/rclcpp/issues/2861>`__)
* Removed warning test_qos (`#2859 <https://github.com/ros2/rclcpp/issues/2859>`__)
* Added missing chrono includes (`#2854 <https://github.com/ros2/rclcpp/issues/2854>`__)
* get_all_data_impl() does not handle null pointers properly, causing segmentation fault (`#2840 <https://github.com/ros2/rclcpp/issues/2840>`__)
* QoSInitialization::from_rmw does not validate invalid history policy values, leading to silent failures (`#2841 <https://github.com/ros2/rclcpp/issues/2841>`__)
* remove get_notify_guard_condition from NodeBaseInterface. (`#2839 <https://github.com/ros2/rclcpp/issues/2839>`__)
* Removed deprecated StaticSingleThreadedExecutor (`#2835 <https://github.com/ros2/rclcpp/issues/2835>`__)
* Removed deprecated rcpputils Path (`#2834 <https://github.com/ros2/rclcpp/issues/2834>`__)
* Add range constraints for applicable array parameters (`#2828 <https://github.com/ros2/rclcpp/issues/2828>`__)
* Update RingBufferImplementation to clear internal data. (`#2837 <https://github.com/ros2/rclcpp/issues/2837>`__)
* Removed deprecated cancel_sleep_or_wait (`#2836 <https://github.com/ros2/rclcpp/issues/2836>`__)
* Add missing 's' to 'NodeParametersInterface' in doc/comment (`#2831 <https://github.com/ros2/rclcpp/issues/2831>`__)
* subordinate node consistent behavior and update docstring. (`#2822 <https://github.com/ros2/rclcpp/issues/2822>`__)
* throws std::invalid_argument if ParameterEvent is NULL. (`#2814 <https://github.com/ros2/rclcpp/issues/2814>`__)
* Removed clang warnings (`#2823 <https://github.com/ros2/rclcpp/issues/2823>`__)
* Contributors: Alberto Soragna, Alejandro Hernández Cordero, Alex Youngs, Alexis Tsogias, Andrianov Roman, Barry Xu, CY Chen, Chris Lalancette, Christophe Bedard, Danil, Emerson Knapp, Ilario A. Azzollini, Ivo Ivanov, Janosch Machowinski, Julien Enoch, Lee, Lucas Wendland, Maurice Alexander Purnawan, Michael Carlstrom, Michael Carroll, Michael Orlov, Michiel Leegwater, Minju, Oren Bell, Patrick Roncagliolo, Peng Wang, Rahat Dhande, Skyler Medeiros, Sriharsha Ghanta, Tim Clephas, Tomoya Fujita, Yadnyeshwar Amol Sakhare, Yuchen966, fabianhirmann, jay, yadunund


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rclcpp_action <https://github.com/ros2/rclcpp/tree/lyrical/rclcpp_action/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* publish_feedback should effect only on executing state. (`#3118 <https://github.com/ros2/rclcpp/issues/3118>`__)
* Support to configure feedback subscription content filter for action client (`#3034 <https://github.com/ros2/rclcpp/issues/3034>`__)
* Use new ROSIDL aggregate CMake target (`#3105 <https://github.com/ros2/rclcpp/issues/3105>`__)
* Fix expiration of action goals when EventsExecutors are used (`#3018 <https://github.com/ros2/rclcpp/issues/3018>`__)
* perf: Optimized out shared_ptr copies (`#3079 <https://github.com/ros2/rclcpp/issues/3079>`__)
* remove default: so that compiler can detect the missing case. (`#3048 <https://github.com/ros2/rclcpp/issues/3048>`__)
* Update exception documentation for goal cancellation in ServerGoalHandle (`#3019 <https://github.com/ros2/rclcpp/issues/3019>`__)
* Fix REP url locations (`#2987 <https://github.com/ros2/rclcpp/issues/2987>`__)
* it misses the iterator second to lock the weakptr. (`#2958 <https://github.com/ros2/rclcpp/issues/2958>`__)
* try aborting before canceling 1st on dtor of ServerGoalHandle. (`#2953 <https://github.com/ros2/rclcpp/issues/2953>`__)
* deprecate rclcpp::spin_some and rclcpp::spin_all (`#2848 <https://github.com/ros2/rclcpp/issues/2848>`__)
* fix cmake deprecation (`#2914 <https://github.com/ros2/rclcpp/issues/2914>`__)
* Replace std::default_random_engine with std::mt19937 (rolling) (`#2843 <https://github.com/ros2/rclcpp/issues/2843>`__)
* Added missing chrono includes (`#2854 <https://github.com/ros2/rclcpp/issues/2854>`__)
* Contributors: Alberto Soragna, Alejandro Hernández Cordero, Andrei Costinescu, Barry Xu, Emerson Knapp, Janosch Machowinski, Skyler Medeiros, Tim Clephas, Tomoya Fujita, keeponoiro, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rclcpp_components <https://github.com/ros2/rclcpp/tree/lyrical/rclcpp_components/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Refactor component containers + Add option for CBG Executor (`#3134 <https://github.com/ros2/rclcpp/issues/3134>`__)
* feat: Add per-node log level support via NodeOptions (`#3092 <https://github.com/ros2/rclcpp/issues/3092>`__)
* Use new ROSIDL aggregate CMake target (`#3105 <https://github.com/ros2/rclcpp/issues/3105>`__)
* Avoid unecessary creation of MultiThreadedExecutor (`#3090 <https://github.com/ros2/rclcpp/issues/3090>`__)
* Fix component registering in subdirectories (`#3064 <https://github.com/ros2/rclcpp/issues/3064>`__)
* Add library dependency to node executable in rclcpp_components_register_node (`#3047 <https://github.com/ros2/rclcpp/issues/3047>`__)
* Updated deprecated ament_index_cpp API (`#3011 <https://github.com/ros2/rclcpp/issues/3011>`__)
* Fix REP url locations (`#2987 <https://github.com/ros2/rclcpp/issues/2987>`__)
* Cleanup the dependencies in rclcpp_components. (`#2918 <https://github.com/ros2/rclcpp/issues/2918>`__)
* fix cmake deprecation (`#2914 <https://github.com/ros2/rclcpp/issues/2914>`__)
* NEW PR: Add component_container for EventsExecutor (`#2885 <https://github.com/ros2/rclcpp/issues/2885>`__)
* make sure that plugin arg includes the double colon. (`#2878 <https://github.com/ros2/rclcpp/issues/2878>`__)
* set thread names by node in component container isolated (`#2871 <https://github.com/ros2/rclcpp/issues/2871>`__)
* Added missing chrono includes (`#2854 <https://github.com/ros2/rclcpp/issues/2854>`__)
* Contributors: Adam Aposhian, Alejandro Hernández Cordero, Chris Lalancette, Emerson Knapp, Mihir Rao, Peng Wang, Skyler Medeiros, Tim Clephas, Tomoya Fujita, YuJin Hong, mosfet80, pum1k, solo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rclcpp_lifecycle <https://github.com/ros2/rclcpp/tree/lyrical/rclcpp_lifecycle/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#3105 <https://github.com/ros2/rclcpp/issues/3105>`__)
* Compatiblity with 'Populate Transitions' `ros2/rcl#1269 <https://github.com/ros2/rcl/issues/1269>`__ (`#2967 <https://github.com/ros2/rclcpp/issues/2967>`__)
* add : get clients, servers info (`#2569 <https://github.com/ros2/rclcpp/issues/2569>`__)
* Fix REP url locations (`#2987 <https://github.com/ros2/rclcpp/issues/2987>`__)
* Add get_parameter_or overload returning value or alternative (`#2973 <https://github.com/ros2/rclcpp/issues/2973>`__)
* deprecate rclcpp::spin_some and rclcpp::spin_all (`#2848 <https://github.com/ros2/rclcpp/issues/2848>`__)
* Clearer warning message, the old one lacked information and was perhaps misleading (`#2927 <https://github.com/ros2/rclcpp/issues/2927>`__)
* fix cmake deprecation (`#2914 <https://github.com/ros2/rclcpp/issues/2914>`__) cmake version < then 3.10 is deprecated
* Added missing chrono includes (`#2854 <https://github.com/ros2/rclcpp/issues/2854>`__)
* introduce rcl_lifecycle_get_transition_label_by_id(). (`#2827 <https://github.com/ros2/rclcpp/issues/2827>`__)
* Contributors: Alberto Soragna, Alejandro Hernández Cordero, Emerson Knapp, Jasper van Brakel, Lee, Minju, Peter Mitrano (AR), Tim Clephas, Tomoya Fujita, Zheng Qu, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rclpy <https://github.com/ros2/rclpy/tree/lyrical/rclpy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Feature: async node (`#1620 <https://github.com/ros2/rclpy/issues/1620>`__)
* Refactor: moved TypeDescriptionService, LoggingService, ParameterService to BaseNode (`#1645 <https://github.com/ros2/rclpy/issues/1645>`__)
* Refactor: base node (`#1637 <https://github.com/ros2/rclpy/issues/1637>`__)
* Bugfix: executor doesn't propagate exception from task that awaited a future (`#1643 <https://github.com/ros2/rclpy/issues/1643>`__)
* Fix: disable flaky executor test (`#1648 <https://github.com/ros2/rclpy/issues/1648>`__) (`#1649 <https://github.com/ros2/rclpy/issues/1649>`__)
* Streamline entity destroy (`#1629 <https://github.com/ros2/rclpy/issues/1629>`__)
* Add acceptable_buffer_backends as subscription option in rclpy (`#1628 <https://github.com/ros2/rclpy/issues/1628>`__)
* publish_feedback should effect only on executing state. (`#1639 <https://github.com/ros2/rclpy/issues/1639>`__)
* Support to configure feedback subscription content filter for action client (`#1633 <https://github.com/ros2/rclpy/issues/1633>`__)
* fix flaky test_multi_threaded_executor_closes_threads. (`#1636 <https://github.com/ros2/rclpy/issues/1636>`__)
* Fix violation (`#1635 <https://github.com/ros2/rclpy/issues/1635>`__)
* Fix test_executor types (`#1632 <https://github.com/ros2/rclpy/issues/1632>`__)
* Refactor: base clock (`#1627 <https://github.com/ros2/rclpy/issues/1627>`__)
* Fix future flake8 (`#1634 <https://github.com/ros2/rclpy/issues/1634>`__)
* Use new ROSIDL aggregate CMake target (`#1630 <https://github.com/ros2/rclpy/issues/1630>`__)
* Update type hints for parameters (`#1631 <https://github.com/ros2/rclpy/issues/1631>`__)
* Add support check for content filter feature in subscription (`#1618 <https://github.com/ros2/rclpy/issues/1618>`__)
* Refactor: base entity classes (`#1624 <https://github.com/ros2/rclpy/issues/1624>`__)
* Fix more test typings and remove unused type aliases (`#1626 <https://github.com/ros2/rclpy/issues/1626>`__)
* Add types to test_waitable (`#1625 <https://github.com/ros2/rclpy/issues/1625>`__)
* Correct typos (`#1619 <https://github.com/ros2/rclpy/issues/1619>`__)
* Fix incorrect action client/server callback type hints (`#1616 <https://github.com/ros2/rclpy/issues/1616>`__)
* avoid stale parameter events in content filter tests. (`#1615 <https://github.com/ros2/rclpy/issues/1615>`__)
* fix violations (`#1614 <https://github.com/ros2/rclpy/issues/1614>`__)
* Typing Regression Fixes (`#1612 <https://github.com/ros2/rclpy/issues/1612>`__)
* CFT is only supported rmw_fastrtps and rmw_connextdds. (`#1611 <https://github.com/ros2/rclpy/issues/1611>`__)
* Prevents the Future result from being set twice. (`#1599 <https://github.com/ros2/rclpy/issues/1599>`__)
* Wrap up ActionClient construction before spining (`#1591 <https://github.com/ros2/rclpy/issues/1591>`__)
* Compatiblity with 'Populate Transitions' `ros2/rcl#1269 <https://github.com/ros2/rcl/issues/1269>`__ (`#1528 <https://github.com/ros2/rclpy/issues/1528>`__)
* Drop invalid waitables from wait set (`#1590 <https://github.com/ros2/rclpy/issues/1590>`__)
* give some time for the discovery for test_on_new_message_callback. (`#1585 <https://github.com/ros2/rclpy/issues/1585>`__)
* print warning message on owner node if the parameter operation fails. (`#1584 <https://github.com/ros2/rclpy/issues/1584>`__)
* Update release version to 10.0.4 (`#1583 <https://github.com/ros2/rclpy/issues/1583>`__)
* Update ``type_support.py`` to use new message abstract base classes  (`#1509 <https://github.com/ros2/rclpy/issues/1509>`__)
* Fix performance bug in MultiThreadedExecutor (hopefully) (`#1547 <https://github.com/ros2/rclpy/issues/1547>`__)
* Expose action graph functions as Node class methods. (`#1574 <https://github.com/ros2/rclpy/issues/1574>`__)
* Improve wildcard parsing and optimize the logic for parsing YAML para… (`#1571 <https://github.com/ros2/rclpy/issues/1571>`__)
* Improve the compatibility of processing YAML parameter files (`#1548 <https://github.com/ros2/rclpy/issues/1548>`__)
* Fix parameter parsing for unspecified target nodes (`#1552 <https://github.com/ros2/rclpy/issues/1552>`__)
* Remove default from switch with enum, so that compiler warns. (`#1566 <https://github.com/ros2/rclpy/issues/1566>`__)
* Use unconditional wait when possible. (`#1563 <https://github.com/ros2/rclpy/issues/1563>`__)
* Increase clock accuracy (`#1564 <https://github.com/ros2/rclpy/issues/1564>`__)
* Fix issues with resuming async tasks awaiting a future (`#1469 <https://github.com/ros2/rclpy/issues/1469>`__)
* ParameterEventHandler support ContentFiltering (`#1531 <https://github.com/ros2/rclpy/issues/1531>`__)
* add : get clients, servers info (`#1307 <https://github.com/ros2/rclpy/issues/1307>`__)
* Allow action servers without execute callback (`#1219 <https://github.com/ros2/rclpy/issues/1219>`__)
* Remove accidental tuple (`#1542 <https://github.com/ros2/rclpy/issues/1542>`__)
* fix(test_events_executor): destroy all nodes before shutdown (`#1538 <https://github.com/ros2/rclpy/issues/1538>`__)
* Remove duplicate future handling from send_goal_async (`#1532 <https://github.com/ros2/rclpy/issues/1532>`__)
* remove unused 'param_type' (`#1524 <https://github.com/ros2/rclpy/issues/1524>`__)
* Fixes Action.*_async futures never complete (`#1308 <https://github.com/ros2/rclpy/issues/1308>`__)
* add spinning state for the Executor classes. (`#1510 <https://github.com/ros2/rclpy/issues/1510>`__)
* EventsExecutor: Handle async callbacks for services and subscriptions (`#1478 <https://github.com/ros2/rclpy/issues/1478>`__)
* Added lock to protect futures for multithreaded executor (`#1477 <https://github.com/ros2/rclpy/issues/1477>`__)
* Add content-filtered-topic interfaces (`#1506 <https://github.com/ros2/rclpy/issues/1506>`__)
* Fix warnings from gcc. (`#1501 <https://github.com/ros2/rclpy/issues/1501>`__)
* Feature: expose event callback setter in subscription, service, client and timer (`#1496 <https://github.com/ros2/rclpy/issues/1496>`__)
* Feature: add executor.create_future() (`#1495 <https://github.com/ros2/rclpy/issues/1495>`__)
* Add More Test Typings (`#1472 <https://github.com/ros2/rclpy/issues/1472>`__)
* Use pybind11 from deb or pixi (`#1497 <https://github.com/ros2/rclpy/issues/1497>`__)
* Do not execute the timer if call_timer_with_info() fails (`#1488 <https://github.com/ros2/rclpy/issues/1488>`__)
* Fix msbuild warnings on ``operator==`` deprecation for pybind11 >=2.2 (`#1483 <https://github.com/ros2/rclpy/issues/1483>`__)
* Cleanup the rclpy dependencies. (`#1482 <https://github.com/ros2/rclpy/issues/1482>`__)
* Feature: add logger_name property to subscription, publisher, service and client (`#1471 <https://github.com/ros2/rclpy/issues/1471>`__)
* Update ``test_node`` Types (`#1464 <https://github.com/ros2/rclpy/issues/1464>`__)
* Add method that get datetime.datetime from Time (`#1443 <https://github.com/ros2/rclpy/issues/1443>`__)
* add ``MessageInfo.publisher_gid`` (`#1466 <https://github.com/ros2/rclpy/issues/1466>`__)
* Add types to ``test_action\_\*.py`` (`#1444 <https://github.com/ros2/rclpy/issues/1444>`__)
* Revert "Fix Duration, Clock, and QoS Docs (`#1428 <https://github.com/ros2/rclpy/issues/1428>`__)" (`#1447 <https://github.com/ros2/rclpy/issues/1447>`__)
* remove all deprecated classes and methods (`#1456 <https://github.com/ros2/rclpy/issues/1456>`__)
* [rclpy] Fix spin() incorrectly removing node from executor if already attached (`#1446 <https://github.com/ros2/rclpy/issues/1446>`__)
* Contributors: Alejandro Hernández Cordero, Alon Borenshtein, Auguste Lalande, Barry Xu, Brad Martin, Brennan Miller-Klugman, Błażej Sowa, CY Chen, Chris Lalancette, Christian Rauch, Clara Berendsen, Emerson Knapp, Florian Vahl, Jasper van Brakel, Jean Paul, Jonathan, Lee, Michael Carlstrom, Michael Tandy, Minju, Nadav Elkabets, Nathan Wiebe Neufeldt, Tim Clephas, Tomoya Fujita, Yuyuan Yuan, mhidalgo-rai


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcpputils <https://github.com/ros2/rcpputils/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Updated note related with tl_expected (`#229 <https://github.com/ros2/rcpputils/issues/229>`__)
* Increase test coverage (`#222 <https://github.com/ros2/rcpputils/issues/222>`__)
* Append copies of BSD and CC0 licenses from the works (`#223 <https://github.com/ros2/rcpputils/issues/223>`__)
* Use std::filesystem in find_library and add more test (`#221 <https://github.com/ros2/rcpputils/issues/221>`__)
* Remove -Werror from Clang compile options (`#220 <https://github.com/ros2/rcpputils/issues/220>`__)
* Remove unnecessary dependencies from rcpputils. (`#216 <https://github.com/ros2/rcpputils/issues/216>`__) It doesn't need to have dependencies on python tests.
* fix cmake deprecation (`#214 <https://github.com/ros2/rcpputils/issues/214>`__)
* add thread naming utilities (`#213 <https://github.com/ros2/rcpputils/issues/213>`__)
* Removed deprecated path (`#212 <https://github.com/ros2/rcpputils/issues/212>`__)
* Contributors: Adam Aposhian, Alejandro Hernández Cordero, Chris Lalancette, Tully Foote, William Woodall, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcutils <https://github.com/ros2/rcutils/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add buildtool_export_depend on ament_cmake_ros_core (`#558 <https://github.com/ros2/rcutils/issues/558>`__)
* fix: typo in parameter documentation for overwrite (`#557 <https://github.com/ros2/rcutils/issues/557>`__)
* Remove ATOMIC_VAR_INIT (`#556 <https://github.com/ros2/rcutils/issues/556>`__)
* Use ``ament_set_default_language_standards`` from ``ament_cmake_core`` (`#548 <https://github.com/ros2/rcutils/issues/548>`__)
* Use uncommon variable name in macro to avoid being overwritten (`#551 <https://github.com/ros2/rcutils/issues/551>`__)
* Remove ``ament_export_link_flags()`` for atomic operations (`#528 <https://github.com/ros2/rcutils/issues/528>`__)
* Use less common variable name in macro (`#550 <https://github.com/ros2/rcutils/issues/550>`__)
* Fix missing include for std::get_time (`#549 <https://github.com/ros2/rcutils/issues/549>`__)
* Fix gcc 15.2.1 warning for discarding 'const' qualifier (`#547 <https://github.com/ros2/rcutils/issues/547>`__)
* Disable warning C5105 for older Windows SDKs in base64.c (`#544 <https://github.com/ros2/rcutils/issues/544>`__)
* Add {short_file_name} as log format option (`#541 <https://github.com/ros2/rcutils/issues/541>`__)
* Add base64 encoding and decoding functions with tests (`#533 <https://github.com/ros2/rcutils/issues/533>`__)
* remove default: so that compiler can detect the missing case. (`#534 <https://github.com/ros2/rcutils/issues/534>`__)
* Check SIZE_MAX for array initialization. (`#527 <https://github.com/ros2/rcutils/issues/527>`__)
* Do not export dl in rcutils_LIBRARIES (`#522 <https://github.com/ros2/rcutils/issues/522>`__)
* rcutils_logging_allocator_initialize() support. (`#419 <https://github.com/ros2/rcutils/issues/419>`__)
* Export -latomic even if BUILD_TESTING is disabled. (`#516 <https://github.com/ros2/rcutils/issues/516>`__)
* Add rcutils_raw_steady_time_now method for slew-free clock (`#507 <https://github.com/ros2/rcutils/issues/507>`__)
* Revert "use getenv_s instead of getenv for Windows. (`#499 <https://github.com/ros2/rcutils/issues/499>`__)" (`#504 <https://github.com/ros2/rcutils/issues/504>`__) This reverts commit 46ab4d4eeb555a2e9e880157b97f0a867d3a256c.
* Hand-code logging_macros.h (`#502 <https://github.com/ros2/rcutils/issues/502>`__)
* Implement rcutils_strnlen. (`#430 <https://github.com/ros2/rcutils/issues/430>`__)
* use getenv_s instead of getenv for Windows. (`#499 <https://github.com/ros2/rcutils/issues/499>`__)
* Make linters happy
* Clean memory in test_process.cpp (`#495 <https://github.com/ros2/rcutils/issues/495>`__)
* Contributors: Alejandro Hernández Cordero, Andrei Kholodnyi, Barry Xu, Chris Lalancette, EddyGharib, Miguel Company, Sai Kishor Kothakota, Shane Loretz, Tomoya Fujita, Tony Najjar


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`resource_retriever <https://github.com/ros/resource_retriever/tree/lyrical/resource_retriever/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Removed python2 code (`#121 <https://github.com/ros/resource_retriever/issues/121>`__)
* Delete resource_retriever/setup.py (`#120 <https://github.com/ros/resource_retriever/issues/120>`__)
* Use get_package_share_path (`#119 <https://github.com/ros/resource_retriever/issues/119>`__)
* Updated deprecated ament_index_cpp API (`#118 <https://github.com/ros/resource_retriever/issues/118>`__)
* removed libcurl_vendor package (`#116 <https://github.com/ros/resource_retriever/issues/116>`__)
* Removed deprecated code (`#113 <https://github.com/ros/resource_retriever/issues/113>`__)
* Fixed clang compile error (`#112 <https://github.com/ros/resource_retriever/issues/112>`__)
* Removed windows warnings (`#111 <https://github.com/ros/resource_retriever/issues/111>`__)
* Add a plugin mechanism to resource_retriever (`#103 <https://github.com/ros/resource_retriever/issues/103>`__)
* uniform  MinCMakeVersion (`#108 <https://github.com/ros/resource_retriever/issues/108>`__)
* Contributors: Alejandro Hernández Cordero, Michael Carlstrom, Michael Carroll, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`resource_retriever_interfaces <https://github.com/ros2/resource_retriever_service/tree/lyrical/resource_retriever_interfaces/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the plugin license (`#17 <https://github.com/ros2/resource_retriever_service/issues/17>`__)
* Contributors: Stoyan Gaydarov


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`resource_retriever_service <https://github.com/ros2/resource_retriever_service/tree/lyrical/resource_retriever_service/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the plugin license (`#17 <https://github.com/ros2/resource_retriever_service/issues/17>`__)
* Contributors: Stoyan Gaydarov


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`resource_retriever_service_plugin <https://github.com/ros2/resource_retriever_service/tree/lyrical/resource_retriever_service_plugin/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the plugin license (`#17 <https://github.com/ros2/resource_retriever_service/issues/17>`__)
* Contributors: Stoyan Gaydarov


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw <https://github.com/ros2/rmw/tree/lyrical/rmw/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* find_package ament_cmake_gtest (`#417 <https://github.com/ros2/rmw/issues/417>`__)
* Add acceptable_buffer_backends field in rmw_subscription_options_s (`#416 <https://github.com/ros2/rmw/issues/416>`__)
* Add is_cft_supported field to rmw_subscription_t for content filtering support (`#415 <https://github.com/ros2/rmw/issues/415>`__)
* Remove default from switch with enum, so that compiler warns. (`#414 <https://github.com/ros2/rmw/issues/414>`__)
* add: get clients, servers info (`#371 <https://github.com/ros2/rmw//issues/371>`__)
* Fix REP url locations (`#406 <https://github.com/ros2/rmw//issues/406>`__)
* Update link to rmw API docs (`#405 <https://github.com/ros2/rmw//issues/405>`__)
* Don't assume a DDS-based implementation in function docs (`#402 <https://github.com/ros2/rmw//issues/402>`__)
* Contributors: Barry Xu, CY Chen, Christophe Bedard, Lee, Minju, Shane Loretz, Tim Clephas, Tomoya Fujita


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_connextdds <https://github.com/ros2/rmw_connextdds/tree/lyrical/rmw_connextdds/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix: Fixed compilation on MSVC 2022 (`#225 <https://github.com/ros2/rmw_connextdds/issues/225>`__)
* Remove default from switch with enum to enable compiler warnings (`#216 <https://github.com/ros2/rmw_connextdds/issues/216>`__)
* add : get clients,servers info (`#154 <https://github.com/ros2/rmw_connextdds/issues/154>`__)
* fix: remove superflous ``buildtool_export_depend`` (`#206 <https://github.com/ros2/rmw_connextdds/issues/206>`__)
* Fix cmake deprecation (`#198 <https://github.com/ros2/rmw_connextdds/issues/198>`__)
* Contributors: Bas Zalmstra, Janosch Machowinski, Lee, Minju, Tomoya Fujita, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_connextdds_common <https://github.com/ros2/rmw_connextdds/tree/lyrical/rmw_connextdds_common/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix content filtering on Windows with modern Connext DDS (`#226 <https://github.com/ros2/rmw_connextdds/issues/226>`__) (`#230 <https://github.com/ros2/rmw_connextdds/issues/230>`__)
* fix: Fixed compilation on MSVC 2022 (`#225 <https://github.com/ros2/rmw_connextdds/issues/225>`__)
* Add variable ``RMW_CONNEXT_USER_TOPICS_PUBLISH_MODE`` and deprecate ``RMW_CONNEXT_USE_DEFAULT_PUBLISH_MODE`` (`#224 <https://github.com/ros2/rmw_connextdds/issues/224>`__)
* Update Connext from 7.3.0 to 7.7.0, disable monitoring library by default, and use synchronous publishing mode (`#219 <https://github.com/ros2/rmw_connextdds/issues/219>`__)
* Enable property ``dds.ros.demangle_topic_and_type_names`` to announce demangled topic name as topic alias (`#221 <https://github.com/ros2/rmw_connextdds/issues/221>`__)
* Enable content filtering flag (`#223 <https://github.com/ros2/rmw_connextdds/issues/223>`__)
* Remove deprecated security properties and use new ones (`#217 <https://github.com/ros2/rmw_connextdds/issues/217>`__)
* Remove default from switch with enum to enable compiler warnings (`#216 <https://github.com/ros2/rmw_connextdds/issues/216>`__)
* Replace ``DDS_ContentFilter_register_filter`` with ``DDS_DomainParticipant_register_contentfilterI`` (`#215 <https://github.com/ros2/rmw_connextdds/issues/215>`__)
* Remove superfluous ``buildtool_export_depend`` (`#210 <https://github.com/ros2/rmw_connextdds/issues/210>`__)
* add : get clients,servers info (`#154 <https://github.com/ros2/rmw_connextdds/issues/154>`__)
* [rmw_connextdds_common]: Remove <member_of_group>rosidl_interface_packages (`#202 <https://github.com/ros2/rmw_connextdds/issues/202>`__)
* Correctly calculate the size of a serialized key (`#200 <https://github.com/ros2/rmw_connextdds/issues/200>`__)
* Fix cmake deprecation (`#198 <https://github.com/ros2/rmw_connextdds/issues/198>`__)
* Fixed serialized minimum sample size callback (`#196 <https://github.com/ros2/rmw_connextdds/issues/196>`__)
* Removed warning (`#187 <https://github.com/ros2/rmw_connextdds/issues/187>`__)
* Contributors: Alejandro Hernández Cordero, Barry Xu, Chris Lalancette, Francisco Gallego Salido, Janosch Machowinski, Lee, Minju, Tomoya Fujita, mergify[bot], mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_connextddsmicro <https://github.com/ros2/rmw_connextdds/tree/lyrical/rmw_connextddsmicro/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix: Fixed compilation on MSVC 2022 (`#225 <https://github.com/ros2/rmw_connextdds/issues/225>`__)
* Remove default from switch with enum to enable compiler warnings (`#216 <https://github.com/ros2/rmw_connextdds/issues/216>`__)
* Remove superfluous ``buildtool_export_depend`` (`#210 <https://github.com/ros2/rmw_connextdds/issues/210>`__)
* add : get clients,servers info (`#154 <https://github.com/ros2/rmw_connextdds/issues/154>`__)
* Fix cmake deprecation (`#198 <https://github.com/ros2/rmw_connextdds/issues/198>`__)
* Contributors: Janosch Machowinski, Lee, Minju, Tomoya Fujita, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_cyclonedds_cpp <https://github.com/ros2/rmw_cyclonedds/tree/lyrical/rmw_cyclonedds_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Silence unused variable warning in Release builds (`#580 <https://github.com/ros2/rmw_cyclonedds/issues/580>`__)
* Add key support and update Cyclone DDS compatibility (`#575 <https://github.com/ros2/rmw_cyclonedds/issues/575>`__)
* Explicitly disable content filtering support (`#574 <https://github.com/ros2/rmw_cyclonedds/issues/574>`__)
* Add tracepoint to ``rmw_take_loan_int`` (`#566 <https://github.com/ros2/rmw_cyclonedds/issues/566>`__)
* Fix warnings about ``may be used uninitialized`` (`#573 <https://github.com/ros2/rmw_cyclonedds/issues/573>`__)
* Improve MessageTypeSupport performance (`#562 <https://github.com/ros2/rmw_cyclonedds/issues/562>`__)
* Improve serialization performance by optimizing ``dynamic_cast`` usage and replacing virtual functions with templates (`#553 <https://github.com/ros2/rmw_cyclonedds/issues/553>`__)
* Remove defaults to trigger proper warnings (`#549 <https://github.com/ros2/rmw_cyclonedds/issues/549>`__)
* add : get clients, servers info (`#499 <https://github.com/ros2/rmw_cyclonedds/issues/499>`__)
* Do not include rosidl_typesupport\_{c,cpp} in rmw impl typesupport list (`#544 <https://github.com/ros2/rmw_cyclonedds/issues/544>`__)
* Update CMake requirement (`#539 <https://github.com/ros2/rmw_cyclonedds/issues/539>`__)
* Contributors: Brandon Simoncic, Christophe Bedard, Janosch Machowinski, Lee, Minju, Oren Bell PhD, Shane Loretz, Tomoya Fujita, eboasson, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_dds_common <https://github.com/ros2/rmw_dds_common/tree/lyrical/rmw_dds_common/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* If no publishers discovered, make the best available QoS for subscription. (`#84 <https://github.com/ros2/rmw_dds_common/issues/84>`__)
* Add get_clients_info_by_service and get_servers_info_by_service; introduce ServiceEntityInfo to handle service type hash in graph cache (`#82 <https://github.com/ros2/rmw_dds_common/issues/82>`__)
* Remove deprecated GraphCache methods without type hash (`#83 <https://github.com/ros2/rmw_dds_common/issues/83>`__)
* Update cmake requirements (`#80 <https://github.com/ros2/rmw_dds_common/issues/80>`__)
* Remove deprecated security utilities (`#79 <https://github.com/ros2/rmw_dds_common/issues/79>`__)
* Contributors: Alejandro Hernández Cordero, Christophe Bedard, Lee, Minju, Tomoya Fujita, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_fastrtps_cpp <https://github.com/ros2/rmw_fastrtps/tree/lyrical/rmw_fastrtps_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Clean up logs for the rosidl::Buffer path (`#886 <https://github.com/ros2/rmw_fastrtps//issues/886>`__) (`#887 <https://github.com/ros2/rmw_fastrtps//issues/887>`__)
* Change the buffer-aware BUFBE: -> bufbe. (backport `#880 <https://github.com/ros2/rmw_fastrtps//issues/880>`__) (`#884 <https://github.com/ros2/rmw_fastrtps//issues/884>`__)
* Fix UB in accessing the keys (`#879 <https://github.com/ros2/rmw_fastrtps//issues/879>`__) (`#882 <https://github.com/ros2/rmw_fastrtps//issues/882>`__)
* Remove warning when compiling with ``lcang`` (`#876 <https://github.com/ros2/rmw_fastrtps/issues/876>`__)
* Add support for rosidl::Buffer-aware per-endpoint pub/sub (`#867 <https://github.com/ros2/rmw_fastrtps/issues/867>`__)
* Use new aggregate rosidl target instead of _TARGETS (`#870 <https://github.com/ros2/rmw_fastrtps/issues/870>`__)
* Enable content filtering flag (`#869 <https://github.com/ros2/rmw_fastrtps/issues/869>`__)
* fix: remove superflous buildtool_export_depend. (`#852 <https://github.com/ros2/rmw_fastrtps/issues/852>`__)
* add : get clients, servers info (`#771 <https://github.com/ros2/rmw_fastrtps/issues/771>`__)
* Do not include rosidl_typesupport\_{c,cpp} in rmw impl typesupport list (`#843 <https://github.com/ros2/rmw_fastrtps/issues/843>`__)
* fix cmake deprecation (`#831 <https://github.com/ros2/rmw_fastrtps/issues/831>`__)
* Contributors: Alejandro Hernández Cordero, Alexis Tsogias, Barry Xu, CY Chen, Christophe Bedard, Lee, Minju, Tomoya Fujita, mergify[bot], mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_fastrtps_dynamic_cpp <https://github.com/ros2/rmw_fastrtps/tree/lyrical/rmw_fastrtps_dynamic_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix UB in accessing the keys (`#879 <https://github.com/ros2/rmw_fastrtps//issues/879>`__) (`#882 <https://github.com/ros2/rmw_fastrtps//issues/882>`__)
* Add support for rosidl::Buffer-aware per-endpoint pub/sub (`#867 <https://github.com/ros2/rmw_fastrtps/issues/867>`__)
* Use new aggregate rosidl target instead of _TARGETS (`#870 <https://github.com/ros2/rmw_fastrtps/issues/870>`__)
* fix: remove superflous buildtool_export_depend. (`#852 <https://github.com/ros2/rmw_fastrtps/issues/852>`__)
* add : get clients, servers info (`#771 <https://github.com/ros2/rmw_fastrtps/issues/771>`__)
* Do not include rosidl_typesupport\_{c,cpp} in rmw impl typesupport list (`#843 <https://github.com/ros2/rmw_fastrtps/issues/843>`__)
* Check remaining size before resizing sequences (`#827 <https://github.com/ros2/rmw_fastrtps/issues/827>`__)
* fix cmake deprecation (`#831 <https://github.com/ros2/rmw_fastrtps/issues/831>`__)
* Contributors: Alexis Tsogias, CY Chen, Christophe Bedard, Lee, Miguel Company, Minju, Tomoya Fujita, mergify[bot], mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_fastrtps_shared_cpp <https://github.com/ros2/rmw_fastrtps/tree/lyrical/rmw_fastrtps_shared_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Change the buffer-aware BUFBE: -> bufbe. (backport `#880 <https://github.com/ros2/rmw_fastrtps//issues/880>`__) (`#884 <https://github.com/ros2/rmw_fastrtps//issues/884>`__)
* feat: set collection header element_flags TryConstructFailAction::DISCARD instead of 0 (`#875 <https://github.com/ros2/rmw_fastrtps/issues/875>`__)
* Add support for rosidl::Buffer-aware per-endpoint pub/sub (`#867 <https://github.com/ros2/rmw_fastrtps/issues/867>`__)
* Added rmw_take tracepoint, because it wasn't being triggered for successful takes (`#871 <https://github.com/ros2/rmw_fastrtps/issues/871>`__)
* Added tracepoint to loaned take (`#868 <https://github.com/ros2/rmw_fastrtps/issues/868>`__)
* fix: remove superflous buildtool_export_depend. (`#852 <https://github.com/ros2/rmw_fastrtps/issues/852>`__)
* add : get clients, servers info (`#771 <https://github.com/ros2/rmw_fastrtps/issues/771>`__)
* Refs `#23861 <https://github.com/ros2/rmw_fastrtps/issues/23861>`__. Use key annotation in TypeObject build (`#849 <https://github.com/ros2/rmw_fastrtps/issues/849>`__)
* fix cmake deprecation (`#831 <https://github.com/ros2/rmw_fastrtps/issues/831>`__)
* Retrieve ``HistoryQoS`` in discovery when available (`#829 <https://github.com/ros2/rmw_fastrtps/issues/829>`__)
* check a local publication to ignore with serialized message. (`#823 <https://github.com/ros2/rmw_fastrtps/issues/823>`__)
* Contributors: CY Chen, Daisuke Nishimatsu, Lee, Mario Domínguez López, Miguel Company, Minju, Oren Bell, Oren Bell PhD, Tomoya Fujita, mergify[bot], mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_implementation <https://github.com/ros2/rmw_implementation/tree/lyrical/rmw_implementation/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add rmw_zenoh_cpp as a build dependency (`#273 <https://github.com/ros2/rmw_implementation/issues/273>`__)
* Updated deprecated ament_index_cpp API (`#272 <https://github.com/ros2/rmw_implementation/issues/272>`__)
* Add rmw_get_clients_info_by_service , rmw_servers_clients_info_by_service (`#238 <https://github.com/ros2/rmw_implementation/issues/238>`__)
* fix cmake deprecation (`#267 <https://github.com/ros2/rmw_implementation/issues/267>`__)
* Explain rosidl_typesupport\_{c,cpp} in rmw impl typesupport list (`#265 <https://github.com/ros2/rmw_implementation/issues/265>`__)
* Fixed windows warning (`#254 <https://github.com/ros2/rmw_implementation/issues/254>`__)
* Contributors: Alejandro Hernández Cordero, Christophe Bedard, Lee, Minju, Tony Najjar, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_test_fixture <https://github.com/ros2/ament_cmake_ros/tree/lyrical/rmw_test_fixture/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add missing dependency from rmw_test_fixture to rmw (`#53 <https://github.com/ros2/ament_cmake_ros/issues/53>`__)
* add find_package call (`#50 <https://github.com/ros2/ament_cmake_ros/issues/50>`__)
* fix cmake deprecation (`#47 <https://github.com/ros2/ament_cmake_ros/issues/47>`__)
* Contributors: Matt Condino, Scott K Logan, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_test_fixture_implementation <https://github.com/ros2/ament_cmake_ros/tree/lyrical/rmw_test_fixture_implementation/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Block signals during Python environment reload in rmw_test_fixture_implementation (`#64 <https://github.com/ros2/ament_cmake_ros/issues/64>`__)
* Add ``ament_ros_defaults`` target (`#62 <https://github.com/ros2/ament_cmake_ros/issues/62>`__)
* Drop dependency group dependency on test fixtures (`#60 <https://github.com/ros2/ament_cmake_ros/issues/60>`__)
* Restore ROS_DOMAIN_ID after isolation is finished (`#58 <https://github.com/ros2/ament_cmake_ros/issues/58>`__)
* default to c++17 due to use of newer methods on std::map (`#55 <https://github.com/ros2/ament_cmake_ros/issues/55>`__)
* fix cmake deprecation (`#47 <https://github.com/ros2/ament_cmake_ros/issues/47>`__)
* On start-after-stop, re-check RMW_IMPLEMENTATION for changes (`#46 <https://github.com/ros2/ament_cmake_ros/issues/46>`__)
* Choose random domain IDs during default RMW isolation (`#39 <https://github.com/ros2/ament_cmake_ros/issues/39>`__)
* Ignore SIGINT *after* child process has been spawned (`#45 <https://github.com/ros2/ament_cmake_ros/issues/45>`__)
* Add some smoke tests for rmw_test_fixture_implementation (`#42 <https://github.com/ros2/ament_cmake_ros/issues/42>`__)
* Copy all environment variables explicitly (`#43 <https://github.com/ros2/ament_cmake_ros/issues/43>`__)
* Split the generator expression for each library (`#36 <https://github.com/ros2/ament_cmake_ros/issues/36>`__)
* Removed clang warnings (`#34 <https://github.com/ros2/ament_cmake_ros/issues/34>`__)
* Contributors: Alejandro Hernández Cordero, Michael Carlstrom, Michael Carroll, Scott K Logan, Tanishq Chaudhary, William Woodall, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_zenoh_cpp <https://github.com/ros2/rmw_zenoh/tree/lyrical/rmw_zenoh_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Bump Zenoh to 1.8.0, fix Windows shutdown hang, and resolve synchronization with ``undeclare`` (`#964 <https://github.com/ros2/rmw_zenoh/issues/964>`__)
* Revert changes to build against rust >= 1.75 and bump zenoh to 1.8.0 (`#960 <https://github.com/ros2/rmw_zenoh/issues/960>`__)
* Prevent deadlock by not holding both locks when processing event data (`#937 <https://github.com/ros2/rmw_zenoh/issues/937>`__)
* Bump zenoh to 1.8.0 (`#935 <https://github.com/ros2/rmw_zenoh/issues/935>`__)
* Explicitly set ``false`` for the content filtering feature (`#938 <https://github.com/ros2/rmw_zenoh/issues/938>`__)
* Add deadline/liveliness QoS events to ``rmw_zenoh_cpp`` (`#934 <https://github.com/ros2/rmw_zenoh/issues/934>`__)
* Catch ``PackageNotFoundError`` during default config URI loading to prevent crash (`#915 <https://github.com/ros2/rmw_zenoh/issues/915>`__)
* Populate ``reception_sequence_number`` and ``advertise_sequence_number`` features (`#920 <https://github.com/ros2/rmw_zenoh/issues/920>`__)
* Use ``get_package_share_path`` (`#913 <https://github.com/ros2/rmw_zenoh/issues/913>`__)
* Address outstanding TODO items (`#896 <https://github.com/ros2/rmw_zenoh/issues/896>`__)
* Expose zenoh session (`#865 <https://github.com/ros2/rmw_zenoh/issues/865>`__)
* Fix config loading with incorrect path variable (`#898 <https://github.com/ros2/rmw_zenoh/issues/898>`__)
* Fix build binary workflow (`#895 <https://github.com/ros2/rmw_zenoh/issues/895>`__)
* Fix line ending in session open error message (`#888 <https://github.com/ros2/rmw_zenoh/issues/888>`__)
* Update deprecated ``ament_index_cpp`` API (`#879 <https://github.com/ros2/rmw_zenoh/issues/879>`__)
* Remove ``default`` from switch with enum to enable compiler warnings (`#871 <https://github.com/ros2/rmw_zenoh/issues/871>`__)
* Use shared SHM transport provider instead of creating a new instance (`#857 <https://github.com/ros2/rmw_zenoh/issues/857>`__)
* Bump ``zenoh`` to 1.7.1 (`#870 <https://github.com/ros2/rmw_zenoh/issues/870>`__)
* Add rmw_get_clients_info_by_service and rmw_get_servers_info_by_service (`#679 <https://github.com/ros2/rmw_zenoh/issues/679>`__)
* Fix REP url locations (`#858 <https://github.com/ros2/rmw_zenoh/issues/858>`__)
* Restore ZENOH_CONFIG_OVERRIDE after isolation is finished (`#855 <https://github.com/ros2/rmw_zenoh/issues/855>`__)
* Fix typo in 'triggered' (`#844 <https://github.com/ros2/rmw_zenoh/issues/844>`__)
* Log details at SHM creation (alloc and threashold sizes) (`#835 <https://github.com/ros2/rmw_zenoh/issues/835>`__)
* Change default value of ZENOH_SHM_ALLOC_SIZE to 48 MiB (`#830 <https://github.com/ros2/rmw_zenoh/issues/830>`__)
* config: increase queries_default_timeout to 10min (`#820 <https://github.com/ros2/rmw_zenoh/issues/820>`__)
* Fix compile with clang (`#819 <https://github.com/ros2/rmw_zenoh/issues/819>`__)
* feat(logging): add contextual information to log messages (`#809 <https://github.com/ros2/rmw_zenoh/issues/809>`__)
* Align the config with upstream Zenoh. (`#785 <https://github.com/ros2/rmw_zenoh/issues/785>`__)
* fix: resolve memory leak when publishing with the default allocator (`#797 <https://github.com/ros2/rmw_zenoh/issues/797>`__)
* Recycle serialization buffers on transmission (`#342 <https://github.com/ros2/rmw_zenoh/issues/342>`__)
* refactor: avoid redundant key expression creation when replying (`#732 <https://github.com/ros2/rmw_zenoh/issues/732>`__)
* Do not include rosidl_typesupport\_{c,cpp} in rmw impl typesupport list (`#748 <https://github.com/ros2/rmw_zenoh/issues/748>`__)
* fixing typo flow to flows in config files (`#740 <https://github.com/ros2/rmw_zenoh/issues/740>`__)
* Shared Memory on C++ API (`#363 <https://github.com/ros2/rmw_zenoh/issues/363>`__)
* Bump Zenoh to v1.5.0 (`#728 <https://github.com/ros2/rmw_zenoh/issues/728>`__)
* rmw_zenoh_cpp: Include algorithm for std::find_if (`#723 <https://github.com/ros2/rmw_zenoh/issues/723>`__)
* Use rfind to avoid issues with service types ending in Request or Response (`#719 <https://github.com/ros2/rmw_zenoh/issues/719>`__)
* Remove the extra copy on the publisher side (`#711 <https://github.com/ros2/rmw_zenoh/issues/711>`__)
* Avoid ambiguity with variable shadowing (`#706 <https://github.com/ros2/rmw_zenoh/issues/706>`__)
* Only configure the timeout of the action-related service ``get_result`` to maximum value. (`#685 <https://github.com/ros2/rmw_zenoh/issues/685>`__)
* Use Zenoh Querier to replace Session.get (`#694 <https://github.com/ros2/rmw_zenoh/issues/694>`__)
* Use data() to avoid potentially dereferencing an empty vector (`#667 <https://github.com/ros2/rmw_zenoh/issues/667>`__)
* Bump Zenoh to 1.4.0 (`#652 <https://github.com/ros2/rmw_zenoh/issues/652>`__)
* fix(comment): correct the QoS incompatibilities (`#644 <https://github.com/ros2/rmw_zenoh/issues/644>`__)
* fix rmw_take_serialized_message. (`#638 <https://github.com/ros2/rmw_zenoh/issues/638>`__)
* Update CMakeLists.txt (`#617 <https://github.com/ros2/rmw_zenoh/issues/617>`__)
* Contributors: Alejandro Hernandez Cordero, Alejandro Hernández Cordero, ChenYing Kuo (CY), Chris Lalancette, Christophe Bedard, Faseel Chemmadan, Filip, Hervé Audren, Jan Vermaete, Julien Enoch, Lee, Mahmoud Mazouz, Minju, Nikola Banović, Scott K Logan, Shane Loretz, Skyler Medeiros, Steven Palma, Tim Clephas, Tomoya Fujita, Yadunund, Yuyuan Yuan, jordanburklund, milidam, mosfet80, yadunund, yellowhatter, Øystein Sture


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`robot_state_publisher <https://github.com/ros/robot_state_publisher/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#246 <https://github.com/ros/robot_state_publisher/issues/246>`__)
* Improvements (`#245 <https://github.com/ros/robot_state_publisher/issues/245>`__)
* Update subscription callback signatures (`#241 <https://github.com/ros/robot_state_publisher/issues/241>`__)
* Add functionality to read description from a topic instead of a parameter (`#234 <https://github.com/ros/robot_state_publisher/issues/234>`__)
* Removed tf2_ros warning (`#239 <https://github.com/ros/robot_state_publisher/issues/239>`__)
* fix cmake deprecation (`#232 <https://github.com/ros/robot_state_publisher/issues/232>`__)
* Removed tf2_ros warning (`#238 <https://github.com/ros/robot_state_publisher/issues/238>`__)
* Removed orocos kdl vendor dependency (`#237 <https://github.com/ros/robot_state_publisher/issues/237>`__)
* Removed warnings in geometry2 (`#236 <https://github.com/ros/robot_state_publisher/issues/236>`__)
* Replace deprecated tf2_ros headers (`#235 <https://github.com/ros/robot_state_publisher/issues/235>`__)
* Removed deprecated command-line argument (`#233 <https://github.com/ros/robot_state_publisher/issues/233>`__)
* Contributors: Alejandro Hernández Cordero, Emerson Knapp, Kenji Brameld (TRACLabs), Maurice Alexander Purnawan, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2action <https://github.com/ros2/ros2cli/tree/lyrical/ros2action/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix ``flake8`` (`#1215 <https://github.com/ros2/ros2cli/issues/1215>`__)
* Add timeout arguments to ``ros2 service call``, ``ros2 action send_goal``, ``ros2 component``, ``ros2 lifecycle``, and ``ros2 param`` (`#1185 <https://github.com/ros2/ros2cli/issues/1185>`__)
* add osrf_pycommon depend for test_exec. (`#1120 <https://github.com/ros2/ros2cli/issues/1120>`__)
* Fujitatomoya/clearup isolated ros2daemon (`#1098 <https://github.com/ros2/ros2cli/issues/1098>`__)
* Restore environment variables after launch tests (`#1086 <https://github.com/ros2/ros2cli/issues/1086>`__)
* Use rmw_test_fixture to isolate ros2cli tests (`#1062 <https://github.com/ros2/ros2cli/issues/1062>`__)
* fix setuptools deprecations (`#1066 <https://github.com/ros2/ros2cli/issues/1066>`__)
* fix ros2action send_goal signal handling. (`#1072 <https://github.com/ros2/ros2cli/issues/1072>`__)
* Fujitatomoya/ros2 action send goal timeout (`#1067 <https://github.com/ros2/ros2cli/issues/1067>`__)
* Make sure to install py.typed files (`#1058 <https://github.com/ros2/ros2cli/issues/1058>`__)
* Relax the check from exact to partial match. (`#1055 <https://github.com/ros2/ros2cli/issues/1055>`__)
* Export Typing information (`#1041 <https://github.com/ros2/ros2cli/issues/1041>`__)
* move QoS methods from ros2topic.api to ros2cli.qos. (`#1053 <https://github.com/ros2/ros2cli/issues/1053>`__)
* remove unnecessary '/' from ros2 action info. (`#1049 <https://github.com/ros2/ros2cli/issues/1049>`__)
* add QoS option to ros2service/ros2action echo commands. (`#1036 <https://github.com/ros2/ros2cli/issues/1036>`__)
* Allow zenoh tests to run with multicast (`#992 <https://github.com/ros2/ros2cli/issues/992>`__)
* Support 'ros2 action echo' (`#978 <https://github.com/ros2/ros2cli/issues/978>`__)
* Correct the license content (`#979 <https://github.com/ros2/ros2cli/issues/979>`__)
* Contributors: Barry Xu, Christophe Bedard, Michael Carlstrom, Michael Carroll, Scott K Logan, Tomoya Fujita, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2bag <https://github.com/ros2/rosbag2/tree/lyrical/ros2bag/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add ``--repeat-all-transient-local`` flag for automatic transient-local topic detection (`#2391 <https://github.com/ros2/rosbag2/issues/2391>`__)
* Repeat transient-local topics: Recorder, CLI, and Python bindings (`#2387 <https://github.com/ros2/rosbag2/issues/2387>`__)
* Suppress multi-threaded process warning from rosbag2 flake8 (`#2329 <https://github.com/ros2/rosbag2/issues/2329>`__)
* Remove deprecated arguments and options from ``ros2bag`` (`#2328 <https://github.com/ros2/rosbag2/issues/2328>`__)
* Implement circular logging by split count (``--max-bag-files``) (`#2218 <https://github.com/ros2/rosbag2/issues/2218>`__)
* Improve ``ros2 bag convert`` performance for fragment cutting and add ``--input-options`` (`#2325 <https://github.com/ros2/rosbag2/issues/2325>`__)
* Add static topics feature for recorder (`#2319 <https://github.com/ros2/rosbag2/issues/2319>`__)
* Add ``--max-cache-duration`` option for time-bounded snapshots (`#2289 <https://github.com/ros2/rosbag2/issues/2289>`__)
* Add ``rosbag2_storage_default_plugins`` to ``exec_depend`` of ``ros2bag`` (`#2227 <https://github.com/ros2/rosbag2/issues/2227>`__)
* Add ``input_serialization_format`` and ``output_serialization_format`` to ``RecordOptions``, deprecating ``rmw_serialization_format`` (`#2215 <https://github.com/ros2/rosbag2/issues/2215>`__)
* Publish messages lost statistics to 'events/messages_lost' topic (`#2150 <https://github.com/ros2/rosbag2/issues/2150>`__)
* Expose more of the player and recorder API to Python, and improve signal handling (`#2062 <https://github.com/ros2/rosbag2/issues/2062>`__)
* Fix setuptools deprecations (`#2087 <https://github.com/ros2/rosbag2/issues/2087>`__)
* Refactor Python player and recorder APIs into classes (`#2047 <https://github.com/ros2/rosbag2/issues/2047>`__)
* Upstream quality changes from Apex.AI part-2 (`#1924 <https://github.com/ros2/rosbag2/issues/1924>`__)
* Contributors: Christophe Bedard, Luke Sy, Michael Orlov, Tomoya Fujita, Tony Najjar, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2cli <https://github.com/ros2/ros2cli/tree/lyrical/ros2cli/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add RMW isolation fixture to enable discovery for ``rmw_zenoh_cpp`` tests (`#1216 <https://github.com/ros2/ros2cli/issues/1216>`__)
* Add support for fish (`#1211 <https://github.com/ros2/ros2cli/issues/1211>`__)
* Fix ``flake8`` (`#1215 <https://github.com/ros2/ros2cli/issues/1215>`__)
* Fix future flake8 regressions (`#1196 <https://github.com/ros2/ros2cli//issues/1196>`__)
* fix deprecated warning for action graph APIs. (`#1188 <https://github.com/ros2/ros2cli//issues/1188>`__)
* Enable always complete (`#1190 <https://github.com/ros2/ros2cli//issues/1190>`__)
* Add fzf-based interactive selection to ros2cli commands (`#1151 <https://github.com/ros2/ros2cli//issues/1151>`__)
* check for invalid ROS discovery configuration and print warning if ne… (`#1178 <https://github.com/ros2/ros2cli//issues/1178>`__)
* skip history and depth check for rmw_connextdds. (`#1064 <https://github.com/ros2/ros2cli/issues/1064>`__)
* Remove importlib packages (`#1117 <https://github.com/ros2/ros2cli/issues/1117>`__)
* add verbose in service-info verb (`#916 <https://github.com/ros2/ros2cli//issues/916>`__)
* Fix handling of empty ROS_DOMAIN_ID in ros2cli (`#1112 <https://github.com/ros2/ros2cli//issues/1112>`__)
* fix: Also catch a TimeoutError (`#1092 <https://github.com/ros2/ros2cli/issues/1092>`__)
* [ros2doctor] Add Action Report (`#1076 <https://github.com/ros2/ros2cli/issues/1076>`__)
* Use rmw_test_fixture to isolate ros2cli tests (`#1062 <https://github.com/ros2/ros2cli/issues/1062>`__)
* fix setuptools deprecations (`#1066 <https://github.com/ros2/ros2cli/issues/1066>`__)
* Add Service report similar to topic report (`#1059 <https://github.com/ros2/ros2cli/issues/1059>`__)
* Make sure to install py.typed files (`#1058 <https://github.com/ros2/ros2cli/issues/1058>`__)
* Export Typing information (`#1041 <https://github.com/ros2/ros2cli/issues/1041>`__)
* move QoS methods from ros2topic.api to ros2cli.qos. (`#1053 <https://github.com/ros2/ros2cli/issues/1053>`__)
* Assert HistoryQoS in test_ros2cli_daemon (`#1040 <https://github.com/ros2/ros2cli/issues/1040>`__)
* remove add_subparsers from ros2cli. (`#1032 <https://github.com/ros2/ros2cli/issues/1032>`__)
* Allow zenoh tests to run with multicast (`#992 <https://github.com/ros2/ros2cli/issues/992>`__)
* Contributors: Christophe Bedard, David V. Lu!!, Kaju-Bubanja, Lee, Mario Domínguez López, Michael Carlstrom, Michael Carroll, Minju, SPeak, Scott K Logan, Tomoya Fujita, Tony Najjar, Yuyuan Yuan, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2cli_common_extensions <https://github.com/ros2/ros2cli_common_extensions/tree/lyrical/ros2cli_common_extensions/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add dependency on ros2plugin in package.xml (`#13 <https://github.com/ros2/ros2cli_common_extensions/issues/13>`__)
* Update CMakeLists.txt (`#11 <https://github.com/ros2/ros2cli_common_extensions/issues/11>`__)
* Contributors: Maurice Alexander Purnawan, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2cli_test_interfaces <https://github.com/ros2/ros2cli/tree/lyrical/ros2cli_test_interfaces/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix cmake deprecation (`#1082 <https://github.com/ros2/ros2cli/issues/1082>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2component <https://github.com/ros2/ros2cli/tree/lyrical/ros2component/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix ``flake8`` (`#1215 <https://github.com/ros2/ros2cli/issues/1215>`__)
* Add timeout arguments to ``ros2 service call``, ``ros2 action send_goal``, ``ros2 component``, ``ros2 lifecycle``, and ``ros2 param`` (`#1185 <https://github.com/ros2/ros2cli/issues/1185>`__)
* Fix future flake8 regressions (`#1196 <https://github.com/ros2/ros2cli//issues/1196>`__)
* fix setuptools deprecations (`#1066 <https://github.com/ros2/ros2cli/issues/1066>`__)
* Make sure to install py.typed files (`#1058 <https://github.com/ros2/ros2cli/issues/1058>`__)
* Export Typing information (`#1041 <https://github.com/ros2/ros2cli/issues/1041>`__)
* Contributors: Christophe Bedard, Michael Carlstrom, Tomoya Fujita, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2doctor <https://github.com/ros2/ros2cli/tree/lyrical/ros2doctor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix future flake8 regressions (`#1196 <https://github.com/ros2/ros2cli//issues/1196>`__)
* Remove importlib packages (`#1117 <https://github.com/ros2/ros2cli/issues/1117>`__)
* Harden ros2doctor system calls. (`#1118 <https://github.com/ros2/ros2cli/issues/1118>`__)
* Add error handling when parsing package locally (`#1108 <https://github.com/ros2/ros2cli//issues/1108>`__)
* Fujitatomoya/clearup isolated ros2daemon (`#1098 <https://github.com/ros2/ros2cli/issues/1098>`__)
* [ros2doctor] Environment Report (`#1045 <https://github.com/ros2/ros2cli/issues/1045>`__)
* Restore environment variables after launch tests (`#1086 <https://github.com/ros2/ros2cli/issues/1086>`__)
* add warning notice for ros2 doctor --report. (`#1079 <https://github.com/ros2/ros2cli/issues/1079>`__)
* [ros2doctor] Add Action Report (`#1076 <https://github.com/ros2/ros2cli/issues/1076>`__)
* Use rmw_test_fixture to isolate ros2cli tests (`#1062 <https://github.com/ros2/ros2cli/issues/1062>`__)
* fix setuptools deprecations (`#1066 <https://github.com/ros2/ros2cli/issues/1066>`__)
* Add Service report similar to topic report (`#1059 <https://github.com/ros2/ros2cli/issues/1059>`__)
* Make sure to install py.typed files (`#1058 <https://github.com/ros2/ros2cli/issues/1058>`__)
* Export Typing information (`#1041 <https://github.com/ros2/ros2cli/issues/1041>`__)
* Fix stringifying InterfaceFlags when the flags are empty. (`#1026 <https://github.com/ros2/ros2cli/issues/1026>`__)
* Allow zenoh tests to run with multicast (`#992 <https://github.com/ros2/ros2cli/issues/992>`__)
* Skip QoS compatibility test on Zenoh (`#985 <https://github.com/ros2/ros2cli/issues/985>`__)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Christophe Bedard, Michael Carlstrom, Michael Carroll, Scott K Logan, Tomoya Fujita, mini-1235, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2interface <https://github.com/ros2/ros2cli/tree/lyrical/ros2interface/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add fzf-based interactive selection to ros2cli commands (`#1151 <https://github.com/ros2/ros2cli//issues/1151>`__)
* fix setuptools deprecations (`#1066 <https://github.com/ros2/ros2cli/issues/1066>`__)
* Make sure to install py.typed files (`#1058 <https://github.com/ros2/ros2cli/issues/1058>`__)
* Export Typing information (`#1041 <https://github.com/ros2/ros2cli/issues/1041>`__)
* Contributors: Christophe Bedard, Michael Carlstrom, Tony Najjar, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2launch <https://github.com/ros2/launch_ros/tree/lyrical/ros2launch/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* correct typos (`#524 <https://github.com/ros2/launch_ros//issues/524>`__)
* fix setuptools deprecations (`#475 <https://github.com/ros2/launch_ros/issues/475>`__)
* user control of log file base names, in ros2 launch (`#461 <https://github.com/ros2/launch_ros/issues/461>`__) Co-authored-by: Katherine Scott <katherineAScott@gmail.com>
* Contributors: Auguste Lalande, Tanishq Chaudhary, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2lifecycle <https://github.com/ros2/ros2cli/tree/lyrical/ros2lifecycle/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add timeout arguments to ``ros2 service call``, ``ros2 action send_goal``, ``ros2 component``, ``ros2 lifecycle``, and ``ros2 param`` (`#1185 <https://github.com/ros2/ros2cli/issues/1185>`__)
* ros2interface output the contents for each node. (`#1163 <https://github.com/ros2/ros2cli//issues/1163>`__)
* Fujitatomoya/clearup isolated ros2daemon (`#1098 <https://github.com/ros2/ros2cli/issues/1098>`__)
* Restore environment variables after launch tests (`#1086 <https://github.com/ros2/ros2cli/issues/1086>`__)
* Use rmw_test_fixture to isolate ros2cli tests (`#1062 <https://github.com/ros2/ros2cli/issues/1062>`__)
* fix setuptools deprecations (`#1066 <https://github.com/ros2/ros2cli/issues/1066>`__)
* Make sure to install py.typed files (`#1058 <https://github.com/ros2/ros2cli/issues/1058>`__)
* Relax the check from exact to partial match. (`#1055 <https://github.com/ros2/ros2cli/issues/1055>`__)
* Export Typing information (`#1041 <https://github.com/ros2/ros2cli/issues/1041>`__)
* Allow zenoh tests to run with multicast (`#992 <https://github.com/ros2/ros2cli/issues/992>`__)
* Contributors: Christophe Bedard, Michael Carlstrom, Michael Carroll, Scott K Logan, Tomoya Fujita, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2lifecycle_test_fixtures <https://github.com/ros2/ros2cli/tree/lyrical/ros2lifecycle_test_fixtures/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix cmake deprecation (`#1082 <https://github.com/ros2/ros2cli/issues/1082>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#973 <https://github.com/ros2/ros2cli/issues/973>`__)
* Contributors: Shane Loretz, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2multicast <https://github.com/ros2/ros2cli/tree/lyrical/ros2multicast/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix setuptools deprecations (`#1066 <https://github.com/ros2/ros2cli/issues/1066>`__)
* Make sure to install py.typed files (`#1058 <https://github.com/ros2/ros2cli/issues/1058>`__)
* Export Typing information (`#1041 <https://github.com/ros2/ros2cli/issues/1041>`__)
* Contributors: Christophe Bedard, Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2node <https://github.com/ros2/ros2cli/tree/lyrical/ros2node/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add fzf-based interactive selection to ros2cli commands (`#1151 <https://github.com/ros2/ros2cli//issues/1151>`__)
* Fujitatomoya/clearup isolated ros2daemon (`#1098 <https://github.com/ros2/ros2cli/issues/1098>`__)
* Restore environment variables after launch tests (`#1086 <https://github.com/ros2/ros2cli/issues/1086>`__)
* Use rmw_test_fixture to isolate ros2cli tests (`#1062 <https://github.com/ros2/ros2cli/issues/1062>`__)
* fix setuptools deprecations (`#1066 <https://github.com/ros2/ros2cli/issues/1066>`__)
* Make sure to install py.typed files (`#1058 <https://github.com/ros2/ros2cli/issues/1058>`__)
* Export Typing information (`#1041 <https://github.com/ros2/ros2cli/issues/1041>`__)
* Allow zenoh tests to run with multicast (`#992 <https://github.com/ros2/ros2cli/issues/992>`__)
* Contributors: Christophe Bedard, Michael Carlstrom, Michael Carroll, Scott K Logan, Tomoya Fujita, Tony Najjar, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2param <https://github.com/ros2/ros2cli/tree/lyrical/ros2param/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add timeout arguments to ``ros2 service call``, ``ros2 action send_goal``, ``ros2 component``, ``ros2 lifecycle``, and ``ros2 param`` (`#1185 <https://github.com/ros2/ros2cli/issues/1185>`__)
* ros2 param set /node_name <param1 value1 param2 value2...> support. (`#1204 <https://github.com/ros2/ros2cli//issues/1204>`__)
* ros2 param get /node_name <param1 param2 param3...> support. (`#1203 <https://github.com/ros2/ros2cli//issues/1203>`__)
* Add per-node timeout option to ros2 param list (`#1170 <https://github.com/ros2/ros2cli//issues/1170>`__)
* Fix Bash completion (`#1182 <https://github.com/ros2/ros2cli//issues/1182>`__)
* Add fzf-based interactive selection to ros2cli commands (`#1151 <https://github.com/ros2/ros2cli//issues/1151>`__)
* Support "ros2 param get <parameter>" across all nodes. (`#1174 <https://github.com/ros2/ros2cli//issues/1174>`__)
* Fix ParameterNameCompleter. (`#1172 <https://github.com/ros2/ros2cli//issues/1172>`__)
* Output node parameters upon each receipt (`#1162 <https://github.com/ros2/ros2cli//issues/1162>`__)
* skip test_verb_load_wildcard for rmw_connextdds. (`#1150 <https://github.com/ros2/ros2cli/issues/1150>`__)
* Fujitatomoya/clearup isolated ros2daemon (`#1098 <https://github.com/ros2/ros2cli/issues/1098>`__)
* Restore environment variables after launch tests (`#1086 <https://github.com/ros2/ros2cli/issues/1086>`__)
* Use rmw_test_fixture to isolate ros2cli tests (`#1062 <https://github.com/ros2/ros2cli/issues/1062>`__)
* fix setuptools deprecations (`#1066 <https://github.com/ros2/ros2cli/issues/1066>`__)
* Make sure to install py.typed files (`#1058 <https://github.com/ros2/ros2cli/issues/1058>`__)
* Relax the check from exact to partial match. (`#1055 <https://github.com/ros2/ros2cli/issues/1055>`__)
* Export Typing information (`#1041 <https://github.com/ros2/ros2cli/issues/1041>`__)
* fix misspelling. (`#1035 <https://github.com/ros2/ros2cli/issues/1035>`__)
* catch ConnectionRefusedError, so that it can fall back to DirectNode. (`#1014 <https://github.com/ros2/ros2cli/issues/1014>`__)
* fails the test properly to avoid TypeError exception. (`#1016 <https://github.com/ros2/ros2cli/issues/1016>`__)
* Fix loading parameter behavior from yaml file (`#864 <https://github.com/ros2/ros2cli/issues/864>`__)
* Allow zenoh tests to run with multicast (`#992 <https://github.com/ros2/ros2cli/issues/992>`__)
* Contributors: Barry Xu, Christophe Bedard, Michael Carlstrom, Michael Carroll, Scott K Logan, Taiga Arai, Tomoya Fujita, Tony Najjar, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2pkg <https://github.com/ros2/ros2cli/tree/lyrical/ros2pkg/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Remove "rclrs" duplicate dependency (`#1197 <https://github.com/ros2/ros2cli//issues/1197>`__)
* Fix future flake8 regressions (`#1196 <https://github.com/ros2/ros2cli//issues/1196>`__)
* Add Native ROS2 Rust Package Create Capability (`#1107 <https://github.com/ros2/ros2cli/issues/1107>`__)
* Remove importlib packages (`#1117 <https://github.com/ros2/ros2cli/issues/1117>`__)
* add mypy (`#1109 <https://github.com/ros2/ros2cli//issues/1109>`__)
* fix cmake deprecation (`#1082 <https://github.com/ros2/ros2cli/issues/1082>`__)
* fix setuptools deprecations (`#1066 <https://github.com/ros2/ros2cli/issues/1066>`__)
* Make sure to install py.typed files (`#1058 <https://github.com/ros2/ros2cli/issues/1058>`__)
* Reduce boilerplate in install(TARGETS for library (`#1056 <https://github.com/ros2/ros2cli/issues/1056>`__)
* Export Typing information (`#1041 <https://github.com/ros2/ros2cli/issues/1041>`__)
* Use modern C++17 syntax. (`#982 <https://github.com/ros2/ros2cli/issues/982>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#973 <https://github.com/ros2/ros2cli/issues/973>`__)
* Try to use the git global user.name for maintainer-name (`#968 <https://github.com/ros2/ros2cli/issues/968>`__)
* Update minimum CMake version CMakeLists.txt.em (`#969 <https://github.com/ros2/ros2cli/issues/969>`__)
* Contributors: Bartlomiej Styczen, Christophe Bedard, Larry Gezelius, Michael Carlstrom, Parth Patel, Sebastian Castro, Shane Loretz, Shynur, Silvio Traversaro, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2plugin <https://github.com/ros/pluginlib/tree/lyrical/ros2plugin/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Implement package option (`#293 <https://github.com/ros/pluginlib/issues/293>`__)
* Improve logging when unable to parse the plugin (`#285 <https://github.com/ros/pluginlib/issues/285>`__)
* Add ros2plugin (`#165 <https://github.com/ros/pluginlib/issues/165>`__)
* Contributors: Alejandro Hernández Cordero, Jeremie Deray, mini-1235


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2run <https://github.com/ros2/ros2cli/tree/lyrical/ros2run/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix Bash completion (`#1182 <https://github.com/ros2/ros2cli//issues/1182>`__)
* Add fzf-based interactive selection to ros2cli commands (`#1151 <https://github.com/ros2/ros2cli//issues/1151>`__)
* fix setuptools deprecations (`#1066 <https://github.com/ros2/ros2cli/issues/1066>`__)
* Make sure to install py.typed files (`#1058 <https://github.com/ros2/ros2cli/issues/1058>`__)
* Export Typing information (`#1041 <https://github.com/ros2/ros2cli/issues/1041>`__)
* Add signal handler SIGIN/SIGTERM to ros2run (`#899 <https://github.com/ros2/ros2cli/issues/899>`__)
* Contributors: Christophe Bedard, Michael Carlstrom, Tomoya Fujita, Tony Najjar, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2service <https://github.com/ros2/ros2cli/tree/lyrical/ros2service/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add timeout arguments to ``ros2 service call``, ``ros2 action send_goal``, ``ros2 component``, ``ros2 lifecycle``, and ``ros2 param`` (`#1185 <https://github.com/ros2/ros2cli/issues/1185>`__)
* Fix Bash completion (`#1182 <https://github.com/ros2/ros2cli//issues/1182>`__)
* Add fzf-based interactive selection to ros2cli commands (`#1151 <https://github.com/ros2/ros2cli//issues/1151>`__)
* add verbose in service-info verb (`#916 <https://github.com/ros2/ros2cli//issues/916>`__)
* Fujitatomoya/clearup isolated ros2daemon (`#1098 <https://github.com/ros2/ros2cli/issues/1098>`__)
* Restore environment variables after launch tests (`#1086 <https://github.com/ros2/ros2cli/issues/1086>`__)
* Use rmw_test_fixture to isolate ros2cli tests (`#1062 <https://github.com/ros2/ros2cli/issues/1062>`__)
* fix setuptools deprecations (`#1066 <https://github.com/ros2/ros2cli/issues/1066>`__)
* Make sure to install py.typed files (`#1058 <https://github.com/ros2/ros2cli/issues/1058>`__)
* Relax the check from exact to partial match. (`#1055 <https://github.com/ros2/ros2cli/issues/1055>`__)
* Export Typing information (`#1041 <https://github.com/ros2/ros2cli/issues/1041>`__)
* move QoS methods from ros2topic.api to ros2cli.qos. (`#1053 <https://github.com/ros2/ros2cli/issues/1053>`__)
* add QoS option to ros2service/ros2action echo commands. (`#1036 <https://github.com/ros2/ros2cli/issues/1036>`__)
* Use ``get_service`` in ``ros2service call`` (`#994 <https://github.com/ros2/ros2cli/issues/994>`__)
* Allow zenoh tests to run with multicast (`#992 <https://github.com/ros2/ros2cli/issues/992>`__)
* Support QoS options for ``ros2 service call`` (`#966 <https://github.com/ros2/ros2cli/issues/966>`__)
* Contributors: Christophe Bedard, Lee, Michael Carlstrom, Michael Carroll, Minju, Scott K Logan, Tomoya Fujita, Tony Najjar, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2test <https://github.com/ros2/ros_testing/tree/lyrical/ros2test/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix setuptools deprecations (`#16 <https://github.com/ros2/ros_testing/issues/16>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2topic <https://github.com/ros2/ros2cli/tree/lyrical/ros2topic/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Improve test isolation and suppress Connext license noise (`#1225 <https://github.com/ros2/ros2cli/issues/1225>`__)
* Add fzf-based interactive selection to ros2cli commands (`#1151 <https://github.com/ros2/ros2cli//issues/1151>`__)
* add "--all/-a" option to "ros2 topic bw" with screen refresh. (`#1130 <https://github.com/ros2/ros2cli/issues/1130>`__)
* return explicitly from internal functions. (`#1128 <https://github.com/ros2/ros2cli/issues/1128>`__)
* support multiple topics for "ros2 topic bw". (`#1124 <https://github.com/ros2/ros2cli/issues/1124>`__)
* add "--all/-a" option to "ros2 topic hz" with screen refresh. (`#1122 <https://github.com/ros2/ros2cli/issues/1122>`__)
* Fujitatomoya/clearup isolated ros2daemon (`#1098 <https://github.com/ros2/ros2cli/issues/1098>`__)
* wait for the publisher before test command is executed. (`#1094 <https://github.com/ros2/ros2cli/issues/1094>`__)
* Enable test isolation on a few remaining ros2topic tests (`#1087 <https://github.com/ros2/ros2cli/issues/1087>`__)
* Restore environment variables after launch tests (`#1086 <https://github.com/ros2/ros2cli/issues/1086>`__)
* Use rmw_test_fixture to isolate ros2cli tests (`#1062 <https://github.com/ros2/ros2cli/issues/1062>`__)
* fix setuptools deprecations (`#1066 <https://github.com/ros2/ros2cli/issues/1066>`__)
* Make sure to install py.typed files (`#1058 <https://github.com/ros2/ros2cli/issues/1058>`__)
* Export Typing information (`#1041 <https://github.com/ros2/ros2cli/issues/1041>`__)
* move QoS methods from ros2topic.api to ros2cli.qos. (`#1053 <https://github.com/ros2/ros2cli/issues/1053>`__)
* Custom Completion Finder for fetching topic prototype (`#995 <https://github.com/ros2/ros2cli/issues/995>`__)
* Documented now and auto keywords (`#1008 <https://github.com/ros2/ros2cli/issues/1008>`__)
* Conditional deserialization of message for ``ros2 topic hz`` (`#1005 <https://github.com/ros2/ros2cli/issues/1005>`__)
* Enable ``ros2 topic echo`` with entries of array fields (`#996 <https://github.com/ros2/ros2cli/issues/996>`__)
* Allow zenoh tests to run with multicast (`#992 <https://github.com/ros2/ros2cli/issues/992>`__)
* Adapt tests to Zenoh (`#988 <https://github.com/ros2/ros2cli/issues/988>`__)
* Adjust topic hz and bw command description (`#987 <https://github.com/ros2/ros2cli/issues/987>`__)
* Add support for topic QOS for ros2topic bw, delay and hz (`#935 <https://github.com/ros2/ros2cli/issues/935>`__)
* Start the simulation from 1 second for the test (`#975 <https://github.com/ros2/ros2cli/issues/975>`__)
* Support QoS options for ``ros2 service call`` (`#966 <https://github.com/ros2/ros2cli/issues/966>`__)
* Support ros2 topic pub yaml file input (`#925 <https://github.com/ros2/ros2cli/issues/925>`__)
* Contributors: Alejandro Hernández Cordero, Anthony Welte, Christophe Bedard, Fabian Thomsen, Florencia, Kostubh Khandelwal, Leander Stephen D'Souza, Martin Pecka, Michael Carlstrom, Michael Carroll, Scott K Logan, Tomoya Fujita, Tony Najjar, mosfet80, nomumu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2trace <https://github.com/ros2/ros2_tracing/tree/lyrical/ros2trace/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Ignore A0005 (`#237 <https://github.com/ros2/ros2_tracing/issues/237>`__)
* Update trace command's doc-string (`#213 <https://github.com/ros2/ros2_tracing/issues/213>`__)
* Allow creating snapshot sessions (`#195 <https://github.com/ros2/ros2_tracing/issues/195>`__)
* Add support for starting tracing at runtime (`#191 <https://github.com/ros2/ros2_tracing/issues/191>`__)
* fix setuptools deprecation (`#189 <https://github.com/ros2/ros2_tracing/issues/189>`__)
* Address typing issues reported by mypy in tracetools_launch (`#184 <https://github.com/ros2/ros2_tracing/issues/184>`__)
* Contributors: Christophe Bedard, Michael Carlstrom, Shravan Deva, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros_environment <https://github.com/ros/ros_environment/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Change default ROS_DISTRO from 'rolling' to 'lyrical'
* fix cmake deprecation (`#42 <https://github.com/ros/ros_environment/issues/42>`__)
* Remove CODEOWNERS. (`#40 <https://github.com/ros/ros_environment/issues/40>`__)
* Contributors: Chris Lalancette, Shane Loretz, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros_testing <https://github.com/ros2/ros_testing/tree/lyrical/ros_testing/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix cmake deprecation (`#17 <https://github.com/ros2/ros_testing/issues/17>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2 <https://github.com/ros2/rosbag2/tree/lyrical/rosbag2/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix CMAKE deprecation (`#2067 <https://github.com/ros2/rosbag2/issues/2067>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_compression <https://github.com/ros2/rosbag2/tree/lyrical/rosbag2_compression/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add validation for empty file path in compression process (`#2398 <https://github.com/ros2/rosbag2/issues/2398>`__)
* Fix a possible race condition in compression writer on close (`#2362 <https://github.com/ros2/rosbag2/issues/2362>`__)
* Update Rosbag2 filename format to ``index+name+timestamp`` (`#2265 <https://github.com/ros2/rosbag2/issues/2265>`__)
* Implement circular logging by split count (``--max-bag-files``) (`#2218 <https://github.com/ros2/rosbag2/issues/2218>`__)
* Make topics persistent between writer's close() and open() API calls (`#2229 <https://github.com/ros2/rosbag2/issues/2229>`__)
* Address recorder test flakiness by increasing cache size (`#2203 <https://github.com/ros2/rosbag2/issues/2203>`__)
* Fix CMAKE deprecation (`#2067 <https://github.com/ros2/rosbag2/issues/2067>`__)
* Add message loss statistics callbacks and logging (`#2039 <https://github.com/ros2/rosbag2/issues/2039>`__)
* Introduce new ``BaseWriteInterface`` methods ``write_messages`` and ``write_message`` to provide operation status, deprecating old write APIs (`#2030 <https://github.com/ros2/rosbag2/issues/2030>`__)
* Bugfix: ``ros2 bag convert`` dropping messages with compression mode message (`#1975 <https://github.com/ros2/rosbag2/issues/1975>`__)
* Contributors: Daisuke Sato, DangitBen, Luke Sy, Michael Orlov, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_compression_zstd <https://github.com/ros2/rosbag2/tree/lyrical/rosbag2_compression_zstd/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Replace ``zstd_vendor`` with ``zstd_cmake_module`` (`#2166 <https://github.com/ros2/rosbag2/issues/2166>`__)
* Fix CMAKE deprecation (`#2067 <https://github.com/ros2/rosbag2/issues/2067>`__)
* Contributors: Alejandro Hernández Cordero, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_cpp <https://github.com/ros2/rosbag2/tree/lyrical/rosbag2_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Removed clang warning (`#2404 <https://github.com/ros2/rosbag2/issues/2404>`__)
* Implement ``transient-local topic`` repetition for Writer API and split/snapshot integration (`#2386 <https://github.com/ros2/rosbag2/issues/2386>`__)
* Add ``TransientLocalMessagesCache`` and ``RecordOptions`` for repeating transient-local topics (`#2385 <https://github.com/ros2/rosbag2/issues/2385>`__)
* Use new ROSIDL aggregate CMake target (`#2384 <https://github.com/ros2/rosbag2/issues/2384>`__)
* Fix a possible race condition in compression writer on close (`#2362 <https://github.com/ros2/rosbag2/issues/2362>`__)
* Fix incorrect serialization format in metadata (`#2372 <https://github.com/ros2/rosbag2/issues/2372>`__)
* Update Rosbag2 filename format to ``index+name+timestamp`` (`#2265 <https://github.com/ros2/rosbag2/issues/2265>`__)
* Support relative includes for IDL in local message definition (`#2241 <https://github.com/ros2/rosbag2/issues/2241>`__)
* Implement circular logging by split count (``--max-bag-files``) (`#2218 <https://github.com/ros2/rosbag2/issues/2218>`__)
* Add ``--max-cache-duration`` option for time-bounded snapshots (`#2289 <https://github.com/ros2/rosbag2/issues/2289>`__)
* Workaround flaky ``bagsize_split_is_at_least_specified_size`` test (`#2311 <https://github.com/ros2/rosbag2/issues/2311>`__)
* Incorporate upstream minor fixes from Apex.AI (`#2240 <https://github.com/ros2/rosbag2/issues/2240>`__)
* Update deprecated ament_index_cpp API (`#2268 <https://github.com/ros2/rosbag2/issues/2268>`__)
* Make topics persistent between writer's close() and open() API calls (`#2229 <https://github.com/ros2/rosbag2/issues/2229>`__)
* Add nullptr check when pushing new messages to the message cache (`#2219 <https://github.com/ros2/rosbag2/issues/2219>`__)
* Address recorder test flakiness by increasing cache size (`#2203 <https://github.com/ros2/rosbag2/issues/2203>`__)
* Log reasoning for not found message definition only in debug log (`#2183 <https://github.com/ros2/rosbag2/issues/2183>`__)
* Improve error handling in rosbag2_cpp with null checks and exception throwing (`#2127 <https://github.com/ros2/rosbag2/issues/2127>`__)
* Add null pointer checks in ``Reader`` constructor and ``open()`` method (`#2135 <https://github.com/ros2/rosbag2/issues/2135>`__)
* Use ``rclcpp typesupport helpers`` in ``rosbag2_cpp`` (`#2017 <https://github.com/ros2/rosbag2/issues/2017>`__)
* Fix callback not called for MESSAGES_LOST event (`#2105 <https://github.com/ros2/rosbag2/issues/2105>`__)
* Improve recorder's MessageCache performance (`#2104 <https://github.com/ros2/rosbag2/issues/2104>`__)
* Fix reindex duration bug when bag file durations overlap (`#2036 <https://github.com/ros2/rosbag2/issues/2036>`__)
* Fix CMAKE deprecation (`#2067 <https://github.com/ros2/rosbag2/issues/2067>`__)
* Add support for searching message definitions in nested subdirectories (`#2055 <https://github.com/ros2/rosbag2/issues/2055>`__)
* Add message loss statistics callbacks and logging (`#2039 <https://github.com/ros2/rosbag2/issues/2039>`__)
* Use cache to determine action interface inner types (`#2052 <https://github.com/ros2/rosbag2/issues/2052>`__)
* Fix service/action message definition issue (`#2041 <https://github.com/ros2/rosbag2/issues/2041>`__)
* Introduce new ``BaseWriteInterface`` methods ``write_messages`` and ``write_message`` to provide operation status, deprecating old write APIs (`#2030 <https://github.com/ros2/rosbag2/issues/2030>`__)
* Improve message publishing timing by avoiding sporadic wakeups and fixing incorrect intervals on player start (`#2025 <https://github.com/ros2/rosbag2/issues/2025>`__)
* Upstream quality changes from Apex.AI part-2 (`#1924 <https://github.com/ros2/rosbag2/issues/1924>`__)
* Address clang warning in the ``TimeControllerClock::wakeup()`` (`#1962 <https://github.com/ros2/rosbag2/issues/1962>`__)
* Contributors: Alejandro Hernández Cordero, Barry Xu, Christophe Bedard, Chui Vanfleet, Daisuke Sato, Emerson Knapp, Hunter L. Allen, José Faria, Luke Sy, Michael Orlov, Tomoya Fujita, Tony Najjar, YuJin Hong, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_examples_cpp <https://github.com/ros2/rosbag2/tree/lyrical/rosbag2_examples/rosbag2_examples_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#2384 <https://github.com/ros2/rosbag2/issues/2384>`__)
* Update subscription callback signatures (`#2225 <https://github.com/ros2/rosbag2/issues/2225>`__)
* Fix CMAKE deprecation (`#2067 <https://github.com/ros2/rosbag2/issues/2067>`__)
* Add examples for compressing bag files (`#1956 <https://github.com/ros2/rosbag2/issues/1956>`__)
* Contributors: Emerson Knapp, Maxime Fleury, mini-1235, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_examples_py <https://github.com/ros2/rosbag2/tree/lyrical/rosbag2_examples/rosbag2_examples_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix setuptools deprecation by removing ``tests_require`` (`#2092 <https://github.com/ros2/rosbag2/issues/2092>`__)
* Add examples for compressing bag files (`#1956 <https://github.com/ros2/rosbag2/issues/1956>`__)
* Upstream quality changes from Apex.AI part-2 (`#1924 <https://github.com/ros2/rosbag2/issues/1924>`__)
* Add a simple example showing how to convert bags to the csv file (`#1974 <https://github.com/ros2/rosbag2/issues/1974>`__)
* Contributors: Christophe Bedard, Maxime Fleury, Michael Orlov, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_interfaces <https://github.com/ros2/rosbag2/tree/lyrical/rosbag2_interfaces/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add support for time based Resume service (`#2357 <https://github.com/ros2/rosbag2/issues/2357>`__)
* Allow pause/resume service calls while not in recording (`#2349 <https://github.com/ros2/rosbag2/issues/2349>`__)
* Implement delayed and time-based recorder and player services, adding new bag split modes (`#2330 <https://github.com/ros2/rosbag2/issues/2330>`__)
* Add error return code to the ``~/stop`` service request (`#2312 <https://github.com/ros2/rosbag2/issues/2312>`__)
* Add Record, Stop, StartDiscovery, StopDiscovery, and IsDiscoveryRunning services for Recorder (`#2248 <https://github.com/ros2/rosbag2/issues/2248>`__)
* Publish messages lost statistics to 'events/messages_lost' topic (`#2150 <https://github.com/ros2/rosbag2/issues/2150>`__)
* Fix CMAKE deprecation (`#2067 <https://github.com/ros2/rosbag2/issues/2067>`__)
* Contributors: Michael Orlov, carlos-apex, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_performance_benchmarking <https://github.com/ros2/rosbag2/tree/lyrical/rosbag2_performance/rosbag2_performance_benchmarking/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#2384 <https://github.com/ros2/rosbag2/issues/2384>`__)
* Remove unnecessary dependencies on ``yaml_cpp_vendor`` (`#2353 <https://github.com/ros2/rosbag2/issues/2353>`__)
* Fix warning by initializing ``number_of_threads`` (`#2121 <https://github.com/ros2/rosbag2/issues/2121>`__)
* Enable ``rosbag2_performance_benchmarking`` package to be built by default (`#2093 <https://github.com/ros2/rosbag2/issues/2093>`__)
* Fix performance benchmarking data generation and environment variable handling (`#2078 <https://github.com/ros2/rosbag2/issues/2078>`__)
* Fix failure in ``benchmark_launch`` when calling ``Process.wait()`` twice (`#2076 <https://github.com/ros2/rosbag2/issues/2076>`__)
* Fix incorrect results from ``prosbag2_performance_benchmarking`` for high-frequency topics (`#2077 <https://github.com/ros2/rosbag2/issues/2077>`__)
* Fix CMAKE deprecation (`#2067 <https://github.com/ros2/rosbag2/issues/2067>`__)
* Upstream quality changes from Apex.AI part-2 (`#1924 <https://github.com/ros2/rosbag2/issues/1924>`__)
* Contributors: Chris Lalancette, Christophe Bedard, Cristóbal Arroyo, Emerson Knapp, Michael Orlov, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_performance_benchmarking_msgs <https://github.com/ros2/rosbag2/tree/lyrical/rosbag2_performance/rosbag2_performance_benchmarking_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Enable ``rosbag2_performance_benchmarking`` package to be built by default (`#2093 <https://github.com/ros2/rosbag2/issues/2093>`__)
* Fix CMAKE deprecation (`#2067 <https://github.com/ros2/rosbag2/issues/2067>`__)
* Contributors: Michael Orlov, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_py <https://github.com/ros2/rosbag2/tree/lyrical/rosbag2_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add ``--repeat-all-transient-local`` flag for automatic transient-local topic detection (`#2391 <https://github.com/ros2/rosbag2/issues/2391>`__)
* Repeat transient-local topics: Recorder, CLI, and Python bindings (`#2387 <https://github.com/ros2/rosbag2/issues/2387>`__)
* Implement circular logging by split count (``--max-bag-files``) (`#2218 <https://github.com/ros2/rosbag2/issues/2218>`__)
* Move to ``build_depend`` (`#2332 <https://github.com/ros2/rosbag2/issues/2332>`__)
* Improve ``ros2 bag convert`` performance for fragment cutting and add ``--input-options`` (`#2325 <https://github.com/ros2/rosbag2/issues/2325>`__)
* Add static topics feature for recorder (`#2319 <https://github.com/ros2/rosbag2/issues/2319>`__)
* Add ``--max-cache-duration`` option for time-bounded snapshots (`#2289 <https://github.com/ros2/rosbag2/issues/2289>`__)
* Incorporate upstream minor fixes from Apex.AI (`#2240 <https://github.com/ros2/rosbag2/issues/2240>`__)
* Add ``input_serialization_format`` and ``output_serialization_format`` to ``RecordOptions``, deprecating ``rmw_serialization_format`` (`#2215 <https://github.com/ros2/rosbag2/issues/2215>`__)
* Use pybind11 from deb or pixi (`#2154 <https://github.com/ros2/rosbag2/issues/2154>`__)
* Publish messages lost statistics to 'events/messages_lost' topic (`#2150 <https://github.com/ros2/rosbag2/issues/2150>`__)
* Ensure test topic discovery by recorder in ``rosbag2_py`` test (`#2132 <https://github.com/ros2/rosbag2/issues/2132>`__)
* Fix CMake list append for env vars in rosbag2_py with clang (`#2116 <https://github.com/ros2/rosbag2/issues/2116>`__)
* Add public API for player's starting time and playback duration (`#2095 <https://github.com/ros2/rosbag2/issues/2095>`__)
* Expose more of the player and recorder API to Python, and improve signal handling (`#2062 <https://github.com/ros2/rosbag2/issues/2062>`__)
* Add ``send_timestamp`` to Python interface for reading serialized messages (`#2061 <https://github.com/ros2/rosbag2/issues/2061>`__)
* Refactor Python player and recorder APIs into classes (`#2047 <https://github.com/ros2/rosbag2/issues/2047>`__)
* Fix service/action message definition issue (`#2041 <https://github.com/ros2/rosbag2/issues/2041>`__)
* Upstream quality changes from Apex.AI part-2 (`#1924 <https://github.com/ros2/rosbag2/issues/1924>`__)
* Bugfix: ``ros2 bag convert`` dropping messages with compression mode message (`#1975 <https://github.com/ros2/rosbag2/issues/1975>`__)
* Contributors: Alejandro Hernández Cordero, Barry Xu, Christophe Bedard, DangitBen, Luke Sy, Michael Carlstrom, Michael Orlov, Om Shivam Verma, Tony Najjar


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_storage <https://github.com/ros2/rosbag2/tree/lyrical/rosbag2_storage/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add ``TransientLocalMessagesCache`` and ``RecordOptions`` for repeating transient-local topics (`#2385 <https://github.com/ros2/rosbag2/issues/2385>`__)
* Implement delayed and time-based recorder and player services, adding new bag split modes (`#2330 <https://github.com/ros2/rosbag2/issues/2330>`__)
* Implement circular logging by split count (``--max-bag-files``) (`#2218 <https://github.com/ros2/rosbag2/issues/2218>`__)
* Improve ``ros2 bag convert`` performance for fragment cutting and add ``--input-options`` (`#2325 <https://github.com/ros2/rosbag2/issues/2325>`__)
* Add ``--max-cache-duration`` option for time-bounded snapshots (`#2289 <https://github.com/ros2/rosbag2/issues/2289>`__)
* Throw ``YAML::Exception`` during conversion if the data type mismatches (`#2262 <https://github.com/ros2/rosbag2/issues/2262>`__)
* Fix decoder and encode mismatch in YAML deserialization (`#2277 <https://github.com/ros2/rosbag2/issues/2277>`__)
* Incorporate upstream minor fixes from Apex.AI (`#2240 <https://github.com/ros2/rosbag2/issues/2240>`__)
* Fix memory leak on ``make_empty_serialized_message()`` (`#2253 <https://github.com/ros2/rosbag2/issues/2253>`__)
* Fix CMAKE deprecation (`#2067 <https://github.com/ros2/rosbag2/issues/2067>`__)
* Introduce new ``BaseWriteInterface`` methods ``write_messages`` and ``write_message`` to provide operation status, deprecating old write APIs (`#2030 <https://github.com/ros2/rosbag2/issues/2030>`__)
* Fix undefined behavior in the ``rosbag2_storage`` and ``rosbag2_storage_sqlite3`` packages (`#1997 <https://github.com/ros2/rosbag2/issues/1997>`__)
* Use DDS queue depth for subscriptions as a maximum value across publishers (`#1960 <https://github.com/ros2/rosbag2/issues/1960>`__)
* Contributors: Luke Sy, Michael Orlov, Tomoya Fujita, Tony Najjar, carlos-apex, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_storage_default_plugins <https://github.com/ros2/rosbag2/tree/lyrical/rosbag2_storage_default_plugins/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix CMAKE deprecation (`#2067 <https://github.com/ros2/rosbag2/issues/2067>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_storage_mcap <https://github.com/ros2/rosbag2/tree/lyrical/rosbag2_storage_mcap/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#2384 <https://github.com/ros2/rosbag2/issues/2384>`__)
* Remove unnecessary dependencies on ``yaml_cpp_vendor`` (`#2353 <https://github.com/ros2/rosbag2/issues/2353>`__)
* Fix MCAPStorage::seek(time) to advance when timestamp matches current time (`#2157 <https://github.com/ros2/rosbag2/issues/2157>`__)
* Fix CMAKE deprecation (`#2067 <https://github.com/ros2/rosbag2/issues/2067>`__)
* Introduce new ``BaseWriteInterface`` methods ``write_messages`` and ``write_message`` to provide operation status, deprecating old write APIs (`#2030 <https://github.com/ros2/rosbag2/issues/2030>`__)
* Update ``index.ros.org/p/`` links for ``rosbag2_storage_mcap`` (`#2034 <https://github.com/ros2/rosbag2/issues/2034>`__)
* Contributors: Chris Lalancette, Christophe Bedard, Emerson Knapp, Michael Orlov, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_storage_sqlite3 <https://github.com/ros2/rosbag2/tree/lyrical/rosbag2_storage_sqlite3/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#2384 <https://github.com/ros2/rosbag2/issues/2384>`__)
* Implement circular logging by split count (``--max-bag-files``) (`#2218 <https://github.com/ros2/rosbag2/issues/2218>`__)
* Add ``--max-cache-duration`` option for time-bounded snapshots (`#2289 <https://github.com/ros2/rosbag2/issues/2289>`__)
* Fix vulnerable string concatenation by using parameterized queries (`#2290 <https://github.com/ros2/rosbag2/issues/2290>`__)
* Remove sqlite3_vendor (`#2164 <https://github.com/ros2/rosbag2/issues/2164>`__)
* Fix CMAKE deprecation (`#2067 <https://github.com/ros2/rosbag2/issues/2067>`__)
* Introduce new ``BaseWriteInterface`` methods ``write_messages`` and ``write_message`` to provide operation status, deprecating old write APIs (`#2030 <https://github.com/ros2/rosbag2/issues/2030>`__)
* Fix undefined behavior in the ``rosbag2_storage`` and ``rosbag2_storage_sqlite3`` packages (`#1997 <https://github.com/ros2/rosbag2/issues/1997>`__)
* Upstream quality changes from Apex.AI part-2 (`#1924 <https://github.com/ros2/rosbag2/issues/1924>`__)
* Contributors: Alejandro Hernández Cordero, Christophe Bedard, Emerson Knapp, Luke Sy, Michael Orlov, Tomoya Fujita, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_test_common <https://github.com/ros2/rosbag2/tree/lyrical/rosbag2_test_common/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Reduce flakiness in rosbag2 recorder end-to-end tests (`#2370 <https://github.com/ros2/rosbag2/issues/2370>`__)
* Use new ROSIDL aggregate CMake target (`#2384 <https://github.com/ros2/rosbag2/issues/2384>`__)
* Update Rosbag2 filename format to ``index+name+timestamp`` (`#2265 <https://github.com/ros2/rosbag2/issues/2265>`__)
* Update subscription callback signatures (`#2225 <https://github.com/ros2/rosbag2/issues/2225>`__)
* Fix CMAKE deprecation (`#2067 <https://github.com/ros2/rosbag2/issues/2067>`__)
* Address test flakiness by waiting for executor spin (`#2001 <https://github.com/ros2/rosbag2/issues/2001>`__)
* Upstream quality changes from Apex.AI part-2 (`#1924 <https://github.com/ros2/rosbag2/issues/1924>`__)
* Use DDS queue depth for subscriptions as a maximum value across publishers (`#1960 <https://github.com/ros2/rosbag2/issues/1960>`__)
* Contributors: Christophe Bedard, Daisuke Sato, Emerson Knapp, Michael Orlov, mini-1235, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_test_msgdefs <https://github.com/ros2/rosbag2/tree/lyrical/rosbag2_test_msgdefs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Support relative includes for IDL in local message definition (`#2241 <https://github.com/ros2/rosbag2/issues/2241>`__)
* Fix CMAKE deprecation (`#2067 <https://github.com/ros2/rosbag2/issues/2067>`__)
* Add support for searching message definitions in nested subdirectories (`#2055 <https://github.com/ros2/rosbag2/issues/2055>`__)
* Contributors: Hunter L. Allen, Michael Orlov, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_tests <https://github.com/ros2/rosbag2/tree/lyrical/rosbag2_tests/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#2384 <https://github.com/ros2/rosbag2/issues/2384>`__)
* Update Rosbag2 filename format to ``index+name+timestamp`` (`#2265 <https://github.com/ros2/rosbag2/issues/2265>`__)
* Add static topics feature for recorder (`#2319 <https://github.com/ros2/rosbag2/issues/2319>`__)
* Add ``--max-cache-duration`` option for time-bounded snapshots (`#2289 <https://github.com/ros2/rosbag2/issues/2289>`__)
* Workaround flaky ``bagsize_split_is_at_least_specified_size`` test (`#2311 <https://github.com/ros2/rosbag2/issues/2311>`__)
* Add ``input_serialization_format`` and ``output_serialization_format`` to ``RecordOptions``, deprecating ``rmw_serialization_format`` (`#2215 <https://github.com/ros2/rosbag2/issues/2215>`__)
* Address recorder test flakiness by increasing cache size (`#2203 <https://github.com/ros2/rosbag2/issues/2203>`__)
* Use ``rclcpp typesupport helpers`` in ``rosbag2_cpp`` (`#2017 <https://github.com/ros2/rosbag2/issues/2017>`__)
* Expose more of the player and recorder API to Python, and improve signal handling (`#2062 <https://github.com/ros2/rosbag2/issues/2062>`__)
* Fix CMAKE deprecation (`#2067 <https://github.com/ros2/rosbag2/issues/2067>`__)
* Fix deadlocks in Rosbag2 player when calling stop API (`#2057 <https://github.com/ros2/rosbag2/issues/2057>`__)
* Introduce new ``BaseWriteInterface`` methods ``write_messages`` and ``write_message`` to provide operation status, deprecating old write APIs (`#2030 <https://github.com/ros2/rosbag2/issues/2030>`__)
* Address test flakiness by waiting for executor spin (`#2001 <https://github.com/ros2/rosbag2/issues/2001>`__)
* Upstream quality changes from Apex.AI part-2 (`#1924 <https://github.com/ros2/rosbag2/issues/1924>`__)
* Contributors: Christophe Bedard, Daisuke Sato, Emerson Knapp, Michael Orlov, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_transport <https://github.com/ros2/rosbag2/tree/lyrical/rosbag2_transport/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Apply /bigobj to all MSVC builds in rosbag2_transport (`#2424 <https://github.com/ros2/rosbag2/issues/2424>`__) (`#2428 <https://github.com/ros2/rosbag2/issues/2428>`__)
* fix: Fixed compile errors in rosbag2_transport for MSVC 2022 and C++20 (`#2407 <https://github.com/ros2/rosbag2/issues/2407>`__)
* Fix QoS overrides ignored when topic name has no leading slash (`#2394 <https://github.com/ros2/rosbag2/issues/2394>`__)
* Refactor transient-local topic detection and logging in RecorderImpl (`#2395 <https://github.com/ros2/rosbag2/issues/2395>`__)
* Fix race condition in ``RecordSrvsSimTimeTest`` by waiting for clock subscriber (`#2396 <https://github.com/ros2/rosbag2/issues/2396>`__)
* Add ``--repeat-all-transient-local`` flag for automatic transient-local topic detection (`#2391 <https://github.com/ros2/rosbag2/issues/2391>`__)
* Repeat transient-local topics: Recorder, CLI, and Python bindings (`#2387 <https://github.com/ros2/rosbag2/issues/2387>`__)
* Implement ``transient-local topic`` repetition for Writer API and split/snapshot integration (`#2386 <https://github.com/ros2/rosbag2/issues/2386>`__)
* Add ``TransientLocalMessagesCache`` and ``RecordOptions`` for repeating transient-local topics (`#2385 <https://github.com/ros2/rosbag2/issues/2385>`__)
* Use new ROSIDL aggregate CMake target (`#2384 <https://github.com/ros2/rosbag2/issues/2384>`__)
* Address flakiness in the ``rosbag2_transport::test_record_services`` tests (`#2368 <https://github.com/ros2/rosbag2/issues/2368>`__)
* Add support for time based Resume service (`#2357 <https://github.com/ros2/rosbag2/issues/2357>`__)
* Address race condition in the ``wait_for_playback_to_start()`` function (`#2344 <https://github.com/ros2/rosbag2/issues/2344>`__)
* Add ``set_on_start_recording_callback()`` to set the callback for when recording starts (`#2340 <https://github.com/ros2/rosbag2/issues/2340>`__)
* Remove unnecessary dependencies on ``yaml_cpp_vendor`` (`#2353 <https://github.com/ros2/rosbag2/issues/2353>`__)
* Allow pause/resume service calls while not in recording (`#2349 <https://github.com/ros2/rosbag2/issues/2349>`__)
* Implement delayed and time-based recorder and player services, adding new bag split modes (`#2330 <https://github.com/ros2/rosbag2/issues/2330>`__)
* Update Rosbag2 filename format to ``index+name+timestamp`` (`#2265 <https://github.com/ros2/rosbag2/issues/2265>`__)
* Address a possible deadlock in ``seek(timestamp)`` (`#2345 <https://github.com/ros2/rosbag2/issues/2345>`__)
* Add missing fields to ``RecordOptions`` YAML encode/decode functions and include a compile-time safeguard (`#2334 <https://github.com/ros2/rosbag2/issues/2334>`__)
* Implement circular logging by split count (``--max-bag-files``) (`#2218 <https://github.com/ros2/rosbag2/issues/2218>`__)
* Improve ``ros2 bag convert`` performance for fragment cutting and add ``--input-options`` (`#2325 <https://github.com/ros2/rosbag2/issues/2325>`__)
* Add static topics feature for recorder (`#2319 <https://github.com/ros2/rosbag2/issues/2319>`__)
* Add ``--max-cache-duration`` option for time-bounded snapshots (`#2289 <https://github.com/ros2/rosbag2/issues/2289>`__)
* Fix the flaky ``can_record_again_after_stop`` test (`#2313 <https://github.com/ros2/rosbag2/issues/2313>`__)
* Add error return code to the ``~/stop`` service request (`#2312 <https://github.com/ros2/rosbag2/issues/2312>`__)
* Add Record, Stop, StartDiscovery, StopDiscovery, and IsDiscoveryRunning services for Recorder (`#2248 <https://github.com/ros2/rosbag2/issues/2248>`__)
* Use QoS override settings for inner Rosbag2 publishing topics (`#2286 <https://github.com/ros2/rosbag2/issues/2286>`__)
* Fix decoder and encode mismatch in YAML deserialization (`#2277 <https://github.com/ros2/rosbag2/issues/2277>`__)
* Incorporate upstream minor fixes from Apex.AI (`#2240 <https://github.com/ros2/rosbag2/issues/2240>`__)
* Fix macOS build: Disable thread-safety annotations in ``locked_priority_queue.hpp`` (`#2245 <https://github.com/ros2/rosbag2/issues/2245>`__)
* Fix C++ Recorder failure when stop() then record() are called with the same bag name (`#2224 <https://github.com/ros2/rosbag2/issues/2224>`__)
* Add a direct API for ``rosbag2_transport::Recorder`` (`#2221 <https://github.com/ros2/rosbag2/issues/2221>`__)
* Add ``input_serialization_format`` and ``output_serialization_format`` to ``RecordOptions``, deprecating ``rmw_serialization_format`` (`#2215 <https://github.com/ros2/rosbag2/issues/2215>`__)
* Enable RMW communication isolation in rosbag2_transport tests (`#2190 <https://github.com/ros2/rosbag2/issues/2190>`__)
* Add topic name and type delimiter for hash map keys to avoid collisions (`#2210 <https://github.com/ros2/rosbag2/issues/2210>`__)
* Add cache for ``TopicFilter`` to avoid performance burden on discovery (`#1486 <https://github.com/ros2/rosbag2/issues/1486>`__)
* Address recorder test flakiness by increasing cache size (`#2203 <https://github.com/ros2/rosbag2/issues/2203>`__)
* Reduce CPU overhead in Rosbag2 recorder discovery by improving discovery logic (`#2201 <https://github.com/ros2/rosbag2/issues/2201>`__)
* Fix data races in ``PlayerProgressBar`` using atomic variables (`#2194 <https://github.com/ros2/rosbag2/issues/2194>`__)
* Fix data races in tests with ``MockSequentialWriter`` (`#2192 <https://github.com/ros2/rosbag2/issues/2192>`__)
* Player now respects original message order for same timestamps (`#2172 <https://github.com/ros2/rosbag2/issues/2172>`__)
* Return player storage options by value in ``get_storage_options()`` to avoid dangling reference (`#2181 <https://github.com/ros2/rosbag2/issues/2181>`__)
* Fix player not playing when ``read_ahead_queue_size`` equals 1 (`#2174 <https://github.com/ros2/rosbag2/issues/2174>`__)
* Fix multiple race conditions and a deadlock in the player (`#2171 <https://github.com/ros2/rosbag2/issues/2171>`__)
* Fix multibag replay stagnation and improve playback performance by managing chronological message order with ``ReadersManager`` (`#2158 <https://github.com/ros2/rosbag2/issues/2158>`__)
* Fix MCAPStorage::seek(time) to advance when timestamp matches current time (`#2157 <https://github.com/ros2/rosbag2/issues/2157>`__)
* Publish messages lost statistics to 'events/messages_lost' topic (`#2150 <https://github.com/ros2/rosbag2/issues/2150>`__)
* Add ``RecorderEventNotifier`` class (`#2144 <https://github.com/ros2/rosbag2/issues/2144>`__)
* Resolve deadlock during multibag replay and update ``wait_for_playback_to_start`` (`#2143 <https://github.com/ros2/rosbag2/issues/2143>`__)
* Use ``rclcpp typesupport helpers`` in ``rosbag2_cpp`` (`#2017 <https://github.com/ros2/rosbag2/issues/2017>`__)
* Fix callback not called for MESSAGES_LOST event (`#2105 <https://github.com/ros2/rosbag2/issues/2105>`__)
* Add public API for player's starting time and playback duration (`#2095 <https://github.com/ros2/rosbag2/issues/2095>`__)
* Fix CMAKE deprecation (`#2067 <https://github.com/ros2/rosbag2/issues/2067>`__)
* Fix deadlocks in Rosbag2 player when calling stop API (`#2057 <https://github.com/ros2/rosbag2/issues/2057>`__)
* Add message loss statistics callbacks and logging (`#2039 <https://github.com/ros2/rosbag2/issues/2039>`__)
* Skip flaky ``can_record_again_after_stop`` test (`#2031 <https://github.com/ros2/rosbag2/issues/2031>`__)
* Fix ``cout`` output when progress bar is disabled (`#2024 <https://github.com/ros2/rosbag2/issues/2024>`__)
* Improve message publishing timing by avoiding sporadic wakeups and fixing incorrect intervals on player start (`#2025 <https://github.com/ros2/rosbag2/issues/2025>`__)
* Fix ``playing_respects_delay`` test flakiness (`#2016 <https://github.com/ros2/rosbag2/issues/2016>`__)
* Address test flakiness by waiting for executor spin (`#2001 <https://github.com/ros2/rosbag2/issues/2001>`__)
* Avoid sending non-existent cancel requests (`#2005 <https://github.com/ros2/rosbag2/issues/2005>`__)
* Fix a maybe-uninitialized warning in player_action_client.cpp (`#1969 <https://github.com/ros2/rosbag2/issues/1969>`__)
* Upstream quality changes from Apex.AI part-2 (`#1924 <https://github.com/ros2/rosbag2/issues/1924>`__)
* Bugfix: ``ros2 bag convert`` dropping messages with compression mode message (`#1975 <https://github.com/ros2/rosbag2/issues/1975>`__)
* Use DDS queue depth for subscriptions as a maximum value across publishers (`#1960 <https://github.com/ros2/rosbag2/issues/1960>`__)
* Contributors: Barry Xu, Chris Lalancette, Christophe Bedard, Daisuke Sato, DangitBen, Dhruv Patel, Emerson Knapp, Janosch Machowinski, Luke Sy, Michael Carroll, Michael Orlov, Sahil Lakhmani, Scott K Logan, Shane Loretz, Tomoya Fujita, Tony Najjar, baranbologur, carlos-apex, mergify[bot], mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosgraph_msgs <https://github.com/ros2/rcl_interfaces/tree/lyrical/rosgraph_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Actually build the new graph description messages (`#192 <https://github.com/ros2/rcl_interfaces/issues/192>`__) (`#198 <https://github.com/ros2/rcl_interfaces/issues/198>`__)
* Add Graph description messages to ``rosgraph_msgs`` (`#188 <https://github.com/ros2/rcl_interfaces/issues/188>`__)
* Fix cmake deprecation (`#180 <https://github.com/ros2/rcl_interfaces/issues/180>`__)
* Contributors: Emerson Knapp, mergify[bot], mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_adapter <https://github.com/ros2/rosidl/tree/lyrical/rosidl_adapter/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix future regressions on flake8 (`#936 <https://github.com/ros2/rosidl//issues/936>`__)
* Fix @optional for string literals (`#905 <https://github.com/ros2/rosidl/issues/905>`__)
* Export typing Information (`#903 <https://github.com/ros2/rosidl/issues/903>`__)
* Add Optional Parsing (`#883 <https://github.com/ros2/rosidl/issues/883>`__)
* Uniform cmake minVersion (`#849 <https://github.com/ros2/rosidl/issues/849>`__)
* Contributors: Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_buffer <https://github.com/ros2/rosidl/tree/lyrical/rosidl_buffer/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add missing std::vector compatible APIs to rosidl::Buffer (`#959 <https://github.com/ros2/rosidl/issues/959>`__)
* Bump rosidl_buffer min CMake version (`#956 <https://github.com/ros2/rosidl/issues/956>`__)
* Update rosidl cpp path to emit rosidl::Buffer for uint8[] type (`#942 <https://github.com/ros2/rosidl/issues/942>`__)
* Add cstdint include to c_helpers.cpp (`#953 <https://github.com/ros2/rosidl/issues/953>`__)
* Add rosidl_buffer and rosidl_buffer_backend for native Buffer type support (`#941 <https://github.com/ros2/rosidl/issues/941>`__)
* Contributors: CY Chen, Shane Loretz, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_buffer_backend <https://github.com/ros2/rosidl/tree/lyrical/rosidl_buffer_backend/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Clarify return of get_descriptor_type_support() (`#958 <https://github.com/ros2/rosidl/issues/958>`__)
* Bump rosidl_buffer min CMake version (`#956 <https://github.com/ros2/rosidl/issues/956>`__)
* Add rosidl_buffer and rosidl_buffer_backend for native Buffer type support (`#941 <https://github.com/ros2/rosidl/issues/941>`__)
* Contributors: CY Chen, Shane Loretz


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_buffer_backend_registry <https://github.com/ros2/rosidl/tree/lyrical/rosidl_buffer_backend_registry/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add missing ament_cmake_gtest dep (`#960 <https://github.com/ros2/rosidl/issues/960>`__)
* Bump rosidl_buffer min CMake version (`#956 <https://github.com/ros2/rosidl/issues/956>`__)
* Add rosidl_buffer_backend_registry (`#944 <https://github.com/ros2/rosidl/issues/944>`__)
* Contributors: CY Chen, Scott K Logan, Shane Loretz


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_buffer_py <https://github.com/ros2/rosidl/tree/lyrical/rosidl_buffer_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Bump rosidl_buffer min CMake version (`#956 <https://github.com/ros2/rosidl/issues/956>`__)
* Fix pybind11 rosdep key (`#955 <https://github.com/ros2/rosidl/issues/955>`__)
* Update rosidl cpp path to emit rosidl::Buffer for uint8[] type (`#942 <https://github.com/ros2/rosidl/issues/942>`__)
* Contributors: CY Chen, Christoph Fröhlich, Shane Loretz


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_cli <https://github.com/ros2/rosidl/tree/lyrical/rosidl_cli/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix future regressions on flake8 (`#936 <https://github.com/ros2/rosidl//issues/936>`__)
* remove importlib-metadata (`#917 <https://github.com/ros2/rosidl/issues/917>`__)
* Export typing Information (`#903 <https://github.com/ros2/rosidl/issues/903>`__)
* fix setuptools deprecations (`#877 <https://github.com/ros2/rosidl/issues/877>`__)
* rosidl_cli: Add type description support (`#857 <https://github.com/ros2/rosidl/issues/857>`__)
* Contributors: Francisco Rossi, Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_cmake <https://github.com/ros2/rosidl/tree/lyrical/rosidl_cmake/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Install interface files to same folder as idl (`#935 <https://github.com/ros2/rosidl/issues/935>`__)
* Create an aggregate target for rosidl generated interfaces targets (`#947 <https://github.com/ros2/rosidl//issues/947>`__)
* Add ``rosidl_auto_generate_interfaces`` function (`#918 <https://github.com/ros2/rosidl/issues/918>`__)
* Export typing Information (`#903 <https://github.com/ros2/rosidl/issues/903>`__)
* remove deprecated rosidl_target_interfaces. (`#898 <https://github.com/ros2/rosidl/issues/898>`__)
* fix cmake <3.10 deprecation (`#875 <https://github.com/ros2/rosidl/issues/875>`__)
* Contributors: Emerson Knapp, Kotaro Yoshimoto, Michael Carlstrom, Tim Wendt, Tomoya Fujita, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_core_generators <https://github.com/ros2/rosidl_core/tree/lyrical/rosidl_core_generators/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Revert "Revert "Added rosidl_generator_rs (`#7 <https://github.com/ros2/rosidl_core/issues/7>`__)" (`#8 <https://github.com/ros2/rosidl_core/issues/8>`__)" (`#9 <https://github.com/ros2/rosidl_core/issues/9>`__)
* fix cmake deprecation (`#10 <https://github.com/ros2/rosidl_core/issues/10>`__)
* Contributors: Esteve Fernandez, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_core_runtime <https://github.com/ros2/rosidl_core/tree/lyrical/rosidl_core_runtime/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add rosidl_buffer_py as build_export_depend with explicit group resolution (`#14 <https://github.com/ros2/rosidl_core/issues/14>`__)
* fix cmake deprecation (`#10 <https://github.com/ros2/rosidl_core/issues/10>`__)
* Contributors: CY Chen, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_default_generators <https://github.com/ros2/rosidl_defaults/tree/lyrical/rosidl_default_generators/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix cmake deprecation (`#31 <https://github.com/ros2/rosidl_defaults/issues/31>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_default_runtime <https://github.com/ros2/rosidl_defaults/tree/lyrical/rosidl_default_runtime/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix cmake deprecation (`#31 <https://github.com/ros2/rosidl_defaults/issues/31>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_dynamic_typesupport <https://github.com/ros2/rosidl_dynamic_typesupport/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Don't automatically enable verbose makefiles (`#17 <https://github.com/ros2/rosidl_dynamic_typesupport/issues/17>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_dynamic_typesupport_fastrtps <https://github.com/ros2/rosidl_dynamic_typesupport_fastrtps/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Merge pull request `#11 <https://github.com/ros2/rosidl_dynamic_typesupport_fastrtps/issues/11>`__ from mosfet80/patch-1
* Don't automatically enable verbose makefiles. (`#9 <https://github.com/ros2/rosidl_dynamic_typesupport_fastrtps/issues/9>`__)
* Contributors: Chris Lalancette, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_generator_c <https://github.com/ros2/rosidl/tree/lyrical/rosidl_generator_c/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Export typing Information (`#903 <https://github.com/ros2/rosidl/issues/903>`__)
* Uniform cmake minVersion (`#849 <https://github.com/ros2/rosidl/issues/849>`__)
* rosidl_cli: Add type description support (`#857 <https://github.com/ros2/rosidl/issues/857>`__)
* Contributors: Francisco Rossi, Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_generator_cpp <https://github.com/ros2/rosidl/tree/lyrical/rosidl_generator_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update rosidl cpp path to emit rosidl::Buffer for uint8[] type (`#942 <https://github.com/ros2/rosidl/issues/942>`__)
* Use IWYU pragma export to avoid clangd warnings for generated headers (`#902 <https://github.com/ros2/rosidl//issues/902>`__)
* rosidl_generator_cpp: constexpr message traits and to_tuple_ref for generated structs (`#928 <https://github.com/ros2/rosidl//issues/928>`__)
* Make ``data_type`` and ``name`` traits constexpr (`#929 <https://github.com/ros2/rosidl//issues/929>`__)
* Export typing Information (`#903 <https://github.com/ros2/rosidl/issues/903>`__)
* Add static_cast (`#884 <https://github.com/ros2/rosidl/issues/884>`__)
* Uniform cmake minVersion (`#849 <https://github.com/ros2/rosidl/issues/849>`__)
* rosidl_cli: Add type description support (`#857 <https://github.com/ros2/rosidl/issues/857>`__)
* Add missing cstdint include (`#864 <https://github.com/ros2/rosidl/issues/864>`__)
* Removed deprecated methods (`#863 <https://github.com/ros2/rosidl/issues/863>`__)
* Contributors: Adam Leeper, Alejandro Hernández Cordero, CY Chen, Francisco Rossi, Michael Carlstrom, mosfet80, Øystein Sture


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_generator_dds_idl <https://github.com/ros2/rosidl_dds/tree/lyrical/rosidl_generator_dds_idl/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update cmake version requirements (`#64 <https://github.com/ros2/rosidl_dds/issues/64>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_generator_py <https://github.com/ros2/rosidl_python/tree/lyrical/rosidl_generator_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use absolute names for type hints (`#258 <https://github.com/ros2/rosidl_python/issues/258>`__)
* Feature: add depends flag for ament_python_install_package (`#254 <https://github.com/ros2/rosidl_python/issues/254>`__)
* Add support for rosidl::Buffer in rosidl Python path for rclpy (`#250 <https://github.com/ros2/rosidl_python/issues/250>`__)
* Cast Sequence to list on assignment (with templates) (`#249 <https://github.com/ros2/rosidl_python/issues/249>`__)
* Fix linter violations with flake8-import-order 0.19.0 (`#248 <https://github.com/ros2/rosidl_python/issues/248>`__)
* Add DEPENDS_EXPLICIT_ONLY to remove implicit dependencies (`#238 <https://github.com/ros2/rosidl_python/issues/238>`__)
* Deprecate using set for container based input (`#243 <https://github.com/ros2/rosidl_python//issues/243>`__)
* Update to use new BaseImpl (`#241 <https://github.com/ros2/rosidl_python/issues/241>`__)
* remove second call (`#232 <https://github.com/ros2/rosidl_python/issues/232>`__)
* Derive Messages from Base Classes (`#230 <https://github.com/ros2/rosidl_python/issues/230>`__)
* Remove NoReturn for now (`#229 <https://github.com/ros2/rosidl_python/issues/229>`__)
* Static typing for Message, Services, and Actions (`#206 <https://github.com/ros2/rosidl_python/issues/206>`__)
* Contributors: Anthony Welte, CY Chen, Michael Carlstrom, Nadav Elkabets


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_generator_rs <https://github.com/ros2-rust/rosidl_rust/tree/main/rosidl_generator_rs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix(rosidl_generator_rs_generate_interfaces): Remove poisoning of global CMAKE_SHARED_LINKER_FLAGS variable (#22)
* Change the package metadata to point to the new ros-env crate (#21)
* Fix TransientParseError on Ubuntu Resolute (#20)
* fix: do not monkey-patch _removesuffix into str (`#18 <https://github.com/ros2-rust/rosidl_rust/issues/18>`__)
* fix: add str.removesuffix() backport for Python < 3.9 (RHEL 8) (`#17 <https://github.com/ros2-rust/rosidl_rust/issues/17>`__)
* feat: relative Module Path Resolution (`#12 <https://github.com/ros2-rust/rosidl_rust/issues/12>`__) * Changed all generated code to use relative symbols instead of ``crate::`` ones. Reworked the rosidl_generator_rs slightly to be a bit simpler. Separate actual templates from the files that reuse them. * WIP For adding documentation to all structs, members, and constants generated from idl's. * Clean up all the surfaced warnings from generated code.
* build: update rosidl_runtime_rs dependency version to 0.6 (`#14 <https://github.com/ros2-rust/rosidl_rust/issues/14>`__)
* fix: update rosidl_runtime_rs dependency version to 0.5 (`#11 <https://github.com/ros2-rust/rosidl_rust/issues/11>`__)
* Fix use of serde (`#9 <https://github.com/ros2-rust/rosidl_rust/issues/9>`__) * Fix use of serde * Include serde for services ---------
* Update to the latest version of Action trait (`#7 <https://github.com/ros2-rust/rosidl_rust/issues/7>`__) * Update to the latest version of Action trait * Fix use of serde ---------
* fix cmake deprecation (`#6 <https://github.com/ros2-rust/rosidl_rust/issues/6>`__) * fix cmake deprecation cmake version < then 3.10 is deprecated * Update CMakeLists.txt
* fix: clean up dependencies (`#5 <https://github.com/ros2-rust/rosidl_rust/issues/5>`__)
* fix: added missing dependency
* clean up changelog. Removed rosidl_runtime_rs as a dependency
* set python executable var to custom cmake commands (`#3 <https://github.com/ros2-rust/rosidl_rust/issues/3>`__)
* Contributors: Esteve Fernandez, Grey, Kimberly N. McGuire, Sam Privett, Shane Loretz, Silvio Traversaro, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_generator_tests <https://github.com/ros2/rosidl/tree/lyrical/rosidl_generator_tests/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new aggregate rosidl target instead of _TARGETS (`#952 <https://github.com/ros2/rosidl/issues/952>`__)
* rosidl_generator_cpp: constexpr message traits and to_tuple_ref for generated structs (`#928 <https://github.com/ros2/rosidl//issues/928>`__)
* Make ``data_type`` and ``name`` traits constexpr (`#929 <https://github.com/ros2/rosidl//issues/929>`__)
* fix cmake <3.10 deprecation (`#875 <https://github.com/ros2/rosidl/issues/875>`__)
* Contributors: Alexis Tsogias, Michael Carlstrom, mosfet80, Øystein Sture


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_generator_type_description <https://github.com/ros2/rosidl/tree/lyrical/rosidl_generator_type_description/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add DEPENDS_EXPLICIT_ONLY to remove implicit dependencies (`#910 <https://github.com/ros2/rosidl//issues/910>`__)
* Add missing dependency on ament_cmake_pytest (`#914 <https://github.com/ros2/rosidl/issues/914>`__)
* Export typing Information (`#903 <https://github.com/ros2/rosidl/issues/903>`__)
* Uniform cmake minVersion (`#849 <https://github.com/ros2/rosidl/issues/849>`__)
* rosidl_cli: Add type description support (`#857 <https://github.com/ros2/rosidl/issues/857>`__)
* Contributors: Anthony Welte, Francisco Rossi, Michael Carlstrom, Scott K Logan, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_parser <https://github.com/ros2/rosidl/tree/lyrical/rosidl_parser/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix future regressions on flake8 (`#936 <https://github.com/ros2/rosidl//issues/936>`__)
* Add Optional Parsing (`#883 <https://github.com/ros2/rosidl/issues/883>`__)
* fix cmake <3.10 deprecation (`#875 <https://github.com/ros2/rosidl/issues/875>`__)
* Contributors: Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_pycommon <https://github.com/ros2/rosidl/tree/lyrical/rosidl_pycommon/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix regressions (`#951 <https://github.com/ros2/rosidl/issues/951>`__)
* Fix future regressions on flake8 (`#936 <https://github.com/ros2/rosidl//issues/936>`__)
* Add BaseImpl (`#912 <https://github.com/ros2/rosidl/issues/912>`__)
* Export typing Information (`#903 <https://github.com/ros2/rosidl/issues/903>`__)
* Provide base classes in ``rosidl_pycommon`` (`#887 <https://github.com/ros2/rosidl/issues/887>`__)
* fix setuptools deprecation (`#880 <https://github.com/ros2/rosidl/issues/880>`__)
* Contributors: Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_runtime_c <https://github.com/ros2/rosidl/tree/lyrical/rosidl_runtime_c/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update rosidl cpp path to emit rosidl::Buffer for uint8[] type (`#942 <https://github.com/ros2/rosidl/issues/942>`__)
* Fix copy/paste errors in type support docs (`#906 <https://github.com/ros2/rosidl/issues/906>`__)
* fix cmake <3.10 deprecation (`#875 <https://github.com/ros2/rosidl/issues/875>`__)
* Add an ament_cmake_gtest dependency to rosidl_runtime_c. (`#865 <https://github.com/ros2/rosidl/issues/865>`__)
* Contributors: CY Chen, Chris Lalancette, Christophe Bedard, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_runtime_cpp <https://github.com/ros2/rosidl/tree/lyrical/rosidl_runtime_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update rosidl cpp path to emit rosidl::Buffer for uint8[] type (`#942 <https://github.com/ros2/rosidl/issues/942>`__)
* rosidl_generator_cpp: constexpr message traits and to_tuple_ref for generated structs (`#928 <https://github.com/ros2/rosidl//issues/928>`__)
* Make ``data_type`` and ``name`` traits constexpr (`#929 <https://github.com/ros2/rosidl//issues/929>`__)
* fix cmake <3.10 deprecation (`#875 <https://github.com/ros2/rosidl/issues/875>`__)
* Add missing cstdint include (`#864 <https://github.com/ros2/rosidl/issues/864>`__)
* Contributors: CY Chen, Michael Carlstrom, mosfet80, Øystein Sture


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_runtime_py <https://github.com/ros2/rosidl_runtime_py/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add support for rosidl::Buffer in rosidl Python path for rclpy (`#39 <https://github.com/ros2/rosidl_runtime_py/issues/39>`__)
* Fix flake8 (`#40 <https://github.com/ros2/rosidl_runtime_py/issues/40>`__)
* Add py.typed to the package (`#37 <https://github.com/ros2/rosidl_runtime_py/issues/37>`__)
* fix setuptools deprecations (`#35 <https://github.com/ros2/rosidl_runtime_py/issues/35>`__)
* Contributors: CY Chen, Michael Carlstrom, Vladimir Gerts, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_c <https://github.com/ros2/rosidl_typesupport/tree/lyrical/rosidl_typesupport_c/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add DEPENDS_EXPLICIT_ONLY to remove implicit dependencies (`#168 <https://github.com/ros2/rosidl_typesupport/issues/168>`__)
* Contributors: Anthony Welte


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_cpp <https://github.com/ros2/rosidl_typesupport/tree/lyrical/rosidl_typesupport_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add DEPENDS_EXPLICIT_ONLY to remove implicit dependencies (`#168 <https://github.com/ros2/rosidl_typesupport/issues/168>`__)
* Remove deprecated rosidl_typesupport_cpp/type_support_map.h (`#167 <https://github.com/ros2/rosidl_typesupport/issues/167>`__)
* De-duplicate type_support_map.h header (`#81 <https://github.com/ros2/rosidl_typesupport/issues/81>`__)
* Contributors: Anthony Welte, Christophe Bedard


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_fastrtps_c <https://github.com/ros2/rosidl_typesupport_fastrtps/tree/lyrical/rosidl_typesupport_fastrtps_c/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update rosidl typesupport to support rosidl::Buffer in nested uint8[] (`#151 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/151>`__) (`#152 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/152>`__)
* Add support for rosidl::Buffer type serialization (`#144 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/144>`__)
* use variable to control shared/static build type (`#138 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/138>`__)
* Switch ament_index_python and rosidl_cli to exec_depend. (`#137 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/137>`__)
* Removed deprecated code (`#135 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/135>`__)
* fix cmake deprecation (`#134 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/134>`__)
* Check remaining size before resizing sequences (`#130 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/130>`__)
* Contributors: Alejandro Hernández Cordero, Anthony Welte, CY Chen, Chris Lalancette, Jay Sridharan, Miguel Company, mergify[bot], mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_fastrtps_cpp <https://github.com/ros2/rosidl_typesupport_fastrtps/tree/lyrical/rosidl_typesupport_fastrtps_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Clean up logs in buffer serialization functions (`#153 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/153>`__) (`#154 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/154>`__)
* Update rosidl typesupport to support rosidl::Buffer in nested uint8[] (`#151 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/151>`__) (`#152 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/152>`__)
* Add missing build dependencies for exported dependencies (`#149 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/149>`__)
* Add support for rosidl::Buffer type serialization (`#144 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/144>`__)
* use variable to control shared/static build type (`#138 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/138>`__)
* Add DEPENDS_EXPLICIT_ONLY to remove implicit dependencies (`#136 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/136>`__)
* Switch ament_index_python and rosidl_cli to exec_depend. (`#137 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/137>`__)
* Removed deprecated code (`#135 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/135>`__)
* fix cmake deprecation (`#134 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/134>`__)
* Check remaining size before resizing sequences (`#130 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/130>`__)
* Contributors: Alejandro Hernández Cordero, Anthony Welte, CY Chen, Chris Lalancette, Jay Sridharan, Miguel Company, Scott K Logan, mergify[bot], mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_interface <https://github.com/ros2/rosidl/tree/lyrical/rosidl_typesupport_interface/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix cmake <3.10 deprecation (`#875 <https://github.com/ros2/rosidl/issues/875>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_introspection_c <https://github.com/ros2/rosidl/tree/lyrical/rosidl_typesupport_introspection_c/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update rosidl cpp path to emit rosidl::Buffer for uint8[] type (`#942 <https://github.com/ros2/rosidl/issues/942>`__)
* Add DEPENDS_EXPLICIT_ONLY to remove implicit dependencies (`#910 <https://github.com/ros2/rosidl//issues/910>`__)
* Export typing Information (`#903 <https://github.com/ros2/rosidl/issues/903>`__)
* Uniform cmake minVersion (`#849 <https://github.com/ros2/rosidl/issues/849>`__)
* rosidl_cli: Add type description support (`#857 <https://github.com/ros2/rosidl/issues/857>`__)
* Contributors: Anthony Welte, CY Chen, Francisco Rossi, Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_introspection_cpp <https://github.com/ros2/rosidl/tree/lyrical/rosidl_typesupport_introspection_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update rosidl cpp path to emit rosidl::Buffer for uint8[] type (`#942 <https://github.com/ros2/rosidl/issues/942>`__)
* Add DEPENDS_EXPLICIT_ONLY to remove implicit dependencies (`#910 <https://github.com/ros2/rosidl//issues/910>`__)
* Export typing Information (`#903 <https://github.com/ros2/rosidl/issues/903>`__)
* Uniform cmake minVersion (`#849 <https://github.com/ros2/rosidl/issues/849>`__)
* rosidl_cli: Add type description support (`#857 <https://github.com/ros2/rosidl/issues/857>`__)
* Contributors: Anthony Welte, CY Chen, Francisco Rossi, Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_introspection_tests <https://github.com/ros2/rosidl/tree/lyrical/rosidl_typesupport_introspection_tests/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update rosidl cpp path to emit rosidl::Buffer for uint8[] type (`#942 <https://github.com/ros2/rosidl/issues/942>`__)
* fix cmake <3.10 deprecation (`#875 <https://github.com/ros2/rosidl/issues/875>`__)
* Disable test failing in coverage jobs, see `#812 <https://github.com/ros2/rosidl/issues/812>`__ (`#853 <https://github.com/ros2/rosidl/issues/853>`__)
* Contributors: CY Chen, Jorge J. Perez, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_tests <https://github.com/ros2/rosidl_typesupport/tree/lyrical/rosidl_typesupport_tests/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* pass all tests for rmw_cyclonedds_cpp. (`#171 <https://github.com/ros2/rosidl_typesupport/issues/171>`__)
* Contributors: Tomoya Fujita


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rpyutils <https://github.com/ros2/rpyutils/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Enforce ament_mypy --ament-strict (`#22 <https://github.com/ros2/rpyutils/issues/22>`__)
* fix setuptools deprecations (`#17 <https://github.com/ros2/rpyutils/issues/17>`__)
* Contributors: Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt <https://github.com/ros-visualization/rqt/tree/lyrical/rqt/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix setuptools deprecations (`#334 <https://github.com/ros-visualization/rqt/issues/334>`__)
* fix setuptools deprecations (`#329 <https://github.com/ros-visualization/rqt/issues/329>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_action <https://github.com/ros-visualization/rqt_action/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix setuptools deprecations (`#19 <https://github.com/ros-visualization/rqt_action/issues/19>`__)
* Remove CODEOWNERS and mirror-rolling-to-main workflow (`#16 <https://github.com/ros-visualization/rqt_action/issues/16>`__)
* Contributors: Alejandro Hernández Cordero, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_bag <https://github.com/ros-visualization/rqt_bag/tree/lyrical/rqt_bag/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix Qt6 issues (backport `#207 <https://github.com/ros-visualization/rqt_bag/issues/207>`__) (`#208 <https://github.com/ros-visualization/rqt_bag/issues/208>`__)
* Support Qt6 (`#206 <https://github.com/ros-visualization/rqt_bag/issues/206>`__)
* Cleanup mislabeled BSD license (`#205 <https://github.com/ros-visualization/rqt_bag/issues/205>`__)
* Better handling of large bag files (`#178 <https://github.com/ros-visualization/rqt_bag/issues/178>`__)
* Display roll, pitch, yaw values for quaternions (`#179 <https://github.com/ros-visualization/rqt_bag/issues/179>`__)
* Fix flake8 error in setup.py (`#192 <https://github.com/ros-visualization/rqt_bag/issues/192>`__)
* Improved raw view to better handle arrays and time objects (`#173 <https://github.com/ros-visualization/rqt_bag/issues/173>`__)
* plot_view: Fixed display of initial message (`#180 <https://github.com/ros-visualization/rqt_bag/issues/180>`__)
* fix setuptools deprecations (`#185 <https://github.com/ros-visualization/rqt_bag/issues/185>`__)
* Fixed timeline resolution (`#175 <https://github.com/ros-visualization/rqt_bag/issues/175>`__)
* Contributors: Alejandro Hernández Cordero, Martin Pecka, Michael Carlstrom, mergify[bot], mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_bag_plugins <https://github.com/ros-visualization/rqt_bag/tree/lyrical/rqt_bag_plugins/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Support Qt6 (`#206 <https://github.com/ros-visualization/rqt_bag/issues/206>`__)
* Display roll, pitch, yaw values for quaternions (`#179 <https://github.com/ros-visualization/rqt_bag/issues/179>`__)
* Fix flake8 error in setup.py (`#192 <https://github.com/ros-visualization/rqt_bag/issues/192>`__)
* Fixed image helper and added support for PNG-coded compressedDepth (`#176 <https://github.com/ros-visualization/rqt_bag/issues/176>`__)
* Improve plot view (`#174 <https://github.com/ros-visualization/rqt_bag/issues/174>`__)
* plot_view: Fixed display of initial message (`#180 <https://github.com/ros-visualization/rqt_bag/issues/180>`__)
* fix setuptools deprecations (`#185 <https://github.com/ros-visualization/rqt_bag/issues/185>`__)
* Contributors: Alejandro Hernández Cordero, Martin Pecka, Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_console <https://github.com/ros-visualization/rqt_console/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Support Qt6 (`#58 <https://github.com/ros-visualization/rqt_console/issues/58>`__)
* fixed copyright test (`#57 <https://github.com/ros-visualization/rqt_console/issues/57>`__)
* added copyright header
* fixed flake8 (`#56 <https://github.com/ros-visualization/rqt_console/issues/56>`__)
* fixed flake8
* remove residual imports in colored output test-file (`#55 <https://github.com/ros-visualization/rqt_console/issues/55>`__)
* remove residual imports
* basic support for colors and bold/bright using ANSI escape codes (`#54 <https://github.com/ros-visualization/rqt_console/issues/54>`__)
* replace colorama dependency with manual ansi codes
* add checkbox in the settings
* support more color codes
* basic support for colors and bold/bright
* fix setuptools deprecations (`#50 <https://github.com/ros-visualization/rqt_console/issues/50>`__)
* Contributors: Alejandro Hernández Cordero, Arne Hitzmann, Peter, mosfet80, peter


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_graph <https://github.com/ros-visualization/rqt_graph/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Support Wheel event in qt6 (backport `#116 <https://github.com/ros-visualization/rqt_graph/issues/116>`__) (`#117 <https://github.com/ros-visualization/rqt_graph/issues/117>`__)
* Fix: broken dependency (`#115 <https://github.com/ros-visualization/rqt_graph//issues/115>`__)
* Support Qt6 (`#114 <https://github.com/ros-visualization/rqt_graph//issues/114>`__)
* add warning for type incompatibilities (`#105 <https://github.com/ros-visualization/rqt_graph/issues/105>`__)
* Remove rqt_graph script. (`#66 <https://github.com/ros-visualization/rqt_graph/issues/66>`__)
* fix setuptools deprecations (`#107 <https://github.com/ros-visualization/rqt_graph/issues/107>`__)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Jonas Otto, Matthew Foran, mergify[bot], mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_gui <https://github.com/ros-visualization/rqt/tree/lyrical/rqt_gui/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix setupTools deprecations (`#322 <https://github.com/ros-visualization/rqt/issues/322>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_gui_cpp <https://github.com/ros-visualization/rqt/tree/lyrical/rqt_gui_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Cleanup headers (`#347 <https://github.com/ros-visualization/rqt/issues/347>`__) (`#350 <https://github.com/ros-visualization/rqt/issues/350>`__)
* Use qt-base-dev / libqtwidgets (`#345 <https://github.com/ros-visualization/rqt/issues/345>`__)
* fix: include unistd.h for getpid (`#341 <https://github.com/ros-visualization/rqt/issues/341>`__)
* Support Qt6 (`#339 <https://github.com/ros-visualization/rqt/issues/339>`__)
* Removed deprecated header (`#340 <https://github.com/ros-visualization/rqt/issues/340>`__)
* Use qt6 as the default dependency from rosdep (`#337 <https://github.com/ros-visualization/rqt/issues/337>`__)
* fix compile with qt6 (`#321 <https://github.com/ros-visualization/rqt/issues/321>`__)
* Contributors: Alejandro Hernández Cordero, Daisuke Nishimatsu, Shane Loretz, mergify[bot], mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_gui_py <https://github.com/ros-visualization/rqt/tree/lyrical/rqt_gui_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Support Qt6 (`#339 <https://github.com/ros-visualization/rqt/issues/339>`__)
* Fix setupTools deprecations (`#322 <https://github.com/ros-visualization/rqt/issues/322>`__)
* Contributors: Alejandro Hernández Cordero, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_msg <https://github.com/ros-visualization/rqt_msg/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Support Qt6 (`#27 <https://github.com/ros-visualization/rqt_msg/issues/27>`__)
* fix setuptools deprecations (`#23 <https://github.com/ros-visualization/rqt_msg/issues/23>`__)
* Remove CODEOWNERS (`#20 <https://github.com/ros-visualization/rqt_msg/issues/20>`__)
* Contributors: Alejandro Hernández Cordero, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_plot <https://github.com/ros-visualization/rqt_plot/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use qt-base-dev / libqtwidgets (`#128 <https://github.com/ros-visualization/rqt_plot/issues/128>`__)
* Support Qt6 (`#127 <https://github.com/ros-visualization/rqt_plot/issues/127>`__)
* fix setuptools deprecations (`#123 <https://github.com/ros-visualization/rqt_plot/issues/123>`__)
* Added missing test dependency (`#118 <https://github.com/ros-visualization/rqt_plot/issues/118>`__)
* Fix for displaying constant curves (`#114 <https://github.com/ros-visualization/rqt_plot/issues/114>`__)
* Contributors: Alejandro Hernández Cordero, Martin Pecka, Shane Loretz, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_publisher <https://github.com/ros-visualization/rqt_publisher/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix flake8 (backport `#57 <https://github.com/ros-visualization/rqt_publisher/issues/57>`__) (`#58 <https://github.com/ros-visualization/rqt_publisher/issues/58>`__)
* Support Qt6 (`#56 <https://github.com/ros-visualization/rqt_publisher/issues/56>`__)
* fix setuptools deprecations (`#52 <https://github.com/ros-visualization/rqt_publisher/issues/52>`__)
* Contributors: Alejandro Hernández Cordero, mergify[bot], mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_py_common <https://github.com/ros-visualization/rqt/tree/lyrical/rqt_py_common/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Cleanup headers (`#347 <https://github.com/ros-visualization/rqt/issues/347>`__) (`#350 <https://github.com/ros-visualization/rqt/issues/350>`__)
* Use qt-base-dev / libqtwidgets (`#345 <https://github.com/ros-visualization/rqt/issues/345>`__)
* Support Qt6 (`#339 <https://github.com/ros-visualization/rqt/issues/339>`__)
* fix compile with qt6 (`#321 <https://github.com/ros-visualization/rqt/issues/321>`__)
* Contributors: Alejandro Hernández Cordero, Shane Loretz, mergify[bot], mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_py_console <https://github.com/ros-visualization/rqt_py_console/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add Qt6 compatibility (`#25 <https://github.com/ros-visualization/rqt_py_console/issues/25>`__) Co-authored-by: Alejandro Hernandez Cordero <ahcorde@gmail.com>
* fix setuptools deprecations (`#21 <https://github.com/ros-visualization/rqt_py_console/issues/21>`__)
* Contributors: Shane Loretz, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_reconfigure <https://github.com/ros-visualization/rqt_reconfigure/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Support Qt6 (`#158 <https://github.com/ros-visualization/rqt_reconfigure/issues/158>`__)
* Harden behavior if double value or limit is Infinity (`#161 <https://github.com/ros-visualization/rqt_reconfigure/issues/161>`__)
* Scale IntegerEditor if range exceeds int32 (`#160 <https://github.com/ros-visualization/rqt_reconfigure/issues/160>`__)
* Ignore A005 for future flake8 (`#159 <https://github.com/ros-visualization/rqt_reconfigure/issues/159>`__)
* Cleanup mislabeled BSD license (`#157 <https://github.com/ros-visualization/rqt_reconfigure/issues/157>`__)
* fix setuptools deprecation (`#153 <https://github.com/ros-visualization/rqt_reconfigure/issues/153>`__)
* If updating remote fails, reflect the failure locally (`#144 <https://github.com/ros-visualization/rqt_reconfigure/issues/144>`__)
* Remove CODEOWNERS (`#147 <https://github.com/ros-visualization/rqt_reconfigure/issues/147>`__)
* Contributors: Alejandro Hernández Cordero, Christoph Fröhlich, Jonathan Selling, Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_service_caller <https://github.com/ros-visualization/rqt_service_caller/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Support Qt6 (`#38 <https://github.com/ros-visualization/rqt_service_caller/issues/38>`__)
* fix setuptools deprecations (`#33 <https://github.com/ros-visualization/rqt_service_caller/issues/33>`__)
* Contributors: Alejandro Hernández Cordero, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_shell <https://github.com/ros-visualization/rqt_shell/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* make linters happy
* Fix setuptools deprecation (`#26 <https://github.com/ros-visualization/rqt_shell/issues/26>`__)
* Contributors: Alejandro Hernandez Cordero, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_srv <https://github.com/ros-visualization/rqt_srv/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix setuptools deprecations (`#16 <https://github.com/ros-visualization/rqt_srv/issues/16>`__)
* Remove CODEOWNERS (`#13 <https://github.com/ros-visualization/rqt_srv/issues/13>`__)
* Contributors: Alejandro Hernández Cordero, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_topic <https://github.com/ros-visualization/rqt_topic/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Support Qt6 (`#67 <https://github.com/ros-visualization/rqt_topic/issues/67>`__)
* Add Qt6 compatibility (`#66 <https://github.com/ros-visualization/rqt_topic/issues/66>`__)
* Tweak expected error in test for Pydantic v2 compat (`#65 <https://github.com/ros-visualization/rqt_topic/issues/65>`__)
* Use choose_qos() from ros2 topic echo (`#55 <https://github.com/ros-visualization/rqt_topic/issues/55>`__)
* Enable flake8 (`#58 <https://github.com/ros-visualization/rqt_topic/issues/58>`__)
* Open source rewrite of rqt_topic (`#47 <https://github.com/ros-visualization/rqt_topic/issues/47>`__) Co-authored-by: Evan Flynn <evan.flynn@apex.ai> Co-authored-by: Alejandro Hernandez Cordero <ahcorde@gmail.com>
* fix setuptools deprecations (`#57 <https://github.com/ros-visualization/rqt_topic/issues/57>`__)
* Contributors: Alejandro Hernández Cordero, Evan Flynn, Romain Reignier, Scott K Logan, Shane Loretz, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rti_connext_dds_cmake_module <https://github.com/ros2/rmw_connextdds/tree/lyrical/rti_connext_dds_cmake_module/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update Connext from 7.3.0 to 7.7.0, disable monitoring library by default, and use synchronous publishing mode (`#219 <https://github.com/ros2/rmw_connextdds/issues/219>`__)
* Fix cmake deprecation (`#198 <https://github.com/ros2/rmw_connextdds/issues/198>`__)
* Contributors: Francisco Gallego Salido, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rttest <https://github.com/ros2/realtime_support/tree/lyrical/rttest/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* cleanups and removed dead code (`#141 <https://github.com/ros2/realtime_support/issues/141>`__) (`#144 <https://github.com/ros2/realtime_support/issues/144>`__)
* Fix cmake deprecation (`#134 <https://github.com/ros2/realtime_support/issues/134>`__)
* Contributors: mergify[bot], mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rviz2 <https://github.com/ros2/rviz/tree/lyrical/rviz2/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use rosdep keys that select Qt5 or Qt6 by platform (`#1720 <https://github.com/ros2/rviz/issues/1720>`__)
* Use new ROSIDL aggregate CMake target (`#1688 <https://github.com/ros2/rviz/issues/1688>`__)
* Fix Qt version resolution when both Qt5 and Qt6 are installed - CMake defaults to ascending resolution and Qt5 will be found when Qt6 is desired (Rolling, L-Turtle, and beyond). (`#1689 <https://github.com/ros2/rviz/issues/1689>`__)
* Use qt6 as the default dependency from rosdep (`#1635 <https://github.com/ros2/rviz/issues/1635>`__)
* get rid of deprecated rclcpp::spin_some() (`#1567 <https://github.com/ros2/rviz/issues/1567>`__)
* feat: support both qt5 and qt6 (`#1187 <https://github.com/ros2/rviz/issues/1187>`__)
* Contributors: Alejandro Hernández Cordero, Daisuke Nishimatsu, Emerson Knapp, Nathan Brooks, Shane Loretz


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rviz_common <https://github.com/ros2/rviz/tree/lyrical/rviz_common/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use rosdep keys that select Qt5 or Qt6 by platform (`#1720 <https://github.com/ros2/rviz/issues/1720>`__)
* Compressed Image Display (`#1288 <https://github.com/ros2/rviz//issues/1288>`__)
* fix: Fixed compilation on MSVC 2022 (`#1706 <https://github.com/ros2/rviz//issues/1706>`__)
* Removed Qt6 warnings (`#1704 <https://github.com/ros2/rviz/issues/1704>`__)
* Fixed regresion is RHEL (`#1703 <https://github.com/ros2/rviz/issues/1703>`__)
* Remove warnings (`#1693 <https://github.com/ros2/rviz/issues/1693>`__)
* Link against ``GTest::gmock`` target (`#1699 <https://github.com/ros2/rviz/issues/1699>`__)
* Reduce ``QFile`` dependency (`#1652 <https://github.com/ros2/rviz/issues/1652>`__)
* Use new ROSIDL aggregate CMake target (`#1688 <https://github.com/ros2/rviz/issues/1688>`__)
* Fix Qt version resolution when both Qt5 and Qt6 are installed - CMake defaults to ascending resolution and Qt5 will be found when Qt6 is desired (Rolling, L-Turtle, and beyond). (`#1689 <https://github.com/ros2/rviz/issues/1689>`__)
* Cleanups in rviz_common (`#1686 <https://github.com/ros2/rviz/issues/1686>`__)
* Add tests for shallow and deep copy in Config (`#1682 <https://github.com/ros2/rviz/issues/1682>`__)
* Build performance optimizations for rviz_common (`#1677 <https://github.com/ros2/rviz/issues/1677>`__)
* Use get_package_share_path (`#1671 <https://github.com/ros2/rviz/issues/1671>`__)
* Fix setHidden regression in PropertyTreeWidget (`#1667 <https://github.com/ros2/rviz/issues/1667>`__)
* Add topic name filtering when adding new visualizations (`#1662 <https://github.com/ros2/rviz/issues/1662>`__)
* use QPointer in QTimer::singleShot to prevent use-after-free (`#1657 <https://github.com/ros2/rviz/issues/1657>`__)
* Fix Not loading plugins due to incorrect package path (`#1651 <https://github.com/ros2/rviz/issues/1651>`__)
* Updated deprecated ament_index_cpp API (`#1647 <https://github.com/ros2/rviz/issues/1647>`__)
* Fix crash with no tools (`#1639 <https://github.com/ros2/rviz/issues/1639>`__)
* Use qt6 as the default dependency from rosdep (`#1635 <https://github.com/ros2/rviz/issues/1635>`__)
* Pointcloud2 display set QoS to best effort (`#1621 <https://github.com/ros2/rviz/issues/1621>`__)
* Cleanup deprecated code (`#1619 <https://github.com/ros2/rviz//issues/1619>`__)
* Removed support for yaml-cpp lower than 0.5 (`#1605 <https://github.com/ros2/rviz//issues/1605>`__)
* Removed duplicated forward class declaration (`#1602 <https://github.com/ros2/rviz//issues/1602>`__)
* resolved TODO in visualization manager (`#1603 <https://github.com/ros2/rviz//issues/1603>`__)
* Fix incorrect Qt signal connection in combo box (`#1596 <https://github.com/ros2/rviz/issues/1596>`__)
* Removed tinyxml2_vendor dependency (`#1591 <https://github.com/ros2/rviz/issues/1591>`__)
* Replace QRegExp with QRegularExpression to support Qt6 (`#1592 <https://github.com/ros2/rviz/issues/1592>`__)
* fix crash (`#1587 <https://github.com/ros2/rviz/issues/1587>`__)
* added option to change filemode (`#1537 <https://github.com/ros2/rviz/issues/1537>`__)
* Removed deprecation warning in tf2 (`#1585 <https://github.com/ros2/rviz/issues/1585>`__)
* Std chrono update in default plugins (`#1579 <https://github.com/ros2/rviz/issues/1579>`__)
* Removed deprecations (`#1556 <https://github.com/ros2/rviz/issues/1556>`__)
* rviz common ros service property (`#1548 <https://github.com/ros2/rviz/issues/1548>`__)
* add ros action property (`#1549 <https://github.com/ros2/rviz/issues/1549>`__)
* Deprecates update(float, float) methods and provides update(std::chrono::duration, std::chrono::duration) replacements. (`#1533 <https://github.com/ros2/rviz//issues/1533>`__)
* Replace deprecated tf2_ros headers (`#1529 <https://github.com/ros2/rviz/issues/1529>`__)
* Postpone hiding of properties until insertion into model is finished (`#1508 <https://github.com/ros2/rviz/issues/1508>`__)
* Don't hide rows of properties not within the model (`#1507 <https://github.com/ros2/rviz/issues/1507>`__)
* Remove redundant check (`#1506 <https://github.com/ros2/rviz/issues/1506>`__)
* Fix panel deletion (`#1037 <https://github.com/ros2/rviz/issues/1037>`__)
* Config::mapGetBool causes segmentation fault when value_out is nullptr (`#1471 <https://github.com/ros2/rviz/issues/1471>`__)
* feat: support both qt5 and qt6 (`#1187 <https://github.com/ros2/rviz/issues/1187>`__)
* Fixed crash when a resource is not available (`#1455 <https://github.com/ros2/rviz/issues/1455>`__)
* Work in progress using the new resource retriever apis (`#1262 <https://github.com/ros2/rviz/issues/1262>`__)
* addTrackedObject Function Fails to Handle Null Pointer, Causing Crash When nullptr is Passed (`#1375 <https://github.com/ros2/rviz/issues/1375>`__)
* Add test to check mapGetString when key is missing (`#1361 <https://github.com/ros2/rviz/issues/1361>`__)
* UniformStringStream::parseFloat Fails to Handle Invalid Float Formats Correctly (`#1360 <https://github.com/ros2/rviz/issues/1360>`__)
* Fix Potential Null Pointer Dereference in VisualizerApp::getRenderWindow() to Prevent Crashes (`#1359 <https://github.com/ros2/rviz/issues/1359>`__)
* Extend support for type adaptation (REP 2007) in rviz_common for TF-filtered displays (`#1346 <https://github.com/ros2/rviz/issues/1346>`__)
* Contributors: Alejandro Hernández Cordero, Daisuke Nishimatsu, David V. Lu!!, Emerson Knapp, Janosch Machowinski, Joshua Supratman, Mark Johnson, Mateusz Żak, Matteo Princisgh, Matthew Foran, Michael Carroll, Nathan Brooks, Oscmoar07, Patrick Roncagliolo, Shane Loretz, mini-1235, nelson, t0k0shi


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rviz_default_plugins <https://github.com/ros2/rviz/tree/lyrical/rviz_default_plugins/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use rosdep keys that select Qt5 or Qt6 by platform (`#1720 <https://github.com/ros2/rviz/issues/1720>`__)
* Compressed Image Display (`#1288 <https://github.com/ros2/rviz//issues/1288>`__)
* Removed Qt6 warnings (`#1704 <https://github.com/ros2/rviz/issues/1704>`__)
* Switch rviz service resource retriever to use new repo's code (`#1698 <https://github.com/ros2/rviz/issues/1698>`__)
* Link against ``GTest::gmock`` target (`#1699 <https://github.com/ros2/rviz/issues/1699>`__)
* Improve marker common (`#1687 <https://github.com/ros2/rviz/issues/1687>`__)
* Reduce ``QFile`` dependency (`#1652 <https://github.com/ros2/rviz/issues/1652>`__)
* Use new ROSIDL aggregate CMake target (`#1688 <https://github.com/ros2/rviz/issues/1688>`__)
* Fix Qt version resolution when both Qt5 and Qt6 are installed - CMake defaults to ascending resolution and Qt5 will be found when Qt6 is desired (Rolling, L-Turtle, and beyond). (`#1689 <https://github.com/ros2/rviz/issues/1689>`__)
* Remove redundant compilation of test fixtures (`#1673 <https://github.com/ros2/rviz/issues/1673>`__)
* Updated deprecated ament_index_cpp API (`#1647 <https://github.com/ros2/rviz/issues/1647>`__)
* Add CameraInfo topic property to DepthCloudDisplay (`#1643 <https://github.com/ros2/rviz/issues/1643>`__)
* Use qt6 as the default dependency from rosdep (`#1635 <https://github.com/ros2/rviz/issues/1635>`__)
* Pointcloud2 display set QoS to best effort (`#1621 <https://github.com/ros2/rviz/issues/1621>`__)
* Fix Translation Issue in XYOrbitViewController (`#1630 <https://github.com/ros2/rviz/issues/1630>`__)
* Overcome 16384 size limit (`#1622 <https://github.com/ros2/rviz/issues/1622>`__)
* Removed already done TODO (`#1604 <https://github.com/ros2/rviz//issues/1604>`__)
* Fixed issue 1593 (`#1598 <https://github.com/ros2/rviz/issues/1598>`__)
* Removed tf2 warning (`#1586 <https://github.com/ros2/rviz/issues/1586>`__)
* Removed deprecation warning in tf2 (`#1585 <https://github.com/ros2/rviz/issues/1585>`__)
* Std chrono update in default plugins (`#1579 <https://github.com/ros2/rviz/issues/1579>`__)
* Fix pointcloud2 display divide by 0 (`#1581 <https://github.com/ros2/rviz/issues/1581>`__)
* add support for ffmpeg_image_transport and point_cloud_transport (`#1568 <https://github.com/ros2/rviz/issues/1568>`__)
* Extend the message filter display for point cloud 2 display (`#1566 <https://github.com/ros2/rviz/issues/1566>`__)
* Support image transport lifecycle (`#1472 <https://github.com/ros2/rviz//issues/1472>`__)
* Fix QoS profile loading for InitialPoseTool from rviz config files (`#1544 <https://github.com/ros2/rviz//issues/1544>`__)
* Replace rmw_qos_profile_t with rclcpp::QoS (`#1525 <https://github.com/ros2/rviz/issues/1525>`__)
* Replace deprecated tf2_ros headers (`#1529 <https://github.com/ros2/rviz/issues/1529>`__)
* fix deprecated include (`#1530 <https://github.com/ros2/rviz/issues/1530>`__)
* point_cloud_transport update API call (`#1526 <https://github.com/ros2/rviz/issues/1526>`__)
* Better handling of missing transport plugins (`#1488 <https://github.com/ros2/rviz/issues/1488>`__)
* Fixed deprecation warning on point_cloud_transport: rmw_qos_profile_t (`#1491 <https://github.com/ros2/rviz/issues/1491>`__)
* Add symbol visibility macros to make*Palette public functions (`#1492 <https://github.com/ros2/rviz/issues/1492>`__)
* Fix /rviz/get_resource (`#1487 <https://github.com/ros2/rviz/issues/1487>`__)
* Removed point_cloud_transport deprecation (`#1474 <https://github.com/ros2/rviz/issues/1474>`__)
* Frame view controller: Removed warnings (`#1470 <https://github.com/ros2/rviz/issues/1470>`__)
* Fix compile with qt6 (`#1475 <https://github.com/ros2/rviz/issues/1475>`__)
* Fix Issue with Quaternion Angular Distance (`#1473 <https://github.com/ros2/rviz/issues/1473>`__)
* PointStampedDisplay: Ignore incoming messages if disabled (`#1036 <https://github.com/ros2/rviz/issues/1036>`__)
* Removed unused headers from resouce retriever (`#1463 <https://github.com/ros2/rviz/issues/1463>`__)
* feat: support both qt5 and qt6 (`#1187 <https://github.com/ros2/rviz/issues/1187>`__)
* [rviz_default_plugins] Add missing export dependencies (`#1461 <https://github.com/ros2/rviz/issues/1461>`__)
* Backported FrameAligned camera (`#1453 <https://github.com/ros2/rviz/issues/1453>`__)
* Changed Marker Displays to allow toggling visibility of namespaces (`#1402 <https://github.com/ros2/rviz/issues/1402>`__)
* Do not use ${Qt5Widgets_INCLUDE_DIRS} to avoid creating non-relocatable CMake config files (`#1450 <https://github.com/ros2/rviz/issues/1450>`__)
* PointCloudDisplay: Fix decay time 0 keeping more than the last message (`#1400 <https://github.com/ros2/rviz/issues/1400>`__)
* Work in progress using the new resource retriever apis (`#1262 <https://github.com/ros2/rviz/issues/1262>`__)
* Include chrono (`#1353 <https://github.com/ros2/rviz/issues/1353>`__)
* Contributors: Alejandro Hernández Cordero, Alexis Tsogias, Antonio Brandi, Daisuke Nishimatsu, Eesha Kumar, Emerson Knapp, Felix Exner (fexner), Georg Flick, Guillaume Doisy, Harrison Chen, Kenji Brameld (TRACLabs), Kosuke Takeuchi, Lennart Reiher, Mark Johnson, Matthew Foran, Michael Carroll, Nathan Brooks, Shane Loretz, Silvio Traversaro, Stefan Fabian, Stoyan Gaydarov, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rviz_ogre_vendor <https://github.com/ros2/rviz/tree/lyrical/rviz_ogre_vendor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add patch to remove ``binary_function`` (`#1691 <https://github.com/ros2/rviz/issues/1691>`__)
* Bump cmake version and suppress warning for rviz_ogre_vendor (`#1684 <https://github.com/ros2/rviz/issues/1684>`__)
* Remove vendoring freetype and zlib on Windows (`#1636 <https://github.com/ros2/rviz/issues/1636>`__)
* Add RVIZ_OGRE_VENDOR_MANGLE_NAME_OF_LIBRARIES_USED_BY_RVIZ option to further mangle ogre libraries used by rviz (`#1493 <https://github.com/ros2/rviz/issues/1493>`__)
* Add missing glew dependency for ogre vendor package (`#1350 <https://github.com/ros2/rviz/issues/1350>`__)
* Contributors: Dhruv Patel, Michael Carroll, Shane Loretz, Silvio Traversaro, Stefan Fabian


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rviz_rendering <https://github.com/ros2/rviz/tree/lyrical/rviz_rendering/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use rosdep keys that select Qt5 or Qt6 by platform (`#1720 <https://github.com/ros2/rviz/issues/1720>`__)
* Fix build for Ubuntu 26 (`#1694 <https://github.com/ros2/rviz/issues/1694>`__)
* Fix Qt version resolution when both Qt5 and Qt6 are installed - CMake defaults to ascending resolution and Qt5 will be found when Qt6 is desired (Rolling, L-Turtle, and beyond). (`#1689 <https://github.com/ros2/rviz/issues/1689>`__)
* Updated deprecated ament_index_cpp API (`#1647 <https://github.com/ros2/rviz/issues/1647>`__)
* Use qt6 as the default dependency from rosdep (`#1635 <https://github.com/ros2/rviz/issues/1635>`__)
* Removed unused files (`#1600 <https://github.com/ros2/rviz//issues/1600>`__)
* Removed assimp vendor package (`#1574 <https://github.com/ros2/rviz/issues/1574>`__)
* Update OGRE mesh files from ROS1 RViz (`#1536 <https://github.com/ros2/rviz//issues/1536>`__) (`#1559 <https://github.com/ros2/rviz//issues/1559>`__)
* add resourceExists check to loadEmbeddedTexture before loading texture (`#1542 <https://github.com/ros2/rviz//issues/1542>`__)
* Assign the geometry to the resource group "rviz_rendering" (`#1502 <https://github.com/ros2/rviz/issues/1502>`__)
* Removed windows warning (`#1486 <https://github.com/ros2/rviz/issues/1486>`__)
* Handle glTF Y-Up frame convention on mesh load (`#1482 <https://github.com/ros2/rviz/issues/1482>`__)
* Removed unused headers from resouce retriever (`#1463 <https://github.com/ros2/rviz/issues/1463>`__)
* feat: support both qt5 and qt6 (`#1187 <https://github.com/ros2/rviz/issues/1187>`__)
* WrenchVisual::setForceColor and setTorqueColor clamp values (`#1437 <https://github.com/ros2/rviz/issues/1437>`__)
* Missing Null Pointer Check in TrianglePolygon Constructor Leads to Crash (`#1434 <https://github.com/ros2/rviz/issues/1434>`__)
* BillboardLine::addPoint() does not throw an exception when exceeding max_points_per_line limit (`#1436 <https://github.com/ros2/rviz/issues/1436>`__)
* Constructor ScrewVisual::ScrewVisual does not handle null pointers, leading to crashes (`#1435 <https://github.com/ros2/rviz/issues/1435>`__)
* Removed Windows warnings (`#1413 <https://github.com/ros2/rviz/issues/1413>`__)
* Memory Access Error When Handling Empty Strings in splitStringIntoTrimmedItems Function (`#1412 <https://github.com/ros2/rviz/issues/1412>`__)
* Crash due to Unhandled Null Pointer in ParameterEventsFilter Constructor (`#1411 <https://github.com/ros2/rviz/issues/1411>`__)
* MovableText constructor does not validate invalid character height, default fallback missing (`#1398 <https://github.com/ros2/rviz/issues/1398>`__)
* Invalid Parameter Handling in CovarianceVisual::CovarianceVisual Constructor (`#1396 <https://github.com/ros2/rviz/issues/1396>`__)
* Lack of Validity Check for Invalid Parameters in EffortVisual::EffortVisual Constructor (`#1395 <https://github.com/ros2/rviz/issues/1395>`__)
* Grid Class Constructor Does Not Handle Null Pointer, Leading to Program Crash (`#1394 <https://github.com/ros2/rviz/issues/1394>`__)
* Crash in MovableText::update() when caption is an empty string due to uninitialized resource usage (`#1393 <https://github.com/ros2/rviz/issues/1393>`__)
* Work in progress using the new resource retriever apis (`#1262 <https://github.com/ros2/rviz/issues/1262>`__)
* Contributors: Alejandro Hernández Cordero, Daisuke Nishimatsu, John TGZ, Michael Carlstrom, Michael Carroll, Michel Hidalgo, Nathan Brooks, Shane Loretz, matthias88, mergify[bot], mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rviz_rendering_tests <https://github.com/ros2/rviz/tree/lyrical/rviz_rendering_tests/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use rosdep keys that select Qt5 or Qt6 by platform (`#1720 <https://github.com/ros2/rviz/issues/1720>`__)
* Fix Qt version resolution when both Qt5 and Qt6 are installed - CMake defaults to ascending resolution and Qt5 will be found when Qt6 is desired (Rolling, L-Turtle, and beyond). (`#1689 <https://github.com/ros2/rviz/issues/1689>`__)
* Updated deprecated ament_index_cpp API (`#1647 <https://github.com/ros2/rviz/issues/1647>`__)
* Use qt6 as the default dependency from rosdep (`#1635 <https://github.com/ros2/rviz/issues/1635>`__)
* feat: support both qt5 and qt6 (`#1187 <https://github.com/ros2/rviz/issues/1187>`__)
* Work in progress using the new resource retriever apis (`#1262 <https://github.com/ros2/rviz/issues/1262>`__)
* Contributors: Alejandro Hernández Cordero, Daisuke Nishimatsu, Michael Carroll, Nathan Brooks, Shane Loretz


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rviz_visual_testing_framework <https://github.com/ros2/rviz/tree/lyrical/rviz_visual_testing_framework/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use rosdep keys that select Qt5 or Qt6 by platform (`#1720 <https://github.com/ros2/rviz/issues/1720>`__)
* Use new ROSIDL aggregate CMake target (`#1688 <https://github.com/ros2/rviz/issues/1688>`__)
* Fix Qt version resolution when both Qt5 and Qt6 are installed - CMake defaults to ascending resolution and Qt5 will be found when Qt6 is desired (Rolling, L-Turtle, and beyond). (`#1689 <https://github.com/ros2/rviz/issues/1689>`__)
* Use get_package_share_path (`#1671 <https://github.com/ros2/rviz/issues/1671>`__)
* Update ament_index_cpp API (`#1649 <https://github.com/ros2/rviz/issues/1649>`__)
* Use qt6 as the default dependency from rosdep (`#1635 <https://github.com/ros2/rviz/issues/1635>`__)
* Removed deprecation warning in tf2 (`#1585 <https://github.com/ros2/rviz/issues/1585>`__)
* Replace deprecated tf2_ros headers (`#1529 <https://github.com/ros2/rviz/issues/1529>`__)
* feat: support both qt5 and qt6 (`#1187 <https://github.com/ros2/rviz/issues/1187>`__)
* Contributors: Alejandro Hernández Cordero, Daisuke Nishimatsu, Emerson Knapp, Nathan Brooks, Shane Loretz


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`sensor_msgs <https://github.com/ros2/common_interfaces/tree/lyrical/sensor_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [ADD] missing PointField type entries (`#301 <https://github.com/ros2/common_interfaces/issues/301>`__)
* Update point_cloud2_iterator.hpp (`#298 <https://github.com/ros2/common_interfaces/issues/298>`__)
* Fix CMAKE deprecation (`#288 <https://github.com/ros2/common_interfaces/issues/288>`__)
* Enhance NV12 and NV21 Support in sensor_msgs::image_encodings (`#264 <https://github.com/ros2/common_interfaces/issues/264>`__)
* Contributors: Adam Leeper, Zhaoyuan Cheng, mosfet80, wodtko


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`sensor_msgs_py <https://github.com/ros2/common_interfaces/tree/lyrical/sensor_msgs_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use structured NumPy points.dtype.itemsize as default point_step in create_cloud (`#295 <https://github.com/ros2/common_interfaces/issues/295>`__)
* fix setuptools deprecation (`#293 <https://github.com/ros2/common_interfaces/issues/293>`__)
* Contributors: mosfet80, xndcn


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`service_msgs <https://github.com/ros2/rcl_interfaces/tree/lyrical/service_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix cmake deprecation (`#180 <https://github.com/ros2/rcl_interfaces/issues/180>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`shape_msgs <https://github.com/ros2/common_interfaces/tree/lyrical/shape_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix CMAKE deprecation (`#288 <https://github.com/ros2/common_interfaces/issues/288>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`spdlog_vendor <https://github.com/ros2/spdlog_vendor/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Remove CODEOWNERS and mirror-rolling-to-master. (`#38 <https://github.com/ros2/spdlog_vendor/issues/38>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`sros2 <https://github.com/ros2/sros2/tree/lyrical/sros2/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix ``load_file_into_BIO: File could not be found, opened or is empty`` error on Windows (`#386 <https://github.com/ros2/sros2/issues/386>`__) (`#387 <https://github.com/ros2/sros2/issues/387>`__)
* python3-pytest-timeout is missing for test dependency. (`#377 <https://github.com/ros2/sros2/issues/377>`__)
* Clean up isolated ros2 daemon process for tests. (`#375 <https://github.com/ros2/sros2/issues/375>`__)
* Remove importlib (`#368 <https://github.com/ros2/sros2/issues/368>`__)
* Timezone aware datetimes + remove hack from `#209 <https://github.com/ros2/sros2/issues/209>`__ (`#300 <https://github.com/ros2/sros2/issues/300>`__)
* fix setuptools deprecations (`#357 <https://github.com/ros2/sros2/issues/357>`__)
* Use rmw_test_fixture to isolate ros2cli tests (`#356 <https://github.com/ros2/sros2/issues/356>`__)
* update utilities to pass instance not class of ec.SECP256R1 (`#352 <https://github.com/ros2/sros2/issues/352>`__)
* suppress multi-threaded warnings. (`#346 <https://github.com/ros2/sros2/issues/346>`__)
* Switch to get_rmw_additional_env (`#339 <https://github.com/ros2/sros2/issues/339>`__)
* Fix github-workflow mypy error (`#336 <https://github.com/ros2/sros2/issues/336>`__)
* Contributors: Michael Carlstrom, Mikael Arguedas, Scott K Logan, Tomoya Fujita, cdisco, mergify[bot], mosfet80, yadunund


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`sros2_cmake <https://github.com/ros2/sros2/tree/lyrical/sros2_cmake/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update CMakeLists.txt (`#344 <https://github.com/ros2/sros2/issues/344>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`statistics_msgs <https://github.com/ros2/rcl_interfaces/tree/lyrical/statistics_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix cmake deprecation (`#180 <https://github.com/ros2/rcl_interfaces/issues/180>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`std_msgs <https://github.com/ros2/common_interfaces/tree/lyrical/std_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix CMAKE deprecation (`#288 <https://github.com/ros2/common_interfaces/issues/288>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`std_srvs <https://github.com/ros2/common_interfaces/tree/lyrical/std_srvs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix CMAKE deprecation (`#288 <https://github.com/ros2/common_interfaces/issues/288>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`stereo_msgs <https://github.com/ros2/common_interfaces/tree/lyrical/stereo_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix CMAKE deprecation (`#288 <https://github.com/ros2/common_interfaces/issues/288>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tango_icons_vendor <https://github.com/ros-visualization/tango_icons_vendor/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix cmake deprecation (`#15 <https://github.com/ros-visualization/tango_icons_vendor/issues/15>`__)
* Remove the mirror-rolling-to-master workflow (`#12 <https://github.com/ros-visualization/tango_icons_vendor/issues/12>`__)
* Remove CODEOWNERS (`#11 <https://github.com/ros-visualization/tango_icons_vendor/issues/11>`__)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_cli <https://github.com/ros2/system_tests/tree/lyrical/test_cli/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix CMAKE deprecation (`#572 <https://github.com/ros2/system_tests/issues/572>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_cli_remapping <https://github.com/ros2/system_tests/tree/lyrical/test_cli_remapping/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#587 <https://github.com/ros2/system_tests/issues/587>`__)
* fix CMAKE deprecation (`#572 <https://github.com/ros2/system_tests/issues/572>`__)
* Contributors: Emerson Knapp, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_communication <https://github.com/ros2/system_tests/tree/lyrical/test_communication/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#587 <https://github.com/ros2/system_tests/issues/587>`__)
* disable interoperability check for CycloneDDS and FastRTPS for WString (`#586 <https://github.com/ros2/system_tests/issues/586>`__)
* Fix index (`#585 <https://github.com/ros2/system_tests/issues/585>`__)
* Update subscription callback signatures (`#575 <https://github.com/ros2/system_tests/issues/575>`__)
* get rid of deprecated rclcpp::spin_some(). (`#574 <https://github.com/ros2/system_tests//issues/574>`__)
* fix CMAKE deprecation (`#572 <https://github.com/ros2/system_tests/issues/572>`__)
* Use EnableRmwIsolation in launch tests (`#571 <https://github.com/ros2/system_tests/issues/571>`__)
* Switch to isolated test fixture macros (`#571 <https://github.com/ros2/system_tests/issues/571>`__)
* Add tests for Keyed types (`#568 <https://github.com/ros2/system_tests/issues/568>`__)
* Remove use of ament_target_dependencies (`#566 <https://github.com/ros2/system_tests/issues/566>`__)
* Skip all multi-vendor pub/sub tests with zenoh (`#560 <https://github.com/ros2/system_tests/issues/560>`__)
* Contributors: Alejandro Hernández Cordero, Emerson Knapp, Francisco Gallego Salido, Janosch Machowinski, Michael Carlstrom, Scott K Logan, Shane Loretz, Tomoya Fujita, mini-1235, mosfet80, yadunund


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_interface_files <https://github.com/ros2/test_interface_files/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update CMakeLists.txt (`#26 <https://github.com/ros2/test_interface_files/issues/26>`__)
* Remove CODEOWNERS and mirror-rolling-to-master workflow. (`#23 <https://github.com/ros2/test_interface_files/issues/23>`__)
* Contributors: Chris Lalancette, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_launch_ros <https://github.com/ros2/launch_ros/tree/lyrical/test_launch_ros/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add tests for new component container refactor (`#536 <https://github.com/ros2/launch_ros/issues/536>`__)
* Surpressing multi-threaded process warning from flake8. (`#520 <https://github.com/ros2/launch_ros//issues/520>`__)
* correct typos (`#524 <https://github.com/ros2/launch_ros//issues/524>`__)
* set PYTHONUNBUFFERED to 1 to avoid hangs due to lost buffers (`#519 <https://github.com/ros2/launch_ros//issues/519>`__)
* Make FindPackage substitutions a Path to get operator / (`#494 <https://github.com/ros2/launch_ros/issues/494>`__)
* Expose lifecycle_node (`#327 <https://github.com/ros2/launch_ros/issues/327>`__) (with test) (`#482 <https://github.com/ros2/launch_ros/issues/482>`__)
* Switch osrf_pycommon dependency to system package (`#431 <https://github.com/ros2/launch_ros/issues/431>`__)
* Fix SetUseSimTime for launch frontends (`#488 <https://github.com/ros2/launch_ros/issues/488>`__)
* fix setuptools deprecations (`#475 <https://github.com/ros2/launch_ros/issues/475>`__)
* Fix: LoadComposableNodes fails to parse wildcard param files correctly (`#460 <https://github.com/ros2/launch_ros/issues/460>`__) (`#465 <https://github.com/ros2/launch_ros/issues/465>`__)
* Contributors: Auguste Lalande, Christophe Bedard, Clara Berendsen, Emerson Knapp, Emre Kuru, Jasper van Brakel, Scott K Logan, Skyler Medeiros, Tomoya Fujita, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_launch_testing <https://github.com/ros2/launch/tree/lyrical/test_launch_testing/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix CMake deprecation (`#899 <https://github.com/ros2/launch/issues/899>`__)
* Allow Path in substitutions, instead of requiring cast to str (`#873 <https://github.com/ros2/launch/issues/873>`__)
* Contributors: Emerson Knapp, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_msgs <https://github.com/ros2/rcl_interfaces/tree/lyrical/test_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add ``ament_cmake_mypy`` to ``test_msgs`` (`#187 <https://github.com/ros2/rcl_interfaces/issues/187>`__)
* Fix cmake deprecation (`#180 <https://github.com/ros2/rcl_interfaces/issues/180>`__)
* Contributors: Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_osrf_testing_tools_cpp <https://github.com/osrf/osrf_testing_tools_cpp/tree/lyrical/test_osrf_testing_tools_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix cmake deprecation  (`#94 <https://github.com/osrf/osrf_testing_tools_cpp/issues/94>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_quality_of_service <https://github.com/ros2/system_tests/tree/lyrical/test_quality_of_service/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#587 <https://github.com/ros2/system_tests/issues/587>`__)
* fix CMAKE deprecation (`#572 <https://github.com/ros2/system_tests/issues/572>`__)
* Switch to isolated test fixture macros (`#571 <https://github.com/ros2/system_tests/issues/571>`__)
* Use rmw_event_type_is_supported to skip tests (`#563 <https://github.com/ros2/system_tests/issues/563>`__)
* Contributors: Alejandro Hernández Cordero, Emerson Knapp, Scott K Logan, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_rclcpp <https://github.com/ros2/system_tests/tree/lyrical/test_rclcpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add tests isolation in test_rclcpp (`#583 <https://github.com/ros2/system_tests/issues/583>`__)
* info message comes from deferred signal handler with another thread. (`#576 <https://github.com/ros2/system_tests/issues/576>`__)
* get rid of deprecated rclcpp::spin_some(). (`#574 <https://github.com/ros2/system_tests//issues/574>`__)
* fix CMAKE deprecation (`#572 <https://github.com/ros2/system_tests/issues/572>`__)
* Use EnableRmwIsolation in launch tests (`#571 <https://github.com/ros2/system_tests/issues/571>`__)
* Ensure test verifies the existence of all spawning nodes (`#558 <https://github.com/ros2/system_tests/issues/558>`__)
* Contributors: Alejandro Hernández Cordero, Julien Enoch, Scott K Logan, Tomoya Fujita, Yuyuan Yuan, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_rmw_implementation <https://github.com/ros2/rmw_implementation/tree/lyrical/test_rmw_implementation/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new aggregate rosidl target instead of _TARGETS (`#276 <https://github.com/ros2/rmw_implementation/issues/276>`__)
* test_rmw_implementation: add test isolation (`#275 <https://github.com/ros2/rmw_implementation/issues/275>`__)
* Add rmw_get_clients_info_by_service , rmw_servers_clients_info_by_service (`#238 <https://github.com/ros2/rmw_implementation/issues/238>`__)
* fix cmake deprecation (`#267 <https://github.com/ros2/rmw_implementation/issues/267>`__)
* Test failing deserialization of invalid sequence length (`#261 <https://github.com/ros2/rmw_implementation/issues/261>`__)
* add ignore_local_publications_serialized test. (`#255 <https://github.com/ros2/rmw_implementation/issues/255>`__)
* Contributors: Alexis Tsogias, Julien Enoch, Lee, Miguel Company, Minju, Tomoya Fujita, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_ros2trace <https://github.com/ros2/ros2_tracing/tree/lyrical/test_ros2trace/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Skip test_ros2trace's tracing tests for now (`#218 <https://github.com/ros2/ros2_tracing/issues/218>`__)
* Allow creating snapshot sessions (`#195 <https://github.com/ros2/ros2_tracing/issues/195>`__)
* Only check test process events in test_runtime_disable (`#193 <https://github.com/ros2/ros2_tracing/issues/193>`__)
* Add runtime tracing opt-out mechanism (`#185 <https://github.com/ros2/ros2_tracing/issues/185>`__)
* fix setuptools deprecation (`#189 <https://github.com/ros2/ros2_tracing/issues/189>`__)
* Use timeout for everything in test_ros2trace tests (`#174 <https://github.com/ros2/ros2_tracing/issues/174>`__)
* Contributors: Christophe Bedard, Michel Hidalgo, Shravan Deva, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_security <https://github.com/ros2/system_tests/tree/lyrical/test_security/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#587 <https://github.com/ros2/system_tests/issues/587>`__)
* fix CMAKE deprecation (`#572 <https://github.com/ros2/system_tests/issues/572>`__)
* Contributors: Emerson Knapp, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_tf2 <https://github.com/ros2/geometry2/tree/lyrical/test_tf2/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix typos (`#921 <https://github.com/ros2/geometry2/issues/921>`__)
* Use new ROSIDL aggregate CMake target (`#907 <https://github.com/ros2/geometry2/issues/907>`__)
* added toMsg for eigen-accel as well as its tests (`#887 <https://github.com/ros2/geometry2/issues/887>`__)
* Move \author tags to \file \brief (`#870 <https://github.com/ros2/geometry2/issues/870>`__)
* Modernize conf.py files to only include modified Copyright, eliminati… (`#865 <https://github.com/ros2/geometry2/issues/865>`__)
* Adding NodeInterfaces API Design (`#714 <https://github.com/ros2/geometry2/issues/714>`__)
* Change tf2_ros C to C++ headers (`#805 <https://github.com/ros2/geometry2/issues/805>`__)
* Uniform cmake min version (`#764 <https://github.com/ros2/geometry2/issues/764>`__)
* Add ``rclcpp::shutdown`` (`#762 <https://github.com/ros2/geometry2/issues/762>`__)
* Contributors: Alireza Moayyedi, Auguste Lalande, Emerson Knapp, Gary Servin, Lucas Wendland, R Kent James, Yuyuan Yuan, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_tracetools <https://github.com/ros2/ros2_tracing/tree/lyrical/test_tracetools/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix: Fixed compiation on MSVC 2022 (`#243 <https://github.com/ros2/ros2_tracing/issues/243>`__)
* Use new ROSIDL aggregate CMake target (`#238 <https://github.com/ros2/ros2_tracing/issues/238>`__)
* Support tracepoints for complex message flow annotation used by ROS 2 plugin of Eclipse Trace Compass (`#233 <https://github.com/ros2/ros2_tracing/issues/233>`__)
* Update subscription callback signatures (`#217 <https://github.com/ros2/ros2_tracing/issues/217>`__)
* Add runtime tracing opt-out mechanism (`#185 <https://github.com/ros2/ros2_tracing/issues/185>`__)
* Update CMakeLists.txt (`#176 <https://github.com/ros2/ros2_tracing/issues/176>`__)
* Contributors: Emerson Knapp, Janosch Machowinski, Michel Hidalgo, Raphael van Kempen, mini-1235, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_tracetools_launch <https://github.com/ros2/ros2_tracing/tree/lyrical/test_tracetools_launch/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Allow creating snapshot sessions (`#195 <https://github.com/ros2/ros2_tracing/issues/195>`__)
* fix setuptools deprecation (`#189 <https://github.com/ros2/ros2_tracing/issues/189>`__)
* Make trace action parameters substitutable for xml and yaml launch files (`#188 <https://github.com/ros2/ros2_tracing/issues/188>`__)
* Make trace action parameters substitutable (`#187 <https://github.com/ros2/ros2_tracing/issues/187>`__)
* Address typing issues reported by mypy in tracetools_launch (`#184 <https://github.com/ros2/ros2_tracing/issues/184>`__)
* Contributors: Christophe Bedard, Shravan Deva, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2 <https://github.com/ros2/geometry2/tree/lyrical/tf2/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Added tests for static cache (`#920 <https://github.com/ros2/geometry2/issues/920>`__)
* Replacing with clean index-based iteration and avoid division by zero (`#901 <https://github.com/ros2/geometry2/issues/901>`__)
* fix typos (`#921 <https://github.com/ros2/geometry2/issues/921>`__)
* Fix StaticCache::getData() returning true on empty cache (`#908 <https://github.com/ros2/geometry2/issues/908>`__)
* Use new ROSIDL aggregate CMake target (`#907 <https://github.com/ros2/geometry2/issues/907>`__)
* Fix CPP style in tf2 (`#902 <https://github.com/ros2/geometry2/issues/902>`__)
* local variable tf2 no longer shadows the tf2:: (`#903 <https://github.com/ros2/geometry2/issues/903>`__)
* Replaced char* with std::string (`#904 <https://github.com/ros2/geometry2/issues/904>`__)
* Fix misleading extrapolation time in buffer_core (`#832 <https://github.com/ros2/geometry2/issues/832>`__) (`#896 <https://github.com/ros2/geometry2/issues/896>`__)
* static function to crate quaternions directly from rotation added (`#881 <https://github.com/ros2/geometry2/issues/881>`__)
* Expose Doxygen output in tf2, showing former Doxygen front page also as README.md (`#871 <https://github.com/ros2/geometry2/issues/871>`__)
* Move \author tags to \file \brief (`#870 <https://github.com/ros2/geometry2/issues/870>`__)
* Modernize conf.py files to only include modified Copyright, eliminati… (`#865 <https://github.com/ros2/geometry2/issues/865>`__)
* Fix various documentation errors in tf2 (`#857 <https://github.com/ros2/geometry2/issues/857>`__)
* Disable TAGFILES in rosdoc2 to separate namespace tf2 documentation into packages (`#856 <https://github.com/ros2/geometry2/issues/856>`__)
* Fix REP url locations (`#847 <https://github.com/ros2/geometry2/issues/847>`__)
* Adding explicit handling for normalization of zero-quaternions (`#839 <https://github.com/ros2/geometry2/issues/839>`__)
* Cleanup TF2 dependencies (`#843 <https://github.com/ros2/geometry2/issues/843>`__)
* Added tf2 documentation to docs.ros.org (`#671 <https://github.com/ros2/geometry2/issues/671>`__)
* Add RPY quaternion constructor (`#806 <https://github.com/ros2/geometry2/issues/806>`__)
* Default initialize TransformStorage's frame_id\_ and child_frame_id\_ with UINT32_MAX (`#783 <https://github.com/ros2/geometry2/issues/783>`__)
* Removed deprecated headers tf2 (`#789 <https://github.com/ros2/geometry2/issues/789>`__)
* Add isnan support (`#780 <https://github.com/ros2/geometry2/issues/780>`__)
* Overflow Issue in durationFromSec() Function when Handling Extremely Large or Small Values (`#785 <https://github.com/ros2/geometry2/issues/785>`__)
* Do not clobber callback handles when cancelling pending transformable requests (`#779 <https://github.com/ros2/geometry2/issues/779>`__)
* Uniform cmake min version (`#764 <https://github.com/ros2/geometry2/issues/764>`__)
* Contributors: Alejandro Hernández Cordero, Alireza Moayyedi, Andreas, Auguste Lalande, Chris Lalancette, Emerson Knapp, Markus Bader, Michael Carlstrom, Pavel Guzenfeld, R Kent James, Selim Ağırman, Simon Jusner, Tim Clephas, Timo Röhling, cramke, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_bullet <https://github.com/ros2/geometry2/tree/lyrical/tf2_bullet/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#907 <https://github.com/ros2/geometry2/issues/907>`__)
* Move \author tags to \file \brief (`#870 <https://github.com/ros2/geometry2/issues/870>`__)
* Modernize conf.py files to only include modified Copyright, eliminati… (`#865 <https://github.com/ros2/geometry2/issues/865>`__)
* Disable TAGFILES in rosdoc2 to separate namespace tf2 documentation into packages (`#856 <https://github.com/ros2/geometry2/issues/856>`__)
* Set Cmake Policy CMP0144 (`#819 <https://github.com/ros2/geometry2/issues/819>`__)
* Change tf2_ros C to C++ headers (`#805 <https://github.com/ros2/geometry2/issues/805>`__)
* Uniform cmake min version (`#764 <https://github.com/ros2/geometry2/issues/764>`__)
* Contributors: Cristóbal Arroyo, Emerson Knapp, Gary Servin, R Kent James, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_eigen <https://github.com/ros2/geometry2/tree/lyrical/tf2_eigen/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix typos (`#921 <https://github.com/ros2/geometry2/issues/921>`__)
* Use new ROSIDL aggregate CMake target (`#907 <https://github.com/ros2/geometry2/issues/907>`__)
* added toMsg for eigen-accel as well as its tests (`#887 <https://github.com/ros2/geometry2/issues/887>`__)
* Move \author tags to \file \brief (`#870 <https://github.com/ros2/geometry2/issues/870>`__)
* Modernize conf.py files to only include modified Copyright, eliminati… (`#865 <https://github.com/ros2/geometry2/issues/865>`__)
* Add fromMsg for converting from Accel to Eigen (`#844 <https://github.com/ros2/geometry2/issues/844>`__)
* Disable TAGFILES in rosdoc2 to separate namespace tf2 documentation into packages (`#856 <https://github.com/ros2/geometry2/issues/856>`__)
* Change tf2_ros C to C++ headers (`#805 <https://github.com/ros2/geometry2/issues/805>`__)
* Uniform cmake min version (`#764 <https://github.com/ros2/geometry2/issues/764>`__)
* Contributors: Alireza Moayyedi, Auguste Lalande, Emerson Knapp, Gary Servin, R Kent James, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_eigen_kdl <https://github.com/ros2/geometry2/tree/lyrical/tf2_eigen_kdl/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Modernize conf.py files to only include modified Copyright, eliminati… (`#865 <https://github.com/ros2/geometry2/issues/865>`__)
* Disable TAGFILES in rosdoc2 to separate namespace tf2 documentation into packages (`#856 <https://github.com/ros2/geometry2/issues/856>`__)
* Cleanup mislabeled BSD license (`#855 <https://github.com/ros2/geometry2/issues/855>`__)
* Removed orocos kdl vendor dependency (`#826 <https://github.com/ros2/geometry2/issues/826>`__)
* Uniform cmake min version (`#764 <https://github.com/ros2/geometry2/issues/764>`__)
* Contributors: Alejandro Hernández Cordero, R Kent James, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_geometry_msgs <https://github.com/ros2/geometry2/tree/lyrical/tf2_geometry_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix typos (`#921 <https://github.com/ros2/geometry2/issues/921>`__)
* fix: doTransform of VelocityStamped added input vector after transform (`#909 <https://github.com/ros2/geometry2/issues/909>`__)
* Use new ROSIDL aggregate CMake target (`#907 <https://github.com/ros2/geometry2/issues/907>`__)
* Copy child_frame_id from input (`#889 <https://github.com/ros2/geometry2/issues/889>`__)
* Move \author tags to \file \brief (`#870 <https://github.com/ros2/geometry2/issues/870>`__)
* Modernize conf.py files to only include modified Copyright, eliminati… (`#865 <https://github.com/ros2/geometry2/issues/865>`__)
* Removed orocos kdl vendor dependency (`#826 <https://github.com/ros2/geometry2/issues/826>`__)
* Change tf2_ros C to C++ headers (`#805 <https://github.com/ros2/geometry2/issues/805>`__)
* Contributors: Alejandro Hernández Cordero, Auguste Lalande, Emerson Knapp, Gary Servin, R Kent James, Yannik Meinken, cramke


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_kdl <https://github.com/ros2/geometry2/tree/lyrical/tf2_kdl/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#907 <https://github.com/ros2/geometry2/issues/907>`__)
* Move \author tags to \file \brief (`#870 <https://github.com/ros2/geometry2/issues/870>`__)
* Documentation fixes for tf2_kdl (`#869 <https://github.com/ros2/geometry2/issues/869>`__)
* Modernize conf.py files to only include modified Copyright, eliminati… (`#865 <https://github.com/ros2/geometry2/issues/865>`__)
* Disable TAGFILES in rosdoc2 to separate namespace tf2 documentation into packages (`#856 <https://github.com/ros2/geometry2/issues/856>`__)
* Removed orocos kdl vendor dependency (`#826 <https://github.com/ros2/geometry2/issues/826>`__)
* Change tf2_ros C to C++ headers (`#805 <https://github.com/ros2/geometry2/issues/805>`__)
* Uniform cmake min version (`#764 <https://github.com/ros2/geometry2/issues/764>`__)
* Fix external docs mappings (`#757 <https://github.com/ros2/geometry2/issues/757>`__)
* Contributors: Alejandro Hernández Cordero, Emerson Knapp, Emmanuel, Gary Servin, R Kent James, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_msgs <https://github.com/ros2/geometry2/tree/lyrical/tf2_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix typos (`#921 <https://github.com/ros2/geometry2/issues/921>`__)
* Modernize conf.py files to only include modified Copyright, eliminati… (`#865 <https://github.com/ros2/geometry2/issues/865>`__)
* Uniform cmake min version (`#764 <https://github.com/ros2/geometry2/issues/764>`__)
* Contributors: Auguste Lalande, R Kent James, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_py <https://github.com/ros2/geometry2/tree/lyrical/tf2_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix typos (`#921 <https://github.com/ros2/geometry2/issues/921>`__)
* Use new ROSIDL aggregate CMake target (`#907 <https://github.com/ros2/geometry2/issues/907>`__)
* Modernize conf.py files to only include modified Copyright, eliminati… (`#865 <https://github.com/ros2/geometry2/issues/865>`__)
* Cleanup TF2 dependencies (`#843 <https://github.com/ros2/geometry2/issues/843>`__)
* Contributors: Auguste Lalande, Chris Lalancette, Emerson Knapp, R Kent James


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_ros <https://github.com/ros2/geometry2/tree/lyrical/tf2_ros/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix typos (`#921 <https://github.com/ros2/geometry2/issues/921>`__)
* Use new ROSIDL aggregate CMake target (`#907 <https://github.com/ros2/geometry2/issues/907>`__)
* Move \author tags to \file \brief (`#870 <https://github.com/ros2/geometry2/issues/870>`__)
* Modernize conf.py files to only include modified Copyright, eliminati… (`#865 <https://github.com/ros2/geometry2/issues/865>`__)
* Disable TAGFILES in rosdoc2 to separate namespace tf2 documentation into packages (`#856 <https://github.com/ros2/geometry2/issues/856>`__)
* Prevent log spam from tf2_ros message_filter (`#851 <https://github.com/ros2/geometry2/issues/851>`__)
* Updated tf2_echo with some other features (`#802 <https://github.com/ros2/geometry2/issues/802>`__) (`#840 <https://github.com/ros2/geometry2/issues/840>`__)
* Replace std::sleep_for with rclcpp::clock::sleep_for (`#835 <https://github.com/ros2/geometry2/issues/835>`__)
* Removed deprecation rclcpp::spin_some(node) (`#824 <https://github.com/ros2/geometry2/issues/824>`__)
* Adding NodeInterfaces API Design (`#714 <https://github.com/ros2/geometry2/issues/714>`__)
* ger rid of deprecated rclcpp::spin_some(). (`#821 <https://github.com/ros2/geometry2/issues/821>`__)
* Ensure variable is considered volatile in message_filter_test (`#812 <https://github.com/ros2/geometry2/issues/812>`__)
* Change tf2_ros C to C++ headers (`#805 <https://github.com/ros2/geometry2/issues/805>`__)
* Fix message filter target frames string (`#803 <https://github.com/ros2/geometry2/issues/803>`__)
* Remove deprecation warnings (`#790 <https://github.com/ros2/geometry2/issues/790>`__)
* Uniform cmake min version (`#764 <https://github.com/ros2/geometry2/issues/764>`__)
* Add ``rclcpp::shutdown`` (`#762 <https://github.com/ros2/geometry2/issues/762>`__)
* Fix external docs mappings (`#757 <https://github.com/ros2/geometry2/issues/757>`__)
* Contributors: Alejandro Hernández Cordero, Auguste Lalande, Emerson Knapp, Emmanuel, Gary Servin, Lucas Wendland, Mirko Ferrati, R Kent James, Sergei Zobov, Tomoya Fujita, Yuyuan Yuan, mergify[bot], mini-1235, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_ros_py <https://github.com/ros2/geometry2/tree/lyrical/tf2_ros_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix typos (`#921 <https://github.com/ros2/geometry2/issues/921>`__)
* flake8 fixes (`#919 <https://github.com/ros2/geometry2/issues/919>`__)
* prevent AttributeError when static_only=true (`#906 <https://github.com/ros2/geometry2/issues/906>`__)
* fixed typoe in buffer.py (`#905 <https://github.com/ros2/geometry2/issues/905>`__)
* Increase robustness of listener and broadcaster test (`#894 <https://github.com/ros2/geometry2/issues/894>`__)
* Modernize conf.py files to only include modified Copyright, eliminati… (`#865 <https://github.com/ros2/geometry2/issues/865>`__)
* Disable TAGFILES in rosdoc2 to separate namespace tf2 documentation into packages (`#856 <https://github.com/ros2/geometry2/issues/856>`__)
* Cleanup TF2 dependencies (`#843 <https://github.com/ros2/geometry2/issues/843>`__)
* Fixed inconsistency of C++ and Python implementations of StaticTransformPublisher (`#820 <https://github.com/ros2/geometry2/issues/820>`__)
* Fix deprecation warning (`#804 <https://github.com/ros2/geometry2/issues/804>`__)
* Remove deprecation warnings (`#790 <https://github.com/ros2/geometry2/issues/790>`__)
* Fix external docs mappings (`#757 <https://github.com/ros2/geometry2/issues/757>`__)
* Contributors: Alejandro Hernández Cordero, Auguste Lalande, Chris Lalancette, Dominik, Emmanuel, Michael Carlstrom, Michael Carroll, R Kent James, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_sensor_msgs <https://github.com/ros2/geometry2/tree/lyrical/tf2_sensor_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix typos (`#921 <https://github.com/ros2/geometry2/issues/921>`__)
* Use new ROSIDL aggregate CMake target (`#907 <https://github.com/ros2/geometry2/issues/907>`__)
* Modernize conf.py files to only include modified Copyright, eliminati… (`#865 <https://github.com/ros2/geometry2/issues/865>`__)
* Solved TODO with copyright in tf2_sensor_msgs (`#836 <https://github.com/ros2/geometry2/issues/836>`__)
* Removed orocos kdl vendor dependency (`#826 <https://github.com/ros2/geometry2/issues/826>`__)
* Add imu & mag support in ``tf2_sensor_msgs`` (`#800 <https://github.com/ros2/geometry2/issues/800>`__) (`#813 <https://github.com/ros2/geometry2/issues/813>`__)
* Change tf2_ros C to C++ headers (`#805 <https://github.com/ros2/geometry2/issues/805>`__)
* Add normals rotation in ``PointCloud2`` ``doTransform`` (`#792 <https://github.com/ros2/geometry2/issues/792>`__)
* Contributors: Alejandro Hernández Cordero, Auguste Lalande, Emerson Knapp, Gary Servin, Patrick Roncagliolo, R Kent James


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_tools <https://github.com/ros2/geometry2/tree/lyrical/tf2_tools/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Modernize conf.py files to only include modified Copyright, eliminati… (`#865 <https://github.com/ros2/geometry2/issues/865>`__)
* Fix Setuptools deprecations (`#809 <https://github.com/ros2/geometry2/issues/809>`__)
* Contributors: R Kent James, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tlsf <https://github.com/ros2/tlsf/tree/lyrical/tlsf/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* update cmake requirements (`#18 <https://github.com/ros2/tlsf/issues/18>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tlsf_cpp <https://github.com/ros2/realtime_support/tree/lyrical/tlsf_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* cleanups and removed dead code (`#141 <https://github.com/ros2/realtime_support/issues/141>`__) (`#144 <https://github.com/ros2/realtime_support/issues/144>`__)
* fix: Removed AllocatorMemoryStrategy (backport `#140 <https://github.com/ros2/realtime_support/issues/140>`__) (`#142 <https://github.com/ros2/realtime_support/issues/142>`__)
* Remove deprecation warnings (`#139 <https://github.com/ros2/realtime_support/issues/139>`__)
* Use new ROSIDL aggregate CMake target (`#137 <https://github.com/ros2/realtime_support/issues/137>`__)
* tlsf_cpp: add test isolation (`#136 <https://github.com/ros2/realtime_support/issues/136>`__)
* Update subscription callback signatures (`#135 <https://github.com/ros2/realtime_support/issues/135>`__)
* Fix cmake deprecation (`#134 <https://github.com/ros2/realtime_support/issues/134>`__)
* Explicitly shutdown context before test exits (`#129 <https://github.com/ros2/realtime_support/issues/129>`__)
* Contributors: Alejandro Hernández Cordero, Emerson Knapp, Julien Enoch, mergify[bot], mini-1235, mosfet80, yadunund


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`topic_monitor <https://github.com/ros2/demos/tree/lyrical/topic_monitor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add mypy config (`#776 <https://github.com/ros2/demos//issues/776>`__)
* Switching to example_interfaces (`#674 <https://github.com/ros2/demos/issues/674>`__)
* fix setuptools deprecations (`#733 <https://github.com/ros2/demos/issues/733>`__)
* Update README.md (`#718 <https://github.com/ros2/demos/issues/718>`__) (`#719 <https://github.com/ros2/demos/issues/719>`__)
* Contributors: Dan Mascarenhas, Lucas Wendland, mergify[bot], mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`topic_statistics_demo <https://github.com/ros2/demos/tree/lyrical/topic_statistics_demo/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use new ROSIDL aggregate CMake target (`#781 <https://github.com/ros2/demos//issues/781>`__)
* Switching to example_interfaces (`#674 <https://github.com/ros2/demos/issues/674>`__)
* Uniform CMAKE min VERSION (`#714 <https://github.com/ros2/demos/issues/714>`__)
* Contributors: Emerson Knapp, Lucas Wendland, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tracetools <https://github.com/ros2/ros2_tracing/tree/lyrical/tracetools/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Support tracepoints for complex message flow annotation used by ROS 2 plugin of Eclipse Trace Compass (`#233 <https://github.com/ros2/ros2_tracing/issues/233>`__)
* Removed warning (`#225 <https://github.com/ros2/ros2_tracing/issues/225>`__)
* Add runtime tracing opt-out mechanism (`#185 <https://github.com/ros2/ros2_tracing/issues/185>`__)
* Fix Clang warnings by using proper function prototypes in macros (`#179 <https://github.com/ros2/ros2_tracing/issues/179>`__)
* Update CMakeLists.txt (`#176 <https://github.com/ros2/ros2_tracing/issues/176>`__)
* Removed clang warning (`#168 <https://github.com/ros2/ros2_tracing/issues/168>`__)
* Contributors: Alejandro Hernández Cordero, Michel Hidalgo, Raphael van Kempen, Shravan Deva, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tracetools_launch <https://github.com/ros2/ros2_tracing/tree/lyrical/tracetools_launch/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* tracetools_launch: use parse_if_substitutions for non-string action params (`#234 <https://github.com/ros2/ros2_tracing/issues/234>`__)
* Add example launch files for snapshot mode (`#206 <https://github.com/ros2/ros2_tracing/issues/206>`__)
* Allow creating snapshot sessions (`#195 <https://github.com/ros2/ros2_tracing/issues/195>`__)
* Add launch files with preconfigured dual session (`#196 <https://github.com/ros2/ros2_tracing/issues/196>`__)
* Add support for starting tracing at runtime (`#191 <https://github.com/ros2/ros2_tracing/issues/191>`__)
* fix setuptools deprecation (`#189 <https://github.com/ros2/ros2_tracing/issues/189>`__)
* Make trace action parameters substitutable for xml and yaml launch files (`#188 <https://github.com/ros2/ros2_tracing/issues/188>`__)
* Make trace action parameters substitutable (`#187 <https://github.com/ros2/ros2_tracing/issues/187>`__)
* Address typing issues reported by mypy in tracetools_launch (`#184 <https://github.com/ros2/ros2_tracing/issues/184>`__)
* Contributors: Christophe Bedard, Sarthak Bagga, Shravan Deva, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tracetools_read <https://github.com/ros2/ros2_tracing/tree/lyrical/tracetools_read/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Work around segfault when reading trace with babeltrace1 Python API (`#246 <https://github.com/ros2/ros2_tracing/issues/246>`__)
* Ignore A0005 (`#237 <https://github.com/ros2/ros2_tracing/issues/237>`__)
* fix setuptools deprecation (`#189 <https://github.com/ros2/ros2_tracing/issues/189>`__)
* Address typing issues reported by mypy in tracetools_launch (`#184 <https://github.com/ros2/ros2_tracing/issues/184>`__)
* Contributors: Christophe Bedard, Michael Carlstrom, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tracetools_test <https://github.com/ros2/ros2_tracing/tree/lyrical/tracetools_test/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Set default values on TraceTestCase to avoid errors on >=8.2.0 pytest (`#236 <https://github.com/ros2/ros2_tracing/issues/236>`__)
* Only check test process events in test_runtime_disable (`#193 <https://github.com/ros2/ros2_tracing/issues/193>`__)
* fix setuptools deprecation (`#189 <https://github.com/ros2/ros2_tracing/issues/189>`__)
* Address typing issues reported by mypy in tracetools_launch (`#184 <https://github.com/ros2/ros2_tracing/issues/184>`__)
* Contributors: Christophe Bedard, Clara Berendsen, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tracetools_trace <https://github.com/ros2/ros2_tracing/tree/lyrical/tracetools_trace/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Support tracepoints for complex message flow annotation used by ROS 2 plugin of Eclipse Trace Compass (`#233 <https://github.com/ros2/ros2_tracing/issues/233>`__)
* Ignore A0005 (`#237 <https://github.com/ros2/ros2_tracing/issues/237>`__)
* Add exec_depend on procps to tracetools_trace for ps command (`#227 <https://github.com/ros2/ros2_tracing/issues/227>`__)
* Handle SIGTERM and gracefully stop tracing in interactive tracing mode (`#219 <https://github.com/ros2/ros2_tracing/issues/219>`__)
* Use overwrite mode for snapshot sessions (`#210 <https://github.com/ros2/ros2_tracing/issues/210>`__)
* Allow creating snapshot sessions (`#195 <https://github.com/ros2/ros2_tracing/issues/195>`__)
* Add support for starting tracing at runtime (`#191 <https://github.com/ros2/ros2_tracing/issues/191>`__)
* fix setuptools deprecation (`#189 <https://github.com/ros2/ros2_tracing/issues/189>`__)
* Address typing issues reported by mypy in tracetools_launch (`#184 <https://github.com/ros2/ros2_tracing/issues/184>`__)
* Warn if kernel might be paranoid about 'perf:thread:' context fields (`#173 <https://github.com/ros2/ros2_tracing/issues/173>`__)
* Fix pluralization in ros2 trace output (`#169 <https://github.com/ros2/ros2_tracing/issues/169>`__)
* Contributors: Christophe Bedard, Michael Carlstrom, Raphael van Kempen, Shravan Deva, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`trajectory_msgs <https://github.com/ros2/common_interfaces/tree/lyrical/trajectory_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix CMAKE deprecation (`#288 <https://github.com/ros2/common_interfaces/issues/288>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`turtlesim <https://github.com/ros/ros_tutorials/tree/lyrical/turtlesim/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add icon for Lyrical Luth (`#196 <https://github.com/ros/ros_tutorials/issues/196>`__) (`#197 <https://github.com/ros/ros_tutorials/issues/197>`__)
* Use rosdep keys that select Qt5 or Qt6 by platform (`#195 <https://github.com/ros/ros_tutorials/issues/195>`__)
* Use new ROSIDL aggregate CMake target (`#194 <https://github.com/ros/ros_tutorials/issues/194>`__)
* Use get_package_share_path (`#193 <https://github.com/ros/ros_tutorials/issues/193>`__)
* fix bug loading turtle images (`#192 <https://github.com/ros/ros_tutorials//issues/192>`__)
* Updated deprecated ament_index_cpp API (`#190 <https://github.com/ros/ros_tutorials/issues/190>`__)
* Use qt6 as the default dependency from rosdep (`#189 <https://github.com/ros/ros_tutorials/issues/189>`__)
* get rid of deprecated rclcpp::spin_some() (`#183 <https://github.com/ros/ros_tutorials/issues/183>`__)
* Support Qt6 (`#170 <https://github.com/ros/ros_tutorials/issues/170>`__)
* Add icon for Kilted Kaiju (`#180 <https://github.com/ros/ros_tutorials/issues/180>`__)
* Contributors: Alejandro Hernández Cordero, Emerson Knapp, Scott K Logan, Shane Loretz, dcconner, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`turtlesim_msgs <https://github.com/ros/ros_tutorials/tree/lyrical/turtlesim_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix: swap action and message file group names in CMakeLists.txt (`#186 <https://github.com/ros/ros_tutorials/issues/186>`__) (`#187 <https://github.com/ros/ros_tutorials/issues/187>`__)
* fix cmake deprecation (`#182 <https://github.com/ros/ros_tutorials/issues/182>`__)
* Contributors: mergify[bot], mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`type_description_interfaces <https://github.com/ros2/rcl_interfaces/tree/lyrical/type_description_interfaces/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix cmake deprecation (`#180 <https://github.com/ros2/rcl_interfaces/issues/180>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`uncrustify_vendor <https://github.com/ament/uncrustify_vendor/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Remove CODEOWNERS and mirror-rolling-to-master workflow. (`#38 <https://github.com/ament/uncrustify_vendor/issues/38>`__)
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`unique_identifier_msgs <https://github.com/ros2/unique_identifier_msgs/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix cmake deprecation (`#33 <https://github.com/ros2/unique_identifier_msgs/issues/33>`__)
* Remove CODEOWNERS and mirror-rolling-to-master workflow. (`#31 <https://github.com/ros2/unique_identifier_msgs/issues/31>`__)
* Contributors: Chris Lalancette, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`urdf <https://github.com/ros2/urdf/tree/lyrical/urdf/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Remove ``urdf_world/types.h`` deprecation (`#54 <https://github.com/ros2/urdf/issues/54>`__)
* Fix CMAKE deprecation (`#48 <https://github.com/ros2/urdf/issues/48>`__)
* Removed tinyxml2_vendor dependency (`#47 <https://github.com/ros2/urdf/issues/47>`__)
* Contributors: Alejandro Hernández Cordero, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`urdf_parser_plugin <https://github.com/ros2/urdf/tree/lyrical/urdf_parser_plugin/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Remove ``urdf_world/types.h`` deprecation (`#54 <https://github.com/ros2/urdf/issues/54>`__)
* Fix CMAKE deprecation (`#48 <https://github.com/ros2/urdf/issues/48>`__) cmake version < then 3.10 is deprecated
* Contributors: Alejandro Hernández Cordero, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`urdfdom <https://github.com/ros/urdfdom/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Support for URDF Specification 1.2 * Extend parsing of acceleration, deceleration and jerk limits from ``limit`` tag (`#212 <https://github.com/ros/urdfdom/issues/212>`__) * Update default limits for the joint limits and safety limits (`#249 <https://github.com/ros/urdfdom/issues/249>`__) * Add invalid data checks to the Geometry data (`#242 <https://github.com/ros/urdfdom/issues/242>`__) * Require urdfdom_headers 3.0.0 (`#257 <https://github.com/ros/urdfdom/issues/257>`__)
* Use URDF_MAJOR_VERSION for SOVERSION (`#248 <https://github.com/ros/urdfdom/issues/248>`__)
* Revert "Extend parsing of acceleration, deceleration and jerk limits from ``limit`` tag (`#212 <https://github.com/ros/urdfdom/issues/212>`__)" This was a breaking change that will be released in 6.0.0
* Prevent CI from failing fast to allow all builds to complete (`#254 <https://github.com/ros/urdfdom/issues/254>`__)
* Remove ``urdf_world/types.h`` deprecation (`#251 <https://github.com/ros/urdfdom/issues/251>`__)
* Extend parsing of acceleration, deceleration and jerk limits from ``limit`` tag (`#212 <https://github.com/ros/urdfdom/issues/212>`__)
* ROS 2 CI: build urdfdom_headers from source (`#246 <https://github.com/ros/urdfdom/issues/246>`__)
* Disable system workflow because ``urdfdom_headers`` isn't available on Ubuntu 24.04 (`#240 <https://github.com/ros/urdfdom/issues/240>`__)
* Fix ROS 2 CI workflow by updating Ubuntu version and checkout action (`#239 <https://github.com/ros/urdfdom/issues/239>`__)
* Support for URDF Specification 1.1 * Add support for capsule geometry type (`#238 <https://github.com/ros/urdfdom/issues/238>`__) * Add documentation about versioning * Require version 2.1.0 of urdfdom_headers Co-authored-by: Steve Peters <scpeters@openrobotics.org> * Support quaternions in URDF 1.1 (`#235 <https://github.com/ros/urdfdom/issues/235>`__) Co-authored-by: Guillaume Doisy <doisyg@users.noreply.github.com>
* Fix multiple format-string vulnerabilities in URDF parser logging (`#243 <https://github.com/ros/urdfdom/issues/243>`__) User-controlled URDF content was passed directly to CONSOLE_BRIDGE_logError() at multiple call sites, allowing printf-style format string interpretation. All affected logging paths now use explicit "%s" format specifiers to ensure input is treated as data and to prevent information disclosure or undefined behavior.
* More logging format string fixes (`#244 <https://github.com/ros/urdfdom/issues/244>`__) * Add explicit "%s" format strings when logging * Use %s format string instead of string addition
* Read cmake version from package.xml (`#236 <https://github.com/ros/urdfdom/issues/236>`__) * Use regex to match version string. Based on suggestion from Chris Lalancette. * Require cmake minimum version 3.10 Co-authored-by: Chris Lalancette <clalancette@gmail.com>
* Revert "Quaternion in urdf (PR123 new attempt) (#231)" (`#231 <https://github.com/ros/urdfdom/issues/231>`__)
* Quaternion in urdf (PR123 new attempt) (`#194 <https://github.com/ros/urdfdom/issues/194>`__)
* Removed tinyxml2_vendor dependency (`#225 <https://github.com/ros/urdfdom/issues/225>`__)
* Relax the version compatibility for urdfdom_headers. (`#222 <https://github.com/ros/urdfdom/issues/222>`__)
* Removed deprecated code (`#217 <https://github.com/ros/urdfdom/issues/217>`__)
* Remove ROS 1 workflows and update ROS 2 (`#218 <https://github.com/ros/urdfdom/issues/218>`__)
* Improvements for the URDF xsd specification  (`#200 <https://github.com/ros/urdfdom/issues/200>`__)
* Update ros2.yaml (`#214 <https://github.com/ros/urdfdom/issues/214>`__)
* fix: missing header (`#216 <https://github.com/ros/urdfdom/issues/216>`__)
* Contributors: Alejandro Hernández Cordero, Amin Ya, Chris Lalancette, Florencia, Guillaume Doisy, Jose Luis Rivero, Pierre Ballif, Sai Kishor Kothakota, Steve Peters, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`urdfdom_headers <https://github.com/ros/urdfdom_headers/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update lower, upper, effort, and velocity default joint limits (`#95 <https://github.com/ros/urdfdom_headers/issues/95>`__)
* Clean up declaration of ModelInterface's SharedPtrs (`#99 <https://github.com/ros/urdfdom_headers/issues/99>`__)
* Extend ``JointLimits`` class to include acceleration, deceleration and jerk limits (`#83 <https://github.com/ros/urdfdom_headers/issues/83>`__)
* Revert "Extend JointLimits class to include acceleration, deceleration and jerk limits (`#83 <https://github.com/ros/urdfdom_headers/issues/83>`__)" This was a breaking change that will be released in 3.0.0
* Clean up declaration of ModelInterface's SharedPtrs (`#99 <https://github.com/ros/urdfdom_headers/issues/99>`__)
* Revert cleanup of ModelInterface's SharedPtrs (`#33 <https://github.com/ros/urdfdom_headers/issues/33>`__)
* Revert fix for assumption that CMAKE_INSTALL_*DIR paths are relative (`#90 <https://github.com/ros/urdfdom_headers/issues/90>`__) (`#97 <https://github.com/ros/urdfdom_headers/issues/97>`__)
* Clean up declaration of ModelInterface's SharedPtrs (`#33 <https://github.com/ros/urdfdom_headers/issues/33>`__)
* Fix assumption that CMAKE_INSTALL_*DIR paths are relative (`#90 <https://github.com/ros/urdfdom_headers/issues/90>`__)
* Extend ``JointLimits`` class to include acceleration, deceleration and jerk limits (`#83 <https://github.com/ros/urdfdom_headers/issues/83>`__)
* Add support for capsule geometry type (`#94 <https://github.com/ros/urdfdom_headers/issues/94>`__)
* 2.0.2
* Read cmake version from package.xml (`#92 <https://github.com/ros/urdfdom_headers/issues/92>`__) Use regex to match version string. Copied from `ros/urdfdom#236 <https://github.com/ros/urdfdom/issues/236>`__.
* quaternions in urdf (PR 51 new attempt) + bump version (`#77 <https://github.com/ros/urdfdom_headers/issues/77>`__)
* fix cmake deprecation (`#89 <https://github.com/ros/urdfdom_headers/issues/89>`__) cmake version < then 3.10 is deprecated
* 2.0.0
* Remove all dependencies from the package.xml. (`#88 <https://github.com/ros/urdfdom_headers/issues/88>`__) This package does not have any header dependencies, so we don't need any of them here.
* Fix package.xml deps to use vendored packages (`#87 <https://github.com/ros/urdfdom_headers/issues/87>`__)
* add package.xml file from release repository (`#85 <https://github.com/ros/urdfdom_headers/issues/85>`__)
* Removed headers, implementation was deprecated and removed (`#86 <https://github.com/ros/urdfdom_headers/issues/86>`__)
* Remove CODEOWNERS. (`#81 <https://github.com/ros/urdfdom_headers/issues/81>`__) It is outdated and no longer serving its intended purpose.
* Contributors: Aarav Gupta, Alejandro Hernández Cordero, Chris Lalancette, Guillaume Doisy, Jorge J. Perez, Lucien Morey, Michal Sojka, Robert Haschke, Sai Kishor Kothakota, Steve Peters, mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`visualization_msgs <https://github.com/ros2/common_interfaces/tree/lyrical/visualization_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix CMAKE deprecation (`#288 <https://github.com/ros2/common_interfaces/issues/288>`__)
* Contributors: mosfet80


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`yaml_cpp_vendor <https://github.com/ros2/yaml_cpp_vendor/tree/lyrical/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Replace ament_vendor with cmake modules (`#56 <https://github.com/ros2/yaml_cpp_vendor/issues/56>`__)
* Remove CODEOWNERS and mirror-rolling-to-master workflow. (`#52 <https://github.com/ros2/yaml_cpp_vendor/issues/52>`__)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`zenoh_cpp_vendor <https://github.com/ros2/rmw_zenoh/tree/lyrical/zenoh_cpp_vendor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use zenoh-cpp 481b71b fixing build with MSVC 2022 in C++20 mode (`#969 <https://github.com/ros2/rmw_zenoh/issues/969>`__)
* Bump Zenoh to 1.8.0, fix Windows shutdown hang, and resolve synchronization with ``undeclare`` (`#964 <https://github.com/ros2/rmw_zenoh/issues/964>`__)
* Revert changes to build against rust >= 1.75 and bump zenoh to 1.8.0 (`#960 <https://github.com/ros2/rmw_zenoh/issues/960>`__)
* Revert patch of Cargo.lock with new Zenoh commit due to Windows test failures (`#959 <https://github.com/ros2/rmw_zenoh/issues/959>`__)
* Update Cargo.lock with new Zenoh commit (`#957 <https://github.com/ros2/rmw_zenoh/issues/957>`__)
* Build against ``rust >= 1.75`` for ROS Lyrical (`#945 <https://github.com/ros2/rmw_zenoh/issues/945>`__)
* Bump zenoh to 1.8.0 (`#935 <https://github.com/ros2/rmw_zenoh/issues/935>`__)
* Allow use of non-vendored Zenoh if present (`#908 <https://github.com/ros2/rmw_zenoh/issues/908>`__)
* Bump ``zenoh`` to 1.7.1 (`#870 <https://github.com/ros2/rmw_zenoh/issues/870>`__)
* Fix REP url locations (`#858 <https://github.com/ros2/rmw_zenoh/issues/858>`__)
* Bump zenoh to 1.6.2 (`#842 <https://github.com/ros2/rmw_zenoh/issues/842>`__)
* Bump Zenoh to 1.5.1 (`#774 <https://github.com/ros2/rmw_zenoh/issues/774>`__)
* Bump Zenoh to v1.5.0 (`#728 <https://github.com/ros2/rmw_zenoh/issues/728>`__)
* Change zenoh-c features to use its default + shared-memory + transport_serial (`#692 <https://github.com/ros2/rmw_zenoh/issues/692>`__)
* Bump Zenoh to 1.4.0 (`#652 <https://github.com/ros2/rmw_zenoh/issues/652>`__)
* fix: pin rust toolchain to v1.75.0 (`#602 <https://github.com/ros2/rmw_zenoh/issues/602>`__)
* fix: use the right commit to bump zenoh to v1.3.2 (`#607 <https://github.com/ros2/rmw_zenoh/issues/607>`__)
* Contributors: ChenYing Kuo (CY), Julien Enoch, Shane Loretz, Tim Clephas, Yadunund, Yuyuan Yuan, Øystein Sture


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`zenoh_security_tools <https://github.com/ros2/rmw_zenoh/tree/lyrical/zenoh_security_tools/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Address outstanding TODO items (`#896 <https://github.com/ros2/rmw_zenoh/issues/896>`__)
* Removed tinyxml2_vendor dependency (`#829 <https://github.com/ros2/rmw_zenoh/issues/829>`__)
* Fix commands in zenoh_security_tools README (`#814 <https://github.com/ros2/rmw_zenoh/issues/814>`__)
* Revert "fix: handle missing enclaves_dir argument for zenoh_security_tools (#…" (`#802 <https://github.com/ros2/rmw_zenoh/issues/802>`__)
* Correct a description error in the zenoh_security_tools README (`#789 <https://github.com/ros2/rmw_zenoh/issues/789>`__)
* fix: handle missing enclaves_dir argument for zenoh_security_tools (`#788 <https://github.com/ros2/rmw_zenoh/issues/788>`__)
* SROS: add ACL rules for TRANSIENT_LOCAL pub/sub (fix `#753 <https://github.com/ros2/rmw_zenoh/issues/753>`__) (`#779 <https://github.com/ros2/rmw_zenoh/issues/779>`__)
* Fix handling of enclave path in zenoh_security_tools (`#770 <https://github.com/ros2/rmw_zenoh/issues/770>`__)
* Update CMakeLists.txt (`#617 <https://github.com/ros2/rmw_zenoh/issues/617>`__)
* Fix warning on Windows (`#615 <https://github.com/ros2/rmw_zenoh/issues/615>`__)
* Contributors: Alejandro Hernández Cordero, Barry Xu, Christophe Bedard, Julien Enoch, Tomoya Fujita, Yadunund, mosfet80
