Iron Irwini Changelog
=====================

This page is a list of the complete changes in all ROS 2 core packages since the previous release.

.. contents:: Table of Contents
   :local:

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`action_msgs <https://github.com/ros2/rcl_interfaces/tree/iron/action_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update common_interfaces to C++17. (`#215 <https://github.com/ros2/rcl_interfaces/issues/215>`__) (`#151 <https://github.com/ros2/rcl_interfaces/issues/151>`__)
* Add service_msgs package (`#143 <https://github.com/ros2/rcl_interfaces/issues/143>`__)
* [rolling] Update maintainers - 2022-11-07 (`#150 <https://github.com/ros2/rcl_interfaces/issues/150>`__)
* Depend on rosidl_core_generators for packages required by actions (`#144 <https://github.com/ros2/rcl_interfaces/issues/144>`__)
* Contributors: Audrow Nash, Brian, Chris Lalancette, Jacob Perron


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`action_tutorials_cpp <https://github.com/ros2/demos/tree/iron/action_tutorials/action_tutorials_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Change all ROS2 -> ROS 2. (`#610 <https://github.com/ros2/demos/issues/610>`__)
* Update the demos to C++17. (`#594 <https://github.com/ros2/demos/issues/594>`__)
* Add README's for action_tutorials. (`#576 <https://github.com/ros2/demos/issues/576>`__)
* [rolling] Update maintainers - 2022-11-07 (`#589 <https://github.com/ros2/demos/issues/589>`__)
* Fix two small bugs in the fibonacci C++ tutorial. (`#564 <https://github.com/ros2/demos/issues/564>`__)
* Contributors: Audrow Nash, Chris Lalancette, kagibson


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`action_tutorials_interfaces <https://github.com/ros2/demos/tree/iron/action_tutorials/action_tutorials_interfaces/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Change all ROS2 -> ROS 2. (`#610 <https://github.com/ros2/demos/issues/610>`__)
* A couple more upgrades to C++17. (`#609 <https://github.com/ros2/demos/issues/609>`__)
* Add README's for action_tutorials. (`#576 <https://github.com/ros2/demos/issues/576>`__)
* [rolling] Update maintainers - 2022-11-07 (`#589 <https://github.com/ros2/demos/issues/589>`__)
* Remove action_msgs dependency (`#580 <https://github.com/ros2/demos/issues/580>`__)
* Contributors: Audrow Nash, Chris Lalancette, Jacob Perron, kagibson


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`action_tutorials_py <https://github.com/ros2/demos/tree/iron/action_tutorials/action_tutorials_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add README's for action_tutorials. (`#576 <https://github.com/ros2/demos/issues/576>`__)
* [rolling] Update maintainers - 2022-11-07 (`#589 <https://github.com/ros2/demos/issues/589>`__)
* Contributors: Audrow Nash, kagibson


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`actionlib_msgs <https://github.com/ros2/common_interfaces/tree/iron/actionlib_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update common_interfaces to C++17. (`#215 <https://github.com/ros2/common_interfaces/issues/215>`__)
* [rolling] Update maintainers - 2022-11-07 (`#210 <https://github.com/ros2/common_interfaces/issues/210>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_clang_format <https://github.com/ament/ament_lint/tree/iron/ament_clang_format/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* ament_clang_format: use open braces for enum definitions (`#426 <https://github.com/ament/ament_lint/issues/426>`__)
* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Audrow Nash, james-rms, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_clang_tidy <https://github.com/ament/ament_lint/tree/iron/ament_clang_tidy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* recommend use of --mixin compile-commands (`#371 <https://github.com/ament/ament_lint/issues/371>`__)
* Improve message and avoid missing new lines between reports from files (`#373 <https://github.com/ament/ament_lint/issues/373>`__)
* Contributors: Audrow Nash, William Woodall, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake <https://github.com/ament/ament_cmake/tree/iron/ament_cmake/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#411 <https://github.com/ament/ament_cmake/issues/411>`__) * Update maintainers to Michael Jeronimo
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_auto <https://github.com/ament/ament_cmake/tree/iron/ament_cmake_auto/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Support INTERFACE on ament_auto_add_library (`#420 <https://github.com/ament/ament_cmake/issues/420>`__)
* Fix ament_auto_add_gtest's parameter passing (`#421 <https://github.com/ament/ament_cmake/issues/421>`__)
* [rolling] Update maintainers - 2022-11-07 (`#411 <https://github.com/ament/ament_cmake/issues/411>`__)
* Rolling: ament_cmake_auto should include dependencies as SYSTEM (`#385 <https://github.com/ament/ament_cmake/issues/385>`__)
* Contributors: Audrow Nash, Christopher Wecht, Joshua Whitley, Rin Iwai


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_clang_format <https://github.com/ament/ament_lint/tree/iron/ament_cmake_clang_format/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_clang_tidy <https://github.com/ament/ament_lint/tree/iron/ament_cmake_clang_tidy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_copyright <https://github.com/ament/ament_lint/tree/iron/ament_cmake_copyright/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* [ament_lint_auto] General file exclusion with AMENT_LINT_AUTO_FILE_EXCLUDE (`#386 <https://github.com/ament/ament_lint/issues/386>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Abrar Rahman Protyasha, Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_core <https://github.com/ament/ament_cmake/tree/iron/ament_cmake_core/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* ament_cmake_uninstall_target: Correct location of install_manifest.txt (`#432 <https://github.com/ament/ament_cmake/issues/432>`__)
* Use file(GENERATE OUTPUT) to create dsv files (`#416 <https://github.com/ament/ament_cmake/issues/416>`__) Using file(WRITE) and file(APPEND) causes the modification stamp of the file to be changed each time CMake configures, resluting in an 'Installing' message rather than an 'Up-to-date' message even though the file content is identical. Using file(GENERATE OUTPUT) updates the timestamp of the file only if the content changes.
* Warn when trying to symlink install an INTERFACE_LIBRARY (`#417 <https://github.com/ament/ament_cmake/issues/417>`__)
* Workaround to exclude Clion's cmake folders from colcon test (`#410 <https://github.com/ament/ament_cmake/issues/410>`__) - Add AMENT_IGNORE to CMAKE_BINARY_DIR to avoid picking up cmake specific folders created by CLion in ``colcon build`` and ``colcon test`` commands
* if (NOT ${UNDEFINED_VAR}) gets evaluated to false, so change to if (NOT UNDEFINED_VAR) so it evaluates to true. (`#407 <https://github.com/ament/ament_cmake/issues/407>`__)
* [rolling] Update maintainers - 2022-11-07 (`#411 <https://github.com/ament/ament_cmake/issues/411>`__) * Update maintainers to Michael Jeronimo
* Implement ament_add_default_options (`#390 <https://github.com/ament/ament_cmake/issues/390>`__)
* Contributors: Audrow Nash, Kenji Brameld, Michael Orlov, Scott K Logan, Shane Loretz, Silvio Traversaro, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_cppcheck <https://github.com/ament/ament_lint/tree/iron/ament_cmake_cppcheck/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* [ament_lint_auto] General file exclusion with AMENT_LINT_AUTO_FILE_EXCLUDE (`#386 <https://github.com/ament/ament_lint/issues/386>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Abrar Rahman Protyasha, Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_cpplint <https://github.com/ament/ament_lint/tree/iron/ament_cmake_cpplint/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* [ament_lint_auto] General file exclusion with AMENT_LINT_AUTO_FILE_EXCLUDE (`#386 <https://github.com/ament/ament_lint/issues/386>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Abrar Rahman Protyasha, Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_export_definitions <https://github.com/ament/ament_cmake/tree/iron/ament_cmake_export_definitions/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#411 <https://github.com/ament/ament_cmake/issues/411>`__) * Update maintainers to Michael Jeronimo
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_export_dependencies <https://github.com/ament/ament_cmake/tree/iron/ament_cmake_export_dependencies/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#411 <https://github.com/ament/ament_cmake/issues/411>`__) * Update maintainers to Michael Jeronimo
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_export_include_directories <https://github.com/ament/ament_cmake/tree/iron/ament_cmake_export_include_directories/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#411 <https://github.com/ament/ament_cmake/issues/411>`__) * Update maintainers to Michael Jeronimo
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_export_interfaces <https://github.com/ament/ament_cmake/tree/iron/ament_cmake_export_interfaces/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#411 <https://github.com/ament/ament_cmake/issues/411>`__) * Update maintainers to Michael Jeronimo
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_export_libraries <https://github.com/ament/ament_cmake/tree/iron/ament_cmake_export_libraries/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#411 <https://github.com/ament/ament_cmake/issues/411>`__) * Update maintainers to Michael Jeronimo
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_export_link_flags <https://github.com/ament/ament_cmake/tree/iron/ament_cmake_export_link_flags/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#411 <https://github.com/ament/ament_cmake/issues/411>`__) * Update maintainers to Michael Jeronimo
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_export_targets <https://github.com/ament/ament_cmake/tree/iron/ament_cmake_export_targets/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#411 <https://github.com/ament/ament_cmake/issues/411>`__) * Update maintainers to Michael Jeronimo
* Support new target export template introduced with CMake 3.24 (`#395 <https://github.com/ament/ament_cmake/issues/395>`__)
* Fix the order in which Export.cmake files are included (`#256 <https://github.com/ament/ament_cmake/issues/256>`__)
* Contributors: Audrow Nash, Timo Röhling


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_flake8 <https://github.com/ament/ament_lint/tree/iron/ament_cmake_flake8/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add flake8 linter ignore support (`#424 <https://github.com/ament/ament_lint/issues/424>`__)
* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Abrar Rahman Protyasha, Audrow Nash, RFRIEDM-Trimble, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_gen_version_h <https://github.com/ament/ament_cmake/tree/iron/ament_cmake_gen_version_h/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Changed version gte macro to make it MSVC compatible. Fix `#433 <https://github.com/ament/ament_cmake/issues/433>`__ (`#434 <https://github.com/ament/ament_cmake/issues/434>`__)
* [rolling] Update maintainers - 2022-11-07 (`#411 <https://github.com/ament/ament_cmake/issues/411>`__) * Update maintainers to Michael Jeronimo
* Contributors: Audrow Nash, iquarobotics


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_gmock <https://github.com/ament/ament_cmake/tree/iron/ament_cmake_gmock/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix compiler warnings related to gtest/gmock (`#408 <https://github.com/ament/ament_cmake/issues/408>`__) * Suppress compiler warnings when building gmock definition of implicit copy constructor ... is deprecated because it has a user-declared copy assignment operator [-Wdeprecated-copy] * Declare gtest/gmock include dirs as SYSTEM PRIVATE for test targets
* [rolling] Update maintainers - 2022-11-07 (`#411 <https://github.com/ament/ament_cmake/issues/411>`__) * Update maintainers to Michael Jeronimo
* Contributors: Audrow Nash, Robert Haschke


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_google_benchmark <https://github.com/ament/ament_cmake/tree/iron/ament_cmake_google_benchmark/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#411 <https://github.com/ament/ament_cmake/issues/411>`__) * Update maintainers to Michael Jeronimo
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_gtest <https://github.com/ament/ament_cmake/tree/iron/ament_cmake_gtest/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix compiler warnings related to gtest/gmock (`#408 <https://github.com/ament/ament_cmake/issues/408>`__) * Suppress compiler warnings when building gmock definition of implicit copy constructor ... is deprecated because it has a user-declared copy assignment operator [-Wdeprecated-copy] * Declare gtest/gmock include dirs as SYSTEM PRIVATE for test targets
* [rolling] Update maintainers - 2022-11-07 (`#411 <https://github.com/ament/ament_cmake/issues/411>`__) * Update maintainers to Michael Jeronimo
* Contributors: Audrow Nash, Robert Haschke


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_include_directories <https://github.com/ament/ament_cmake/tree/iron/ament_cmake_include_directories/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#411 <https://github.com/ament/ament_cmake/issues/411>`__) * Update maintainers to Michael Jeronimo
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_libraries <https://github.com/ament/ament_cmake/tree/iron/ament_cmake_libraries/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#411 <https://github.com/ament/ament_cmake/issues/411>`__) * Update maintainers to Michael Jeronimo
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_lint_cmake <https://github.com/ament/ament_lint/tree/iron/ament_cmake_lint_cmake/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_mypy <https://github.com/ament/ament_lint/tree/iron/ament_cmake_mypy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_pclint <https://github.com/ament/ament_lint/tree/iron/ament_cmake_pclint/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_pep257 <https://github.com/ament/ament_lint/tree/iron/ament_cmake_pep257/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_pycodestyle <https://github.com/ament/ament_lint/tree/iron/ament_cmake_pycodestyle/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_pyflakes <https://github.com/ament/ament_lint/tree/iron/ament_cmake_pyflakes/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_pytest <https://github.com/ament/ament_cmake/tree/iron/ament_cmake_pytest/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix test skipping logic for missing pytest module (`#441 <https://github.com/ament/ament_cmake/issues/441>`__)
* Add missing buildtool_depend on python3-pytest (`#440 <https://github.com/ament/ament_cmake/issues/440>`__)
* ament_cmake_pytest needs a buildtool_depend on ament_cmake_test. (`#439 <https://github.com/ament/ament_cmake/issues/439>`__)
* Fix pytest-cov version detection with pytest >=7.0.0 (`#436 <https://github.com/ament/ament_cmake/issues/436>`__)
* use the error handler replace to allow non-utf8 to be decoded (`#381 <https://github.com/ament/ament_cmake/issues/381>`__)
* [rolling] Update maintainers - 2022-11-07 (`#411 <https://github.com/ament/ament_cmake/issues/411>`__) * Update maintainers to Michael Jeronimo
* Add NOCAPTURE option to ament_add_pytest_test (`#393 <https://github.com/ament/ament_cmake/issues/393>`__)
* Contributors: Audrow Nash, Chris Lalancette, Christophe Bedard, El Jawad Alaa, Jacob Perron, Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_python <https://github.com/ament/ament_cmake/tree/iron/ament_cmake_python/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Support Debian-specific install dir for ament_cmake_python (`#431 <https://github.com/ament/ament_cmake/issues/431>`__)
* [rolling] Update maintainers - 2022-11-07 (`#411 <https://github.com/ament/ament_cmake/issues/411>`__) * Update maintainers to Michael Jeronimo
* Document ament_cmake_python (`#387 <https://github.com/ament/ament_cmake/issues/387>`__)
* Contributors: Audrow Nash, Shane Loretz, Timo Röhling


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_ros <https://github.com/ros2/ament_cmake_ros/tree/iron/ament_cmake_ros/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#16 <https://github.com/ros2/ament_cmake_ros/issues/16>`__)
* Update maintainers (`#15 <https://github.com/ros2/ament_cmake_ros/issues/15>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_target_dependencies <https://github.com/ament/ament_cmake/tree/iron/ament_cmake_target_dependencies/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#411 <https://github.com/ament/ament_cmake/issues/411>`__) * Update maintainers to Michael Jeronimo
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_test <https://github.com/ament/ament_cmake/tree/iron/ament_cmake_test/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* use the error handler replace to allow non-utf8 to be decoded (`#381 <https://github.com/ament/ament_cmake/issues/381>`__)
* [rolling] Update maintainers - 2022-11-07 (`#411 <https://github.com/ament/ament_cmake/issues/411>`__) * Update maintainers to Michael Jeronimo
* Contributors: Audrow Nash, El Jawad Alaa


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_uncrustify <https://github.com/ament/ament_lint/tree/iron/ament_cmake_uncrustify/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* [ament_lint_auto] General file exclusion with AMENT_LINT_AUTO_FILE_EXCLUDE (`#386 <https://github.com/ament/ament_lint/issues/386>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Abrar Rahman Protyasha, Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_vendor_package <https://github.com/ament/ament_cmake/tree/iron/ament_cmake_vendor_package/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix the version number of ament_cmake_vendor_package.
* Add ament_cmake_vendor_package package (`#429 <https://github.com/ament/ament_cmake/issues/429>`__)
* Contributors: Chris Lalancette, Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_version <https://github.com/ament/ament_cmake/tree/iron/ament_cmake_version/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#411 <https://github.com/ament/ament_cmake/issues/411>`__) * Update maintainers to Michael Jeronimo
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cmake_xmllint <https://github.com/ament/ament_lint/tree/iron/ament_cmake_xmllint/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_copyright <https://github.com/ament/ament_lint/tree/iron/ament_copyright/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Support for matching license header within multiline comment block (`#361 <https://github.com/ament/ament_lint/issues/361>`__)
* Improved licencse matching (`#358 <https://github.com/ament/ament_lint/issues/358>`__)
* Updated regex and adding test cases for copyright search (`#363 <https://github.com/ament/ament_lint/issues/363>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Audrow Nash, Will, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cppcheck <https://github.com/ament/ament_lint/tree/iron/ament_cppcheck/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_cpplint <https://github.com/ament/ament_lint/tree/iron/ament_cpplint/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* [ament_cpplint] Process errors without linenums (`#385 <https://github.com/ament/ament_lint/issues/385>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Consider files with '.hh' extension as C++ headers (`#374 <https://github.com/ament/ament_lint/issues/374>`__)
* Contributors: Abrar Rahman Protyasha, Audrow Nash, Jacob Perron, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_flake8 <https://github.com/ament/ament_lint/tree/iron/ament_flake8/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Fix exclude regression (`#387 <https://github.com/ament/ament_lint/issues/387>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_index_cpp <https://github.com/ament/ament_index/tree/iron/ament_index_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#89 <https://github.com/ament/ament_index/issues/89>`__)
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_index_python <https://github.com/ament/ament_index/tree/iron/ament_index_python/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#89 <https://github.com/ament/ament_index/issues/89>`__)
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_lint <https://github.com/ament/ament_lint/tree/iron/ament_lint/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_lint_auto <https://github.com/ament/ament_lint/tree/iron/ament_lint_auto/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add flake8 linter ignore support (`#424 <https://github.com/ament/ament_lint/issues/424>`__)
* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* [ament_lint_auto] General file exclusion with AMENT_LINT_AUTO_FILE_EXCLUDE (`#386 <https://github.com/ament/ament_lint/issues/386>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Abrar Rahman Protyasha, Audrow Nash, RFRIEDM-Trimble, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_lint_cmake <https://github.com/ament/ament_lint/tree/iron/ament_lint_cmake/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_lint_common <https://github.com/ament/ament_lint/tree/iron/ament_lint_common/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_mypy <https://github.com/ament/ament_lint/tree/iron/ament_mypy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_package <https://github.com/ament/ament_package/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add support for comment lines in dsv files (`#139 <https://github.com/ament/ament_package/issues/139>`__)
* [rolling] Update maintainers - 2022-11-07 (`#138 <https://github.com/ament/ament_package/issues/138>`__)
* Mirror rolling to master
* Remove unused isolated prefix level templates (`#133 <https://github.com/ament/ament_package/issues/133>`__)
* Contributors: Audrow Nash, Scott K Logan, Shane Loretz


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_pclint <https://github.com/ament/ament_lint/tree/iron/ament_pclint/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_pep257 <https://github.com/ament/ament_lint/tree/iron/ament_pep257/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* updating ref to pep257 docs (`#433 <https://github.com/ament/ament_lint/issues/433>`__)
* Added underscore to ignore new pydocstyle item (`#428 <https://github.com/ament/ament_lint/issues/428>`__)
* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* [ament_pep257][master] redirecting error prints to stderr (`#390 <https://github.com/ament/ament_lint/issues/390>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Audrow Nash, Christian Henkel, Cristóbal Arroyo, Mirco Colosi (CR/AAS3), methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_pycodestyle <https://github.com/ament/ament_lint/tree/iron/ament_pycodestyle/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* ament_pycodestyle - fix crash caused by reporting on ignored errors (`#435 <https://github.com/ament/ament_lint/issues/435>`__)
* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Audrow Nash, Shane Loretz, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_pyflakes <https://github.com/ament/ament_lint/tree/iron/ament_pyflakes/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_uncrustify <https://github.com/ament/ament_lint/tree/iron/ament_uncrustify/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ament_xmllint <https://github.com/ament/ament_lint/tree/iron/ament_xmllint/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#421 <https://github.com/ament/ament_lint/issues/421>`__)
* Update maintainers (`#379 <https://github.com/ament/ament_lint/issues/379>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`builtin_interfaces <https://github.com/ros2/rcl_interfaces/tree/iron/builtin_interfaces/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update common_interfaces to C++17. (`#215 <https://github.com/ros2/rcl_interfaces/issues/215>`__) (`#151 <https://github.com/ros2/rcl_interfaces/issues/151>`__)
* [rolling] Update maintainers - 2022-11-07 (`#150 <https://github.com/ros2/rcl_interfaces/issues/150>`__)
* Depend on rosidl_core_generators for packages required by actions (`#144 <https://github.com/ros2/rcl_interfaces/issues/144>`__)
* Fix documented range (`#139 <https://github.com/ros2/rcl_interfaces/issues/139>`__)
* Contributors: Audrow Nash, Chris Lalancette, Jacob Perron, Tully Foote


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`camera_calibration_parsers <https://github.com/ros-perception/image_common/tree/iron/camera_calibration_parsers/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update image_common to C++17. (`#267 <https://github.com/ros-perception/image_common/issues/267>`__)
* Add alias library targets for all libraries (`#259 <https://github.com/ros-perception/image_common/issues/259>`__)
* Add support for missing ROI and binning fields (`#254 <https://github.com/ros-perception/image_common/issues/254>`__)
* Contributors: AndreasR30, Chris Lalancette, RFRIEDM-Trimble


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`camera_info_manager <https://github.com/ros-perception/image_common/tree/iron/camera_info_manager/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update image_common to C++17. (`#267 <https://github.com/ros-perception/image_common/issues/267>`__)
* Add alias library targets for all libraries (`#259 <https://github.com/ros-perception/image_common/issues/259>`__)
* Add lifecycle node compatibility to camera_info_manager (`#190 <https://github.com/ros-perception/image_common/issues/190>`__)
* Contributors: Chris Lalancette, RFRIEDM-Trimble, Ramon Wijnands


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`class_loader <https://github.com/ros/class_loader/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* make sanitizer happy (`#205 <https://github.com/ros/class_loader/issues/205>`__)
* [rolling] Update maintainers - 2022-11-07 (`#206 <https://github.com/ros/class_loader/issues/206>`__)
* Remove appveyor configuration. (`#204 <https://github.com/ros/class_loader/issues/204>`__)
* Just fix a typo in a comment. (`#203 <https://github.com/ros/class_loader/issues/203>`__)
* make the meta-object alive in the lifecycle of the registered plugin (`#201 <https://github.com/ros/class_loader/issues/201>`__)
* Mirror rolling to ros2
* Contributors: Audrow Nash, Chen Lihui, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`common_interfaces <https://github.com/ros2/common_interfaces/tree/iron/common_interfaces/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#210 <https://github.com/ros2/common_interfaces/issues/210>`__)
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`composition <https://github.com/ros2/demos/tree/iron/composition/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Change all ROS2 -> ROS 2. (`#610 <https://github.com/ros2/demos/issues/610>`__)
* update launch file name format to match documentation (`#588 <https://github.com/ros2/demos/issues/588>`__)
* Added README.md for composition (`#598 <https://github.com/ros2/demos/issues/598>`__)
* Update the demos to C++17. (`#594 <https://github.com/ros2/demos/issues/594>`__)
* [rolling] Update maintainers - 2022-11-07 (`#589 <https://github.com/ros2/demos/issues/589>`__)
* fix memory leak (`#585 <https://github.com/ros2/demos/issues/585>`__)
* Contributors: Audrow Nash, Chen Lihui, Chris Lalancette, Gary Bey, Patrick Wspanialy


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`composition_interfaces <https://github.com/ros2/rcl_interfaces/tree/iron/composition_interfaces/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update common_interfaces to C++17. (`#215 <https://github.com/ros2/rcl_interfaces/issues/215>`__) (`#151 <https://github.com/ros2/rcl_interfaces/issues/151>`__)
* [rolling] Update maintainers - 2022-11-07 (`#150 <https://github.com/ros2/rcl_interfaces/issues/150>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`demo_nodes_cpp <https://github.com/ros2/demos/tree/iron/demo_nodes_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Change all ROS2 -> ROS 2. (`#610 <https://github.com/ros2/demos/issues/610>`__)
* Add matched event demo for rclcpp and rclpy (`#607 <https://github.com/ros2/demos/issues/607>`__)
* Fix the set_parameters_callback example program. (`#608 <https://github.com/ros2/demos/issues/608>`__)
* [demo_nodes_cpp] Add YAML launch demos for topics (`#605 <https://github.com/ros2/demos/issues/605>`__)
* update launch file name format to match documentation (`#588 <https://github.com/ros2/demos/issues/588>`__)
* Service introspection (`#602 <https://github.com/ros2/demos/issues/602>`__) * Add in a rclcpp and rclpy demo of introspection.
* Added README.md for demo_cpp_nodes (`#599 <https://github.com/ros2/demos/issues/599>`__)
* Update the demos to C++17. (`#594 <https://github.com/ros2/demos/issues/594>`__)
* [rolling] Update maintainers - 2022-11-07 (`#589 <https://github.com/ros2/demos/issues/589>`__)
* Demo for pre and post set parameter callback support (`#565 <https://github.com/ros2/demos/issues/565>`__) * local parameter callback support
* counter starts from 1, not 2. (`#562 <https://github.com/ros2/demos/issues/562>`__)
* add a demo of content filter listener (`#557 <https://github.com/ros2/demos/issues/557>`__)
* Contributors: Audrow Nash, Barry Xu, Chen Lihui, Chris Lalancette, Damien LaRocque, Deepanshu Bansal, Gary Bey, Patrick Wspanialy, Tomoya Fujita


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`demo_nodes_cpp_native <https://github.com/ros2/demos/tree/iron/demo_nodes_cpp_native/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Change all ROS2 -> ROS 2. (`#610 <https://github.com/ros2/demos/issues/610>`__)
* Added README.md for demo_cpp_nodes_native (`#597 <https://github.com/ros2/demos/issues/597>`__)
* Update the demos to C++17. (`#594 <https://github.com/ros2/demos/issues/594>`__)
* [rolling] Update maintainers - 2022-11-07 (`#589 <https://github.com/ros2/demos/issues/589>`__)
* Make demo_nodes_cpp_native install stuff only when it builds (`#590 <https://github.com/ros2/demos/issues/590>`__)
* Contributors: Audrow Nash, Chris Lalancette, Gary Bey, Shane Loretz


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`demo_nodes_py <https://github.com/ros2/demos/tree/iron/demo_nodes_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Change all ROS2 -> ROS 2. (`#610 <https://github.com/ros2/demos/issues/610>`__)
* Add matched event demo for rclcpp and rclpy (`#607 <https://github.com/ros2/demos/issues/607>`__)
* Enable document generation using rosdoc2 (`#606 <https://github.com/ros2/demos/issues/606>`__)
* Service introspection (`#602 <https://github.com/ros2/demos/issues/602>`__)
* Added README.md for demo_nodes_py (`#600 <https://github.com/ros2/demos/issues/600>`__)
* [rolling] Update maintainers - 2022-11-07 (`#589 <https://github.com/ros2/demos/issues/589>`__)
* Demo for pre and post set parameter callback support (`#565 <https://github.com/ros2/demos/issues/565>`__)
* Add demo for rclpy parameter client (`#566 <https://github.com/ros2/demos/issues/566>`__)
* Exit with code 0 if ExternalShutdownException is raised (`#581 <https://github.com/ros2/demos/issues/581>`__)
* Contributors: Audrow Nash, Barry Xu, Brian, Chris Lalancette, Deepanshu Bansal, Gary Bey, Jacob Perron, Yadu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`diagnostic_msgs <https://github.com/ros2/common_interfaces/tree/iron/diagnostic_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update common_interfaces to C++17. (`#215 <https://github.com/ros2/common_interfaces/issues/215>`__)
* [rolling] Update maintainers - 2022-11-07 (`#210 <https://github.com/ros2/common_interfaces/issues/210>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`domain_coordinator <https://github.com/ros2/ament_cmake_ros/tree/iron/domain_coordinator/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#16 <https://github.com/ros2/ament_cmake_ros/issues/16>`__)
* Update maintainers (`#15 <https://github.com/ros2/ament_cmake_ros/issues/15>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`dummy_map_server <https://github.com/ros2/demos/tree/iron/dummy_robot/dummy_map_server/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Change all ROS2 -> ROS 2. (`#610 <https://github.com/ros2/demos/issues/610>`__)
* Update the demos to C++17. (`#594 <https://github.com/ros2/demos/issues/594>`__)
* [rolling] Update maintainers - 2022-11-07 (`#589 <https://github.com/ros2/demos/issues/589>`__)
* Added README.md for dummy_map_server (`#572 <https://github.com/ros2/demos/issues/572>`__)
* Contributors: Audrow Nash, Chris Lalancette, Gary Bey


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`dummy_robot_bringup <https://github.com/ros2/demos/tree/iron/dummy_robot/dummy_robot_bringup/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* update launch file name format to match documentation (`#588 <https://github.com/ros2/demos/issues/588>`__)
* [rolling] Update maintainers - 2022-11-07 (`#589 <https://github.com/ros2/demos/issues/589>`__)
* Contributors: Audrow Nash, Patrick Wspanialy


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`dummy_sensors <https://github.com/ros2/demos/tree/iron/dummy_robot/dummy_sensors/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix unstable LaserScan status for rviz2 (`#616 <https://github.com/ros2/demos/issues/616>`__)
* Added README.md for dummy_sensors (`#573 <https://github.com/ros2/demos/issues/573>`__)
* Update the demos to C++17. (`#594 <https://github.com/ros2/demos/issues/594>`__)
* [rolling] Update maintainers - 2022-11-07 (`#589 <https://github.com/ros2/demos/issues/589>`__)
* Contributors: Audrow Nash, Chen Lihui, Chris Lalancette, Gary Bey


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`eigen3_cmake_module <https://github.com/ros2/eigen3_cmake_module/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#6 <https://github.com/ros2/eigen3_cmake_module/issues/6>`__)
* Mirror rolling to master
* Update maintainers (`#4 <https://github.com/ros2/eigen3_cmake_module/issues/4>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`example_interfaces <https://github.com/ros2/example_interfaces/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#17 <https://github.com/ros2/example_interfaces/issues/17>`__)
* Remove action_msgs dependency (`#16 <https://github.com/ros2/example_interfaces/issues/16>`__)
* Mirror rolling to master
* Contributors: Audrow Nash, Jacob Perron


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_async_client <https://github.com/ros2/examples/tree/iron/rclcpp/services/async_client/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the examples to C++17. (`#353 <https://github.com/ros2/examples/issues/353>`__)
* [rolling] Update maintainers - 2022-11-07 (`#352 <https://github.com/ros2/examples/issues/352>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_cbg_executor <https://github.com/ros2/examples/tree/iron/rclcpp/executors/cbg_executor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the examples to C++17. (`#353 <https://github.com/ros2/examples/issues/353>`__)
* [rolling] Update maintainers - 2022-11-07 (`#352 <https://github.com/ros2/examples/issues/352>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_minimal_action_client <https://github.com/ros2/examples/tree/iron/rclcpp/actions/minimal_action_client/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the examples to C++17. (`#353 <https://github.com/ros2/examples/issues/353>`__)
* [rolling] Update maintainers - 2022-11-07 (`#352 <https://github.com/ros2/examples/issues/352>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_minimal_action_server <https://github.com/ros2/examples/tree/iron/rclcpp/actions/minimal_action_server/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the examples to C++17. (`#353 <https://github.com/ros2/examples/issues/353>`__)
* [rolling] Update maintainers - 2022-11-07 (`#352 <https://github.com/ros2/examples/issues/352>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_minimal_client <https://github.com/ros2/examples/tree/iron/rclcpp/services/minimal_client/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the examples to C++17. (`#353 <https://github.com/ros2/examples/issues/353>`__)
* [rolling] Update maintainers - 2022-11-07 (`#352 <https://github.com/ros2/examples/issues/352>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_minimal_composition <https://github.com/ros2/examples/tree/iron/rclcpp/composition/minimal_composition/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the examples to C++17. (`#353 <https://github.com/ros2/examples/issues/353>`__)
* [rolling] Update maintainers - 2022-11-07 (`#352 <https://github.com/ros2/examples/issues/352>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_minimal_publisher <https://github.com/ros2/examples/tree/iron/rclcpp/topics/minimal_publisher/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the examples to C++17. (`#353 <https://github.com/ros2/examples/issues/353>`__)
* [rolling] Update maintainers - 2022-11-07 (`#352 <https://github.com/ros2/examples/issues/352>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_minimal_service <https://github.com/ros2/examples/tree/iron/rclcpp/services/minimal_service/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the examples to C++17. (`#353 <https://github.com/ros2/examples/issues/353>`__)
* [rolling] Update maintainers - 2022-11-07 (`#352 <https://github.com/ros2/examples/issues/352>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_minimal_subscriber <https://github.com/ros2/examples/tree/iron/rclcpp/topics/minimal_subscriber/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the examples to C++17. (`#353 <https://github.com/ros2/examples/issues/353>`__)
* [rolling] Update maintainers - 2022-11-07 (`#352 <https://github.com/ros2/examples/issues/352>`__)
* add ContentFilteredTopic example. (`#341 <https://github.com/ros2/examples/issues/341>`__)
* Contributors: Audrow Nash, Chris Lalancette, Tomoya Fujita


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_minimal_timer <https://github.com/ros2/examples/tree/iron/rclcpp/timers/minimal_timer/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the examples to C++17. (`#353 <https://github.com/ros2/examples/issues/353>`__)
* [rolling] Update maintainers - 2022-11-07 (`#352 <https://github.com/ros2/examples/issues/352>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_multithreaded_executor <https://github.com/ros2/examples/tree/iron/rclcpp/executors/multithreaded_executor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the examples to C++17. (`#353 <https://github.com/ros2/examples/issues/353>`__)
* [rolling] Update maintainers - 2022-11-07 (`#352 <https://github.com/ros2/examples/issues/352>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclcpp_wait_set <https://github.com/ros2/examples/tree/iron/rclcpp/wait_set/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the examples to C++17. (`#353 <https://github.com/ros2/examples/issues/353>`__)
* [rolling] Update maintainers - 2022-11-07 (`#352 <https://github.com/ros2/examples/issues/352>`__)
* Add test linting to wait_set and fix issues. (`#346 <https://github.com/ros2/examples/issues/346>`__) (`#347 <https://github.com/ros2/examples/issues/347>`__)
* Contributors: Audrow Nash, Chris Lalancette, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclpy_executors <https://github.com/ros2/examples/tree/iron/rclpy/executors/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#352 <https://github.com/ros2/examples/issues/352>`__)
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclpy_guard_conditions <https://github.com/ros2/examples/tree/iron/rclpy/guard_conditions/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#352 <https://github.com/ros2/examples/issues/352>`__)
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclpy_minimal_action_client <https://github.com/ros2/examples/tree/iron/rclpy/actions/minimal_action_client/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Enable document generation using rosdoc2 for ament_python pkgs (`#357 <https://github.com/ros2/examples/issues/357>`__) * Add missing action_msgs dep * Add exec_deps for launch_testing_examples
* [rolling] Update maintainers - 2022-11-07 (`#352 <https://github.com/ros2/examples/issues/352>`__)
* Contributors: Audrow Nash, Yadu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclpy_minimal_action_server <https://github.com/ros2/examples/tree/iron/rclpy/actions/minimal_action_server/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#352 <https://github.com/ros2/examples/issues/352>`__)
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclpy_minimal_client <https://github.com/ros2/examples/tree/iron/rclpy/services/minimal_client/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#352 <https://github.com/ros2/examples/issues/352>`__)
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclpy_minimal_publisher <https://github.com/ros2/examples/tree/iron/rclpy/topics/minimal_publisher/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#352 <https://github.com/ros2/examples/issues/352>`__)
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclpy_minimal_service <https://github.com/ros2/examples/tree/iron/rclpy/services/minimal_service/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#352 <https://github.com/ros2/examples/issues/352>`__)
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclpy_minimal_subscriber <https://github.com/ros2/examples/tree/iron/rclpy/topics/minimal_subscriber/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#352 <https://github.com/ros2/examples/issues/352>`__)
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_rclpy_pointcloud_publisher <https://github.com/ros2/examples/tree/iron/rclpy/topics/pointcloud_publisher/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#352 <https://github.com/ros2/examples/issues/352>`__)
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`examples_tf2_py <https://github.com/ros2/geometry2/tree/iron/examples_tf2_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Enable document generation using rosdoc2 for ament_python pkgs (`#587 <https://github.com/ros2/geometry2/issues/587>`__)
* Update maintainers (`#560 <https://github.com/ros2/geometry2/issues/560>`__)
* Contributors: Audrow Nash, Yadu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`fastrtps_cmake_module <https://github.com/ros2/rosidl_typesupport_fastrtps/tree/iron/fastrtps_cmake_module/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#93 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/93>`__)
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`foonathan_memory_vendor <https://github.com/eProsima/foonathan_memory_vendor/tree/master/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Added support for QNX 7.1 build (#65)
* Update upstream to release 0.7-3 (#62)(#63)
* Fix CMake minimum required version (#60)


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`geometry2 <https://github.com/ros2/geometry2/tree/iron/geometry2/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainers (`#560 <https://github.com/ros2/geometry2/issues/560>`__)
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`geometry_msgs <https://github.com/ros2/common_interfaces/tree/iron/geometry_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update common_interfaces to C++17. (`#215 <https://github.com/ros2/common_interfaces/issues/215>`__)
* [rolling] Update maintainers - 2022-11-07 (`#210 <https://github.com/ros2/common_interfaces/issues/210>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`google_benchmark_vendor <https://github.com/ament/google_benchmark_vendor/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Actually update to 1.6.1. (`#25 <https://github.com/ament/google_benchmark_vendor/issues/25>`__) We claimed we were, but in fact we were pinned to the 1.5.3 git hash.
* Remove set but unused variable (`#24 <https://github.com/ament/google_benchmark_vendor/issues/24>`__) Clang checks -Wunused-but-set-variable. This fails the build with -Werror also enabled.
* [rolling] Update maintainers - 2022-11-07 (`#22 <https://github.com/ament/google_benchmark_vendor/issues/22>`__)
* Mirror rolling to main
* Contributors: Audrow Nash, Chris Lalancette, Michael Carroll


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ignition_cmake2_vendor <https://github.com/gazebo-release/gz_cmake2_vendor/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Set target version to 2.14.0 (`#5 <https://github.com/gazebo-release/gz_cmake2_vendor/issues/5>`__)
* Mirror rolling to main
* Contributors: Audrow Nash, Yadu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ignition_math6_vendor <https://github.com/gazebo-release/gz_math6_vendor/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Forward CMAKE_PREFIX_PATH when building vendor package (`#8 <https://github.com/gazebo-release/gz_math6_vendor/issues/8>`__)
* Contributors: Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`image_tools <https://github.com/ros2/demos/tree/iron/image_tools/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Added README.md for image_tools - [Clean] (`#596 <https://github.com/ros2/demos/issues/596>`__)
* Update the demos to C++17. (`#594 <https://github.com/ros2/demos/issues/594>`__)
* [rolling] Update maintainers - 2022-11-07 (`#589 <https://github.com/ros2/demos/issues/589>`__)
* Contributors: Audrow Nash, Chris Lalancette, Gary Bey


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`image_transport <https://github.com/ros-perception/image_common/tree/iron/image_transport/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update image_common to C++17. (`#267 <https://github.com/ros-perception/image_common/issues/267>`__)
* Add alias library targets for all libraries (`#259 <https://github.com/ros-perception/image_common/issues/259>`__)
* Remove subscriber and publisher impl methods without options (`#252 <https://github.com/ros-perception/image_common/issues/252>`__)
* Deprecate impl without options (`#249 <https://github.com/ros-perception/image_common/issues/249>`__)
* opt-in to qos overriding for publisher (`#246 <https://github.com/ros-perception/image_common/issues/246>`__)
* Add qos option to override qos (`#208 <https://github.com/ros-perception/image_common/issues/208>`__)
* Contributors: Brian, Chris Lalancette, Daisuke Nishimatsu, Kenji Brameld, RFRIEDM-Trimble


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`interactive_markers <https://github.com/ros-visualization/interactive_markers/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update interactive_markers to C++17. (`#99 <https://github.com/ros-visualization/interactive_markers/issues/99>`__)
* Update maintainers (`#98 <https://github.com/ros-visualization/interactive_markers/issues/98>`__)
* Mirror rolling to ros2
* update maintainer (`#92 <https://github.com/ros-visualization/interactive_markers/issues/92>`__)
* Contributors: Audrow Nash, Chris Lalancette, Dharini Dutia


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`intra_process_demo <https://github.com/ros2/demos/tree/iron/intra_process_demo/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix executable name in README (`#619 <https://github.com/ros2/demos/issues/619>`__)
* Change all ROS2 -> ROS 2. (`#610 <https://github.com/ros2/demos/issues/610>`__)
* Added README.md for intra_process_demo (`#595 <https://github.com/ros2/demos/issues/595>`__)
* Update the demos to C++17. (`#594 <https://github.com/ros2/demos/issues/594>`__)
* [rolling] Update maintainers - 2022-11-07 (`#589 <https://github.com/ros2/demos/issues/589>`__)
* Contributors: Audrow Nash, Chris Lalancette, Gary Bey, Yadunund


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`kdl_parser <https://github.com/ros/kdl_parser/tree/iron/kdl_parser/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Switch some tests to use unique pointers instead of raw pointers. (`#74 <https://github.com/ros/kdl_parser/issues/74>`__)
* log link children as DEBUG instead of INFO (`#71 <https://github.com/ros/kdl_parser/issues/71>`__)
* Enable the kdl_parser tests in ROS 2 (`#68 <https://github.com/ros/kdl_parser/issues/68>`__)
* Add in a LICENSE file and fix up copyright headers (`#66 <https://github.com/ros/kdl_parser/issues/66>`__)
* Use orocos_kdl_vendor and orocos-kdl target (`#64 <https://github.com/ros/kdl_parser/issues/64>`__)
* Use the rcutils logger instead of printf (`#65 <https://github.com/ros/kdl_parser/issues/65>`__)
* Contributors: Chris Lalancette, Joseph Schornak, Scott K Logan, yuraSomatic


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`keyboard_handler <https://github.com/ros-tooling/keyboard_handler/tree/iron/keyboard_handler/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Force exit from main thread on signal handling in ``keyboard_handler`` (`#23 <https://github.com/ros-tooling/keyboard_handler/issues/23>`__)
* Contributors: Michael Orlov


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`laser_geometry <https://github.com/ros-perception/laser_geometry/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update laser_geometry to C++17. (`#90 <https://github.com/ros-perception/laser_geometry/issues/90>`__)
* Update Maintainers (`#88 <https://github.com/ros-perception/laser_geometry/issues/88>`__)
* Mirror rolling to ros2
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch <https://github.com/ros2/launch/tree/iron/launch/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Document LaunchService.{run,run_async}() return value (`#702 <https://github.com/ros2/launch/issues/702>`__)
* [rosdoc2] Fix document generation on buildfarm (`#701 <https://github.com/ros2/launch/issues/701>`__)
* Enable document generation using rosdoc2 for ament_python pkgs (`#697 <https://github.com/ros2/launch/issues/697>`__)
* Remove the import of Literal from entity.py. (`#694 <https://github.com/ros2/launch/issues/694>`__)
* Fix flake8 errors. (`#695 <https://github.com/ros2/launch/issues/695>`__)
* add symlink to latest log directory (`#686 <https://github.com/ros2/launch/issues/686>`__)
* Improve type checking (`#679 <https://github.com/ros2/launch/issues/679>`__)
* Fixed typos (`#692 <https://github.com/ros2/launch/issues/692>`__)
* Pass modules to PythonExpression (`#655 <https://github.com/ros2/launch/issues/655>`__)
* Allow ReadyToTest() usage in event handler (`#665 <https://github.com/ros2/launch/issues/665>`__)
* Expose emulate_tty to xml and yaml launch (`#669 <https://github.com/ros2/launch/issues/669>`__)
* Expose sigterm_timeout and sigkill_timeout to xml frontend (`#667 <https://github.com/ros2/launch/issues/667>`__)
* [rolling] Update maintainers - 2022-11-07 (`#671 <https://github.com/ros2/launch/issues/671>`__)
* Expect deprecation warnings in tests (`#657 <https://github.com/ros2/launch/issues/657>`__)
* Fix the restoring of os.environ to maintain type. (`#656 <https://github.com/ros2/launch/issues/656>`__)
* Implement Any, All, Equals, and NotEquals substitutions (`#649 <https://github.com/ros2/launch/issues/649>`__)
* add LaunchLogDir substitution, replacing log_dir frontend only substitution (`#652 <https://github.com/ros2/launch/issues/652>`__)
* Add special cases to coerce "1" and "0" to bool when using bool coercion only (`#651 <https://github.com/ros2/launch/issues/651>`__)
* Update launch/test/launch/test_execute_local.py
* Added unit test ensuring that output dictionary works with ExecuteLocal
* Addresses issue `#588 <https://github.com/ros2/launch/issues/588>`__, allowing dict for 'output'
* Remove unused variables. (`#612 <https://github.com/ros2/launch/issues/612>`__)
* Expose shutdown action to xml frontend (`#611 <https://github.com/ros2/launch/issues/611>`__)
* Contributors: Aditya Pande, Alejandro Hernández Cordero, Audrow Nash, Blake Anderson, Chris Lalancette, Christophe Bedard, Hervé Audren, Jacob Perron, Matthew Elwin, Michael Jeronimo, Nikolai Morin, Welte, William Woodall, Yadu, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch_pytest <https://github.com/ros2/launch/tree/iron/launch_pytest/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fixed typos (`#692 <https://github.com/ros2/launch/issues/692>`__)
* Drop unused data_files entry for example_processes (`#680 <https://github.com/ros2/launch/issues/680>`__)
* Spelling correction
* [rolling] Update maintainers - 2022-11-07 (`#671 <https://github.com/ros2/launch/issues/671>`__)
* Contributors: Alejandro Hernández Cordero, Audrow Nash, Geoffrey Biggs, Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch_ros <https://github.com/ros2/launch_ros/tree/iron/launch_ros/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use SomeEntitiesType for type checking. (`#358 <https://github.com/ros2/launch_ros/issues/358>`__)
* Fix normalize_parameters_dict for multiple nodes in the same namespace (`#347 <https://github.com/ros2/launch_ros/issues/347>`__)
* Implement None check for ComposableNodeContainer (`#341 <https://github.com/ros2/launch_ros/issues/341>`__)
* Add LifecyleTransition action (`#317 <https://github.com/ros2/launch_ros/issues/317>`__)
* Improve evaluate_paramenter_dict exceptions error message (`#320 <https://github.com/ros2/launch_ros/issues/320>`__)
* Ensure load_composable_nodes respects condition (`#339 <https://github.com/ros2/launch_ros/issues/339>`__)
* fix: return text value to avoid exception (`#338 <https://github.com/ros2/launch_ros/issues/338>`__)
* [rolling] Update maintainers - 2022-11-07 (`#331 <https://github.com/ros2/launch_ros/issues/331>`__)
* RosTimer -> ROSTimer and PushRosNamespace -> PushROSNamespace, to follow PEP8 (`#326 <https://github.com/ros2/launch_ros/issues/326>`__)
* add SetROSLogDir action (`#325 <https://github.com/ros2/launch_ros/issues/325>`__)
* Support default values in parameter substitution (`#313 <https://github.com/ros2/launch_ros/issues/313>`__)
* Run condition for composable nodes (`#311 <https://github.com/ros2/launch_ros/issues/311>`__)
* Contributors: Aditya Pande, Alexey Merzlyakov, Audrow Nash, Chris Lalancette, Christoph Hellmann Santos, Daisuke Nishimatsu, Felipe Gomes de Melo, Kenji Miyake, William Woodall, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch_testing <https://github.com/ros2/launch/tree/iron/launch_testing/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Improve type checking (`#679 <https://github.com/ros2/launch/issues/679>`__)
* Fixed typos (`#692 <https://github.com/ros2/launch/issues/692>`__)
* Allow ReadyToTest() usage in event handler (`#665 <https://github.com/ros2/launch/issues/665>`__)
* Inherit markers from generate_test_description (`#670 <https://github.com/ros2/launch/issues/670>`__)
* [rolling] Update maintainers - 2022-11-07 (`#671 <https://github.com/ros2/launch/issues/671>`__)
* Fix Typo (`#641 <https://github.com/ros2/launch/issues/641>`__)
* ReadyToTest action timeout using decorator (`#625 <https://github.com/ros2/launch/issues/625>`__)
* Switch to using a comprehension for process_names. (`#614 <https://github.com/ros2/launch/issues/614>`__)
* Contributors: Alejandro Hernández Cordero, Audrow Nash, Chris Lalancette, Deepanshu Bansal, Hervé Audren, Kenji Brameld, Nikolai Morin, Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch_testing_ament_cmake <https://github.com/ros2/launch/tree/iron/launch_testing_ament_cmake/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#671 <https://github.com/ros2/launch/issues/671>`__)
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch_testing_examples <https://github.com/ros2/examples/tree/iron/launch_testing/launch_testing_examples/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Enable document generation using rosdoc2 for ament_python pkgs (`#357 <https://github.com/ros2/examples/issues/357>`__)
* increase the timeout for window platform to avoid flaky test (`#355 <https://github.com/ros2/examples/issues/355>`__)
* [rolling] Update maintainers - 2022-11-07 (`#352 <https://github.com/ros2/examples/issues/352>`__)
* Increase the WaitForNode timeout. (`#350 <https://github.com/ros2/examples/issues/350>`__)
* Contributors: Audrow Nash, Chen Lihui, Chris Lalancette, Yadu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch_testing_ros <https://github.com/ros2/launch_ros/tree/iron/launch_testing_ros/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Increase the timeouts in wait_for_topic_launch_test. (`#360 <https://github.com/ros2/launch_ros/issues/360>`__)
* Enable document generation using rosdoc2 (`#359 <https://github.com/ros2/launch_ros/issues/359>`__)
* exit() methods should not reraise the passed-in exception (`#357 <https://github.com/ros2/launch_ros/issues/357>`__)
* Inherit markers from generate_test_description (`#330 <https://github.com/ros2/launch_ros/issues/330>`__)
* [rolling] Update maintainers - 2022-11-07 (`#331 <https://github.com/ros2/launch_ros/issues/331>`__)
* Fix long wait during shutdown in WaitForTopics (`#314 <https://github.com/ros2/launch_ros/issues/314>`__)
* Contributors: Audrow Nash, Chris Lalancette, Giorgio Pintaudi, Keng12, Scott K Logan, Yadu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch_xml <https://github.com/ros2/launch/tree/iron/launch_xml/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fixed typos (`#692 <https://github.com/ros2/launch/issues/692>`__)
* Expose emulate_tty to xml and yaml launch (`#669 <https://github.com/ros2/launch/issues/669>`__)
* Expose sigterm_timeout and sigkill_timeout to xml frontend (`#667 <https://github.com/ros2/launch/issues/667>`__)
* [rolling] Update maintainers - 2022-11-07 (`#671 <https://github.com/ros2/launch/issues/671>`__)
* Contributors: Aditya Pande, Alejandro Hernández Cordero, Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`launch_yaml <https://github.com/ros2/launch/tree/iron/launch_yaml/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Expose emulate_tty to xml and yaml launch (`#669 <https://github.com/ros2/launch/issues/669>`__)
* Expose sigterm_timeout and sigkill_timeout to xml frontend (`#667 <https://github.com/ros2/launch/issues/667>`__)
* [rolling] Update maintainers - 2022-11-07 (`#671 <https://github.com/ros2/launch/issues/671>`__)
* Contributors: Aditya Pande, Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`libcurl_vendor <https://github.com/ros/resource_retriever/tree/iron/libcurl_vendor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* merge libcurl_vendor build instructions (`#81 <https://github.com/ros/resource_retriever/issues/81>`__)
* Sets CMP0135 policy behavior to NEW (`#79 <https://github.com/ros/resource_retriever/issues/79>`__)
* Fixes policy CMP0135 warning for CMake >= 3.24
* Contributors: Cristóbal Arroyo, Crola1702, schrodinbug


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`libstatistics_collector <https://github.com/ros-tooling/libstatistics_collector/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Bump hmarr/auto-approve-action from 3.2.0 to 3.2.1
* Mark benchmark _ as unused. (`#158 <https://github.com/ros-tooling/libstatistics_collector/issues/158>`__)
* Bump hmarr/auto-approve-action from 3.1.0 to 3.2.0
* Bump ros-tooling/action-ros-ci from 0.2 to 0.3
* Bump pascalgn/automerge-action from 0.15.5 to 0.15.6
* Update libstatistics_collector to C++17. (`#154 <https://github.com/ros-tooling/libstatistics_collector/issues/154>`__)
* Remove unnecessary build dependency on std_msgs. (`#145 <https://github.com/ros-tooling/libstatistics_collector/issues/145>`__)
* Bump pascalgn/automerge-action from 0.15.2 to 0.15.3
* Cleanup the CI jobs on this repository. (`#146 <https://github.com/ros-tooling/libstatistics_collector/issues/146>`__)
* Check if message has a "header" field with a stamp subfield of type builtin_interfaces::msg::Time (`#54 <https://github.com/ros-tooling/libstatistics_collector/issues/54>`__)
* Mirror rolling to master
* Contributors: Audrow Nash, Chris Lalancette, Scott Mende, dependabot[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`libyaml_vendor <https://github.com/ros2/libyaml_vendor/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix system package dependency (`#54 <https://github.com/ros2/libyaml_vendor/issues/54>`__)
* Update libyaml_vendor to C++17. (`#55 <https://github.com/ros2/libyaml_vendor/issues/55>`__)
* [rolling] Update maintainers - 2022-11-07 (`#53 <https://github.com/ros2/libyaml_vendor/issues/53>`__)
* Remove a warning message. (`#51 <https://github.com/ros2/libyaml_vendor/issues/51>`__)
* check if libyaml is already present before building it (take 2) (`#45 <https://github.com/ros2/libyaml_vendor/issues/45>`__)
* Mirror rolling to master
* Support WindowsStore builds for ROS2 (`#50 <https://github.com/ros2/libyaml_vendor/issues/50>`__) * libyaml for uwp
* Contributors: Audrow Nash, Chris Lalancette, Lou Amadio, Scott K Logan, Silvio Traversaro


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`lifecycle <https://github.com/ros2/demos/tree/iron/lifecycle/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* update launch file name format to match documentation (`#588 <https://github.com/ros2/demos/issues/588>`__)
* Update the demos to C++17. (`#594 <https://github.com/ros2/demos/issues/594>`__)
* [rolling] Update maintainers - 2022-11-07 (`#589 <https://github.com/ros2/demos/issues/589>`__)
* Contributors: Audrow Nash, Chris Lalancette, Patrick Wspanialy


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`lifecycle_msgs <https://github.com/ros2/rcl_interfaces/tree/iron/lifecycle_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update common_interfaces to C++17. (`#215 <https://github.com/ros2/rcl_interfaces/issues/215>`__) (`#151 <https://github.com/ros2/rcl_interfaces/issues/151>`__)
* [rolling] Update maintainers - 2022-11-07 (`#150 <https://github.com/ros2/rcl_interfaces/issues/150>`__)
* lifecycle_msgs: remove non-ASCII chars from field comments (`#147 <https://github.com/ros2/rcl_interfaces/issues/147>`__)
* Contributors: Audrow Nash, Chris Lalancette, G.A. vd. Hoorn


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`lifecycle_py <https://github.com/ros2/demos/tree/iron/lifecycle_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Enable document generation using rosdoc2 (`#606 <https://github.com/ros2/demos/issues/606>`__)
* update launch file name format to match documentation (`#588 <https://github.com/ros2/demos/issues/588>`__)
* Cleanup lifecycle_py to conform to ROS 2 standards. (`#604 <https://github.com/ros2/demos/issues/604>`__)
* [rolling] Update maintainers - 2022-11-07 (`#589 <https://github.com/ros2/demos/issues/589>`__)
* Install the launch file for lifecycle_py. (`#586 <https://github.com/ros2/demos/issues/586>`__)
* Contributors: Audrow Nash, Chris Lalancette, Patrick Wspanialy, Yadu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`logging_demo <https://github.com/ros2/demos/tree/iron/logging_demo/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the demos to C++17. (`#594 <https://github.com/ros2/demos/issues/594>`__)
* [rolling] Update maintainers - 2022-11-07 (`#589 <https://github.com/ros2/demos/issues/589>`__)
* Change dependency from 'rosidl_cmake' to 'rosidl_default_generators' (`#578 <https://github.com/ros2/demos/issues/578>`__)
* Contributors: Audrow Nash, Chris Lalancette, Jacob Perron


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`map_msgs <https://github.com/ros-planning/navigation_msgs/tree/iron/map_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainers
* Contributors: Audrow Nash, Steve Macenski


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`mcap_vendor <https://github.com/ros2/rosbag2/tree/iron/mcap_vendor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* mcap_vendor: add readme with versioning procedure (`#1230 <https://github.com/ros2/rosbag2/issues/1230>`__)
* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`__)
* mcap_vendor: only install public headers (`#1207 <https://github.com/ros2/rosbag2/issues/1207>`__)
* Fixes policy CMP0135 warning for CMake >= 3.24 for mcap_vendor (`#1208 <https://github.com/ros2/rosbag2/issues/1208>`__)
* mcap_vendor: download MCAP source via tarball (`#1204 <https://github.com/ros2/rosbag2/issues/1204>`__)
* rosbag2_cpp: test more than one storage plugin (`#1196 <https://github.com/ros2/rosbag2/issues/1196>`__)
* rosbag2_storage_mcap: merge into rosbag2 repo (`#1163 <https://github.com/ros2/rosbag2/issues/1163>`__)
* Fix Windows build (`#73 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/73>`__) Update mcap version to newest windows-compatible release. Add visibility macros for tests. Add clang-format preprocessor indentation for visibility_control to be readable.
* mcap_vendor: update to v0.6.0 (`#69 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/69>`__)
* Cleanup in ``mcap_vendor`` package (`#62 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/62>`__)
* Switch to using the vendored zstd library. (`#59 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/59>`__)
* Support timestamp-ordered playback (`#50 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/50>`__)
* Support regex topic filtering
* Add all lz4 sources to fix undefined symbols at runtime (`#46 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/46>`__)
* Upgrade mcap to fix LZ4 error and segfault (`#42 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/42>`__) Incorporates fixes from https://github.com/foxglove/mcap/pull/478 and https://github.com/foxglove/mcap/pull/482
* Add missing buildtool_depend on git (`#37 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/37>`__) This vendor package uses git to fetch sources for other packages. It should declare a dependency on that build tool. This should address the current cause of RPM build failures for RHEL: https://build.ros2.org/view/Rbin_rhel_el864/job/Rbin_rhel_el864__mcap_vendor__rhel_8_x86_64__binary/
* Test Foxy & Galactic in CI, fix missing test_depends in mcap_vendor/package.xml (`#33 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/33>`__)
* fix: minor issues (`#31 <https://github.com/wep21/rosbag2_storage_mcap/issues/31>`__) * remove unnecessary block * use target_link_libraries instead of ament_target_dependencies * remove ros environment * add prefix to compile definition
* Update email address for Foxglove maintainers (`#32 <https://github.com/wep21/rosbag2_storage_mcap/issues/32>`__)
* Added mcap_vendor package. Updated CMakeLists.txt to fetch dependencies with FetchContent rather than Conan.
* Contributors: Chris Lalancette, Cristóbal Arroyo, Daisuke Nishimatsu, Emerson Knapp, Jacob Bandes-Storch, James Smith, Michael Orlov, Scott K Logan, james-rms


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`message_filters <https://github.com/ros2/message_filters/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update message_filters to C++17. (`#88 <https://github.com/ros2/message_filters/issues/88>`__)
* Fix cache.h std::placeholder namespace (`#87 <https://github.com/ros2/message_filters/issues/87>`__)
* [rolling] Update maintainers - 2022-11-07 (`#85 <https://github.com/ros2/message_filters/issues/85>`__)
* Add a simpler aproximate time sync policy: ApproximateEpsilonTime (`#84 <https://github.com/ros2/message_filters/issues/84>`__)
* Add latest time zero-order-hold sync policy (`#73 <https://github.com/ros2/message_filters/issues/73>`__)
* Fix python examples and add a new example in documentation (`#79 <https://github.com/ros2/message_filters/issues/79>`__)
* Mirror rolling to master
* Adding fix to subscribe() call with raw node pointer and subscriber options (`#76 <https://github.com/ros2/message_filters/issues/76>`__)
* Corrected function arguments in example description (`#35 <https://github.com/ros2/message_filters/issues/35>`__)
* Changed invocation to ``add`` to conform template syntax (`#1388 <https://github.com/ros2/message_filters/issues/1388>`__)
* fix sphinx warning (`#1371 <https://github.com/ros2/message_filters/issues/1371>`__)
* change invocation to ``add`` to conform template syntax (`#1388 <https://github.com/ros/ros_comm/issues/1388>`__)
* fix sphinx warning (`#1371 <https://github.com/ros/ros_comm/issues/1371>`__)
* Contributors: Audrow Nash, Carlos Andrés Álvarez Restrepo, Chris Lalancette, Haoru Xue, Ivan Santiago Paunovic, Martin Ganeff, Steve Macenski, andermi


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`mimick_vendor <https://github.com/ros2/mimick_vendor/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#29 <https://github.com/ros2/mimick_vendor/issues/29>`__)
* Mirror rolling to master
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`nav_msgs <https://github.com/ros2/common_interfaces/tree/iron/nav_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update common_interfaces to C++17. (`#215 <https://github.com/ros2/common_interfaces/issues/215>`__)
* [rolling] Update maintainers - 2022-11-07 (`#210 <https://github.com/ros2/common_interfaces/issues/210>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`orocos_kdl_vendor <https://github.com/ros2/orocos_kdl_vendor/tree/iron/orocos_kdl_vendor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Make sure to quote orocos variables when setting targets. (`#12 <https://github.com/ros2/orocos_kdl_vendor/issues/12>`__)
* Ensure orocos-kdl is available as a target (`#10 <https://github.com/ros2/orocos_kdl_vendor/issues/10>`__)
* Ensure orocos-kdl target references Eigen (`#6 <https://github.com/ros2/orocos_kdl_vendor/issues/6>`__)
* Contributors: Chris Lalancette, Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`osrf_pycommon <https://github.com/osrf/osrf_pycommon/tree/master/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [master] Update maintainers - 2022-11-07 (`#89 <https://github.com/osrf/osrf_pycommon/issues/89>`__)
* Declare test dependencies in [test] extra (`#86 <https://github.com/osrf/osrf_pycommon/issues/86>`__)
* Contributors: Audrow Nash, Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`osrf_testing_tools_cpp <https://github.com/osrf/osrf_testing_tools_cpp/tree/iron/osrf_testing_tools_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix mpark/variant conditional for MSVC (`#77 <https://github.com/osrf/osrf_testing_tools_cpp/issues/77>`__)
* Changing C++ Compile Version (`#76 <https://github.com/osrf/osrf_testing_tools_cpp/issues/76>`__)
* Update maintainers (`#74 <https://github.com/osrf/osrf_testing_tools_cpp/issues/74>`__)
* Sets CMP0135 policy behavior to NEW (`#73 <https://github.com/osrf/osrf_testing_tools_cpp/issues/73>`__)
* Fixes policy CMP0135 warning in CMake 3.24 (`#71 <https://github.com/osrf/osrf_testing_tools_cpp/issues/71>`__)
* Add cstring include. (`#70 <https://github.com/osrf/osrf_testing_tools_cpp/issues/70>`__)
* Contributors: Audrow Nash, Chris Lalancette, Cristóbal Arroyo, Lucas Wendland, Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`pendulum_control <https://github.com/ros2/demos/tree/iron/pendulum_control/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the demos to C++17. (`#594 <https://github.com/ros2/demos/issues/594>`__)
* [rolling] Update maintainers - 2022-11-07 (`#589 <https://github.com/ros2/demos/issues/589>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`pendulum_msgs <https://github.com/ros2/demos/tree/iron/pendulum_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Change all ROS2 -> ROS 2. (`#610 <https://github.com/ros2/demos/issues/610>`__)
* A couple more upgrades to C++17. (`#609 <https://github.com/ros2/demos/issues/609>`__)
* Added README.md for pendulum_msgs. (`#577 <https://github.com/ros2/demos/issues/577>`__)
* [rolling] Update maintainers - 2022-11-07 (`#589 <https://github.com/ros2/demos/issues/589>`__)
* Contributors: Audrow Nash, Chris Lalancette, Gary Bey


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`performance_test_fixture <https://github.com/ros2/performance_test_fixture/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Resolve use-after-free compiler warnings (`#24 <https://github.com/ros2/performance_test_fixture/issues/24>`__)
* Update performance_test_fixture to C++17. (`#21 <https://github.com/ros2/performance_test_fixture/issues/21>`__)
* [rolling] Update maintainers - 2022-11-07 (`#20 <https://github.com/ros2/performance_test_fixture/issues/20>`__)
* Mirror rolling to main
* Add "cstring" to the list of includes (`#19 <https://github.com/ros2/performance_test_fixture/issues/19>`__)
* Contributors: Audrow Nash, Chris Lalancette, Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`pluginlib <https://github.com/ros/pluginlib/tree/iron/pluginlib/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update maintainers
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`pybind11_vendor <https://github.com/ros2/pybind11_vendor/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add a modified patch from upstream to support Python 3.11 (`#22 <https://github.com/ros2/pybind11_vendor/issues/22>`__)
* Add missing buildtool dependency on git (`#19 <https://github.com/ros2/pybind11_vendor/issues/19>`__)
* Update maintainers (`#17 <https://github.com/ros2/pybind11_vendor/issues/17>`__)
* Force pybind11 to find Python 3. (`#15 <https://github.com/ros2/pybind11_vendor/issues/15>`__)
* Mirror rolling to master
* Update maintainers (`#14 <https://github.com/ros2/pybind11_vendor/issues/14>`__)
* Update to pybind11 2.9.1.
* Rename patch file for history continuity.
* Contributors: Audrow Nash, Chris Lalancette, Scott K Logan, Steven! Ragnarök, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`python_cmake_module <https://github.com/ros2/python_cmake_module/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#13 <https://github.com/ros2/python_cmake_module/issues/13>`__)
* Mirror rolling to master
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`python_orocos_kdl_vendor <https://github.com/ros2/orocos_kdl_vendor/tree/iron/python_orocos_kdl_vendor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fixes policy CMP0135 warning for CMake >= 3.24 (`#16 <https://github.com/ros2/orocos_kdl_vendor/issues/16>`__)
* Workaround pybind11 CMake error (`#9 <https://github.com/ros2/orocos_kdl_vendor/issues/9>`__)
* Contributors: Cristóbal Arroyo, Jacob Perron


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`python_qt_binding <https://github.com/ros-visualization/python_qt_binding/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix to allow ninja to use make for generators (`#123 <https://github.com/ros-visualization/python_qt_binding/issues/123>`__)
* Fix flake8 linter regression (`#125 <https://github.com/ros-visualization/python_qt_binding/issues/125>`__)
* Remove pyqt from default binding order for macOS (`#118 <https://github.com/ros-visualization/python_qt_binding/issues/118>`__)
* Demote missing SIP message from WARNING to STATUS (`#122 <https://github.com/ros-visualization/python_qt_binding/issues/122>`__)
* [rolling] Update maintainers - 2022-11-07 (`#120 <https://github.com/ros-visualization/python_qt_binding/issues/120>`__)
* Contributors: Audrow Nash, Christoph Hellmann Santos, Cristóbal Arroyo, Michael Carroll, Rhys Mainwaring, Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`qt_dotgraph <https://github.com/ros-visualization/qt_gui_core/tree/iron/qt_dotgraph/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add in LICENSE file
* Cast drawLine input arguments to int (`#264 <https://github.com/ros-visualization/qt_gui_core/issues/264>`__) (`#265 <https://github.com/ros-visualization/qt_gui_core/issues/265>`__)
* Contributors: Chris Lalancette, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`qt_gui <https://github.com/ros-visualization/qt_gui_core/tree/iron/qt_gui/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add in LICENSE file
* Fix flake8 errors introduced by the previous commit. (`#262 <https://github.com/ros-visualization/qt_gui_core/issues/262>`__)
* Enable basic help information if no plugins are running (`#261 <https://github.com/ros-visualization/qt_gui_core/issues/261>`__)
* Contributors: Chris Lalancette, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`qt_gui_app <https://github.com/ros-visualization/qt_gui_core/tree/iron/qt_gui_app/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add in LICENSE file
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`qt_gui_core <https://github.com/ros-visualization/qt_gui_core/tree/iron/qt_gui_core/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add in LICENSE file
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`qt_gui_cpp <https://github.com/ros-visualization/qt_gui_core/tree/iron/qt_gui_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix ClassLoader warning by unloading plugin providers. (`#275 <https://github.com/ros-visualization/qt_gui_core/issues/275>`__)
* Chen Lihui
* fix shiboken error (`#267 <https://github.com/ros-visualization/qt_gui_core/issues/267>`__)
* Conditionally run import tests when generators are built (`#269 <https://github.com/ros-visualization/qt_gui_core/issues/269>`__)
* Add in LICENSE file
* Contributors: Chris Lalancette, Christoph Hellmann Santos, Michael Carroll, Rhys Mainwaring, Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`qt_gui_py_common <https://github.com/ros-visualization/qt_gui_core/tree/iron/qt_gui_py_common/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add in LICENSE file
* Contributors: Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`quality_of_service_demo_cpp <https://github.com/ros2/demos/tree/iron/quality_of_service_demo/rclcpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the demos to C++17. (`#594 <https://github.com/ros2/demos/issues/594>`__)
* [rolling] Update maintainers - 2022-11-07 (`#589 <https://github.com/ros2/demos/issues/589>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`quality_of_service_demo_py <https://github.com/ros2/demos/tree/iron/quality_of_service_demo/rclpy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use non-deprecated rclpy import. (`#617 <https://github.com/ros2/demos/issues/617>`__)
* Change all ROS2 -> ROS 2. (`#610 <https://github.com/ros2/demos/issues/610>`__)
* Enable document generation using rosdoc2 (`#606 <https://github.com/ros2/demos/issues/606>`__)
* [rolling] Update maintainers - 2022-11-07 (`#589 <https://github.com/ros2/demos/issues/589>`__)
* Exit with code 0 if ExternalShutdownException is raised (`#581 <https://github.com/ros2/demos/issues/581>`__)
* Contributors: Audrow Nash, Chris Lalancette, Jacob Perron, Yadu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl <https://github.com/ros2/rcl/tree/iron/rcl/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Honor ROS_LOCALHOST_ONLY if enabled. (`#1071 <https://github.com/ros2/rcl/issues/1071>`__)
* fix flaky test (`#1063 <https://github.com/ros2/rcl/issues/1063>`__)
* Add enable_type_description_service node option - API only (`#1060 <https://github.com/ros2/rcl/issues/1060>`__)
* Dynamic Subscription (BONUS: Allocators): rcl (`#1057 <https://github.com/ros2/rcl/issues/1057>`__)
* Runtime Interface Reflection: rcl (`#1025 <https://github.com/ros2/rcl/issues/1025>`__)
* [rcl] Improve handling of dynamic discovery  (`#1023 <https://github.com/ros2/rcl/issues/1023>`__)
* Use get_type_hash_func for typesupports (`#1055 <https://github.com/ros2/rcl/issues/1055>`__)
* publish for rosout topic multiple times to avoid flaky test (`#1054 <https://github.com/ros2/rcl/issues/1054>`__)
* Switch to target_link_libraries in rcl. (`#1051 <https://github.com/ros2/rcl/issues/1051>`__)
* Calculate type hash from TypeDescription (rep2011) (`#1027 <https://github.com/ros2/rcl/issues/1027>`__)
* Implement matched event (`#1033 <https://github.com/ros2/rcl/issues/1033>`__)
* use user-defined allocator to configure logging. (`#1047 <https://github.com/ros2/rcl/issues/1047>`__)
* user defined allocator should be used for rosout publisher. (`#1044 <https://github.com/ros2/rcl/issues/1044>`__)
* Add in inconsistent_topic implementation. (`#1024 <https://github.com/ros2/rcl/issues/1024>`__)
* doc update, ROS message accessibility depends on RMW implementation. (`#1043 <https://github.com/ros2/rcl/issues/1043>`__)
* Fix some warnings from clang. (`#1042 <https://github.com/ros2/rcl/issues/1042>`__)
* avoid unnecessary copy for rcutils_char_array_vsprintf. (`#1035 <https://github.com/ros2/rcl/issues/1035>`__)
* Service introspection (`#997 <https://github.com/ros2/rcl/issues/997>`__)
* Cache disable flag to avoid reading environmental variable. (`#1029 <https://github.com/ros2/rcl/issues/1029>`__)
* use parent logger (`#921 <https://github.com/ros2/rcl/issues/921>`__)
* Add timer on reset callback (`#995 <https://github.com/ros2/rcl/issues/995>`__)
* Update rcl to C++17. (`#1031 <https://github.com/ros2/rcl/issues/1031>`__)
* Make sure to check the return value of rcl_clock_init in tests. (`#1030 <https://github.com/ros2/rcl/issues/1030>`__)
* Implement rcl_clock_time_started (`#1021 <https://github.com/ros2/rcl/issues/1021>`__)
* Make sure to reset errors more places in the tests. (`#1020 <https://github.com/ros2/rcl/issues/1020>`__) This makes it so we don't get as many warnings when the tests are running.
* [rolling] Update maintainers - 2022-11-07 (`#1017 <https://github.com/ros2/rcl/issues/1017>`__)
* Small cleanups to rcl (`#1013 <https://github.com/ros2/rcl/issues/1013>`__)
* use int64_t for period. (`#1010 <https://github.com/ros2/rcl/issues/1010>`__)
* fixed rcl_wait return error when timer cancelled (`#1003 <https://github.com/ros2/rcl/issues/1003>`__)
* remove duplicate packages in find_package and reorder (`#994 <https://github.com/ros2/rcl/issues/994>`__)
* Fix buffer overflow in argument parsing caused by lexer returning length beyond length of string (`#979 <https://github.com/ros2/rcl/issues/979>`__)
* Fix leak in test_subscription_content_filter_options.cpp (`#978 <https://github.com/ros2/rcl/issues/978>`__)
* Contributors: Audrow Nash, Barry Xu, Brian, Chen Lihui, Chris Lalancette, Emerson Knapp, Geoffrey Biggs, Shane Loretz, Tomoya Fujita, mauropasse, methylDragon, 정찬희


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl_action <https://github.com/ros2/rcl/tree/iron/rcl_action/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* doc update, ROS message accessibility depends on RMW implementation. (`#1043 <https://github.com/ros2/rcl/issues/1043>`__)
* Update rcl to C++17. (`#1031 <https://github.com/ros2/rcl/issues/1031>`__)
* Reduce result_timeout to 10 seconds. (`#1012 <https://github.com/ros2/rcl/issues/1012>`__)
* [rolling] Update maintainers - 2022-11-07 (`#1017 <https://github.com/ros2/rcl/issues/1017>`__)
* Contributors: Audrow Nash, Chris Lalancette, Tomoya Fujita


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl_interfaces <https://github.com/ros2/rcl_interfaces/tree/iron/rcl_interfaces/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add interfaces for logging service. (`#154 <https://github.com/ros2/rcl_interfaces/issues/154>`__)
* Update common_interfaces to C++17. (`#215 <https://github.com/ros2/rcl_interfaces/issues/215>`__) (`#151 <https://github.com/ros2/rcl_interfaces/issues/151>`__)
* [rolling] Update maintainers - 2022-11-07 (`#150 <https://github.com/ros2/rcl_interfaces/issues/150>`__)
* Contributors: Audrow Nash, Chris Lalancette, Lei Liu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl_lifecycle <https://github.com/ros2/rcl/tree/iron/rcl_lifecycle/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update rcl to C++17. (`#1031 <https://github.com/ros2/rcl/issues/1031>`__)
* [rolling] Update maintainers - 2022-11-07 (`#1017 <https://github.com/ros2/rcl/issues/1017>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl_logging_interface <https://github.com/ros2/rcl_logging/tree/iron/rcl_logging_interface/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update rcl_logging to C++17. (`#98 <https://github.com/ros2/rcl_logging/issues/98>`__)
* Updated maintainers - 2022-11-07 (`#96 <https://github.com/ros2/rcl_logging/issues/96>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl_logging_noop <https://github.com/ros2/rcl_logging/tree/iron/rcl_logging_noop/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update rcl_logging to C++17. (`#98 <https://github.com/ros2/rcl_logging/issues/98>`__)
* Updated maintainers - 2022-11-07 (`#96 <https://github.com/ros2/rcl_logging/issues/96>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl_logging_spdlog <https://github.com/ros2/rcl_logging/tree/iron/rcl_logging_spdlog/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Mark the benchmark _ as unused. (`#99 <https://github.com/ros2/rcl_logging/issues/99>`__)
* Update rcl_logging to C++17. (`#98 <https://github.com/ros2/rcl_logging/issues/98>`__)
* change flushing behavior for spdlog log files, and add env var to use old style (no explicit flushing) (`#95 <https://github.com/ros2/rcl_logging/issues/95>`__) * now flushes every ERROR message and periodically every 5 seconds * can set ``RCL_LOGGING_SPDLOG_EXPERIMENTAL_OLD_FLUSHING_BEHAVIOR=1`` to get old behavior
* Updated maintainers - 2022-11-07 (`#96 <https://github.com/ros2/rcl_logging/issues/96>`__)
* Disable cppcheck for rcl_logging_spdlog. (`#93 <https://github.com/ros2/rcl_logging/issues/93>`__)
* ament_export_dependencies any package with targets we linked against (`#89 <https://github.com/ros2/rcl_logging/issues/89>`__)
* Contributors: Audrow Nash, Chris Lalancette, Shane Loretz, William Woodall


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcl_yaml_param_parser <https://github.com/ros2/rcl/tree/iron/rcl_yaml_param_parser/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix some warnings from clang. (`#1042 <https://github.com/ros2/rcl/issues/1042>`__)
* Cleanup the dependencies in rcl_yaml_param_parser. (`#1014 <https://github.com/ros2/rcl/issues/1014>`__)
* Update rcl to C++17. (`#1031 <https://github.com/ros2/rcl/issues/1031>`__)
* Support yaml string tag '!!str' (`#999 <https://github.com/ros2/rcl/issues/999>`__)
* [rolling] Update maintainers - 2022-11-07 (`#1017 <https://github.com/ros2/rcl/issues/1017>`__)
* Contributors: Audrow Nash, Barry Xu, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rclcpp <https://github.com/ros2/rclcpp/tree/iron/rclcpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix delivered message kind (`#2175 <https://github.com/ros2/rclcpp/issues/2175>`__) (`#2178 <https://github.com/ros2/rclcpp/issues/2178>`__)
* Add support for logging service. (`#2122 <https://github.com/ros2/rclcpp/issues/2122>`__)
* Picking ABI-incompatible executor changes (`#2170 <https://github.com/ros2/rclcpp/issues/2170>`__)
* add events-executor and timers-manager in rclcpp (`#2155 <https://github.com/ros2/rclcpp/issues/2155>`__)
* Create common structures for executors to use (`#2143 <https://github.com/ros2/rclcpp/issues/2143>`__)
* Implement deliver message kind (`#2168 <https://github.com/ros2/rclcpp/issues/2168>`__)
* applied tracepoints for ring_buffer (`#2091 <https://github.com/ros2/rclcpp/issues/2091>`__)
* Dynamic Subscription (REP-2011 Subset): Stubs for rclcpp (`#2165 <https://github.com/ros2/rclcpp/issues/2165>`__)
* Add type_hash to cpp TopicEndpointInfo (`#2137 <https://github.com/ros2/rclcpp/issues/2137>`__)
* Trigger the intraprocess guard condition with data (`#2164 <https://github.com/ros2/rclcpp/issues/2164>`__)
* Minor grammar fix (`#2149 <https://github.com/ros2/rclcpp/issues/2149>`__)
* Fix unnecessary allocations in executor.cpp (`#2135 <https://github.com/ros2/rclcpp/issues/2135>`__)
* add Logger::get_effective_level(). (`#2141 <https://github.com/ros2/rclcpp/issues/2141>`__)
* Remove deprecated header (`#2139 <https://github.com/ros2/rclcpp/issues/2139>`__)
* Implement matched event (`#2105 <https://github.com/ros2/rclcpp/issues/2105>`__)
* use allocator via init_options argument. (`#2129 <https://github.com/ros2/rclcpp/issues/2129>`__)
* Fixes to silence some clang warnings. (`#2127 <https://github.com/ros2/rclcpp/issues/2127>`__)
* Documentation improvements on the executor (`#2125 <https://github.com/ros2/rclcpp/issues/2125>`__)
* Avoid losing waitable handles while using MultiThreadedExecutor (`#2109 <https://github.com/ros2/rclcpp/issues/2109>`__)
* Hook up the incompatible type event inside of rclcpp (`#2069 <https://github.com/ros2/rclcpp/issues/2069>`__)
* Update all rclcpp packages to C++17. (`#2121 <https://github.com/ros2/rclcpp/issues/2121>`__)
* Fix clang warning: bugprone-use-after-move (`#2116 <https://github.com/ros2/rclcpp/issues/2116>`__)
* Fix memory leak in tracetools::get_symbol() (`#2104 <https://github.com/ros2/rclcpp/issues/2104>`__)
* Service introspection (`#1985 <https://github.com/ros2/rclcpp/issues/1985>`__)
* Allow publishing borrowed messages with intra-process enabled (`#2108 <https://github.com/ros2/rclcpp/issues/2108>`__)
* to fix flaky test about TestTimeSource.callbacks (`#2111 <https://github.com/ros2/rclcpp/issues/2111>`__)
* to create a sublogger while getting child of Logger (`#1717 <https://github.com/ros2/rclcpp/issues/1717>`__)
* Fix documentation of Context class (`#2107 <https://github.com/ros2/rclcpp/issues/2107>`__)
* fixes for rmw callbacks in qos_event class (`#2102 <https://github.com/ros2/rclcpp/issues/2102>`__)
* Add support for timers on reset callback (`#1979 <https://github.com/ros2/rclcpp/issues/1979>`__)
* Topic node guard condition in executor (`#2074 <https://github.com/ros2/rclcpp/issues/2074>`__)
* Fix bug on the disorder of calling shutdown callback (`#2097 <https://github.com/ros2/rclcpp/issues/2097>`__)
* Add default constructor to NodeInterfaces (`#2094 <https://github.com/ros2/rclcpp/issues/2094>`__)
* Fix clock state cached time to be a copy, not a reference. (`#2092 <https://github.com/ros2/rclcpp/issues/2092>`__)
* Fix -Wmaybe-uninitialized warning (`#2081 <https://github.com/ros2/rclcpp/issues/2081>`__)
* Fix the keep_last warning when using system defaults. (`#2082 <https://github.com/ros2/rclcpp/issues/2082>`__)
* Add in a fix for older compilers. (`#2075 <https://github.com/ros2/rclcpp/issues/2075>`__)
* Implement Unified Node Interface (NodeInterfaces class) (`#2041 <https://github.com/ros2/rclcpp/issues/2041>`__)
* Do not throw exception if trying to dequeue an empty intra-process buffer (`#2061 <https://github.com/ros2/rclcpp/issues/2061>`__)
* Move event callback binding to PublisherBase and SubscriptionBase (`#2066 <https://github.com/ros2/rclcpp/issues/2066>`__)
* Implement validity checks for rclcpp::Clock (`#2040 <https://github.com/ros2/rclcpp/issues/2040>`__)
* Explicitly set callback type (`#2059 <https://github.com/ros2/rclcpp/issues/2059>`__)
* Fix logging macros to build with msvc and cpp20 (`#2063 <https://github.com/ros2/rclcpp/issues/2063>`__)
* Add clock type to node_options (`#1982 <https://github.com/ros2/rclcpp/issues/1982>`__)
* Fix nullptr dereference in prune_requests_older_than (`#2008 <https://github.com/ros2/rclcpp/issues/2008>`__)
* Remove templating on to_rcl_subscription_options (`#2056 <https://github.com/ros2/rclcpp/issues/2056>`__)
* Fix SharedFuture from async_send_request never becoming valid (`#2044 <https://github.com/ros2/rclcpp/issues/2044>`__)
* Add in a warning for a KeepLast depth of 0. (`#2048 <https://github.com/ros2/rclcpp/issues/2048>`__)
* Mark rclcpp::Clock::now() as const (`#2050 <https://github.com/ros2/rclcpp/issues/2050>`__)
* Fix a case that did not throw ParameterUninitializedException (`#2036 <https://github.com/ros2/rclcpp/issues/2036>`__)
* Update maintainers (`#2043 <https://github.com/ros2/rclcpp/issues/2043>`__)
* MultiThreadExecutor number of threads is at least 2+ in default. (`#2032 <https://github.com/ros2/rclcpp/issues/2032>`__)
* Fix bug that a callback not reached (`#1640 <https://github.com/ros2/rclcpp/issues/1640>`__)
* Set the minimum number of threads of the Multithreaded executor to 2 (`#2030 <https://github.com/ros2/rclcpp/issues/2030>`__)
* check thread whether joinable before join (`#2019 <https://github.com/ros2/rclcpp/issues/2019>`__)
* Set cpplint test timeout to 3 minutes (`#2022 <https://github.com/ros2/rclcpp/issues/2022>`__)
* Make sure to include-what-you-use in the node_interfaces. (`#2018 <https://github.com/ros2/rclcpp/issues/2018>`__)
* Do not clear entities callbacks on destruction (`#2002 <https://github.com/ros2/rclcpp/issues/2002>`__)
* fix mismatched issue if using zero_allocate (`#1995 <https://github.com/ros2/rclcpp/issues/1995>`__)
* Make ParameterService and Sync/AsyncParameterClient accept rclcpp::QoS (`#1978 <https://github.com/ros2/rclcpp/issues/1978>`__)
* support regex match for parameter client (`#1992 <https://github.com/ros2/rclcpp/issues/1992>`__)
* operator+= and operator-= for Duration (`#1988 <https://github.com/ros2/rclcpp/issues/1988>`__)
* Revert "Revert "Add a create_timer method to Node and ``LifecycleNode`` classes (`#1975 <https://github.com/ros2/rclcpp/issues/1975>`__)" (`#2009 <https://github.com/ros2/rclcpp/issues/2009>`__) (`#2010 <https://github.com/ros2/rclcpp/issues/2010>`__)
* force compiler warning if callback handles not captured (`#2000 <https://github.com/ros2/rclcpp/issues/2000>`__)
* Revert "Add a ``create_timer`` method to ``Node`` and ``LifecycleNode`` classes (`#1975 <https://github.com/ros2/rclcpp/issues/1975>`__)" (`#2009 <https://github.com/ros2/rclcpp/issues/2009>`__)
* Add a ``create_timer`` method to ``Node`` and ``LifecycleNode`` classes (`#1975 <https://github.com/ros2/rclcpp/issues/1975>`__)
* [docs] add note about callback lifetime for {on, post}_set_parameter_callback (`#1981 <https://github.com/ros2/rclcpp/issues/1981>`__)
* fix memory leak (`#1994 <https://github.com/ros2/rclcpp/issues/1994>`__)
* Support pre-set and post-set parameter callbacks in addition to on-set-parameter-callback. (`#1947 <https://github.com/ros2/rclcpp/issues/1947>`__)
* Make create_service accept rclcpp::QoS (`#1969 <https://github.com/ros2/rclcpp/issues/1969>`__)
* Make create_client accept rclcpp::QoS (`#1964 <https://github.com/ros2/rclcpp/issues/1964>`__)
* Fix the documentation for rclcpp::ok to be accurate. (`#1965 <https://github.com/ros2/rclcpp/issues/1965>`__)
* use regex for wildcard matching (`#1839 <https://github.com/ros2/rclcpp/issues/1839>`__)
* Revert "Introduce executors new spin_for method, replace spin_until_future_complete with spin_until_complete. (`#1821 <https://github.com/ros2/rclcpp/issues/1821>`__) (`#1874 <https://github.com/ros2/rclcpp/issues/1874>`__)" (`#1956 <https://github.com/ros2/rclcpp/issues/1956>`__)
* Introduce executors new spin_for method, replace spin_until_future_complete with spin_until_complete. (`#1821 <https://github.com/ros2/rclcpp/issues/1821>`__) (`#1874 <https://github.com/ros2/rclcpp/issues/1874>`__)
* test adjustment for LoanedMessage. (`#1951 <https://github.com/ros2/rclcpp/issues/1951>`__)
* fix virtual dispatch issues identified by clang-tidy (`#1816 <https://github.com/ros2/rclcpp/issues/1816>`__)
* Remove unused on_parameters_set_callback\_ (`#1945 <https://github.com/ros2/rclcpp/issues/1945>`__)
* Fix subscription.is_serialized() for callbacks with message info (`#1950 <https://github.com/ros2/rclcpp/issues/1950>`__)
* wait for subscriptions on another thread. (`#1940 <https://github.com/ros2/rclcpp/issues/1940>`__)
* Fix documentation of ``RCLCPP\_[INFO,WARN,...]`` (`#1943 <https://github.com/ros2/rclcpp/issues/1943>`__)
* Always trigger guard condition waitset (`#1923 <https://github.com/ros2/rclcpp/issues/1923>`__)
* Add statistics for handle_loaned_message (`#1927 <https://github.com/ros2/rclcpp/issues/1927>`__)
* Drop wrong template specialization (`#1926 <https://github.com/ros2/rclcpp/issues/1926>`__)
* Update get_parameter_from_event to follow the function description (`#1922 <https://github.com/ros2/rclcpp/issues/1922>`__)
* Add 'best available' QoS enum values and methods (`#1920 <https://github.com/ros2/rclcpp/issues/1920>`__)
* use reinterpret_cast for function pointer conversion. (`#1919 <https://github.com/ros2/rclcpp/issues/1919>`__)
* Contributors: Alberto Soragna, Alexander Hans, Alexis Paques, Andrew Symington, Audrow Nash, Barry Xu, Brian, Chen Lihui, Chris Lalancette, Christophe Bedard, Christopher Wecht, Cristóbal Arroyo, Daniel Reuter, Deepanshu Bansal, Emerson Knapp, Hubert Liberacki, Ivan Santiago Paunovic, Jacob Perron, Jeffery Hsu, Jochen Sprickerhof, Lei Liu, Mateusz Szczygielski, Michael Carroll, Miguel Company, Nikolai Morin, Shane Loretz, Silvio Traversaro, Tomoya Fujita, Tyler Weaver, William Woodall, Yadu, andrei, mauropasse, mergify[bot], methylDragon, schrodinbug, uupks, ymski


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rclcpp_action <https://github.com/ros2/rclcpp/tree/iron/rclcpp_action/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* extract the result response before the callback is issued. (`#2132 <https://github.com/ros2/rclcpp/issues/2132>`__)
* Update all rclcpp packages to C++17. (`#2121 <https://github.com/ros2/rclcpp/issues/2121>`__)
* Fix the GoalUUID to_string representation (`#1999 <https://github.com/ros2/rclcpp/issues/1999>`__)
* Explicitly set callback type (`#2059 <https://github.com/ros2/rclcpp/issues/2059>`__)
* Update maintainers (`#2043 <https://github.com/ros2/rclcpp/issues/2043>`__)
* Do not clear entities callbacks on destruction (`#2002 <https://github.com/ros2/rclcpp/issues/2002>`__)
* Revert "Introduce executors new spin_for method, replace spin_until_future_complete with spin_until_complete. (`#1821 <https://github.com/ros2/rclcpp/issues/1821>`__) (`#1874 <https://github.com/ros2/rclcpp/issues/1874>`__)" (`#1956 <https://github.com/ros2/rclcpp/issues/1956>`__)
* Introduce executors new spin_for method, replace spin_until_future_complete with spin_until_complete. (`#1821 <https://github.com/ros2/rclcpp/issues/1821>`__) (`#1874 <https://github.com/ros2/rclcpp/issues/1874>`__)
* Contributors: Audrow Nash, Chris Lalancette, Hubert Liberacki, Nathan Wiebe Neufeldt, Tomoya Fujita, William Woodall, mauropasse


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rclcpp_components <https://github.com/ros2/rclcpp/tree/iron/rclcpp_components/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update all rclcpp packages to C++17. (`#2121 <https://github.com/ros2/rclcpp/issues/2121>`__)
* Improve component_manager_isolated shutdown (`#2085 <https://github.com/ros2/rclcpp/issues/2085>`__)
* Update maintainers (`#2043 <https://github.com/ros2/rclcpp/issues/2043>`__)
* use unique ptr and remove unuseful container (`#2013 <https://github.com/ros2/rclcpp/issues/2013>`__)
* Revert "Introduce executors new spin_for method, replace spin_until_future_complete with spin_until_complete. (`#1821 <https://github.com/ros2/rclcpp/issues/1821>`__) (`#1874 <https://github.com/ros2/rclcpp/issues/1874>`__)" (`#1956 <https://github.com/ros2/rclcpp/issues/1956>`__)
* Introduce executors new spin_for method, replace spin_until_future_complete with spin_until_complete. (`#1821 <https://github.com/ros2/rclcpp/issues/1821>`__) (`#1874 <https://github.com/ros2/rclcpp/issues/1874>`__)
* Contributors: Audrow Nash, Chen Lihui, Chris Lalancette, Hubert Liberacki, Michael Carroll, William Woodall


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rclcpp_lifecycle <https://github.com/ros2/rclcpp/tree/iron/rclcpp_lifecycle/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add support for logging service. (`#2122 <https://github.com/ros2/rclcpp/issues/2122>`__)
* Support publishing loaned messages in LifecyclePublisher (`#2159 <https://github.com/ros2/rclcpp/issues/2159>`__)
* Fixes to silence some clang warnings. (`#2127 <https://github.com/ros2/rclcpp/issues/2127>`__)
* Update all rclcpp packages to C++17. (`#2121 <https://github.com/ros2/rclcpp/issues/2121>`__)
* Use the correct macro for LifecycleNode::get_fully_qualified_name (`#2117 <https://github.com/ros2/rclcpp/issues/2117>`__)
* add get_fully_qualified_name to rclcpp_lifecycle (`#2115 <https://github.com/ros2/rclcpp/issues/2115>`__)
* Implement Unified Node Interface (NodeInterfaces class) (`#2041 <https://github.com/ros2/rclcpp/issues/2041>`__)
* Add clock type to node_options (`#1982 <https://github.com/ros2/rclcpp/issues/1982>`__)
* Update maintainers (`#2043 <https://github.com/ros2/rclcpp/issues/2043>`__)
* LifecycleNode on_configure doc fix. (`#2034 <https://github.com/ros2/rclcpp/issues/2034>`__)
* Bugfix 20210810 get current state (`#1756 <https://github.com/ros2/rclcpp/issues/1756>`__)
* Make lifecycle impl get_current_state() const. (`#2031 <https://github.com/ros2/rclcpp/issues/2031>`__)
* Cleanup the lifecycle implementation (`#2027 <https://github.com/ros2/rclcpp/issues/2027>`__)
* Cleanup the rclcpp_lifecycle dependencies. (`#2021 <https://github.com/ros2/rclcpp/issues/2021>`__)
* Revert "Revert "Add a create_timer method to Node and ``LifecycleNode`` classes (`#1975 <https://github.com/ros2/rclcpp/issues/1975>`__)" (`#2009 <https://github.com/ros2/rclcpp/issues/2009>`__) (`#2010 <https://github.com/ros2/rclcpp/issues/2010>`__)
* Revert "Add a ``create_timer`` method to ``Node`` and ``LifecycleNode`` classes (`#1975 <https://github.com/ros2/rclcpp/issues/1975>`__)" (`#2009 <https://github.com/ros2/rclcpp/issues/2009>`__)
* Add a ``create_timer`` method to ``Node`` and ``LifecycleNode`` classes (`#1975 <https://github.com/ros2/rclcpp/issues/1975>`__)
* Support pre-set and post-set parameter callbacks in addition to on-set-parameter-callback. (`#1947 <https://github.com/ros2/rclcpp/issues/1947>`__)
* Make create_service accept rclcpp::QoS (`#1969 <https://github.com/ros2/rclcpp/issues/1969>`__)
* Make create_client accept rclcpp::QoS (`#1964 <https://github.com/ros2/rclcpp/issues/1964>`__)
* Contributors: Andrew Symington, Audrow Nash, Chris Lalancette, Deepanshu Bansal, Ivan Santiago Paunovic, Jeffery Hsu, Lei Liu, Michael Babenko, Shane Loretz, Steve Macenski, Tomoya Fujita, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rclpy <https://github.com/ros2/rclpy/tree/iron/rclpy/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix type in Node init args (`#1115 <https://github.com/ros2/rclpy/issues/1115>`__) (`#1122 <https://github.com/ros2/rclpy/issues/1122>`__)
* Logging service support (`#1102 <https://github.com/ros2/rclpy/issues/1102>`__)
* Use custom sourcedir for conf.py (`#1109 <https://github.com/ros2/rclpy/issues/1109>`__)
* ServerGoalHandle should be destroyed before removing. (`#1113 <https://github.com/ros2/rclpy/issues/1113>`__)
* Fix unnecessary list comprehension flake8 (`#1112 <https://github.com/ros2/rclpy/issues/1112>`__)
* Stub type hash value line in TopicEndpointInfo string (`#1110 <https://github.com/ros2/rclpy/issues/1110>`__)
* Support documentation generation using rosdoc2 (`#1103 <https://github.com/ros2/rclpy/issues/1103>`__)
* Fix Time and Duration raising exception when compared to another type (`#1007 <https://github.com/ros2/rclpy/issues/1007>`__)
* Make rcl_interfaces a build and exec dependency. (`#1100 <https://github.com/ros2/rclpy/issues/1100>`__)
* Solving Atomic undefined on OSX with clang (`#1096 <https://github.com/ros2/rclpy/issues/1096>`__)
* Implement matched event (`#1083 <https://github.com/ros2/rclpy/issues/1083>`__)
* Update service.py documentation (`#1094 <https://github.com/ros2/rclpy/issues/1094>`__)
* Allow space or empty strings when using ros2 param set (`#1093 <https://github.com/ros2/rclpy/issues/1093>`__)
* Hook up the incompatible type event inside of rclpy (`#1058 <https://github.com/ros2/rclpy/issues/1058>`__)
* Switch to using module instead of module\_ (`#1090 <https://github.com/ros2/rclpy/issues/1090>`__)
* Add in subscription.get_publisher_count() (`#1089 <https://github.com/ros2/rclpy/issues/1089>`__)
* Service introspection (`#988 <https://github.com/ros2/rclpy/issues/988>`__)
* to create a sublogger while getting child of Logger (`#1084 <https://github.com/ros2/rclpy/issues/1084>`__)
* Fix `#983 <https://github.com/ros2/rclpy/issues/983>`__ by saving future and checking for + raising any exceptions (`#1073 <https://github.com/ros2/rclpy/issues/1073>`__)
* Force C++17 support on. (`#1076 <https://github.com/ros2/rclpy/issues/1076>`__)
* Use RCPPUTILS_SCOPE_EXIT to cleanup unparsed_indices_c. (`#1075 <https://github.com/ros2/rclpy/issues/1075>`__)
* Explicitly link atomic when building with Clang (`#1065 <https://github.com/ros2/rclpy/issues/1065>`__)
* Fix test_publisher linter for pydocstyle 6.2.2 (`#1063 <https://github.com/ros2/rclpy/issues/1063>`__)
* Add default preset qos profile (`#1062 <https://github.com/ros2/rclpy/issues/1062>`__)
* Add on_parameter_event method to the AsyncParameterClient. (`#1061 <https://github.com/ros2/rclpy/issues/1061>`__)
* Add documentation page for rclpy.clock (`#1055 <https://github.com/ros2/rclpy/issues/1055>`__)
* Rewrite test code without depending on parameter client (`#1045 <https://github.com/ros2/rclpy/issues/1045>`__)
* Add parallel callback test (`#1044 <https://github.com/ros2/rclpy/issues/1044>`__)
* decorator should not be callable. (`#1050 <https://github.com/ros2/rclpy/issues/1050>`__)
* typo fix. (`#1049 <https://github.com/ros2/rclpy/issues/1049>`__)
* Add in a warning for a depth of 0 with KEEP_LAST. (`#1048 <https://github.com/ros2/rclpy/issues/1048>`__)
* Add feature of wait for message (`#953 <https://github.com/ros2/rclpy/issues/953>`__). (`#960 <https://github.com/ros2/rclpy/issues/960>`__)
* Document rclpy.time.Time class (`#1040 <https://github.com/ros2/rclpy/issues/1040>`__)
* Deal with ParameterUninitializedException for parameter service (`#1033 <https://github.com/ros2/rclpy/issues/1033>`__)
* Improve documentation in rclpy.utilities (`#1038 <https://github.com/ros2/rclpy/issues/1038>`__)
* Document rclpy.utilities.remove_ros_args (`#1036 <https://github.com/ros2/rclpy/issues/1036>`__)
* Fix incorrect comparsion on whether parameter type is NOT_SET (`#1032 <https://github.com/ros2/rclpy/issues/1032>`__)
* [rolling] Update maintainers (`#1035 <https://github.com/ros2/rclpy/issues/1035>`__)
* Set the default number of threads of the MultiThreadedExecutor to 2 (`#1031 <https://github.com/ros2/rclpy/issues/1031>`__)
* Update the rclpy method documentation. (`#1026 <https://github.com/ros2/rclpy/issues/1026>`__)
* Revert "Raise user handler exception in MultiThreadedExecutor. (`#984 <https://github.com/ros2/rclpy/issues/984>`__)" (`#1017 <https://github.com/ros2/rclpy/issues/1017>`__)
* Waitable should check callback_group if it can be executed. (`#1001 <https://github.com/ros2/rclpy/issues/1001>`__)
* support wildcard matching for params file (`#987 <https://github.com/ros2/rclpy/issues/987>`__)
* Raise user handler exception in MultiThreadedExecutor. (`#984 <https://github.com/ros2/rclpy/issues/984>`__)
* Add wait_for_node method (`#930 <https://github.com/ros2/rclpy/issues/930>`__)
* Create sublogger for action server and action client (`#982 <https://github.com/ros2/rclpy/issues/982>`__)
* Support for pre-set and post-set parameter callback. (`#966 <https://github.com/ros2/rclpy/issues/966>`__)
* fix gcc 7.5 build errors (`#977 <https://github.com/ros2/rclpy/issues/977>`__)
* make _on_parameter_event return result correctly (`#817 <https://github.com/ros2/rclpy/issues/817>`__)
* Fix a small typo in documentation. (`#967 <https://github.com/ros2/rclpy/issues/967>`__)
* Add Parameter Client (`#959 <https://github.com/ros2/rclpy/issues/959>`__)
* Change sphinx theme to readthedocs (`#950 <https://github.com/ros2/rclpy/issues/950>`__)
* Name and type in descriptor(s) is ignored via declare_parameter(s). (`#957 <https://github.com/ros2/rclpy/issues/957>`__)
* Typo fix (`#951 <https://github.com/ros2/rclpy/issues/951>`__)
* Add py.typed to package (`#946 <https://github.com/ros2/rclpy/issues/946>`__)
* Fix rclpy.shutdown() from hanging when triggered from callback (`#947 <https://github.com/ros2/rclpy/pull/947>`__)
* Check if the context is already shutdown. (`#939 <https://github.com/ros2/rclpy/issues/939>`__)
* Avoid causing infinite loop when message is empty (`#935 <https://github.com/ros2/rclpy/issues/935>`__)
* Expose 'best available' QoS policies (`#928 <https://github.com/ros2/rclpy/issues/928>`__)
* remove feedback callback when the goal has been completed. (`#927 <https://github.com/ros2/rclpy/issues/927>`__)
* Allow to create a subscription with a callback that also receives the message info (`#922 <https://github.com/ros2/rclpy/issues/922>`__)
* Contributors: Achille Verheye, Audrow Nash, Barry Xu, Brian, Brian Chen, Chen Lihui, Chris Lalancette, Cristóbal Arroyo, Deepanshu Bansal, Emerson Knapp, Erki Suurjaak, Felix Divo, Florian Vahl, Gonzo, GuiHome, Ivan Santiago Paunovic, Jacob Perron, Lei Liu, Lucas Wendland, Michael Carroll, Sebastian Freitag, Seulbae Kim, Shane Loretz, Steve Nogar, Takeshi Ishita, Tomoya Fujita, Tony Najjar, Yadu, Yuki Igarashi, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcpputils <https://github.com/ros2/rcpputils/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add missing header for strlen (`#169 <https://github.com/ros2/rcpputils/issues/169>`__)
* issue-167 (`#172 <https://github.com/ros2/rcpputils/issues/172>`__)
* [rolling] Update maintainers - 2022-11-07 (`#166 <https://github.com/ros2/rcpputils/issues/166>`__)
* require C++17 and deprecate the rcppmath namespace (`#165 <https://github.com/ros2/rcpputils/issues/165>`__)
* Mirror rolling to master
* Fix possible race condition in create_directories() (`#162 <https://github.com/ros2/rcpputils/issues/162>`__)
* Contributors: Artem Shumov, Audrow Nash, Sebastian Freitag, William Woodall, bijoua29


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rcutils <https://github.com/ros2/rcutils/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix memory leak (`#423 <https://github.com/ros2/rcutils/issues/423>`__)
* Add convenience error handling macros (`#421 <https://github.com/ros2/rcutils/issues/421>`__)
* Calculate the next power-of-two for the user in hash_map_init. (`#420 <https://github.com/ros2/rcutils/issues/420>`__)
* update cast to modern style (`#418 <https://github.com/ros2/rcutils/issues/418>`__)
* Remove deprecated header get_env.h (`#417 <https://github.com/ros2/rcutils/issues/417>`__)
* Updates to rcutils to make rosdoc2 generation happier. (`#416 <https://github.com/ros2/rcutils/issues/416>`__)
* add RCUTILS_LOGGING_AUTOINIT_WITH_ALLOCATOR. (`#415 <https://github.com/ros2/rcutils/issues/415>`__)
* Fix memory leak in string_map.c in rcutils (`#411 <https://github.com/ros2/rcutils/issues/411>`__)
* avoid unnecessary copy for rcutils_char_array_vsprintf. (`#412 <https://github.com/ros2/rcutils/issues/412>`__)
* Add missing stddef include for size_t (`#410 <https://github.com/ros2/rcutils/issues/410>`__)
* Add SHA256 utility implementation (`#408 <https://github.com/ros2/rcutils/issues/408>`__)
* Upgrade rcutils to C++17. (`#392 <https://github.com/ros2/rcutils/issues/392>`__)
* [rolling] Update maintainers - 2022-11-07 (`#404 <https://github.com/ros2/rcutils/issues/404>`__)
* Fix build on OpenHarmony (`#395 <https://github.com/ros2/rcutils/issues/395>`__)
* regression of thread-safety for logging macros (`#393 <https://github.com/ros2/rcutils/issues/393>`__)
* add portable nonnull macros (`#382 <https://github.com/ros2/rcutils/issues/382>`__)
* Fix memory leak when adding the same key to the logger hash map multiple times (`#391 <https://github.com/ros2/rcutils/issues/391>`__)
* time_unix: uses ZEPHYR_VERSION_CODE instead (`#390 <https://github.com/ros2/rcutils/issues/390>`__)
* Cleanup time_unix.c (`#389 <https://github.com/ros2/rcutils/issues/389>`__)
* time_unix: namespace zephyr headers (`#383 <https://github.com/ros2/rcutils/issues/383>`__)
* Restrict overmatching MACH ifdef to only trigger on OSX and Mach (`#386 <https://github.com/ros2/rcutils/issues/386>`__)
* Optimize rcutils_logging_get_logger_effective_level() (`#381 <https://github.com/ros2/rcutils/issues/381>`__)
* Change syntax __VAR_ARGS_\_ to __VA_ARGS_\_ (`#376 <https://github.com/ros2/rcutils/issues/376>`__)
* Fix a bug in hash_map_get_next_key_and_data. (`#375 <https://github.com/ros2/rcutils/issues/375>`__)
* More fixes from review.
* Fixes from review.
* Make g_rcutils_logging_output_handler static.
* Make g_rcutils_logging_default_logger_level static.
* Optimize rcutils_find_lastn where possible.
* Don't bother computing the hash_map key if the hash map is empty.
* Make sure to expand char_array by at least 1.5x.
* Optimize index computation in hash_map_find.
* Improve the performance of rcutils_logging_format_message. (`#372 <https://github.com/ros2/rcutils/issues/372>`__)
* Get rid of unnecessary ret variable.
* Get rid of unnecessary ifdef cplusplus checks in the C file.
* Get rid of unnecessary rcutils_custom_add\_{gtest,gmock}
* Get rid of unnecessary and unused RMW switching for logging tests.
* Remove unnecessary IS_OUTPUT_COLORIZED macro.
* Rename logging internal structures to use our new convention.
* Make all of the logging 'expand' methods static.
* Fix up error checking for RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED.
* Cleanup error handling for the RCUTILS_CONSOLE_OUTPUT_FORMAT checks.
* Revamp error handling in rcutils_logging_initialize_with_allocator.
* Revamp rcutils_logging_initialize_with_allocator.
* Make a few logging global variables static.
* Optimize calls via the RCUTILS_LOG macros. (`#369 <https://github.com/ros2/rcutils/issues/369>`__)
* time_unix: add zephyr posix time (`#368 <https://github.com/ros2/rcutils/issues/368>`__)
* Optimize the implementation of rcutils_char_array_strncpy. (`#367 <https://github.com/ros2/rcutils/issues/367>`__)
* strdup.c: fix arbitrary length overread (`#366 <https://github.com/ros2/rcutils/issues/366>`__)
* Mirror rolling to master
* strdup.c: fix 1 byte buffer overread (`#363 <https://github.com/ros2/rcutils/issues/363>`__)
* Clarify duration arg description in logging macros (`#359 <https://github.com/ros2/rcutils/issues/359>`__)
* Update rcutils_steady_time_now to return the same data as std::chrono (`#357 <https://github.com/ros2/rcutils/issues/357>`__)
* Contributors: AIxWall, Abrar Rahman Protyasha, Audrow Nash, Chen Lihui, Chris Lalancette, Emerson Knapp, Felipe Neves, Jacob Perron, Mario Prats, Maximilian Downey Twiss, Nikolai Morin, Tomoya Fujita, William Woodall, Yakumoo, guijan, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw <https://github.com/ros2/rmw/tree/iron/rmw/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Dynamic Subscription (BONUS: Allocators): rmw (`#353 <https://github.com/ros2/rmw/issues/353>`__)
* Runtime Interface Reflection: rmw (`#340 <https://github.com/ros2/rmw/issues/340>`__)
* [rmw] Improve handling of dynamic discovery (`#338 <https://github.com/ros2/rmw/issues/338>`__)
* rmw_send_reqponse returns RMW_RET_TIMEOUT. (`#350 <https://github.com/ros2/rmw/issues/350>`__)
* Add a note about asynchronicity of discovery. (`#352 <https://github.com/ros2/rmw/issues/352>`__)
* Add matched event support (`#331 <https://github.com/ros2/rmw/issues/331>`__)
* Add type hash to rmw_topic_endpoint_info_t (rep2011) (`#348 <https://github.com/ros2/rmw/issues/348>`__)
* Add in inconsistent topic defines and data structures. (`#339 <https://github.com/ros2/rmw/issues/339>`__)
* Update documented expectations for GIDs (`#335 <https://github.com/ros2/rmw/issues/335>`__)
* Fix rmw->rwm typo (`#347 <https://github.com/ros2/rmw/issues/347>`__)
* Add rmw count clients, services (`#334 <https://github.com/ros2/rmw/issues/334>`__)
* make writer_guid uint8_t[] instead of int8_t for consistency with rmw_gid_t (`#329 <https://github.com/ros2/rmw/issues/329>`__)
* Update rmw to C++17. (`#346 <https://github.com/ros2/rmw/issues/346>`__)
* Reduce GID storage to 16 bytes. (`#345 <https://github.com/ros2/rmw/issues/345>`__)
* Move the RMW_CHECK_TYPE_IDENTIFIERS_MATCH macro to a C header. (`#343 <https://github.com/ros2/rmw/issues/343>`__)
* [rolling] Update maintainers - 2022-11-07 (`#337 <https://github.com/ros2/rmw/issues/337>`__)
* Remove unused test_loaned_message_sequence.cpp (`#336 <https://github.com/ros2/rmw/issues/336>`__)
* callback can be NULL to clear in Listener APIs. (`#332 <https://github.com/ros2/rmw/issues/332>`__)
* Add rmw_get_gid_for_client method (`#327 <https://github.com/ros2/rmw/issues/327>`__)
* Add 'best available' QoS policies (`#320 <https://github.com/ros2/rmw/issues/320>`__) The best available policy should select the highest level of service for the QoS setting while matching with the majority of endpoints. For example, in the case of a DDS middleware subscription, this means: * Prefer reliable reliability if all existing publishers on the same topic are reliable, otherwise use best effort. * Prefer transient local durability if all existing publishers on the same topic are transient local, otherwise use volatile. * Prefer manual by topic liveliness if all existing publishers on the same topic are manual by topic, otherwise use automatic. * Use a deadline that is equal to the largest deadline of existing publishers on the same topic. * Use a liveliness lease duration that is equal to the largest lease duration of existing publishers on the same topic.
* Move statuses definitions to rmw/events_statuses/ (`#232 <https://github.com/ros2/rmw/issues/232>`__)
* Contributors: Audrow Nash, Barry Xu, Brian, Chris Lalancette, Emerson Knapp, Geoffrey Biggs, Jacob Perron, Lee, Minju, Nikolai Morin, Tomoya Fujita, William Woodall, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_connextdds <https://github.com/ros2/rmw_connextdds/tree/iron/rmw_connextdds/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Dynamic Subscription (BONUS: Allocators): rmw_connextdds (`#115 <https://github.com/ros2/rmw_connextdds/issues/115>`__)
* Revert "Refactor serialization support to use allocators and refs"
* Refactor serialization support to use allocators and refs
* Add stubs for new rmw interfaces (`#111 <https://github.com/ros2/rmw_connextdds/issues/111>`__)
* Add rmw_get_gid_for_client impl (`#92 <https://github.com/ros2/rmw_connextdds/issues/92>`__)
* Switch ROS2 -> ROS 2 everywhere (`#83 <https://github.com/ros2/rmw_connextdds/issues/83>`__)
* Contributors: Brian, Chris Lalancette, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_connextdds_common <https://github.com/ros2/rmw_connextdds/tree/iron/rmw_connextdds_common/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rmw_connextdds] New RMW discovery options (`#108 <https://github.com/ros2/rmw_connextdds/issues/108>`__)
* Call get_type_hash_func (`#113 <https://github.com/ros2/rmw_connextdds/issues/113>`__)
* Type hash distribution during discovery (rep2011) (`#104 <https://github.com/ros2/rmw_connextdds/issues/104>`__)
* Implement matched event (`#101 <https://github.com/ros2/rmw_connextdds/issues/101>`__)
* Add in implementation of inconsistent topic. (`#103 <https://github.com/ros2/rmw_connextdds/issues/103>`__)
* Add rmw_get_gid_for_client impl (`#92 <https://github.com/ros2/rmw_connextdds/issues/92>`__)
* Fix assert statement to allow the seconds field of a DDS_Duration_t to be zero (`#88 <https://github.com/ros2/rmw_connextdds/issues/88>`__)
* Handle 'best_available' QoS policies in common  (`#85 <https://github.com/ros2/rmw_connextdds/issues/85>`__)
* Resolve build error with RTI Connext DDS 5.3.1 (`#82 <https://github.com/ros2/rmw_connextdds/issues/82>`__)
* Contributors: Andrea Sorbini, Barry Xu, Brian, Chris Lalancette, Emerson Knapp, Grey, Jose Luis Rivero, Michael Carroll, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_connextddsmicro <https://github.com/ros2/rmw_connextdds/tree/iron/rmw_connextddsmicro/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Dynamic Subscription (BONUS: Allocators): rmw_connextdds (`#115 <https://github.com/ros2/rmw_connextdds/issues/115>`__)
* Add stubs for new rmw interfaces (`#111 <https://github.com/ros2/rmw_connextdds/issues/111>`__)
* Add rmw_get_gid_for_client impl (`#92 <https://github.com/ros2/rmw_connextdds/issues/92>`__)
* Switch ROS2 -> ROS 2 everywhere (`#83 <https://github.com/ros2/rmw_connextdds/issues/83>`__)
* Contributors: Brian, Chris Lalancette, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_cyclonedds_cpp <https://github.com/ros2/rmw_cyclonedds/tree/iron/rmw_cyclonedds_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Dynamic Subscription (BONUS: Allocators): rmw_cyclonedds (`#451 <https://github.com/ros2/rmw_cyclonedds/issues/451>`__)
* Add stubs for new rmw interfaces (`#447 <https://github.com/ros2/rmw_cyclonedds/issues/447>`__)
* [rmw_cyclonedds] Improve handling of dynamic discovery (`#429 <https://github.com/ros2/rmw_cyclonedds/issues/429>`__)
* Call get_type_hash_func (`#448 <https://github.com/ros2/rmw_cyclonedds/issues/448>`__)
* Type hash distribution in discovery (rep2011) (`#437 <https://github.com/ros2/rmw_cyclonedds/issues/437>`__)
* Disable inconsistent topic events. (`#444 <https://github.com/ros2/rmw_cyclonedds/issues/444>`__)
* Implement matched event (`#435 <https://github.com/ros2/rmw_cyclonedds/issues/435>`__)
* Implement inconsistent topic. (`#431 <https://github.com/ros2/rmw_cyclonedds/issues/431>`__)
* Make sure to add semicolons to the CHECK_TYPE_IDENTIFIER_MATCH. (`#432 <https://github.com/ros2/rmw_cyclonedds/issues/432>`__)
* [rolling] Update maintainers - 2022-11-07 (`#428 <https://github.com/ros2/rmw_cyclonedds/issues/428>`__)
* Export CycloneDDS dependency (`#424 <https://github.com/ros2/rmw_cyclonedds/issues/424>`__)
* add NULL check before accessing object. (`#423 <https://github.com/ros2/rmw_cyclonedds/issues/423>`__)
* Add rmw_get_gid_for_client impl (`#402 <https://github.com/ros2/rmw_cyclonedds/issues/402>`__)
* Makes topic_name a const ref
* Adds topic name to error msg when create_topic fails
* Improve error message when create_topic fails (`#405 <https://github.com/ros2/rmw_cyclonedds/issues/405>`__)
* Change wrong use of %d to print uint32_t to PRIu32 (`#253 <https://github.com/ros2/rmw_cyclonedds/issues/253>`__)
* Add cstring include. (`#393 <https://github.com/ros2/rmw_cyclonedds/issues/393>`__)
* Handle 'best_available' QoS policies (`#389 <https://github.com/ros2/rmw_cyclonedds/issues/389>`__)
* Contributors: Audrow Nash, Barry Xu, Brian, Chris Lalancette, Emerson Knapp, Geoffrey Biggs, Jose Luis Rivero, Shane Loretz, Tomoya Fujita, Tully Foote, Voldivh, eboasson, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_dds_common <https://github.com/ros2/rmw_dds_common/tree/iron/rmw_dds_common/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Type hash in GraphCache, user_data encoding tools (`#70 <https://github.com/ros2/rmw_dds_common/issues/70>`__)
* Mark benchmark _ as unused. (`#71 <https://github.com/ros2/rmw_dds_common/issues/71>`__)
* Update rmw_dds_common to C++17. (`#69 <https://github.com/ros2/rmw_dds_common/issues/69>`__)
* Change Gid.msg to be 16 bytes. (`#68 <https://github.com/ros2/rmw_dds_common/issues/68>`__)
* Minor cleanups of test_qos. (`#67 <https://github.com/ros2/rmw_dds_common/issues/67>`__)
* [rolling] Update maintainers - 2022-11-07 (`#65 <https://github.com/ros2/rmw_dds_common/issues/65>`__)
* build shared lib only if BUILD_SHARED_LIBS is set (`#62 <https://github.com/ros2/rmw_dds_common/issues/62>`__)
* Update maintainers (`#61 <https://github.com/ros2/rmw_dds_common/issues/61>`__)
* Add functions for resolving 'best available' QoS policies (`#60 <https://github.com/ros2/rmw_dds_common/issues/60>`__) Given a QoS profile and set of endpoints for the same topic, overwrite any policies set to BEST_AVAILABLE with a policy such that it matches all endpoints while maintaining a high level of service. Add testable functions for updating BEST_AVAILABLE policies, * qos_profile_get_best_available_for_subscription * qos_profile_get_best_available_for_publisher and add convenience functions that actual query the graph for RMW implementations to use, * qos_profile_get_best_available_for_topic_subscription * qos_profile_get_best_available_for_topic_publisher
* Contributors: Audrow Nash, Chris Lalancette, Emerson Knapp, Jacob Perron, hannes09, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_fastrtps_cpp <https://github.com/ros2/rmw_fastrtps/tree/iron/rmw_fastrtps_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Dynamic Subscription (BONUS: Allocators): rmw_fastrtps (`#687 <https://github.com/ros2/rmw_fastrtps/issues/687>`__)
* Runtime Interface Reflection: rmw_fastrtps (`#655 <https://github.com/ros2/rmw_fastrtps/issues/655>`__)
* [rmw_fastrtps] Improve handling of dynamic discovery (`#653 <https://github.com/ros2/rmw_fastrtps/issues/653>`__)
* Call get_type_hash_func (`#680 <https://github.com/ros2/rmw_fastrtps/issues/680>`__)
* Type hash distribution in discovery (rep2011) (`#671 <https://github.com/ros2/rmw_fastrtps/issues/671>`__)
* Implement inconsistent topic event (`#654 <https://github.com/ros2/rmw_fastrtps/issues/654>`__)
* Update all rmw_fastrtps packages to C++17. (`#674 <https://github.com/ros2/rmw_fastrtps/issues/674>`__)
* Rewrite how Topics are tracked in rmw_fastrtps_cpp. (`#669 <https://github.com/ros2/rmw_fastrtps/issues/669>`__)
* Allow loaned messages without data-sharing (`#568 <https://github.com/ros2/rmw_fastrtps/issues/568>`__)
* Fix incoherent dissociate_writer to dissociate_reader (`#647 <https://github.com/ros2/rmw_fastrtps/issues/647>`__) (`#649 <https://github.com/ros2/rmw_fastrtps/issues/649>`__)
* [rolling] Update maintainers - 2022-11-07 (`#643 <https://github.com/ros2/rmw_fastrtps/issues/643>`__)
* Add rmw_get_gid_for_client impl (`#631 <https://github.com/ros2/rmw_fastrtps/issues/631>`__)
* Use Fast-DDS Waitsets instead of listeners (`#619 <https://github.com/ros2/rmw_fastrtps/issues/619>`__)
* Remove rosidl_cmake dependency (`#629 <https://github.com/ros2/rmw_fastrtps/issues/629>`__)
* Revert "add line feed for RCUTILS_SAFE_FWRITE_TO_STDERR (`#608 <https://github.com/ros2/rmw_fastrtps/issues/608>`__)" (`#612 <https://github.com/ros2/rmw_fastrtps/issues/612>`__)
* add line feed for RCUTILS_SAFE_FWRITE_TO_STDERR (`#608 <https://github.com/ros2/rmw_fastrtps/issues/608>`__)
* Allow null arguments in the EventsExecutor parameters (`#602 <https://github.com/ros2/rmw_fastrtps/issues/602>`__)
* Add RMW_CHECKS to rmw_fastrtps_cpp EventsExecutor implementation
* Handle 'best_available' QoS policies (`#598 <https://github.com/ros2/rmw_fastrtps/issues/598>`__)
* Contributors: Audrow Nash, Brian, Chris Lalancette, Emerson Knapp, Geoffrey Biggs, Jacob Perron, Jose Luis Rivero, Miguel Company, Oscarchoi, Ricardo González, Tomoya Fujita, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_fastrtps_dynamic_cpp <https://github.com/ros2/rmw_fastrtps/tree/iron/rmw_fastrtps_dynamic_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Dynamic Subscription (BONUS: Allocators): rmw_fastrtps (`#687 <https://github.com/ros2/rmw_fastrtps/issues/687>`__)
* Runtime Interface Reflection: rmw_fastrtps (`#655 <https://github.com/ros2/rmw_fastrtps/issues/655>`__)
* [rmw_fastrtps] Improve handling of dynamic discovery (`#653 <https://github.com/ros2/rmw_fastrtps/issues/653>`__)
* Call get_type_hash_func (`#680 <https://github.com/ros2/rmw_fastrtps/issues/680>`__)
* Type hash distribution in discovery (rep2011) (`#671 <https://github.com/ros2/rmw_fastrtps/issues/671>`__)
* Implement inconsistent topic event (`#654 <https://github.com/ros2/rmw_fastrtps/issues/654>`__)
* Update all rmw_fastrtps packages to C++17. (`#674 <https://github.com/ros2/rmw_fastrtps/issues/674>`__)
* Rewrite how Topics are tracked in rmw_fastrtps_cpp. (`#669 <https://github.com/ros2/rmw_fastrtps/issues/669>`__)
* Allow loaned messages without data-sharing (`#568 <https://github.com/ros2/rmw_fastrtps/issues/568>`__)
* Fix incoherent dissociate_writer to dissociate_reader (`#647 <https://github.com/ros2/rmw_fastrtps/issues/647>`__) (`#649 <https://github.com/ros2/rmw_fastrtps/issues/649>`__)
* [rolling] Update maintainers - 2022-11-07 (`#643 <https://github.com/ros2/rmw_fastrtps/issues/643>`__)
* Add rmw_get_gid_for_client impl (`#631 <https://github.com/ros2/rmw_fastrtps/issues/631>`__)
* Use Fast-DDS Waitsets instead of listeners (`#619 <https://github.com/ros2/rmw_fastrtps/issues/619>`__)
* Revert "add line feed for RCUTILS_SAFE_FWRITE_TO_STDERR (`#608 <https://github.com/ros2/rmw_fastrtps/issues/608>`__)" (`#612 <https://github.com/ros2/rmw_fastrtps/issues/612>`__)
* add line feed for RCUTILS_SAFE_FWRITE_TO_STDERR (`#608 <https://github.com/ros2/rmw_fastrtps/issues/608>`__)
* Allow null arguments in the EventsExecutor parameters (`#602 <https://github.com/ros2/rmw_fastrtps/issues/602>`__)
* Add EventExecutor to rmw_fastrtps_dynamic_cpp
* Fix cpplint error (`#601 <https://github.com/ros2/rmw_fastrtps/issues/601>`__)
* Handle 'best_available' QoS policies (`#598 <https://github.com/ros2/rmw_fastrtps/issues/598>`__)
* Contributors: Audrow Nash, Brian, Chris Lalancette, Emerson Knapp, Geoffrey Biggs, Jacob Perron, Jose Luis Rivero, Miguel Company, Oscarchoi, Ricardo González, Tomoya Fujita, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_fastrtps_shared_cpp <https://github.com/ros2/rmw_fastrtps/tree/iron/rmw_fastrtps_shared_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix matched event issues (`#683 <https://github.com/ros2/rmw_fastrtps/issues/683>`__)
* Dynamic Subscription (BONUS: Allocators): rmw_fastrtps (`#687 <https://github.com/ros2/rmw_fastrtps/issues/687>`__)
* Check for triggered guard conditions before waiting (`#685 <https://github.com/ros2/rmw_fastrtps/issues/685>`__)
* Runtime Interface Reflection: rmw_fastrtps (`#655 <https://github.com/ros2/rmw_fastrtps/issues/655>`__)
* [rmw_fastrtps] Improve handling of dynamic discovery (`#653 <https://github.com/ros2/rmw_fastrtps/issues/653>`__)
* Type hash distribution in discovery (rep2011) (`#671 <https://github.com/ros2/rmw_fastrtps/issues/671>`__)
* Implement matched event (`#645 <https://github.com/ros2/rmw_fastrtps/issues/645>`__)
* Implement inconsistent topic event (`#654 <https://github.com/ros2/rmw_fastrtps/issues/654>`__)
* Update all rmw_fastrtps packages to C++17. (`#674 <https://github.com/ros2/rmw_fastrtps/issues/674>`__)
* Rewrite how Topics are tracked in rmw_fastrtps_cpp. (`#669 <https://github.com/ros2/rmw_fastrtps/issues/669>`__)
* Delay lock on message callback setters (`#657 <https://github.com/ros2/rmw_fastrtps/issues/657>`__)
* Make sure to add semicolons to the CHECK_TYPE_IDENTIFIER_MATCH. (`#658 <https://github.com/ros2/rmw_fastrtps/issues/658>`__)
* Allow loaned messages without data-sharing (`#568 <https://github.com/ros2/rmw_fastrtps/issues/568>`__)
* Fix incoherent dissociate_writer to dissociate_reader (`#647 <https://github.com/ros2/rmw_fastrtps/issues/647>`__) (`#649 <https://github.com/ros2/rmw_fastrtps/issues/649>`__)
* [rolling] Update maintainers - 2022-11-07 (`#643 <https://github.com/ros2/rmw_fastrtps/issues/643>`__)
* Remove duplicated code (`#637 <https://github.com/ros2/rmw_fastrtps/issues/637>`__)
* Call callbacks only if unread count > 0 (`#634 <https://github.com/ros2/rmw_fastrtps/issues/634>`__)
* Add rmw_get_gid_for_client impl (`#631 <https://github.com/ros2/rmw_fastrtps/issues/631>`__)
* Use Fast-DDS Waitsets instead of listeners (`#619 <https://github.com/ros2/rmw_fastrtps/issues/619>`__)
* Take all available samples on service/client on_data_available. (`#616 <https://github.com/ros2/rmw_fastrtps/issues/616>`__)
* Revert "add line feed for RCUTILS_SAFE_FWRITE_TO_STDERR (`#608 <https://github.com/ros2/rmw_fastrtps/issues/608>`__)" (`#612 <https://github.com/ros2/rmw_fastrtps/issues/612>`__)
* add line feed for RCUTILS_SAFE_FWRITE_TO_STDERR (`#608 <https://github.com/ros2/rmw_fastrtps/issues/608>`__)
* Contributors: Audrow Nash, Barry Xu, Brian, Chris Lalancette, Emerson Knapp, Geoffrey Biggs, Michael Carroll, Miguel Company, Oscarchoi, Ricardo González, Tomoya Fujita, mauropasse, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_implementation <https://github.com/ros2/rmw_implementation/tree/iron/rmw_implementation/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Dynamic Subscription (BONUS: Allocators): rmw_implementation (`#219 <https://github.com/ros2/rmw_implementation/issues/219>`__)
* Runtime Interface Reflection: rmw_implementation (`#215 <https://github.com/ros2/rmw_implementation/issues/215>`__)
* Mark the benchmark _ variables as unused. (`#218 <https://github.com/ros2/rmw_implementation/issues/218>`__)
* Update rmw_implementation to C++17. (`#214 <https://github.com/ros2/rmw_implementation/issues/214>`__)
* [rolling] Update maintainers - 2022-11-07 (`#212 <https://github.com/ros2/rmw_implementation/issues/212>`__)
* Build-time RMW selection does not need ament_index_cpp (`#210 <https://github.com/ros2/rmw_implementation/issues/210>`__)
* Add rmw_get_gid_for_client & tests (`#206 <https://github.com/ros2/rmw_implementation/issues/206>`__)
* Contributors: Audrow Nash, Brian, Chris Lalancette, G.A. vd. Hoorn, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rmw_implementation_cmake <https://github.com/ros2/rmw/tree/iron/rmw_implementation_cmake/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#337 <https://github.com/ros2/rmw/issues/337>`__)
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`robot_state_publisher <https://github.com/ros/robot_state_publisher/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update robot_state_publisher to C++17. (`#204 <https://github.com/ros/robot_state_publisher/issues/204>`__)
* [rolling] Update maintainers - 2022-11-07 (`#203 <https://github.com/ros/robot_state_publisher/issues/203>`__)
* Mirror rolling to ros2
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2action <https://github.com/ros2/ros2cli/tree/iron/ros2action/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Make all of the dependencies in pure Python packages exec_depend. (`#823 <https://github.com/ros2/ros2cli/issues/823>`__)
* [rolling] Update maintainers - 2022-11-07 (`#776 <https://github.com/ros2/ros2cli/issues/776>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2bag <https://github.com/ros2/rosbag2/tree/iron/ros2bag/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Cleanup the help text for ros2 bag record. (`#1329 <https://github.com/ros2/rosbag2/issues/1329>`__) (`#1333 <https://github.com/ros2/rosbag2/issues/1333>`__)
* Enable document generation using rosdoc2 for ament_python pkgs (`#1260 <https://github.com/ros2/rosbag2/issues/1260>`__)
* CLI: Get storage-specific values from plugin (`#1209 <https://github.com/ros2/rosbag2/issues/1209>`__)
* Fix up some of the wording in the record help text. (`#1228 <https://github.com/ros2/rosbag2/issues/1228>`__)
* Add topic_name option to info verb (`#1217 <https://github.com/ros2/rosbag2/issues/1217>`__)
* rosbag2_storage: set MCAP as default plugin (`#1160 <https://github.com/ros2/rosbag2/issues/1160>`__)
* rosbag2_py: parametrize tests across storage plugins (`#1203 <https://github.com/ros2/rosbag2/issues/1203>`__)
* Added option to change node name for the recorder from the Python API (`#1180 <https://github.com/ros2/rosbag2/issues/1180>`__)
* rosbag2_cpp: test more than one storage plugin (`#1196 <https://github.com/ros2/rosbag2/issues/1196>`__)
* rosbag2_storage: expose default storage ID as method (`#1146 <https://github.com/ros2/rosbag2/issues/1146>`__)
* Fix for ros2 bag play exit with non-zero code on SIGINT (`#1126 <https://github.com/ros2/rosbag2/issues/1126>`__)
* ros2bag: move storage preset validation to sqlite3 plugin (`#1135 <https://github.com/ros2/rosbag2/issues/1135>`__)
* Add option to prevent message loss while converting (`#1058 <https://github.com/ros2/rosbag2/issues/1058>`__)
* Added support for excluding topics via regular expressions (`#1046 <https://github.com/ros2/rosbag2/issues/1046>`__)
* Readers/info can accept a single bag storage file, and detect its storage id automatically (`#1072 <https://github.com/ros2/rosbag2/issues/1072>`__)
* Add short -v option to ros2 bag list for verbose (`#1065 <https://github.com/ros2/rosbag2/issues/1065>`__)
* Use a single variable for evaluating the filter regex (`#1053 <https://github.com/ros2/rosbag2/issues/1053>`__)
* Add additional mode of publishing sim time updates triggered by replayed messages (`#1050 <https://github.com/ros2/rosbag2/issues/1050>`__)
* Renamed --topics-regex to --regex and -e in Player class to be consistent with Recorder (`#1045 <https://github.com/ros2/rosbag2/issues/1045>`__)
* Use first available writer in recording if default ``sqlite3`` not available. (`#1044 <https://github.com/ros2/rosbag2/issues/1044>`__)
* Add the ability to record any key/value pair in 'custom' field in metadata.yaml (`#1038 <https://github.com/ros2/rosbag2/issues/1038>`__)
* Added support for filtering topics via regular expressions on Playback (`#1034 <https://github.com/ros2/rosbag2/issues/1034>`__)
* Fix incorrect boundary check for ``playback_duration`` and ``play_until_timestamp`` (`#1032 <https://github.com/ros2/rosbag2/issues/1032>`__)
* Adds play until timestamp functionality (`#1005 <https://github.com/ros2/rosbag2/issues/1005>`__)
* Add CLI verb for burst mode of playback (`#980 <https://github.com/ros2/rosbag2/issues/980>`__)
* Add play-for specified number of seconds functionality (`#960 <https://github.com/ros2/rosbag2/issues/960>`__)
* Make unpublished topics unrecorded by default (`#968 <https://github.com/ros2/rosbag2/issues/968>`__)
* Contributors: Agustin Alba Chicar, Chris Lalancette, DensoADAS, Emerson Knapp, EsipovPA, Esteve Fernandez, Geoffrey Biggs, Hunter L.Allen, Keisuke Shima, Michael Orlov, Sean Kelly, Tony Peng, Yadu, james-rms, kylemarcey, mergify[bot], ricardo-manriquez


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2cli <https://github.com/ros2/ros2cli/tree/iron/ros2cli/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Set automatically_declare_parameters_from_overrides in DirectNode. (`#813 <https://github.com/ros2/ros2cli/issues/813>`__)
* Enable document generation using rosdoc2 (`#811 <https://github.com/ros2/ros2cli/issues/811>`__)
* Fix linters (`#808 <https://github.com/ros2/ros2cli/issues/808>`__)
* add timeout option for ros2param to find node. (`#802 <https://github.com/ros2/ros2cli/issues/802>`__)
* Save method list via connection check to XMLRPC server. (`#796 <https://github.com/ros2/ros2cli/issues/796>`__)
* ZSH argcomplete: call compinit only if needed (`#750 <https://github.com/ros2/ros2cli/issues/750>`__)
* Fix network aware node issue (`#785 <https://github.com/ros2/ros2cli/issues/785>`__)
* [rolling] Update maintainers - 2022-11-07 (`#776 <https://github.com/ros2/ros2cli/issues/776>`__)
* XMLRPC server accepts request from all local IP addresses. (`#729 <https://github.com/ros2/ros2cli/issues/729>`__)
* Contributors: Audrow Nash, Chris Lalancette, Cristóbal Arroyo, Ivan Santiago Paunovic, Tomoya Fujita, Yadu, mjbogusz


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2cli_common_extensions <https://github.com/ros2/ros2cli_common_extensions/tree/iron/ros2cli_common_extensions/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#7 <https://github.com/ros2/ros2cli_common_extensions/issues/7>`__)
* Update maintainers (`#6 <https://github.com/ros2/ros2cli_common_extensions/issues/6>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2cli_test_interfaces <https://github.com/ros2/ros2cli/tree/iron/ros2cli_test_interfaces/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#776 <https://github.com/ros2/ros2cli/issues/776>`__)
* Remove action_msgs dependency (`#743 <https://github.com/ros2/ros2cli/issues/743>`__)
* Contributors: Audrow Nash, Jacob Perron


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2component <https://github.com/ros2/ros2cli/tree/iron/ros2component/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Enable document generation using rosdoc2 (`#811 <https://github.com/ros2/ros2cli/issues/811>`__)
* [rolling] Update maintainers - 2022-11-07 (`#776 <https://github.com/ros2/ros2cli/issues/776>`__)
* Fix the component load help to mention load, not unload. (`#756 <https://github.com/ros2/ros2cli/issues/756>`__)
* Remove unused arguments from ros2 component types. (`#711 <https://github.com/ros2/ros2cli/issues/711>`__)
* Contributors: Audrow Nash, Chris Lalancette, Yadu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2doctor <https://github.com/ros2/ros2cli/tree/iron/ros2doctor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Shutdown ros2doctor hello when ctrl-c is received (`#829 <https://github.com/ros2/ros2cli/issues/829>`__)
* Make all of the dependencies in pure Python packages exec_depend. (`#823 <https://github.com/ros2/ros2cli/issues/823>`__)
* Enable document generation using rosdoc2 (`#811 <https://github.com/ros2/ros2cli/issues/811>`__) * Fix warnings for ros2component, ros2doctor, ros2interface, and ros2node
* [rolling] Update maintainers - 2022-11-07 (`#776 <https://github.com/ros2/ros2cli/issues/776>`__)
* Contributors: Audrow Nash, Chris Lalancette, Michael Carroll, Yadu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2interface <https://github.com/ros2/ros2cli/tree/iron/ros2interface/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Make all of the dependencies in pure Python packages exec_depend. (`#823 <https://github.com/ros2/ros2cli/issues/823>`__)
* Enable document generation using rosdoc2 (`#811 <https://github.com/ros2/ros2cli/issues/811>`__)
* [rolling] Update maintainers - 2022-11-07 (`#776 <https://github.com/ros2/ros2cli/issues/776>`__)
* Contributors: Audrow Nash, Chris Lalancette, Yadu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2launch <https://github.com/ros2/launch_ros/tree/iron/ros2launch/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#331 <https://github.com/ros2/launch_ros/issues/331>`__)
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2lifecycle <https://github.com/ros2/ros2cli/tree/iron/ros2lifecycle/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Make all of the dependencies in pure Python packages exec_depend. (`#823 <https://github.com/ros2/ros2cli/issues/823>`__)
* [rolling] Update maintainers - 2022-11-07 (`#776 <https://github.com/ros2/ros2cli/issues/776>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2lifecycle_test_fixtures <https://github.com/ros2/ros2cli/tree/iron/ros2lifecycle_test_fixtures/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the ros2cli test fixture to C++17. (`#789 <https://github.com/ros2/ros2cli/issues/789>`__)
* [rolling] Update maintainers - 2022-11-07 (`#776 <https://github.com/ros2/ros2cli/issues/776>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2multicast <https://github.com/ros2/ros2cli/tree/iron/ros2multicast/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Make all of the dependencies in pure Python packages exec_depend. (`#823 <https://github.com/ros2/ros2cli/issues/823>`__)
* [rolling] Update maintainers - 2022-11-07 (`#776 <https://github.com/ros2/ros2cli/issues/776>`__)
* Add --group and --port options to ros2 multicast (`#770 <https://github.com/ros2/ros2cli/issues/770>`__)
* Contributors: Audrow Nash, Chris Lalancette, Shane Loretz


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2node <https://github.com/ros2/ros2cli/tree/iron/ros2node/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Make all of the dependencies in pure Python packages exec_depend. (`#823 <https://github.com/ros2/ros2cli/issues/823>`__)
* Enable document generation using rosdoc2 (`#811 <https://github.com/ros2/ros2cli/issues/811>`__) * Fix warnings for ros2component, ros2doctor, ros2interface, and ros2node
* Fix linters (`#808 <https://github.com/ros2/ros2cli/issues/808>`__)
* add timeout option for ros2param to find node. (`#802 <https://github.com/ros2/ros2cli/issues/802>`__)
* [rolling] Update maintainers - 2022-11-07 (`#776 <https://github.com/ros2/ros2cli/issues/776>`__)
* Updated wording in list.py (`#775 <https://github.com/ros2/ros2cli/issues/775>`__)
* Contributors: Audrow Nash, Chris Lalancette, Cristóbal Arroyo, Michael Wrock, Tomoya Fujita, Yadu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2param <https://github.com/ros2/ros2cli/tree/iron/ros2param/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* remove deprecated options (`#824 <https://github.com/ros2/ros2cli/issues/824>`__)
* Make all of the dependencies in pure Python packages exec_depend. (`#823 <https://github.com/ros2/ros2cli/issues/823>`__)
* add timeout option for ros2param to find node. (`#802 <https://github.com/ros2/ros2cli/issues/802>`__)
* Fix printing of integer and double arrays. (`#804 <https://github.com/ros2/ros2cli/issues/804>`__)
* [rolling] Update maintainers - 2022-11-07 (`#776 <https://github.com/ros2/ros2cli/issues/776>`__)
* refactor: make ros2param use rclpy.parameter_client (`#716 <https://github.com/ros2/ros2cli/issues/716>`__)
* Contributors: Audrow Nash, Brian, Chris Lalancette, Tomoya Fujita


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2pkg <https://github.com/ros2/ros2cli/tree/iron/ros2pkg/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix typo in ros2pkg warning message. (`#828 <https://github.com/ros2/ros2cli/issues/828>`__)
* Make all of the dependencies in pure Python packages exec_depend. (`#823 <https://github.com/ros2/ros2cli/issues/823>`__)
* resolve `#790 <https://github.com/ros2/ros2cli/issues/790>`__ (`#801 <https://github.com/ros2/ros2cli/issues/801>`__)
* Add alias library targets for CMake (`#718 <https://github.com/ros2/ros2cli/issues/718>`__)
* [rolling] Update maintainers - 2022-11-07 (`#776 <https://github.com/ros2/ros2cli/issues/776>`__)
* Contributors: Audrow Nash, Chris Lalancette, Kenji Brameld, RFRIEDM-Trimble, Tomoya Fujita


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2run <https://github.com/ros2/ros2cli/tree/iron/ros2run/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Make all of the dependencies in pure Python packages exec_depend. (`#823 <https://github.com/ros2/ros2cli/issues/823>`__)
* [rolling] Update maintainers - 2022-11-07 (`#776 <https://github.com/ros2/ros2cli/issues/776>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2service <https://github.com/ros2/ros2cli/tree/iron/ros2service/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Make all of the dependencies in pure Python packages exec_depend. (`#823 <https://github.com/ros2/ros2cli/issues/823>`__)
* [rolling] Update maintainers - 2022-11-07 (`#776 <https://github.com/ros2/ros2cli/issues/776>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2test <https://github.com/ros2/ros_testing/tree/iron/ros2test/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#12 <https://github.com/ros2/ros_testing/issues/12>`__)
* update maintainer
* Contributors: Audrow Nash, Dharini Dutia, quarkytale


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2topic <https://github.com/ros2/ros2cli/tree/iron/ros2topic/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* remove deprecated options (`#824 <https://github.com/ros2/ros2cli/issues/824>`__)
* Make all of the dependencies in pure Python packages exec_depend. (`#823 <https://github.com/ros2/ros2cli/issues/823>`__)
* Expect type hash cli output in test (`#822 <https://github.com/ros2/ros2cli/issues/822>`__)
* Fix the type annotation in pub.py. (`#814 <https://github.com/ros2/ros2cli/issues/814>`__)
* Switch to using new event_handler instead of qos_event. (`#787 <https://github.com/ros2/ros2cli/issues/787>`__)
* avoid flaky test that subscriber might not receive the message (`#810 <https://github.com/ros2/ros2cli/issues/810>`__)
* Adds a ``--max-wait-time`` option to ``ros2 topic pub``  (`#800 <https://github.com/ros2/ros2cli/issues/800>`__)
* Fix some flake8 warnings related to style. (`#805 <https://github.com/ros2/ros2cli/issues/805>`__)
* Adds a timeout feature to rostopic echo (`#792 <https://github.com/ros2/ros2cli/issues/792>`__)
* Refactor common types (`#791 <https://github.com/ros2/ros2cli/issues/791>`__)
* Allow configuring liveliness in ros2 topic echo and pub (`#788 <https://github.com/ros2/ros2cli/issues/788>`__)
* Extend timeout to shutdown the command line process. (`#783 <https://github.com/ros2/ros2cli/issues/783>`__)
* [rolling] Update maintainers - 2022-11-07 (`#776 <https://github.com/ros2/ros2cli/issues/776>`__)
* a couple of typo fixes. (`#774 <https://github.com/ros2/ros2cli/issues/774>`__)
* Add support use_sim_time for ros2 topic hz/bw/pub. (`#754 <https://github.com/ros2/ros2cli/issues/754>`__)
* Use set_message_fields from rosidl_runtime_py (`#761 <https://github.com/ros2/ros2cli/issues/761>`__)
* Expand auto to the current time when passed to a Header field (`#749 <https://github.com/ros2/ros2cli/issues/749>`__)
* Add verbose option to echo that also prints the associated message info (`#707 <https://github.com/ros2/ros2cli/issues/707>`__)
* update docs for bandwidth functions. (`#709 <https://github.com/ros2/ros2cli/issues/709>`__)
* Split the bandwidth functions into a get and print. (`#708 <https://github.com/ros2/ros2cli/issues/708>`__)
* Contributors: Arjo Chakravarty, Audrow Nash, Chen Lihui, Chris Lalancette, Emerson Knapp, Esteve Fernandez, Ivan Santiago Paunovic, Lei Liu, Tomoya Fujita


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros2trace <https://github.com/ros2/ros2_tracing/tree/iron/ros2trace/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Move ros2trace tests to new test_ros2trace package (`#63 <https://github.com/ros2/ros2_tracing/issues/63>`__)
* Error out if trace already exists unless 'append' option is used (`#58 <https://github.com/ros2/ros2_tracing/issues/58>`__)
* Improve 'ros2 trace' command error handling & add end-to-end tests (`#54 <https://github.com/ros2/ros2_tracing/issues/54>`__)
* Contributors: Christophe Bedard


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ros_testing <https://github.com/ros2/ros_testing/tree/iron/ros_testing/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#12 <https://github.com/ros2/ros_testing/issues/12>`__)
* update maintainer
* Contributors: Audrow Nash, Dharini Dutia, quarkytale


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2 <https://github.com/ros2/rosbag2/tree/iron/rosbag2/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`__)
* Move sqlite3 storage implementation to rosbag2_storage_sqlite3 package (`#1113 <https://github.com/ros2/rosbag2/issues/1113>`__)
* Contributors: Emerson Knapp, Michael Orlov


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_compression <https://github.com/ros2/rosbag2/tree/iron/rosbag2_compression/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add in a missing cstdint include. (`#1321 <https://github.com/ros2/rosbag2/issues/1321>`__) (`#1322 <https://github.com/ros2/rosbag2/issues/1322>`__)
* Fix warning from ClassLoader in sequential compression reader and writer (`#1299 <https://github.com/ros2/rosbag2/issues/1299>`__) (`#1316 <https://github.com/ros2/rosbag2/issues/1316>`__)
* Add message definition read API (`#1292 <https://github.com/ros2/rosbag2/issues/1292>`__)
* rosbag2_storage: add type description hash to topic metadata (`#1272 <https://github.com/ros2/rosbag2/issues/1272>`__)
* rosbag2_cpp: move local message definition source out of MCAP plugin (`#1265 <https://github.com/ros2/rosbag2/issues/1265>`__)
* Update rosbag2 to C++17. (`#1238 <https://github.com/ros2/rosbag2/issues/1238>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#1202 <https://github.com/ros2/rosbag2/issues/1202>`__)
* set_read_order: return success (`#1177 <https://github.com/ros2/rosbag2/issues/1177>`__)
* Add ``update_metadata(BagMetadata)`` API for storage plugin interface (`#1149 <https://github.com/ros2/rosbag2/issues/1149>`__)
* Reverse read order API and sqlite storage implementation (`#1083 <https://github.com/ros2/rosbag2/issues/1083>`__)
* Add option to prevent message loss while converting (`#1058 <https://github.com/ros2/rosbag2/issues/1058>`__)
* set default metadata of compressed message (in case compressor does not set it) (`#1060 <https://github.com/ros2/rosbag2/issues/1060>`__)
* Speed optimization: Preparing copyless publish/subscribing by using const message for writing (`#1010 <https://github.com/ros2/rosbag2/issues/1010>`__)
* Add the ability to record any key/value pair in 'custom' field in metadata.yaml (`#1038 <https://github.com/ros2/rosbag2/issues/1038>`__)
* Contributors: Chris Lalancette, Daisuke Nishimatsu, DensoADAS, Emerson Knapp, Hunter L. Allen, Joshua Hampp, Michael Orlov, Tony Peng, james-rms, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_compression_zstd <https://github.com/ros2/rosbag2/tree/iron/rosbag2_compression_zstd/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update rosbag2 to C++17. (`#1238 <https://github.com/ros2/rosbag2/issues/1238>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#1202 <https://github.com/ros2/rosbag2/issues/1202>`__)
* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`__)
* Speed optimization: Preparing copyless publish/subscribing by using const message for writing (`#1010 <https://github.com/ros2/rosbag2/issues/1010>`__)
* Contributors: Chris Lalancette, Daisuke Nishimatsu, DensoADAS, Joshua Hampp, Michael Orlov


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_cpp <https://github.com/ros2/rosbag2/tree/iron/rosbag2_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add recorder stop() API (`#1300 <https://github.com/ros2/rosbag2/issues/1300>`__) (`#1334 <https://github.com/ros2/rosbag2/issues/1334>`__)
* Add type_hash in MessageDefinition struct (`#1296 <https://github.com/ros2/rosbag2/issues/1296>`__)
* Add message definition read API (`#1292 <https://github.com/ros2/rosbag2/issues/1292>`__)
* rosbag2_storage: add type description hash to topic metadata (`#1272 <https://github.com/ros2/rosbag2/issues/1272>`__)
* Fix for flaky ``TimeControllerClockTest::unpaused_sleep_returns_true`` test (`#1290 <https://github.com/ros2/rosbag2/issues/1290>`__)
* rosbag2_cpp: move local message definition source out of MCAP plugin (`#1265 <https://github.com/ros2/rosbag2/issues/1265>`__)
* Update rosbag2 to C++17. (`#1238 <https://github.com/ros2/rosbag2/issues/1238>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#1202 <https://github.com/ros2/rosbag2/issues/1202>`__)
* Fix rwm->rmw spelling (`#1249 <https://github.com/ros2/rosbag2/issues/1249>`__)
* Expose more Writer methods in python interface (`#1220 <https://github.com/ros2/rosbag2/issues/1220>`__)
* rosbag2_storage: set MCAP as default plugin (`#1160 <https://github.com/ros2/rosbag2/issues/1160>`__)
* Parametrize all rosbag2_tests for both supported storage plugins (`#1221 <https://github.com/ros2/rosbag2/issues/1221>`__)
* rosbag2_cpp: test more than one storage plugin (`#1196 <https://github.com/ros2/rosbag2/issues/1196>`__)
* Replace language for "db3"/"db"/"database" (`#1194 <https://github.com/ros2/rosbag2/issues/1194>`__)
* set_read_order: return success (`#1177 <https://github.com/ros2/rosbag2/issues/1177>`__)
* Remove explicit sqlite3 from code (`#1166 <https://github.com/ros2/rosbag2/issues/1166>`__)
* Add ``update_metadata(BagMetadata)`` API for storage plugin interface (`#1149 <https://github.com/ros2/rosbag2/issues/1149>`__)
* Reader and writer can use default storage by not specifying (`#1167 <https://github.com/ros2/rosbag2/issues/1167>`__)
* rosbag2_storage: expose default storage ID as method (`#1146 <https://github.com/ros2/rosbag2/issues/1146>`__)
* Don't reopen file for every seek if we don't have to. Search directionally for the correct file (`#1117 <https://github.com/ros2/rosbag2/issues/1117>`__)
* Add SplitBagfile recording service. (`#1115 <https://github.com/ros2/rosbag2/issues/1115>`__)
* Reverse read order API and sqlite storage implementation (`#1083 <https://github.com/ros2/rosbag2/issues/1083>`__)
* Replace ``std::filesystem::path(..)`` with ``rcpputils::fs::path(..)`` (`#1104 <https://github.com/ros2/rosbag2/issues/1104>`__)
* Fix issue where sequentialwriter only sets metadata duration to the duration of the final file (`#1098 <https://github.com/ros2/rosbag2/issues/1098>`__)
* Delete obsolete compression_options.cpp from rosbag2_cpp (`#1078 <https://github.com/ros2/rosbag2/issues/1078>`__)
* Readers/info can accept a single bag storage file, and detect its storage id automatically (`#1072 <https://github.com/ros2/rosbag2/issues/1072>`__)
* Remove deprecated rosbag2_cpp/storage_options.hpp, for post-Humble releases (`#1064 <https://github.com/ros2/rosbag2/issues/1064>`__)
* Speed optimization: Preparing copyless publish/subscribing by using const message for writing (`#1010 <https://github.com/ros2/rosbag2/issues/1010>`__)
* Add the ability to record any key/value pair in 'custom' field in metadata.yaml (`#1038 <https://github.com/ros2/rosbag2/issues/1038>`__)
* Notification of significant events during bag recording and playback (`#908 <https://github.com/ros2/rosbag2/issues/908>`__)
* Bugfix for "Playing the bags recorded with split by duration/size is playing only the last recorded .db3." (`#1022 <https://github.com/ros2/rosbag2/issues/1022>`__)
* Improve test_time_controller test (`#1012 <https://github.com/ros2/rosbag2/issues/1012>`__)
* Contributors: Chris Lalancette, Daisuke Nishimatsu, DensoADAS, Emerson Knapp, Geoffrey Biggs, Hunter L. Allen, Jorge Perez, Joshua Hampp, Kaju-Bubanja, Michael Orlov, Tony Peng, james-rms, mergify[bot], rshanor


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_examples_cpp <https://github.com/ros2/rosbag2/tree/iron/rosbag2_examples/rosbag2_examples_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* rosbag2_storage: add type description hash to topic metadata (`#1272 <https://github.com/ros2/rosbag2/issues/1272>`__)
* Update rosbag2 to C++17. (`#1238 <https://github.com/ros2/rosbag2/issues/1238>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#1202 <https://github.com/ros2/rosbag2/issues/1202>`__)
* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`__)
* Add API samples on main branch - Rolling C++ API examples (`#1068 <https://github.com/ros2/rosbag2/issues/1068>`__)
* Contributors: Chris Lalancette, Daisuke Nishimatsu, Emerson Knapp, Michael Orlov, james-rms


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_examples_py <https://github.com/ros2/rosbag2/tree/iron/rosbag2_examples/rosbag2_examples_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix a warning from python setuptools. (`#1312 <https://github.com/ros2/rosbag2/issues/1312>`__) (`#1314 <https://github.com/ros2/rosbag2/issues/1314>`__)
* Add API samples for Python [rebased] (`#1253 <https://github.com/ros2/rosbag2/issues/1253>`__) * Add API samples for Python * Package Renaming and Move * linting + copyright * more linting --------- Co-authored-by: Geoffrey Biggs <gbiggs@killbots.net>
* Contributors: David V. Lu!!, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_interfaces <https://github.com/ros2/rosbag2/tree/iron/rosbag2_interfaces/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update rosbag2 to C++17. (`#1238 <https://github.com/ros2/rosbag2/issues/1238>`__)
* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`__)
* Add SplitBagfile recording service. (`#1115 <https://github.com/ros2/rosbag2/issues/1115>`__)
* Adds stop operation for rosbag2::Player (`#1007 <https://github.com/ros2/rosbag2/issues/1007>`__)
* Notification of significant events during bag recording and playback (`#908 <https://github.com/ros2/rosbag2/issues/908>`__)
* Adds play until timestamp functionality (`#1005 <https://github.com/ros2/rosbag2/issues/1005>`__)
* Add CLI verb for burst mode of playback (`#980 <https://github.com/ros2/rosbag2/issues/980>`__)
* Add play-for specified number of seconds functionality (`#960 <https://github.com/ros2/rosbag2/issues/960>`__)
* Contributors: Agustin Alba Chicar, Chris Lalancette, Geoffrey Biggs, Michael Orlov, Misha Shalem, rshanor


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_performance_benchmarking <https://github.com/ros2/rosbag2/tree/iron/rosbag2_performance/rosbag2_performance_benchmarking/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add tests for rosbag2_performance_benchmarking pkg (`#1268 <https://github.com/ros2/rosbag2/issues/1268>`__)
* Fix expectations for rosbag2 return code in rosbag2_performance_benchmarking (`#1267 <https://github.com/ros2/rosbag2/issues/1267>`__)
* Update rosbag2 to C++17. (`#1238 <https://github.com/ros2/rosbag2/issues/1238>`__)
* Use thread pool to run benchmark publishers in rosbag2_performance_benchmarking (`#1250 <https://github.com/ros2/rosbag2/issues/1250>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#1202 <https://github.com/ros2/rosbag2/issues/1202>`__)
* Skip ament_package() call when not building rosbag2_performance_benchmarking (`#1242 <https://github.com/ros2/rosbag2/issues/1242>`__)
* Add option to specify a message type (`#1153 <https://github.com/ros2/rosbag2/issues/1153>`__)
* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`__)
* Replace language for "db3"/"db"/"database" (`#1194 <https://github.com/ros2/rosbag2/issues/1194>`__)
* Remove explicit sqlite3 from code (`#1166 <https://github.com/ros2/rosbag2/issues/1166>`__)
* Contributors: Chris Lalancette, Daisuke Nishimatsu, Emerson Knapp, Michael Orlov, Shane Loretz, carlossvg


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_performance_benchmarking_msgs <https://github.com/ros2/rosbag2/tree/iron/rosbag2_performance/rosbag2_performance_benchmarking_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add tests for rosbag2_performance_benchmarking pkg (`#1268 <https://github.com/ros2/rosbag2/issues/1268>`__)
* Skip ament_package() call when not building rosbag2_performance_benchmarking (`#1242 <https://github.com/ros2/rosbag2/issues/1242>`__)
* [rolling] Bump to 0.19.0 (`#1232 <https://github.com/ros2/rosbag2/issues/1232>`__)
* Add option to specify a message type (`#1153 <https://github.com/ros2/rosbag2/issues/1153>`__)
* Contributors: Audrow Nash, Michael Orlov, Shane Loretz, carlossvg


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_py <https://github.com/ros2/rosbag2/tree/iron/rosbag2_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add binding to close the writer (`#1339 <https://github.com/ros2/rosbag2/issues/1339>`__) (`#1340 <https://github.com/ros2/rosbag2/issues/1340>`__)
* Add type_hash in MessageDefinition struct (`#1296 <https://github.com/ros2/rosbag2/issues/1296>`__)
* Store message definitions in SQLite3 storage plugin (`#1293 <https://github.com/ros2/rosbag2/issues/1293>`__)
* Add message definition read API (`#1292 <https://github.com/ros2/rosbag2/issues/1292>`__)
* rosbag2_storage: add type description hash to topic metadata (`#1272 <https://github.com/ros2/rosbag2/issues/1272>`__)
* rosbag2_cpp: move local message definition source out of MCAP plugin (`#1265 <https://github.com/ros2/rosbag2/issues/1265>`__)
* Update rosbag2 to C++17. (`#1238 <https://github.com/ros2/rosbag2/issues/1238>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#1202 <https://github.com/ros2/rosbag2/issues/1202>`__)
* Expose more Writer methods in python interface (`#1220 <https://github.com/ros2/rosbag2/issues/1220>`__)
* rosbag2_storage: set MCAP as default plugin (`#1160 <https://github.com/ros2/rosbag2/issues/1160>`__)
* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`__)
* rosbag2_py: parametrize tests across storage plugins (`#1203 <https://github.com/ros2/rosbag2/issues/1203>`__)
* Added option to change node name for the recorder from the Python API (`#1180 <https://github.com/ros2/rosbag2/issues/1180>`__)
* Replace language for "db3"/"db"/"database" (`#1194 <https://github.com/ros2/rosbag2/issues/1194>`__)
* Remove explicit sqlite3 from code (`#1166 <https://github.com/ros2/rosbag2/issues/1166>`__)
* Move python get_default_storage_id to storage module instead of writer (`#1165 <https://github.com/ros2/rosbag2/issues/1165>`__)
* rosbag2_storage: expose default storage ID as method (`#1146 <https://github.com/ros2/rosbag2/issues/1146>`__)
* rosbag2_py: set defaults for config when bag rewriting (`#1121 <https://github.com/ros2/rosbag2/issues/1121>`__)
* Reverse read order API and sqlite storage implementation (`#1083 <https://github.com/ros2/rosbag2/issues/1083>`__)
* expose py Reader metadata, improve ``rosbag2_py.BagMetadata`` usability (`#1082 <https://github.com/ros2/rosbag2/issues/1082>`__)
* Added support for excluding topics via regular expressions (`#1046 <https://github.com/ros2/rosbag2/issues/1046>`__)
* Use a single variable for evaluating the filter regex (`#1053 <https://github.com/ros2/rosbag2/issues/1053>`__)
* Add additional mode of publishing sim time updates triggered by replayed messages (`#1050 <https://github.com/ros2/rosbag2/issues/1050>`__)
* Renamed --topics-regex to --regex and -e in Player class to be consistent with Recorder (`#1045 <https://github.com/ros2/rosbag2/issues/1045>`__)
* Add the ability to record any key/value pair in 'custom' field in metadata.yaml (`#1038 <https://github.com/ros2/rosbag2/issues/1038>`__)
* Added support for filtering topics via regular expressions on Playback (`#1034 <https://github.com/ros2/rosbag2/issues/1034>`__)
* Adds play until timestamp functionality (`#1005 <https://github.com/ros2/rosbag2/issues/1005>`__)
* Add CLI verb for burst mode of playback (`#980 <https://github.com/ros2/rosbag2/issues/980>`__)
* Add play-for specified number of seconds functionality (`#960 <https://github.com/ros2/rosbag2/issues/960>`__)
* Make unpublished topics unrecorded by default (`#968 <https://github.com/ros2/rosbag2/issues/968>`__)
* Fix test rosbag2_py test compatibility with Python < 3.8 (`#987 <https://github.com/ros2/rosbag2/issues/987>`__)
* Contributors: Agustin Alba Chicar, Chris Lalancette, Daisuke Nishimatsu, Emerson Knapp, Esteve Fernandez, Geoffrey Biggs, Hunter L. Allen, Michael Orlov, Scott K Logan, Sean Kelly, Tony Peng, james-rms, kylemarcey, mergify[bot], ricardo-manriquez


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_storage <https://github.com/ros2/rosbag2/tree/iron/rosbag2_storage/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add type_hash in MessageDefinition struct (`#1296 <https://github.com/ros2/rosbag2/issues/1296>`__)
* Add message definition read API (`#1292 <https://github.com/ros2/rosbag2/issues/1292>`__)
* rosbag2_storage: add type description hash to topic metadata (`#1272 <https://github.com/ros2/rosbag2/issues/1272>`__)
* rosbag2_cpp: move local message definition source out of MCAP plugin (`#1265 <https://github.com/ros2/rosbag2/issues/1265>`__)
* Update rosbag2 to C++17. (`#1238 <https://github.com/ros2/rosbag2/issues/1238>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#1202 <https://github.com/ros2/rosbag2/issues/1202>`__)
* rosbag2_storage: set MCAP as default plugin (`#1160 <https://github.com/ros2/rosbag2/issues/1160>`__)
* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`__)
* set_read_order: return success (`#1177 <https://github.com/ros2/rosbag2/issues/1177>`__)
* Remove explicit sqlite3 from code (`#1166 <https://github.com/ros2/rosbag2/issues/1166>`__)
* Add ``update_metadata(BagMetadata)`` API for storage plugin interface (`#1149 <https://github.com/ros2/rosbag2/issues/1149>`__)
* rosbag2_storage: expose default storage ID as method (`#1146 <https://github.com/ros2/rosbag2/issues/1146>`__)
* Don't reopen file for every seek if we don't have to. Search directionally for the correct file (`#1117 <https://github.com/ros2/rosbag2/issues/1117>`__)
* Reverse read order API and sqlite storage implementation (`#1083 <https://github.com/ros2/rosbag2/issues/1083>`__)
* Remove YAML_CPP_DLL define (`#964 <https://github.com/ros2/rosbag2/issues/964>`__)
* Added support for excluding topics via regular expressions (`#1046 <https://github.com/ros2/rosbag2/issues/1046>`__)
* Readers/info can accept a single bag storage file, and detect its storage id automatically (`#1072 <https://github.com/ros2/rosbag2/issues/1072>`__)
* Use a single variable for evaluating the filter regex (`#1053 <https://github.com/ros2/rosbag2/issues/1053>`__)
* Speed optimization: Preparing copyless publish/subscribing by using const message for writing (`#1010 <https://github.com/ros2/rosbag2/issues/1010>`__)
* Renamed --topics-regex to --regex and -e in Player class to be consistent with Recorder (`#1045 <https://github.com/ros2/rosbag2/issues/1045>`__)
* Add the ability to record any key/value pair in 'custom' field in metadata.yaml (`#1038 <https://github.com/ros2/rosbag2/issues/1038>`__)
* Added support for filtering topics via regular expressions on Playback (`#1034 <https://github.com/ros2/rosbag2/issues/1034>`__)
* Contributors: Akash, Chris Lalancette, Daisuke Nishimatsu, DensoADAS, Emerson Knapp, Esteve Fernandez, Hunter L. Allen, Joshua Hampp, Michael Orlov, Tony Peng, james-rms


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_storage_default_plugins <https://github.com/ros2/rosbag2/tree/iron/rosbag2_storage_default_plugins/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* rosbag2_storage: set MCAP as default plugin (`#1160 <https://github.com/ros2/rosbag2/issues/1160>`__)
* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`__)
* Move sqlite3 storage implementation to rosbag2_storage_sqlite3 package (`#1113 <https://github.com/ros2/rosbag2/issues/1113>`__)
* Reverse read order API and sqlite storage implementation (`#1083 <https://github.com/ros2/rosbag2/issues/1083>`__)
* Add support for old db3 schema used on distros prior to Foxy (`#1090 <https://github.com/ros2/rosbag2/issues/1090>`__)
* Added support for excluding topics via regular expressions (`#1046 <https://github.com/ros2/rosbag2/issues/1046>`__)
* Contributors: Emerson Knapp, Esteve Fernandez, Michael Orlov, james-rms


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_storage_mcap <https://github.com/ros2/rosbag2/tree/iron/rosbag2_storage_mcap/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add type_hash in MessageDefinition struct (`#1296 <https://github.com/ros2/rosbag2/issues/1296>`__)
* Add message definition read API (`#1292 <https://github.com/ros2/rosbag2/issues/1292>`__)
* rosbag2_storage: add type description hash to topic metadata (`#1272 <https://github.com/ros2/rosbag2/issues/1272>`__)
* rosbag2_cpp: move local message definition source out of MCAP plugin (`#1265 <https://github.com/ros2/rosbag2/issues/1265>`__)
* Update rosbag2 to C++17. (`#1238 <https://github.com/ros2/rosbag2/issues/1238>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#1202 <https://github.com/ros2/rosbag2/issues/1202>`__)
* CLI: Get storage-specific values from plugin (`#1209 <https://github.com/ros2/rosbag2/issues/1209>`__)
* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`__)
* rosbag2_cpp: test more than one storage plugin (`#1196 <https://github.com/ros2/rosbag2/issues/1196>`__)
* set_read_order: return success (`#1177 <https://github.com/ros2/rosbag2/issues/1177>`__)
* rosbag2_storage_mcap: merge into rosbag2 repo (`#1163 <https://github.com/ros2/rosbag2/issues/1163>`__)
* mcap_storage: 'none' is a valid storage preset profile (`#86 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/86>`__)
* mcap_storage: handle update_metadata call (`#83 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/83>`__)
* Update clang-format rules to fit ROS 2 style guide (`#80 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/80>`__)
* Revert "read_order: throw exception from set_read_order for unsupported orders"
* read_order: throw exception from set_read_order for unsupported orders
* Fix compile flags to work on rosbag_storage:0.17.x (`#78 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/78>`__)
* Fix Windows build (`#73 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/73>`__)
* set defaults for SQLite plugin parity (`#68 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/68>`__)
* rosbag2_storage_mcap: add storage preset profiles (`#57 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/57>`__)
* rename test_fixture_interfaces package to testdata (`#64 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/64>`__)
* Switch to using the vendored zstd library. (`#59 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/59>`__)
* Add set_read_order reader API (`#54 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/54>`__)
* Some minor improvements in rosbag2_storage_mcap after review (`#58 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/58>`__)
* Revert "rosbag2_storage_mcap: add storage preset profiles"
* rosbag2_storage_mcap: add storage preset profiles
* Store IDL message definitions in Schema records when no MSG definition is available (`#43 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/43>`__)
* Support timestamp-ordered playback (`#50 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/50>`__)
* Support regex topic filtering
* Add all lz4 sources to fix undefined symbols at runtime (`#46 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/46>`__)
* Upgrade mcap to fix LZ4 error and segfault (`#42 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/42>`__)
* Fix build for Foxy (`#34 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/34>`__)
* fix: minor issues (`#31 <https://github.com/wep21/rosbag2_storage_mcap/issues/31>`__) * remove unnecessary block * use target_link_libraries instead of ament_target_dependencies * remove ros environment * add prefix to compile definition
* Update email address for Foxglove maintainers (`#32 <https://github.com/wep21/rosbag2_storage_mcap/issues/32>`__)
* Added mcap_vendor package. Updated CMakeLists.txt to fetch dependencies with FetchContent rather than Conan.
* CMake build script will now execute pip install conan automatically.
* [1.0.0] Use Summary section for get_metadata() and seek(), implement remaining methods (`#17 <https://github.com/wep21/rosbag2_storage_mcap/issues/17>`__)
* feat: add play impl (`#16 <https://github.com/wep21/rosbag2_storage_mcap/issues/16>`__)
* chore: refine package.xml (`#15 <https://github.com/wep21/rosbag2_storage_mcap/issues/15>`__)
* Don't throw when READ_WRITE mode is used; add .mcap file extension to recorded files (`#14 <https://github.com/wep21/rosbag2_storage_mcap/issues/14>`__)
* Add dynamic message definition lookup (`#13 <https://github.com/wep21/rosbag2_storage_mcap/issues/13>`__)
* Switch C++ formatter to clang-format (`#12 <https://github.com/wep21/rosbag2_storage_mcap/issues/12>`__)
* Merge pull request `#7 <https://github.com/wep21/rosbag2_storage_mcap/issues/7>`__ from ros-tooling/jhurliman/reader-writer
* uninitialized struct
* lint
* lint
* lint
* Reader and writer implementation
* Merge pull request `#6 <https://github.com/wep21/rosbag2_storage_mcap/issues/6>`__ from wep21/add-metadata-impl
* feat: add metadata impl
* Merge pull request `#5 <https://github.com/wep21/rosbag2_storage_mcap/issues/5>`__ from wep21/mcap-storage-impl
* chore: update cmake minimum version
* chore: install mcap header
* chore: include mcap header
* fix: move fetch content into rosbag2 storage mcap
* Merge pull request `#3 <https://github.com/wep21/rosbag2_storage_mcap/issues/3>`__ from ros-tooling/emersonknapp/mcap_plugin_skeleton
* Add rosbag2_storage_mcap skeleton
* Contributors: Andrew Symington, Chris Lalancette, Daisuke Nishimatsu, Emerson Knapp, Jacob Bandes-Storch, James Smith, John Hurliman, Michael Orlov, james-rms, wep21


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_storage_sqlite3 <https://github.com/ros2/rosbag2/tree/iron/rosbag2_storage_sqlite3/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add type_hash in MessageDefinition struct (`#1296 <https://github.com/ros2/rosbag2/issues/1296>`__)
* Store message definitions in SQLite3 storage plugin (`#1293 <https://github.com/ros2/rosbag2/issues/1293>`__)
* Add message definition read API (`#1292 <https://github.com/ros2/rosbag2/issues/1292>`__)
* rosbag2_storage: add type description hash to topic metadata (`#1272 <https://github.com/ros2/rosbag2/issues/1272>`__)
* rosbag2_cpp: move local message definition source out of MCAP plugin (`#1265 <https://github.com/ros2/rosbag2/issues/1265>`__)
* Update rosbag2 to C++17. (`#1238 <https://github.com/ros2/rosbag2/issues/1238>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#1202 <https://github.com/ros2/rosbag2/issues/1202>`__)
* CLI: Get storage-specific values from plugin (`#1209 <https://github.com/ros2/rosbag2/issues/1209>`__)
* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`__)
* Remove sqlite3-specific info from main README, make it more storage agnostic and point to plugin-specific README (`#1193 <https://github.com/ros2/rosbag2/issues/1193>`__)
* set_read_order: return success (`#1177 <https://github.com/ros2/rosbag2/issues/1177>`__)
* Add ``update_metadata(BagMetadata)`` API for storage plugin interface (`#1149 <https://github.com/ros2/rosbag2/issues/1149>`__)
* Store db schema version and ROS_DISTRO name in db3 files (`#1156 <https://github.com/ros2/rosbag2/issues/1156>`__)
* ros2bag: move storage preset validation to sqlite3 plugin (`#1135 <https://github.com/ros2/rosbag2/issues/1135>`__)
* Move sqlite3 storage implementation to rosbag2_storage_sqlite3 package (`#1113 <https://github.com/ros2/rosbag2/issues/1113>`__)
* Use a single variable for evaluating the filter regex (`#1053 <https://github.com/ros2/rosbag2/issues/1053>`__)
* Renamed --topics-regex to --regex and -e in Player class to be consistent with Recorder (`#1045 <https://github.com/ros2/rosbag2/issues/1045>`__)
* Added support for filtering topics via regular expressions on Playback (`#1034 <https://github.com/ros2/rosbag2/issues/1034>`__)
* Contributors: Chris Lalancette, Daisuke Nishimatsu, Emerson Knapp, Esteve Fernandez, Michael Orlov, james-rms


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_test_common <https://github.com/ros2/rosbag2/tree/iron/rosbag2_test_common/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Address flakiness in rosbag2_play_end_to_end tests (`#1297 <https://github.com/ros2/rosbag2/issues/1297>`__) (`#1330 <https://github.com/ros2/rosbag2/issues/1330>`__)
* Update rosbag2 to C++17. (`#1238 <https://github.com/ros2/rosbag2/issues/1238>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#1202 <https://github.com/ros2/rosbag2/issues/1202>`__)
* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`__)
* rosbag2_py: parametrize tests across storage plugins (`#1203 <https://github.com/ros2/rosbag2/issues/1203>`__)
* Fix for ros2 bag play exit with non-zero code on SIGINT (`#1126 <https://github.com/ros2/rosbag2/issues/1126>`__)
* Split up the include of rclcpp.hpp (`#1027 <https://github.com/ros2/rosbag2/issues/1027>`__)
* Contributors: Chris Lalancette, Daisuke Nishimatsu, Michael Orlov, james-rms, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_test_msgdefs <https://github.com/ros2/rosbag2/tree/iron/rosbag2_test_msgdefs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* rosbag2_cpp: move local message definition source out of MCAP plugin (`#1265 <https://github.com/ros2/rosbag2/issues/1265>`__) The intention of this PR is to move the message-definition-finding capability outside of rosbag2_storage_mcap, and allow any rosbag2 storage plugin to store message definitions.
* Contributors: james-rms


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_tests <https://github.com/ros2/rosbag2/tree/iron/rosbag2_tests/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Address flakiness in rosbag2_play_end_to_end tests (`#1297 <https://github.com/ros2/rosbag2/issues/1297>`__) (`#1330 <https://github.com/ros2/rosbag2/issues/1330>`__)
* Add type_hash in MessageDefinition struct (`#1296 <https://github.com/ros2/rosbag2/issues/1296>`__)
* rosbag2_cpp: move local message definition source out of MCAP plugin (`#1265 <https://github.com/ros2/rosbag2/issues/1265>`__)
* Update rosbag2 to C++17. (`#1238 <https://github.com/ros2/rosbag2/issues/1238>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#1202 <https://github.com/ros2/rosbag2/issues/1202>`__)
* rosbag2_storage: set MCAP as default plugin (`#1160 <https://github.com/ros2/rosbag2/issues/1160>`__)
* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`__)
* Parametrize all rosbag2_tests for both supported storage plugins (`#1221 <https://github.com/ros2/rosbag2/issues/1221>`__)
* Make rosbag2_tests agnostic to storage implementation (`#1192 <https://github.com/ros2/rosbag2/issues/1192>`__)
* Get rid from attempt to open DB file in ``wait_for_db()`` test fixture (`#1141 <https://github.com/ros2/rosbag2/issues/1141>`__)
* Fix for ros2 bag play exit with non-zero code on SIGINT (`#1126 <https://github.com/ros2/rosbag2/issues/1126>`__)
* Move sqlite3 storage implementation to rosbag2_storage_sqlite3 package (`#1113 <https://github.com/ros2/rosbag2/issues/1113>`__)
* Readers/info can accept a single bag storage file, and detect its storage id automatically (`#1072 <https://github.com/ros2/rosbag2/issues/1072>`__)
* Add the ability to record any key/value pair in 'custom' field in metadata.yaml (`#1038 <https://github.com/ros2/rosbag2/issues/1038>`__)
* Contributors: Chris Lalancette, Daisuke Nishimatsu, Emerson Knapp, Hunter L. Allen, Michael Orlov, Tony Peng, james-rms, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosbag2_transport <https://github.com/ros2/rosbag2/tree/iron/rosbag2_transport/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Change subscriptions from GenericSubscripton to SubscriptionBase (`#1338 <https://github.com/ros2/rosbag2/issues/1338>`__)
* Add recorder stop() API (`#1300 <https://github.com/ros2/rosbag2/issues/1300>`__) (`#1334 <https://github.com/ros2/rosbag2/issues/1334>`__)
* Read message definitions from input files in bag_rewrite (`#1295 <https://github.com/ros2/rosbag2/issues/1295>`__)
* Add message definition read API (`#1292 <https://github.com/ros2/rosbag2/issues/1292>`__)
* Move rosbag2_transport::Recorder implementation to pimpl (`#1291 <https://github.com/ros2/rosbag2/issues/1291>`__)
* rosbag2_storage: add type description hash to topic metadata (`#1272 <https://github.com/ros2/rosbag2/issues/1272>`__)
* rosbag2_cpp: move local message definition source out of MCAP plugin (`#1265 <https://github.com/ros2/rosbag2/issues/1265>`__)
* Use RMW methods to initialize endpoint info instead of brace initializer to guard against upcoming struct change (`#1257 <https://github.com/ros2/rosbag2/issues/1257>`__)
* Update rosbag2 to C++17. (`#1238 <https://github.com/ros2/rosbag2/issues/1238>`__)
* Use target_link_libraries instead of ament_target_dependencies (`#1202 <https://github.com/ros2/rosbag2/issues/1202>`__)
* Print "Hidden topics are not recorded" only once. (`#1225 <https://github.com/ros2/rosbag2/issues/1225>`__)
* rosbag2_storage: set MCAP as default plugin (`#1160 <https://github.com/ros2/rosbag2/issues/1160>`__)
* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`__)
* rosbag2_transport: parametrize test_rewrite (`#1206 <https://github.com/ros2/rosbag2/issues/1206>`__)
* rosbag2_cpp: test more than one storage plugin (`#1196 <https://github.com/ros2/rosbag2/issues/1196>`__)
* Replace language for "db3"/"db"/"database" (`#1194 <https://github.com/ros2/rosbag2/issues/1194>`__)
* set_read_order: return success (`#1177 <https://github.com/ros2/rosbag2/issues/1177>`__)
* Remove explicit sqlite3 from code (`#1166 <https://github.com/ros2/rosbag2/issues/1166>`__)
* Add pause and resume service calls for rosbag2 recorder (`#1131 <https://github.com/ros2/rosbag2/issues/1131>`__)
* Redesign record_services tests to make them more deterministic (`#1122 <https://github.com/ros2/rosbag2/issues/1122>`__)
* Add SplitBagfile recording service. (`#1115 <https://github.com/ros2/rosbag2/issues/1115>`__)
* Reverse read order API and sqlite storage implementation (`#1083 <https://github.com/ros2/rosbag2/issues/1083>`__)
* make recorder node composable by inheritance (`#1093 <https://github.com/ros2/rosbag2/issues/1093>`__)
* Mark ``test_play_services`` as xfail for FastRTPS and CycloneDDS (`#1091 <https://github.com/ros2/rosbag2/issues/1091>`__)
* fixed typo (`#1057 <https://github.com/ros2/rosbag2/issues/1057>`__)
* Fix hangout in rosbag2 player and recorder when pressing ``CTRL+C`` (`#1081 <https://github.com/ros2/rosbag2/issues/1081>`__)
* Added support for excluding topics via regular expressions (`#1046 <https://github.com/ros2/rosbag2/issues/1046>`__)
* Use a single variable for evaluating the filter regex (`#1053 <https://github.com/ros2/rosbag2/issues/1053>`__)
* Add additional mode of publishing sim time updates triggered by replayed messages (`#1050 <https://github.com/ros2/rosbag2/issues/1050>`__)
* Speed optimization: Preparing copyless publish/subscribing by using const message for writing (`#1010 <https://github.com/ros2/rosbag2/issues/1010>`__)
* Renamed --topics-regex to --regex and -e in Player class to be consistent with Recorder (`#1045 <https://github.com/ros2/rosbag2/issues/1045>`__)
* Refactor play until and duration tests (`#1024 <https://github.com/ros2/rosbag2/issues/1024>`__)
* Added support for filtering topics via regular expressions on Playback (`#1034 <https://github.com/ros2/rosbag2/issues/1034>`__)
* Adds stop operation for rosbag2::Player (`#1007 <https://github.com/ros2/rosbag2/issues/1007>`__)
* Fix incorrect boundary check for ``playback_duration`` and ``play_until_timestamp`` (`#1032 <https://github.com/ros2/rosbag2/issues/1032>`__)
* Split up the include of rclcpp.hpp (`#1027 <https://github.com/ros2/rosbag2/issues/1027>`__)
* Notification of significant events during bag recording and playback (`#908 <https://github.com/ros2/rosbag2/issues/908>`__)
* Adds play until timestamp functionality (`#1005 <https://github.com/ros2/rosbag2/issues/1005>`__)
* Add CLI verb for burst mode of playback (`#980 <https://github.com/ros2/rosbag2/issues/980>`__)
* Add on play message callbacks to the ``rosbag2::Player`` class (`#1004 <https://github.com/ros2/rosbag2/issues/1004>`__)
* Add play-for specified number of seconds functionality (`#960 <https://github.com/ros2/rosbag2/issues/960>`__)
* Reduce message spam when topics to be recorded do not exist (`#1018 <https://github.com/ros2/rosbag2/issues/1018>`__)
* Address flakiness in record_all_with_sim_time test (`#1014 <https://github.com/ros2/rosbag2/issues/1014>`__)
* Add debug instrumentation for ``test_play_services`` (`#1013 <https://github.com/ros2/rosbag2/issues/1013>`__)
* Fix for rosbag2::Player freeze when pressing ctrl+c in pause mode (`#1002 <https://github.com/ros2/rosbag2/issues/1002>`__)
* Add the /bigobj flag to Windows Debug builds. (`#1009 <https://github.com/ros2/rosbag2/issues/1009>`__)
* Make unpublished topics unrecorded by default (`#968 <https://github.com/ros2/rosbag2/issues/968>`__)
* Make peek_next_message_from_queue return a SharedPtr. (`#993 <https://github.com/ros2/rosbag2/issues/993>`__)
* Change the topic names in test_record.cpp (`#988 <https://github.com/ros2/rosbag2/issues/988>`__)
* Contributors: Agustin Alba Chicar, Bernardo Taveira, Brian, Chris Lalancette, Cristóbal Arroyo, Daisuke Nishimatsu, DensoADAS, Emerson Knapp, Esteve Fernandez, Geoffrey Biggs, Jorge Perez, Joshua Hampp, Michael Orlov, Misha Shalem, Sean Kelly, Tony Peng, james-rms, kylemarcey, mergify[bot], rshanor


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosgraph_msgs <https://github.com/ros2/rcl_interfaces/tree/iron/rosgraph_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update common_interfaces to C++17. (`#215 <https://github.com/ros2/rcl_interfaces/issues/215>`__) (`#151 <https://github.com/ros2/rcl_interfaces/issues/151>`__)
* [rolling] Update maintainers - 2022-11-07 (`#150 <https://github.com/ros2/rcl_interfaces/issues/150>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_adapter <https://github.com/ros2/rosidl/tree/iron/rosidl_adapter/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* rosidl_adapter/cmake/rosidl_adapt_interfaces.cmake: Make ament free (`#709 <https://github.com/ros2/rosidl/issues/709>`__)
* [service introspection] generate service_event messages (`#700 <https://github.com/ros2/rosidl/issues/700>`__)
* Adding tests for unicode support in message comments. (`#720 <https://github.com/ros2/rosidl/issues/720>`__)
* [rolling] Update maintainers - 2022-11-07 (`#717 <https://github.com/ros2/rosidl/issues/717>`__)
* Add action2idl script (`#654 <https://github.com/ros2/rosidl/issues/654>`__)
* Contributors: Audrow Nash, Brian, Guilherme Henrique Galelli Christmann, John Daktylidis, Yasushi SHOJI


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_cli <https://github.com/ros2/rosidl/tree/iron/rosidl_cli/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix warnings (`#726 <https://github.com/ros2/rosidl/issues/726>`__)
* [rolling] Update maintainers - 2022-11-07 (`#717 <https://github.com/ros2/rosidl/issues/717>`__)
* Contributors: Audrow Nash, Yadu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_cmake <https://github.com/ros2/rosidl/tree/iron/rosidl_cmake/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Type Description Codegen and Typesupport  (rep2011) (`#727 <https://github.com/ros2/rosidl/issues/727>`__)
* Type hash in interface codegen (rep2011) (`#722 <https://github.com/ros2/rosidl/issues/722>`__)
* [service introspection] generate service_event messages (`#700 <https://github.com/ros2/rosidl/issues/700>`__)
* [rolling] Update maintainers - 2022-11-07 (`#717 <https://github.com/ros2/rosidl/issues/717>`__)
* Skip rosidl_generate_interfaces dependency export on SKIP_INSTALL. (`#708 <https://github.com/ros2/rosidl/issues/708>`__)
* Move rosidl_cmake Python module to a new package rosidl_pycommon (`#696 <https://github.com/ros2/rosidl/issues/696>`__) Deprecate the Python module in rosidl_cmake and move the implementation to the new package rosidl_pycommon.
* Fix comment in camel case conversion function (`#683 <https://github.com/ros2/rosidl/issues/683>`__)
* Protect rosidl_target_interfaces from using NOTFOUND in include_directories (`#679 <https://github.com/ros2/rosidl/issues/679>`__)
* Contributors: Audrow Nash, Brian, Chris Lalancette, Emerson Knapp, Jacob Perron, Jose Luis Rivero, Shane Loretz


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_core_generators <https://github.com/ros2/rosidl_core/tree/iron/rosidl_core_generators/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#2 <https://github.com/ros2/rosidl_core/issues/2>`__)
* Add generators and runtime configuration packages (`#1 <https://github.com/ros2/rosidl_core/issues/1>`__) Moved (and renamed) from rosidl_defaults. Related PR: https://github.com/ros2/rosidl_defaults/pull/22
* Contributors: Audrow Nash, Jacob Perron


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_core_runtime <https://github.com/ros2/rosidl_core/tree/iron/rosidl_core_runtime/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#2 <https://github.com/ros2/rosidl_core/issues/2>`__)
* Add generators and runtime configuration packages (`#1 <https://github.com/ros2/rosidl_core/issues/1>`__) Moved (and renamed) from rosidl_defaults. Related PR: https://github.com/ros2/rosidl_defaults/pull/22
* Contributors: Audrow Nash, Jacob Perron


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_default_generators <https://github.com/ros2/rosidl_defaults/tree/iron/rosidl_default_generators/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* add service_msgs depend (`#24 <https://github.com/ros2/rosidl_defaults/issues/24>`__)
* [rolling] Update maintainers - 2022-11-07 (`#25 <https://github.com/ros2/rosidl_defaults/issues/25>`__)
* Move dependencies to rosidl_core and depend on action_msgs (`#22 <https://github.com/ros2/rosidl_defaults/issues/22>`__) Move implementation to new packages rosidl_core_generators and rosidl_runtime_generators The new packages are located in a separate repository: https://github.com/ros2/rosidl_core.git rosidl_defaults now depends on the new packages, plus message definitions required for Actions (namely action_msgs). This allows users to avoid having to explictly depend on action_msgs.
* Contributors: Audrow Nash, Brian, Jacob Perron


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_default_runtime <https://github.com/ros2/rosidl_defaults/tree/iron/rosidl_default_runtime/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* add service_msgs depend (`#24 <https://github.com/ros2/rosidl_defaults/issues/24>`__)
* [rolling] Update maintainers - 2022-11-07 (`#25 <https://github.com/ros2/rosidl_defaults/issues/25>`__)
* Move dependencies to rosidl_core and depend on action_msgs (`#22 <https://github.com/ros2/rosidl_defaults/issues/22>`__) Move implementation to new packages rosidl_core_generators and rosidl_runtime_generators The new packages are located in a separate repository: https://github.com/ros2/rosidl_core.git rosidl_defaults now depends on the new packages, plus message definitions required for Actions (namely action_msgs). This allows users to avoid having to explictly depend on action_msgs.
* Contributors: Audrow Nash, Brian, Jacob Perron


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_dynamic_typesupport <https://github.com/ros2/rosidl_dynamic_typesupport/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix up the exports for rosidl_dynamic_typesupport. (`#5 <https://github.com/ros2/rosidl_dynamic_typesupport/issues/5>`__)
* Refactor dynamic message type support impl to use allocators (`#2 <https://github.com/ros2/rosidl_dynamic_typesupport/issues/2>`__)
* Runtime Interface Reflection: rosidl_dynamic_typesupport (`#1 <https://github.com/ros2/rosidl_dynamic_typesupport/issues/1>`__)
* Contributors: Chris Lalancette, William Woodall, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_dynamic_typesupport_fastrtps <https://github.com/ros2/rosidl_dynamic_typesupport_fastrtps/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Remove more unnecessary semicolons (`#4 <https://github.com/ros2/rosidl_dynamic_typesupport_fastrtps/issues/4>`__)
* Dynamic Subscription (BONUS: Allocators): rosidl_dynamic_typesupport_fastrtps (`#3 <https://github.com/ros2/rosidl_dynamic_typesupport_fastrtps/issues/3>`__)
* Remove unnecessary semicolons. (`#2 <https://github.com/ros2/rosidl_dynamic_typesupport_fastrtps/issues/2>`__)
* Runtime Interface Reflection: rosidl_dynamic_typesupport_fastrtps (`#1 <https://github.com/ros2/rosidl_dynamic_typesupport_fastrtps/issues/1>`__)
* Contributors: Chris Lalancette, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_generator_c <https://github.com/ros2/rosidl/tree/iron/rosidl_generator_c/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Type Description Codegen and Typesupport  (rep2011) (`#727 <https://github.com/ros2/rosidl/issues/727>`__)
* Expose type hash on typesupports (rep2011) (`#729 <https://github.com/ros2/rosidl/issues/729>`__)
* Type hash in interface codegen (rep2011) (`#722 <https://github.com/ros2/rosidl/issues/722>`__)
* [service introspection] generate service_event messages (`#700 <https://github.com/ros2/rosidl/issues/700>`__)
* [rolling] Update maintainers - 2022-11-07 (`#717 <https://github.com/ros2/rosidl/issues/717>`__)
* Move rosidl_generator_c/cpp tests to a separate package (`#701 <https://github.com/ros2/rosidl/issues/701>`__)
* Move rosidl_cmake Python module to a new package rosidl_pycommon (`#696 <https://github.com/ros2/rosidl/issues/696>`__) Deprecate the Python module in rosidl_cmake and move the implementation to the new package rosidl_pycommon.
* Add namespaced ALIAS target to easily consume generated libraries via add_subdirectory (`#605 <https://github.com/ros2/rosidl/issues/605>`__)
* Contributors: Audrow Nash, Brian, Emerson Knapp, Jacob Perron, Silvio Traversaro


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_generator_cpp <https://github.com/ros2/rosidl/tree/iron/rosidl_generator_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Type Description Codegen and Typesupport  (rep2011) (`#727 <https://github.com/ros2/rosidl/issues/727>`__)
* Expose type hash on typesupports (rep2011) (`#729 <https://github.com/ros2/rosidl/issues/729>`__)
* Type hash in interface codegen (rep2011) (`#722 <https://github.com/ros2/rosidl/issues/722>`__)
* [service introspection] generate service_event messages (`#700 <https://github.com/ros2/rosidl/issues/700>`__)
* [rolling] Update maintainers - 2022-11-07 (`#717 <https://github.com/ros2/rosidl/issues/717>`__)
* Move rosidl_generator_c/cpp tests to a separate package (`#701 <https://github.com/ros2/rosidl/issues/701>`__)
* Move rosidl_cmake Python module to a new package rosidl_pycommon (`#696 <https://github.com/ros2/rosidl/issues/696>`__) Deprecate the Python module in rosidl_cmake and move the implementation to the new package rosidl_pycommon.
* Add namespaced ALIAS target to easily consume generated libraries via add_subdirectory (`#605 <https://github.com/ros2/rosidl/issues/605>`__)
* Contributors: Audrow Nash, Brian, Emerson Knapp, Jacob Perron, Silvio Traversaro


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_generator_dds_idl <https://github.com/ros2/rosidl_dds/tree/iron/rosidl_generator_dds_idl/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#60 <https://github.com/ros2/rosidl_dds/issues/60>`__)
* Replace rosidl_cmake imports with rosidl_pycommon (`#59 <https://github.com/ros2/rosidl_dds/issues/59>`__)
* Contributors: Audrow Nash, Jacob Perron


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_generator_py <https://github.com/ros2/rosidl_python/tree/iron/rosidl_generator_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Hides the assertions that checks the data types of the message fields. (`#194 <https://github.com/ros2/rosidl_python/issues/194>`__)
* Service introspection (`#178 <https://github.com/ros2/rosidl_python/issues/178>`__)
* [rolling] Update maintainers - 2022-11-07 (`#189 <https://github.com/ros2/rosidl_python/issues/189>`__)
* Remove stray numpy import (`#185 <https://github.com/ros2/rosidl_python/issues/185>`__)
* :man_farmer: Fix NaN values bound numpy windows version (`#182 <https://github.com/ros2/rosidl_python/issues/182>`__)
* Allow NaN values to pass floating point bounds check. (`#167 <https://github.com/ros2/rosidl_python/issues/167>`__)
* Replace rosidl_cmake imports with rosidl_pycommon (`#177 <https://github.com/ros2/rosidl_python/issues/177>`__)
* Change decode error mode to replace (`#176 <https://github.com/ros2/rosidl_python/issues/176>`__)
* Merge pull request `#173 <https://github.com/ros2/rosidl_python/issues/173>`__ from ros2/quarkytale/fix_import_order
* fix flake
* sorting after conversion
* Revert "Use modern cmake targets to avoid absolute paths to appear in binary archives (`#160 <https://github.com/ros2/rosidl_python/issues/160>`__)" (`#166 <https://github.com/ros2/rosidl_python/issues/166>`__)
* Use modern cmake targets to avoid absolute paths to appear in binary archives (`#160 <https://github.com/ros2/rosidl_python/issues/160>`__)
* michel as author
* adding maintainer
* Contributors: Audrow Nash, Ben Wolsieffer, Brian, Cristóbal Arroyo, Dharini Dutia, Eloy Briceno, Ivan Santiago Paunovic, Jacob Perron, Tomoya Fujita, quarkytale, Øystein Sture


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_generator_tests <https://github.com/ros2/rosidl/tree/iron/rosidl_generator_tests/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Type Description Codegen and Typesupport  (rep2011) (`#727 <https://github.com/ros2/rosidl/issues/727>`__)
* Type hash in interface codegen (rep2011) (`#722 <https://github.com/ros2/rosidl/issues/722>`__)
* [service introspection] generate service_event messages (`#700 <https://github.com/ros2/rosidl/issues/700>`__) * add service event message
* [rolling] Update maintainers - 2022-11-07 (`#717 <https://github.com/ros2/rosidl/issues/717>`__)
* Move rosidl_generator_c/cpp tests to a separate package (`#701 <https://github.com/ros2/rosidl/issues/701>`__)
* Contributors: Audrow Nash, Brian, Emerson Knapp, Jacob Perron


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_generator_type_description <https://github.com/ros2/rosidl/tree/iron/rosidl_generator_type_description/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Type Description Codegen and Typesupport  (rep2011) (`#727 <https://github.com/ros2/rosidl/issues/727>`__)
* Expose type hash on typesupports (rep2011) (`#729 <https://github.com/ros2/rosidl/issues/729>`__)
* Type hash in interface codegen (rep2011) (`#722 <https://github.com/ros2/rosidl/issues/722>`__)
* Contributors: Emerson Knapp


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_parser <https://github.com/ros2/rosidl/tree/iron/rosidl_parser/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [service introspection] generate service_event messages (`#700 <https://github.com/ros2/rosidl/issues/700>`__)
* [rolling] Update maintainers - 2022-11-07 (`#717 <https://github.com/ros2/rosidl/issues/717>`__)
* Always include whitespace in string literals (`#688 <https://github.com/ros2/rosidl/issues/688>`__)
* Contributors: Audrow Nash, Brian, Shane Loretz


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_pycommon <https://github.com/ros2/rosidl/tree/iron/rosidl_pycommon/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Type Description Codegen and Typesupport  (rep2011) (`#727 <https://github.com/ros2/rosidl/issues/727>`__)
* Type hash in interface codegen (rep2011) (`#722 <https://github.com/ros2/rosidl/issues/722>`__)
* [rolling] Update maintainers - 2022-11-07 (`#717 <https://github.com/ros2/rosidl/issues/717>`__)
* Move rosidl_cmake Python module to a new package rosidl_pycommon (`#696 <https://github.com/ros2/rosidl/issues/696>`__) Deprecate the Python module in rosidl_cmake and move the implementation to the new package rosidl_pycommon.
* Contributors: Audrow Nash, Emerson Knapp, Jacob Perron


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_runtime_c <https://github.com/ros2/rosidl/tree/iron/rosidl_runtime_c/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Dynamic Subscription (BONUS: Allocators): rosidl (`#737 <https://github.com/ros2/rosidl/issues/737>`__)
* Runtime Interface Reflection: rosidl (`#728 <https://github.com/ros2/rosidl/issues/728>`__)
* Type Description Codegen and Typesupport  (rep2011) (`#727 <https://github.com/ros2/rosidl/issues/727>`__)
* Copied type_description_interfaces structs (rep2011) (`#732 <https://github.com/ros2/rosidl/issues/732>`__)
* Expose type hash on typesupports (rep2011) (`#729 <https://github.com/ros2/rosidl/issues/729>`__)
* Type hash in interface codegen (rep2011) (`#722 <https://github.com/ros2/rosidl/issues/722>`__)
* [service introspection] generate service_event messages (`#700 <https://github.com/ros2/rosidl/issues/700>`__)
* [rolling] Update maintainers - 2022-11-07 (`#717 <https://github.com/ros2/rosidl/issues/717>`__)
* Contributors: Audrow Nash, Brian, Emerson Knapp, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_runtime_cpp <https://github.com/ros2/rosidl/tree/iron/rosidl_runtime_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Type Description Codegen and Typesupport  (rep2011) (`#727 <https://github.com/ros2/rosidl/issues/727>`__)
* Copied type_description_interfaces structs (rep2011) (`#732 <https://github.com/ros2/rosidl/issues/732>`__)
* Fix a few more clang analysis problems. (`#731 <https://github.com/ros2/rosidl/issues/731>`__)
* Return reference from BoundedVector::emplace_back (`#730 <https://github.com/ros2/rosidl/issues/730>`__)
* [service introspection] generate service_event messages (`#700 <https://github.com/ros2/rosidl/issues/700>`__)
* [rolling] Update maintainers - 2022-11-07 (`#717 <https://github.com/ros2/rosidl/issues/717>`__)
* fix conversion to ‘std::streamsize’ {aka ‘long int’} from ‘size_t’ {aka ‘long unsigned int’} may change the sign of the result (`#715 <https://github.com/ros2/rosidl/issues/715>`__)
* Contributors: Alexander Hans, Audrow Nash, Brian, Chris Lalancette, Emerson Knapp, ralwing


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_runtime_py <https://github.com/ros2/rosidl_runtime_py/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Replace the use __slots_\_ for the appropiate API (`#23 <https://github.com/ros2/rosidl_runtime_py/issues/23>`__)
* fix(typing): ``get_interface_packages`` returns a dict (`#22 <https://github.com/ros2/rosidl_runtime_py/issues/22>`__)
* [rolling] Update maintainers - 2022-11-07 (`#21 <https://github.com/ros2/rosidl_runtime_py/issues/21>`__)
* Expand timestamps for std_msgs.msg.Header and builtin_interfaces.msg.Time if 'auto' and 'now' are passed as values (`#19 <https://github.com/ros2/rosidl_runtime_py/issues/19>`__)
* Document a missing parameter in message_to_yaml. (`#18 <https://github.com/ros2/rosidl_runtime_py/issues/18>`__)
* Mirror rolling to master
* Contributors: Audrow Nash, Chris Lalancette, Eloy Briceno, Esteve Fernandez, 兰陈昕


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_c <https://github.com/ros2/rosidl_typesupport/tree/iron/rosidl_typesupport_c/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Type Description Nested Support (`#141 <https://github.com/ros2/rosidl_typesupport/issues/141>`__)
* Fix rosidl_typesupport_c/cpp exec dependencies. (`#140 <https://github.com/ros2/rosidl_typesupport/issues/140>`__)
* Type hashes in typesupport (rep2011) (`#135 <https://github.com/ros2/rosidl_typesupport/issues/135>`__)
* Mark benchmark _ as UNUSED. (`#134 <https://github.com/ros2/rosidl_typesupport/issues/134>`__)
* Service introspection (`#127 <https://github.com/ros2/rosidl_typesupport/issues/127>`__)
* Update rosidl_typesupport to C++17. (`#131 <https://github.com/ros2/rosidl_typesupport/issues/131>`__)
* [rolling] Update maintainers - 2022-11-07 (`#130 <https://github.com/ros2/rosidl_typesupport/issues/130>`__)
* Replace rosidl_cmake imports with rosidl_pycommon (`#126 <https://github.com/ros2/rosidl_typesupport/issues/126>`__)
* [service introspection] Use stddef.h instead of cstddef (`#125 <https://github.com/ros2/rosidl_typesupport/issues/125>`__)
* Contributors: Audrow Nash, Brian, Chris Lalancette, Emerson Knapp, Jacob Perron


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_cpp <https://github.com/ros2/rosidl_typesupport/tree/iron/rosidl_typesupport_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Type Description Nested Support (`#141 <https://github.com/ros2/rosidl_typesupport/issues/141>`__)
* Fix rosidl_typesupport_c/cpp exec dependencies. (`#140 <https://github.com/ros2/rosidl_typesupport/issues/140>`__)
* Type hashes in typesupport (rep2011) (`#135 <https://github.com/ros2/rosidl_typesupport/issues/135>`__)
* Mark benchmark _ as UNUSED. (`#134 <https://github.com/ros2/rosidl_typesupport/issues/134>`__)
* Service introspection (`#127 <https://github.com/ros2/rosidl_typesupport/issues/127>`__)
* Update rosidl_typesupport to C++17. (`#131 <https://github.com/ros2/rosidl_typesupport/issues/131>`__)
* [rolling] Update maintainers - 2022-11-07 (`#130 <https://github.com/ros2/rosidl_typesupport/issues/130>`__)
* Replace rosidl_cmake imports with rosidl_pycommon (`#126 <https://github.com/ros2/rosidl_typesupport/issues/126>`__)
* Contributors: Audrow Nash, Brian, Chris Lalancette, Emerson Knapp, Jacob Perron


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_fastrtps_c <https://github.com/ros2/rosidl_typesupport_fastrtps/tree/iron/rosidl_typesupport_fastrtps_c/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Type Description Nested Support (`#101 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/101>`__)
* Type hashes on typesupport (rep2011) (`#98 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/98>`__)
* Expose type hash to typesupport structs (rep2011) (`#95 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/95>`__)
* Mark benchmark _ as UNUSED. (`#96 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/96>`__)
* Service introspection (`#92 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/92>`__)
* Update rosidl_typesupport_fastrtps to C++17. (`#94 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/94>`__)
* [rolling] Update maintainers - 2022-11-07 (`#93 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/93>`__)
* Replace rosidl_cmake imports with rosidl_pycommon (`#91 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/91>`__)
* Contributors: Audrow Nash, Brian, Chris Lalancette, Emerson Knapp, Jacob Perron


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_fastrtps_cpp <https://github.com/ros2/rosidl_typesupport_fastrtps/tree/iron/rosidl_typesupport_fastrtps_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Type Description Nested Support (`#101 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/101>`__)
* Type hashes on typesupport (rep2011) (`#98 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/98>`__)
* Depend on ament_cmake_ros to default SHARED to ON (`#99 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/99>`__)
* Expose type hash to typesupport structs (rep2011) (`#95 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/95>`__)
* Mark benchmark _ as UNUSED. (`#96 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/96>`__)
* Service introspection (`#92 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/92>`__)
* Update rosidl_typesupport_fastrtps to C++17. (`#94 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/94>`__)
* [rolling] Update maintainers - 2022-11-07 (`#93 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/93>`__)
* Replace rosidl_cmake imports with rosidl_pycommon (`#91 <https://github.com/ros2/rosidl_typesupport_fastrtps/issues/91>`__)
* Contributors: Audrow Nash, Brian, Chris Lalancette, Emerson Knapp, Jacob Perron, Tyler Weaver


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_interface <https://github.com/ros2/rosidl/tree/iron/rosidl_typesupport_interface/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [service introspection] generate service_event messages (`#700 <https://github.com/ros2/rosidl/issues/700>`__)
* [rolling] Update maintainers - 2022-11-07 (`#717 <https://github.com/ros2/rosidl/issues/717>`__)
* Contributors: Audrow Nash, Brian


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_introspection_c <https://github.com/ros2/rosidl/tree/iron/rosidl_typesupport_introspection_c/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Type Description Codegen and Typesupport  (rep2011) (`#727 <https://github.com/ros2/rosidl/issues/727>`__)
* Expose type hash on typesupports (rep2011) (`#729 <https://github.com/ros2/rosidl/issues/729>`__)
* Type hash in interface codegen (rep2011) (`#722 <https://github.com/ros2/rosidl/issues/722>`__)
* [service introspection] generate service_event messages (`#700 <https://github.com/ros2/rosidl/issues/700>`__)
* [rolling] Update maintainers - 2022-11-07 (`#717 <https://github.com/ros2/rosidl/issues/717>`__)
* Move rosidl_cmake Python module to a new package rosidl_pycommon (`#696 <https://github.com/ros2/rosidl/issues/696>`__) Deprecate the Python module in rosidl_cmake and move the implementation to the new package rosidl_pycommon.
* Fix build export dependencies in C introspection package (`#695 <https://github.com/ros2/rosidl/issues/695>`__)
* Add namespaced ALIAS target to easily consume generated libraries via add_subdirectory (`#605 <https://github.com/ros2/rosidl/issues/605>`__)
* Contributors: Audrow Nash, Brian, Emerson Knapp, Jacob Perron, Silvio Traversaro


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_introspection_cpp <https://github.com/ros2/rosidl/tree/iron/rosidl_typesupport_introspection_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Type Description Codegen and Typesupport  (rep2011) (`#727 <https://github.com/ros2/rosidl/issues/727>`__)
* Expose type hash on typesupports (rep2011) (`#729 <https://github.com/ros2/rosidl/issues/729>`__)
* Type hash in interface codegen (rep2011) (`#722 <https://github.com/ros2/rosidl/issues/722>`__)
* Make sure to add the event message to typesupport introspection cpp. (`#724 <https://github.com/ros2/rosidl/issues/724>`__)
* [service introspection] generate service_event messages (`#700 <https://github.com/ros2/rosidl/issues/700>`__)
* [rolling] Update maintainers - 2022-11-07 (`#717 <https://github.com/ros2/rosidl/issues/717>`__)
* Move rosidl_cmake Python module to a new package rosidl_pycommon (`#696 <https://github.com/ros2/rosidl/issues/696>`__) Deprecate the Python module in rosidl_cmake and move the implementation to the new package rosidl_pycommon.
* Add namespaced ALIAS target to easily consume generated libraries via add_subdirectory (`#605 <https://github.com/ros2/rosidl/issues/605>`__)
* Contributors: Audrow Nash, Brian, Chris Lalancette, Emerson Knapp, Jacob Perron, Silvio Traversaro


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_introspection_tests <https://github.com/ros2/rosidl/tree/iron/rosidl_typesupport_introspection_tests/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix a few more clang analysis problems. (`#731 <https://github.com/ros2/rosidl/issues/731>`__) In particular, make sure to mark the fact that we are C++17 (as the emplace_back signature changed), and also add in a few more (void)_ for benchmark tests.
* [service introspection] generate service_event messages (`#700 <https://github.com/ros2/rosidl/issues/700>`__)
* [rolling] Update maintainers - 2022-11-07 (`#717 <https://github.com/ros2/rosidl/issues/717>`__)
* Contributors: Audrow Nash, Brian, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rosidl_typesupport_tests <https://github.com/ros2/rosidl_typesupport/tree/iron/rosidl_typesupport_tests/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* typesupport_tests needs to be updated to C++17 (`#137 <https://github.com/ros2/rosidl_typesupport/issues/137>`__)
* Fix Typesupport Introspection tests (`#133 <https://github.com/ros2/rosidl_typesupport/issues/133>`__)
* Make rosidl_typesupport_tests depend on rosidl_generator_cpp. (`#132 <https://github.com/ros2/rosidl_typesupport/issues/132>`__)
* Service introspection (`#127 <https://github.com/ros2/rosidl_typesupport/issues/127>`__)
* Contributors: Brian, Chris Lalancette, Cristóbal Arroyo, Lucas Wendland


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rpyutils <https://github.com/ros2/rpyutils/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#10 <https://github.com/ros2/rpyutils/issues/10>`__)
* Mirror rolling to master
* updating maintainer
* Contributors: Audrow Nash, Dharini Dutia


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt <https://github.com/ros-visualization/rqt/tree/iron/rqt/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* fix build of ``rqt`` with ``setuptools>=v61.0.0`` (`#271 <https://github.com/ros-visualization/rqt/issues/271>`__)
* [rolling] Update maintainers - 2022-11-07 (`#283 <https://github.com/ros-visualization/rqt/issues/283>`__)
* Fix up the package description. (`#250 <https://github.com/ros-visualization/rqt/issues/250>`__)
* Contributors: Audrow Nash, Chris Lalancette, Daniel Reuter, Dharini Dutia, quarkytale


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_action <https://github.com/ros-visualization/rqt_action/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#14 <https://github.com/ros-visualization/rqt_action/issues/14>`__)
* Small cleanups to the rqt_action plugin (`#13 <https://github.com/ros-visualization/rqt_action/issues/13>`__)
* Mirror rolling to ros2
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_bag <https://github.com/ros-visualization/rqt_bag/tree/iron/rqt_bag/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use default storage id (`#140 <https://github.com/ros-visualization/rqt_bag/issues/140>`__)
* Use rosbag2_py API instead of direct bag parsing
* [rolling] Update maintainers - 2022-11-07 (`#132 <https://github.com/ros-visualization/rqt_bag/issues/132>`__)
* For get_entry_after, bump by 1 nanosecond otherwise always get the same message equal to the timestamp
* Use rosbag2_py.reader for all message queries, remove sqlite3 direct usage
* Cleanup for review
* Improved logging
* Use a rosbag2_py.Reader to get bag metadata
* Disable reading from bag while recording - use direct caching to index for timeline
* Increase publishing checkbox size (`#122 <https://github.com/ros-visualization/rqt_bag/issues/122>`__)
* Fix toggle thumbnails button (`#117 <https://github.com/ros-visualization/rqt_bag/issues/117>`__)
* ensure data types match what PyQt expects (`#118 <https://github.com/ros-visualization/rqt_bag/issues/118>`__)
* Visualize topics being published and highlight topic being selected (`#116 <https://github.com/ros-visualization/rqt_bag/issues/116>`__)
* Be able to scroll up and down, not only zoom-in and out the timeline (`#114 <https://github.com/ros-visualization/rqt_bag/issues/114>`__)
* [Fixes] Fix crash when no qos metadata, make scroll bar appear if needed, add gitignore (`#113 <https://github.com/ros-visualization/rqt_bag/issues/113>`__)
* Fix the types being passed into QFont and QColor. (`#109 <https://github.com/ros-visualization/rqt_bag/issues/109>`__)
* Fix tuples for bisect calls (`#67 <https://github.com/ros-visualization/rqt_bag/issues/67>`__) (`#76 <https://github.com/ros-visualization/rqt_bag/issues/76>`__)
* fix long topic names (`#114 <https://github.com/ros-visualization/rqt_common_plugins/issues/114>`__)
* fix zoom behavior (`#76 <https://github.com/ros-visualization/rqt_common_plugins/issues/76>`__)
* Contributors: Audrow Nash, Chris Lalancette, Emerson Knapp, Ivan Santiago Paunovic, Kenji Brameld, Yadunund


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_bag_plugins <https://github.com/ros-visualization/rqt_bag/tree/iron/rqt_bag_plugins/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Changes the use of __slots_\_ for the field and field type getter (`#138 <https://github.com/ros-visualization/rqt_bag/issues/138>`__)
* [rolling] Update maintainers - 2022-11-07 (`#132 <https://github.com/ros-visualization/rqt_bag/issues/132>`__)
* Contributors: Audrow Nash, Eloy Briceno


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_console <https://github.com/ros-visualization/rqt_console/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#39 <https://github.com/ros-visualization/rqt_console/issues/39>`__)
* added new maintainer
* Contributors: Arne Hitzmann, Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_graph <https://github.com/ros-visualization/rqt_graph/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Refresh rosgraph when params checkbox is clicked (`#87 <https://github.com/ros-visualization/rqt_graph/issues/87>`__)
* [rolling] Update maintainers - 2022-11-07 (`#83 <https://github.com/ros-visualization/rqt_graph/issues/83>`__)
* Minor cleanup (`#80 <https://github.com/ros-visualization/rqt_graph/issues/80>`__)
* Mirror rolling to galactic-devel
* graph load/save into DOT file corrections for py3 (`#78 <https://github.com/ros-visualization/rqt_graph/issues/78>`__)
* Remove repeated prefixes from buttons
* Contributors: Audrow Nash, Chris Lalancette, David V. Lu!!, Yadunund, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_gui <https://github.com/ros-visualization/rqt/tree/iron/rqt_gui/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#283 <https://github.com/ros-visualization/rqt/issues/283>`__)
* Display basic help information when no plugins are loaded (`#268 <https://github.com/ros-visualization/rqt/issues/268>`__)
* Contributors: Audrow Nash, Dharini Dutia, Michael Jeronimo, quarkytale


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_gui_cpp <https://github.com/ros-visualization/rqt/tree/iron/rqt_gui_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update rqt to C++17. (`#285 <https://github.com/ros-visualization/rqt/issues/285>`__)
* [rolling] Update maintainers - 2022-11-07 (`#283 <https://github.com/ros-visualization/rqt/issues/283>`__)
* Contributors: Audrow Nash, Chris Lalancette, Dharini Dutia, quarkytale


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_gui_py <https://github.com/ros-visualization/rqt/tree/iron/rqt_gui_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix an exception raised when terminating with Ctrl+c (`#292 <https://github.com/ros-visualization/rqt/issues/292>`__)
* [rolling] Update maintainers - 2022-11-07 (`#283 <https://github.com/ros-visualization/rqt/issues/283>`__)
* Contributors: Audrow Nash, Chen Lihui, Dharini Dutia, quarkytale


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_msg <https://github.com/ros-visualization/rqt_msg/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#17 <https://github.com/ros-visualization/rqt_msg/issues/17>`__)
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_plot <https://github.com/ros-visualization/rqt_plot/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix regression from #87 (`#91 <https://github.com/ros-visualization/rqt_plot/issues/91>`__)
* Changes the use of __slots_\_ for the field and field type getter (`#87 <https://github.com/ros-visualization/rqt_plot/issues/87>`__)
* [rolling] Update maintainers - 2022-11-07 (`#83 <https://github.com/ros-visualization/rqt_plot/issues/83>`__)
* Fix fixed-size Array visualization (`#81 <https://github.com/ros-visualization/rqt_plot/issues/81>`__)
* Contributors: Audrow Nash, Eloy Briceno, Jacob Perron, Michael Jeronimo, Yadunund


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_publisher <https://github.com/ros-visualization/rqt_publisher/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Changes the use of __slots_\_ for the field and field type getter
* [rolling] Update maintainers - 2022-11-07 (`#36 <https://github.com/ros-visualization/rqt_publisher/issues/36>`__)
* Minor cleanups in rqt_publisher for ROS 2 (`#35 <https://github.com/ros-visualization/rqt_publisher/issues/35>`__)
* Delete sync to foxy-devel workflow
* Merge pull request `#33 <https://github.com/ros-visualization/rqt_publisher/issues/33>`__ from NBadyal/improve-evaluation-of-types
* Use regex matching to strip errors from input
* Change slot_type verification strategy
* Mirror rolling to foxy-devel
* Contributors: Audrow Nash, Chris Lalancette, Geoffrey Biggs, Michael Jeronimo, Nicholas Badyal, Voldivh


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_py_common <https://github.com/ros-visualization/rqt/tree/iron/rqt_py_common/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Changes the use of __slots_\_ for the field and field type getter (`#289 <https://github.com/ros-visualization/rqt/issues/289>`__)
* [rolling] Update maintainers - 2022-11-07 (`#283 <https://github.com/ros-visualization/rqt/issues/283>`__)
* Contributors: Audrow Nash, Dharini Dutia, Eloy Briceno, quarkytale


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_py_console <https://github.com/ros-visualization/rqt_py_console/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#13 <https://github.com/ros-visualization/rqt_py_console/issues/13>`__)
* Contributors: Audrow Nash, Jacob Perron


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_reconfigure <https://github.com/ros-visualization/rqt_reconfigure/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* reorder imports to fix flake8 warning (`#129 <https://github.com/ros-visualization/rqt_reconfigure/issues/129>`__)
* Fixed validator locale when float value is not bound in a range. (`#121 <https://github.com/ros-visualization/rqt_reconfigure/issues/121>`__)
* get parameter type from descriptor
* [rolling] Update maintainers - 2022-11-07 (`#122 <https://github.com/ros-visualization/rqt_reconfigure/issues/122>`__)
* Cleanup mislabeled BSD license (`#66 <https://github.com/ros-visualization/rqt_reconfigure/issues/66>`__)
* Add support for array types (`#108 <https://github.com/ros-visualization/rqt_reconfigure/issues/108>`__)
* Fix float slider step size (`#117 <https://github.com/ros-visualization/rqt_reconfigure/issues/117>`__)
* update maintainer
* Fixed package to run with ros2 run (`#81 <https://github.com/ros-visualization/rqt_reconfigure/issues/81>`__)
* fix updating range limits (`#108 <https://github.com/ros-visualization/rqt_common_plugins/issues/108>`__)
* Improvement; "GUI hangs for awhile or completely, when any one of nodes doesn't return any value" (`#81 <https://github.com/ros-visualization/rqt_common_plugins/issues/81>`__)
* Contributors: Aris Synodinos, Audrow Nash, Christian Rauch, Dharini Dutia, Florian Vahl, Jacob Perron, Shrijit Singh, Tully Foote, quarkytale


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_service_caller <https://github.com/ros-visualization/rqt_service_caller/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#25 <https://github.com/ros-visualization/rqt_service_caller/issues/25>`__)
* Contributors: Audrow Nash, Jacob Perron


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_shell <https://github.com/ros-visualization/rqt_shell/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#17 <https://github.com/ros-visualization/rqt_shell/issues/17>`__)
* Contributors: Audrow Nash, Jacob Perron


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_srv <https://github.com/ros-visualization/rqt_srv/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#10 <https://github.com/ros-visualization/rqt_srv/issues/10>`__)
* Contributors: Audrow Nash, Jacob Perron


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_topic <https://github.com/ros-visualization/rqt_topic/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#43 <https://github.com/ros-visualization/rqt_topic/issues/43>`__)
* Implement bandwidth monitoring (`#40 <https://github.com/ros-visualization/rqt_topic/issues/40>`__)
* Fix the display of array type elements. (`#41 <https://github.com/ros-visualization/rqt_topic/issues/41>`__)
* Fix removal of topics while they are being monitored. (`#39 <https://github.com/ros-visualization/rqt_topic/issues/39>`__)
* Contributors: Audrow Nash, Chris Lalancette, Jacob Perron


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rti_connext_dds_cmake_module <https://github.com/ros2/rmw_connextdds/tree/iron/rti_connext_dds_cmake_module/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Use unified approach for checking the existence of environment variables (`#117 <https://github.com/ros2/rmw_connextdds/issues/117>`__)
* Contributors: Christopher Wecht


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rttest <https://github.com/ros2/realtime_support/tree/iron/rttest/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#121 <https://github.com/ros2/realtime_support/issues/121>`__)
* Addressing issues found in Humble testing (`#116 <https://github.com/ros2/realtime_support/issues/116>`__)
* Contributors: Audrow Nash, Michael Carroll


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rviz2 <https://github.com/ros2/rviz/tree/iron/rviz2/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Make rviz1_to_rviz2.py accept configs with missing values (`#945 <https://github.com/ros2/rviz/issues/945>`__)
* Update rviz to C++17. (`#939 <https://github.com/ros2/rviz/issues/939>`__)
* [rolling] Update maintainers - 2022-11-07 (`#923 <https://github.com/ros2/rviz/issues/923>`__)
* Add rviz1_to_rviz2.py conversion script (`#882 <https://github.com/ros2/rviz/issues/882>`__)
* Contributors: Audrow Nash, Chris Lalancette, Shane Loretz


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rviz_assimp_vendor <https://github.com/ros2/rviz/tree/iron/rviz_assimp_vendor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* If vendored assimp is present, always prefer that (`#970 <https://github.com/ros2/rviz/issues/970>`__)
* [rolling] Update maintainers - 2022-11-07 (`#923 <https://github.com/ros2/rviz/issues/923>`__)
* Fixes policy CMP0135 warning for CMake >= 3.24 (`#898 <https://github.com/ros2/rviz/issues/898>`__)
* Contributors: Audrow Nash, Cristóbal Arroyo, Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rviz_common <https://github.com/ros2/rviz/tree/iron/rviz_common/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update Frame shortcut (`#958 <https://github.com/ros2/rviz/issues/958>`__) * Update Frame shortcut
* Update rviz to C++17. (`#939 <https://github.com/ros2/rviz/issues/939>`__)
* [rolling] Update maintainers - 2022-11-07 (`#923 <https://github.com/ros2/rviz/issues/923>`__)
* Remove YAML_CPP_DLL define (`#831 <https://github.com/ros2/rviz/issues/831>`__)
* Document getTransform() time behavior (`#893 <https://github.com/ros2/rviz/issues/893>`__)
* Ogre 1.12.10 upgrade (`#878 <https://github.com/ros2/rviz/issues/878>`__)
* Add RVIZ_COMMON_PUBLIC macro (`#865 <https://github.com/ros2/rviz/issues/865>`__)
* Add time jump handler (`#752 <https://github.com/ros2/rviz/issues/752>`__) (`#791 <https://github.com/ros2/rviz/issues/791>`__)
* Make sure not to dereference a null Renderable pointer. (`#850 <https://github.com/ros2/rviz/issues/850>`__)
* Contributors: Akash, Audrow Nash, Chris Lalancette, David V. Lu!!, Kenji Brameld, Marcel Zeilinger, Shane Loretz, juchajam


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rviz_default_plugins <https://github.com/ros2/rviz/tree/iron/rviz_default_plugins/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix ODR errors with gmock (`#967 <https://github.com/ros2/rviz/issues/967>`__)
* Update Frame shortcut (`#958 <https://github.com/ros2/rviz/issues/958>`__)
* point_marker: fix bug where the number of rendered points accumulates over time (`#949 <https://github.com/ros2/rviz/issues/949>`__)
* Update rviz to C++17. (`#939 <https://github.com/ros2/rviz/issues/939>`__)
* Fix tolerance calculation precision (`#934 <https://github.com/ros2/rviz/issues/934>`__)
* Fix MeshResourceMarker for mesh with color-based embedded material (`#928 <https://github.com/ros2/rviz/issues/928>`__)
* [rolling] Update maintainers - 2022-11-07 (`#923 <https://github.com/ros2/rviz/issues/923>`__)
* Add Map Display binary option (`#846 <https://github.com/ros2/rviz/issues/846>`__)
* Delete frame_locked_markers when reusing marker (`#907 <https://github.com/ros2/rviz/issues/907>`__)
* Consider region of interest in CameraDisplay (`#864 <https://github.com/ros2/rviz/issues/864>`__)
* std::copy fix - OccupancyGridUpdate - Data is not being processed correctly (`#895 <https://github.com/ros2/rviz/issues/895>`__)
* Set error status when duplicate markers are in the same MarkerArray (`#891 <https://github.com/ros2/rviz/issues/891>`__)
* Make Axes display use latest transform (`#892 <https://github.com/ros2/rviz/issues/892>`__)
* Show link names in inertia error message (`#874 <https://github.com/ros2/rviz/issues/874>`__)
* Ogre 1.12.10 upgrade (`#878 <https://github.com/ros2/rviz/issues/878>`__)
* Use make_shared to construct PointCloud2 (`#869 <https://github.com/ros2/rviz/issues/869>`__)
* Fix include order (`#858 <https://github.com/ros2/rviz/issues/858>`__)
* Contributors: AndreasR30, Audrow Nash, Chris Lalancette, David V. Lu!!, Eric, Hunter L. Allen, Jacob Perron, Kenji Brameld, Patrick Roncagliolo, Shane Loretz, Timon Engelke, Xavier BROQUERE, Xenofon Karamanos, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rviz_ogre_vendor <https://github.com/ros2/rviz/tree/iron/rviz_ogre_vendor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix build failures on macOS + Apple Silicon (`#944 <https://github.com/ros2/rviz/issues/944>`__)
* [rolling] Update maintainers - 2022-11-07 (`#923 <https://github.com/ros2/rviz/issues/923>`__)
* Remove broken rviz_ogre_vendor::RenderSystem_GL target (`#920 <https://github.com/ros2/rviz/issues/920>`__)
* Fixes policy CMP0135 warning for CMake >= 3.24 (`#898 <https://github.com/ros2/rviz/issues/898>`__)
* Ogre 1.12.10 upgrade (`#878 <https://github.com/ros2/rviz/issues/878>`__)
* Make resource file paths relative (`#862 <https://github.com/ros2/rviz/issues/862>`__)
* Use CMAKE_STAGING_PREFIX for staging OGRE installation (`#861 <https://github.com/ros2/rviz/issues/861>`__)
* Contributors: Audrow Nash, Cristóbal Arroyo, Kenji Brameld, Scott K Logan, Shane Loretz, Yadu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rviz_rendering <https://github.com/ros2/rviz/tree/iron/rviz_rendering/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#923 <https://github.com/ros2/rviz/issues/923>`__)
* add test to ensure binary STL files from SOLIDWORKS get imported without a warning (`#917 <https://github.com/ros2/rviz/issues/917>`__)
* Ogre 1.12.10 upgrade (`#878 <https://github.com/ros2/rviz/issues/878>`__)
* Stop using glsl150 resources for now. (`#851 <https://github.com/ros2/rviz/issues/851>`__)
* Contributors: Audrow Nash, Chris Lalancette, Kenji Brameld


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rviz_rendering_tests <https://github.com/ros2/rviz/tree/iron/rviz_rendering_tests/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#923 <https://github.com/ros2/rviz/issues/923>`__)
* add test to ensure binary STL files from SOLIDWORKS get imported without a warning (`#917 <https://github.com/ros2/rviz/issues/917>`__)
* Contributors: Audrow Nash, Kenji Brameld


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rviz_visual_testing_framework <https://github.com/ros2/rviz/tree/iron/rviz_visual_testing_framework/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update rviz to C++17. (`#939 <https://github.com/ros2/rviz/issues/939>`__)
* [rolling] Update maintainers - 2022-11-07 (`#923 <https://github.com/ros2/rviz/issues/923>`__)
* Ogre 1.12.10 upgrade (`#878 <https://github.com/ros2/rviz/issues/878>`__)
* Contributors: Audrow Nash, Chris Lalancette, Kenji Brameld


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`sensor_msgs <https://github.com/ros2/common_interfaces/tree/iron/sensor_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* update YUV format codes and documentation (`#214 <https://github.com/ros2/common_interfaces/issues/214>`__)
* sensor_msgs/Range lacks variance field (`#181 <https://github.com/ros2/common_interfaces/issues/181>`__)
* Update common_interfaces to C++17. (`#215 <https://github.com/ros2/common_interfaces/issues/215>`__)
* [rolling] Update maintainers - 2022-11-07 (`#210 <https://github.com/ros2/common_interfaces/issues/210>`__)
* Replaced non-ASCII dash symbol with ASCII dash (`#208 <https://github.com/ros2/common_interfaces/issues/208>`__)
* Add NV21 and NV24 to colour formats (`#205 <https://github.com/ros2/common_interfaces/issues/205>`__)
* Update BatteryState.msg (`#206 <https://github.com/ros2/common_interfaces/issues/206>`__)
* use regex for matching cv types (`#202 <https://github.com/ros2/common_interfaces/issues/202>`__)
* Fix outdated file path for image_encodings (`#200 <https://github.com/ros2/common_interfaces/issues/200>`__)
* Use uint32_t for pointcloud2 resize method (`#195 <https://github.com/ros2/common_interfaces/issues/195>`__)
* Retain width and height after resize for master (`#193 <https://github.com/ros2/common_interfaces/issues/193>`__)
* Contributors: Audrow Nash, Borong Yuan, Chris Lalancette, Christian Rauch, El Jawad Alaa, Geoffrey Biggs, Ivan Zatevakhin, Kenji Brameld, Tianyu Li


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`sensor_msgs_py <https://github.com/ros2/common_interfaces/tree/iron/sensor_msgs_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add missing dep for sensor_msgs_py (`#217 <https://github.com/ros2/common_interfaces/issues/217>`__)
* [rolling] Update maintainers - 2022-11-07 (`#210 <https://github.com/ros2/common_interfaces/issues/210>`__)
* Add support for non standard point step sizes (`#199 <https://github.com/ros2/common_interfaces/issues/199>`__)
* Remove reference to old implementation (`#198 <https://github.com/ros2/common_interfaces/issues/198>`__)
* Contributors: Audrow Nash, Florian Vahl, Yadu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`service_msgs <https://github.com/ros2/rcl_interfaces/tree/iron/service_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update common_interfaces to C++17. (`#215 <https://github.com/ros2/rcl_interfaces/issues/215>`__) (`#151 <https://github.com/ros2/rcl_interfaces/issues/151>`__)
* Add service_msgs package (`#143 <https://github.com/ros2/rcl_interfaces/issues/143>`__)
* Contributors: Brian, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`shape_msgs <https://github.com/ros2/common_interfaces/tree/iron/shape_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update common_interfaces to C++17. (`#215 <https://github.com/ros2/common_interfaces/issues/215>`__)
* [rolling] Update maintainers - 2022-11-07 (`#210 <https://github.com/ros2/common_interfaces/issues/210>`__)
* Fix SolidPrimitive.msg to contain a single Polygon (`#189 <https://github.com/ros2/common_interfaces/issues/189>`__)
* Contributors: Audrow Nash, Chris Lalancette, M. Fatih Cırıt


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`shared_queues_vendor <https://github.com/ros2/rosbag2/tree/iron/shared_queues_vendor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`__)
* Fixes policy CMP0135 warning for CMake >= 3.24 (`#1084 <https://github.com/ros2/rosbag2/issues/1084>`__)
* Contributors: Cristóbal Arroyo, Michael Orlov


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`spdlog_vendor <https://github.com/ros2/spdlog_vendor/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update to spdlog 1.9.2 (`#33 <https://github.com/ros2/spdlog_vendor/issues/33>`__)
* [rolling] Update maintainers - 2022-11-07 (`#31 <https://github.com/ros2/spdlog_vendor/issues/31>`__)
* Update to spdlog 1.9.1 (`#27 <https://github.com/ros2/spdlog_vendor/issues/27>`__)
* Fixes policy CMP0135 warning for CMake >= 3.24 (`#30 <https://github.com/ros2/spdlog_vendor/issues/30>`__)
* build shared lib only if BUILD_SHARED_LIBS is set (`#29 <https://github.com/ros2/spdlog_vendor/issues/29>`__)
* Mirror rolling to master
* xml tag order
* updating maintainer
* Contributors: Audrow Nash, Chris Lalancette, Cristóbal Arroyo, Dharini Dutia, Scott K Logan, hannes09


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`sqlite3_vendor <https://github.com/ros2/rosbag2/tree/iron/sqlite3_vendor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update to sqlite3 3.37.2 (`#1274 <https://github.com/ros2/rosbag2/issues/1274>`__) This matches version distributed in Ubuntu Jammy.
* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`__)
* Fixes policy CMP0135 warning for CMake >= 3.24 (`#1084 <https://github.com/ros2/rosbag2/issues/1084>`__)
* Contributors: Cristóbal Arroyo, Michael Orlov, Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`sros2 <https://github.com/ros2/sros2/tree/iron/sros2/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix SSH commands in SROS2_Linux.md (`#286 <https://github.com/ros2/sros2/issues/286>`__)
* Make type of get_package_share_directory apparent for sphinx (`#284 <https://github.com/ros2/sros2/issues/284>`__)
* Contributors: Boris Boutillier, Yadu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`statistics_msgs <https://github.com/ros2/rcl_interfaces/tree/iron/statistics_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update common_interfaces to C++17. (`#215 <https://github.com/ros2/rcl_interfaces/issues/215>`__) (`#151 <https://github.com/ros2/rcl_interfaces/issues/151>`__)
* [rolling] Update maintainers - 2022-11-07 (`#150 <https://github.com/ros2/rcl_interfaces/issues/150>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`std_msgs <https://github.com/ros2/common_interfaces/tree/iron/std_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update common_interfaces to C++17. (`#215 <https://github.com/ros2/common_interfaces/issues/215>`__)
* [rolling] Update maintainers - 2022-11-07 (`#210 <https://github.com/ros2/common_interfaces/issues/210>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`std_srvs <https://github.com/ros2/common_interfaces/tree/iron/std_srvs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update common_interfaces to C++17. (`#215 <https://github.com/ros2/common_interfaces/issues/215>`__)
* [rolling] Update maintainers - 2022-11-07 (`#210 <https://github.com/ros2/common_interfaces/issues/210>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`stereo_msgs <https://github.com/ros2/common_interfaces/tree/iron/stereo_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update common_interfaces to C++17. (`#215 <https://github.com/ros2/common_interfaces/issues/215>`__)
* [rolling] Update maintainers - 2022-11-07 (`#210 <https://github.com/ros2/common_interfaces/issues/210>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tango_icons_vendor <https://github.com/ros-visualization/tango_icons_vendor/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#10 <https://github.com/ros-visualization/tango_icons_vendor/issues/10>`__)
* Mirror rolling to master
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_cli <https://github.com/ros2/system_tests/tree/iron/test_cli/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the system tests to C++17. (`#510 <https://github.com/ros2/system_tests/issues/510>`__)
* [rolling] Update maintainers - 2022-11-07 (`#509 <https://github.com/ros2/system_tests/issues/509>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_cli_remapping <https://github.com/ros2/system_tests/tree/iron/test_cli_remapping/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the system tests to C++17. (`#510 <https://github.com/ros2/system_tests/issues/510>`__)
* [rolling] Update maintainers - 2022-11-07 (`#509 <https://github.com/ros2/system_tests/issues/509>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_communication <https://github.com/ros2/system_tests/tree/iron/test_communication/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the system tests to C++17. (`#510 <https://github.com/ros2/system_tests/issues/510>`__)
* [rolling] Update maintainers - 2022-11-07 (`#509 <https://github.com/ros2/system_tests/issues/509>`__)
* Revert "Replace deprecated spin_until_future_complete with spin_until_complete (`#499 <https://github.com/ros2/system_tests/issues/499>`__)" (`#504 <https://github.com/ros2/system_tests/issues/504>`__)
* Replace deprecated spin_until_future_complete with spin_until_complete (`#499 <https://github.com/ros2/system_tests/issues/499>`__)
* Contributors: Audrow Nash, Chris Lalancette, Hubert Liberacki, William Woodall


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_interface_files <https://github.com/ros2/test_interface_files/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#21 <https://github.com/ros2/test_interface_files/issues/21>`__)
* Mirror rolling to master
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_launch_ros <https://github.com/ros2/launch_ros/tree/iron/test_launch_ros/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Enable document generation using rosdoc2 (`#359 <https://github.com/ros2/launch_ros/issues/359>`__)
* Fix normalize_parameters_dict for multiple nodes in the same namespace (`#347 <https://github.com/ros2/launch_ros/issues/347>`__)
* Implement None check for ComposableNodeContainer (`#341 <https://github.com/ros2/launch_ros/issues/341>`__)
* Add LifecyleTransition action (`#317 <https://github.com/ros2/launch_ros/issues/317>`__)
* Ensure load_composable_nodes respects condition (`#339 <https://github.com/ros2/launch_ros/issues/339>`__)
* [rolling] Update maintainers - 2022-11-07 (`#331 <https://github.com/ros2/launch_ros/issues/331>`__)
* RosTimer -> ROSTimer and PushRosNamespace -> PushROSNamespace, to follow PEP8 (`#326 <https://github.com/ros2/launch_ros/issues/326>`__)
* add SetROSLogDir action (`#325 <https://github.com/ros2/launch_ros/issues/325>`__)
* Support default values in parameter substitution (`#313 <https://github.com/ros2/launch_ros/issues/313>`__)
* Run condition for composable nodes (`#311 <https://github.com/ros2/launch_ros/issues/311>`__)
* Load composable nodes in sequence (`#315 <https://github.com/ros2/launch_ros/issues/315>`__)
* Contributors: Aditya Pande, Alexey Merzlyakov, Audrow Nash, Christoph Hellmann Santos, Kenji Miyake, Shane Loretz, William Woodall, Yadu, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_launch_testing <https://github.com/ros2/launch/tree/iron/test_launch_testing/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#671 <https://github.com/ros2/launch/issues/671>`__)
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_msgs <https://github.com/ros2/rcl_interfaces/tree/iron/test_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update common_interfaces to C++17. (`#215 <https://github.com/ros2/rcl_interfaces/issues/215>`__) (`#151 <https://github.com/ros2/rcl_interfaces/issues/151>`__)
* [rolling] Update maintainers - 2022-11-07 (`#150 <https://github.com/ros2/rcl_interfaces/issues/150>`__)
* Depend on rosidl_core_generators for packages required by actions (`#144 <https://github.com/ros2/rcl_interfaces/issues/144>`__)
* Make the functions in the header static inline (`#140 <https://github.com/ros2/rcl_interfaces/issues/140>`__)
* Contributors: Audrow Nash, Chris Lalancette, Jacob Perron


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_osrf_testing_tools_cpp <https://github.com/osrf/osrf_testing_tools_cpp/tree/iron/test_osrf_testing_tools_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Changing C++ Compile Version (`#76 <https://github.com/osrf/osrf_testing_tools_cpp/issues/76>`__)
* Update maintainers (`#74 <https://github.com/osrf/osrf_testing_tools_cpp/issues/74>`__)
* Contributors: Audrow Nash, Lucas Wendland


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_quality_of_service <https://github.com/ros2/system_tests/tree/iron/test_quality_of_service/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix ODR errors with gtest (`#514 <https://github.com/ros2/system_tests/issues/514>`__)
* Avoid flaky test (`#513 <https://github.com/ros2/system_tests/issues/513>`__)
* Update the system tests to C++17. (`#510 <https://github.com/ros2/system_tests/issues/510>`__)
* [rolling] Update maintainers - 2022-11-07 (`#509 <https://github.com/ros2/system_tests/issues/509>`__)
* Pass rclcpp::QoS to create_service (`#507 <https://github.com/ros2/system_tests/issues/507>`__)
* Pass rclcpp::QoS to create_client (`#506 <https://github.com/ros2/system_tests/issues/506>`__)
* Remove Werror from test_quality_of_service. (`#503 <https://github.com/ros2/system_tests/issues/503>`__)
* Revert "Replace deprecated spin_until_future_complete with spin_until_complete (`#499 <https://github.com/ros2/system_tests/issues/499>`__)" (`#504 <https://github.com/ros2/system_tests/issues/504>`__)
* Replace deprecated spin_until_future_complete with spin_until_complete (`#499 <https://github.com/ros2/system_tests/issues/499>`__)
* Add tests for 'best available' QoS policies (`#501 <https://github.com/ros2/system_tests/issues/501>`__)
* Contributors: Audrow Nash, Chen Lihui, Chris Lalancette, Hubert Liberacki, Jacob Perron, Shane Loretz, William Woodall, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_rclcpp <https://github.com/ros2/system_tests/tree/iron/test_rclcpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the system tests to C++17. (`#510 <https://github.com/ros2/system_tests/issues/510>`__)
* [rolling] Update maintainers - 2022-11-07 (`#509 <https://github.com/ros2/system_tests/issues/509>`__)
* Pass rclcpp::QoS to create_service (`#507 <https://github.com/ros2/system_tests/issues/507>`__)
* Pass rclcpp::QoS to create_client (`#506 <https://github.com/ros2/system_tests/issues/506>`__)
* Revert "Replace deprecated spin_until_future_complete with spin_until_complete (`#499 <https://github.com/ros2/system_tests/issues/499>`__)" (`#504 <https://github.com/ros2/system_tests/issues/504>`__)
* Replace deprecated spin_until_future_complete with spin_until_complete (`#499 <https://github.com/ros2/system_tests/issues/499>`__)
* Contributors: Audrow Nash, Chris Lalancette, Hubert Liberacki, Shane Loretz, William Woodall


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_rmw_implementation <https://github.com/ros2/rmw_implementation/tree/iron/test_rmw_implementation/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add tests for rmw matched event (`#216 <https://github.com/ros2/rmw_implementation/issues/216>`__)
* Update rmw_implementation to C++17. (`#214 <https://github.com/ros2/rmw_implementation/issues/214>`__)
* [rolling] Update maintainers - 2022-11-07 (`#212 <https://github.com/ros2/rmw_implementation/issues/212>`__)
* Add rmw_get_gid_for_client & tests (`#206 <https://github.com/ros2/rmw_implementation/issues/206>`__)
* Contributors: Audrow Nash, Barry Xu, Brian, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_ros2trace <https://github.com/ros2/ros2_tracing/tree/iron/test_ros2trace/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Move ros2trace tests to new test_ros2trace package (`#63 <https://github.com/ros2/ros2_tracing/issues/63>`__)
* Contributors: Christophe Bedard


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_security <https://github.com/ros2/system_tests/tree/iron/test_security/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#509 <https://github.com/ros2/system_tests/issues/509>`__)
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_tf2 <https://github.com/ros2/geometry2/tree/iron/test_tf2/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the demos to C++17. (`#578 <https://github.com/ros2/geometry2/issues/578>`__)
* Update maintainers (`#560 <https://github.com/ros2/geometry2/issues/560>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_tracetools <https://github.com/ros2/ros2_tracing/tree/iron/test_tracetools/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Disable tracing on Android (`#72 <https://github.com/ros2/ros2_tracing/issues/72>`__)
* Add intra-process tracepoints (`#30 <https://github.com/ros2/ros2_tracing/issues/30>`__)
* Allow requiring minimum lttng package version for is_lttng_installed (`#59 <https://github.com/ros2/ros2_tracing/issues/59>`__)
* Disable tracing on macOS (`#53 <https://github.com/ros2/ros2_tracing/issues/53>`__)
* Include tracepoints by default on Linux (`#31 <https://github.com/ros2/ros2_tracing/issues/31>`__)
* Fix memory leak in tracetools::get_symbol() (`#43 <https://github.com/ros2/ros2_tracing/issues/43>`__)
* Update tracing to C++17. (`#33 <https://github.com/ros2/ros2_tracing/issues/33>`__)
* Contributors: Chris Lalancette, Christophe Bedard, Przemysław Dąbrowski, ymski


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`test_tracetools_launch <https://github.com/ros2/ros2_tracing/tree/iron/test_tracetools_launch/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Error out if trace already exists unless 'append' option is used (`#58 <https://github.com/ros2/ros2_tracing/issues/58>`__)
* Make subbuffer size configurable with Trace action (`#51 <https://github.com/ros2/ros2_tracing/issues/51>`__)
* Allow requiring minimum lttng package version for is_lttng_installed (`#59 <https://github.com/ros2/ros2_tracing/issues/59>`__)
* Enable document generation using rosdoc2 for ament_python pkgs (`#50 <https://github.com/ros2/ros2_tracing/issues/50>`__)
* Contributors: Christophe Bedard, Christopher Wecht, Yadu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2 <https://github.com/ros2/geometry2/tree/iron/tf2/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Fix error code returned in BufferCore::walkToTopParent (`#602 <https://github.com/ros2/geometry2/issues/602>`__)
* Depend on ament_cmake_ros to default SHARED to ON (`#591 <https://github.com/ros2/geometry2/issues/591>`__)
* Fix a potential crash in TimeCache::findClosest (`#592 <https://github.com/ros2/geometry2/issues/592>`__)
* Extend TimeCache API to provide rich ExtrapolationException infos (`#586 <https://github.com/ros2/geometry2/issues/586>`__)
* Update geometry2 to C++17 (`#584 <https://github.com/ros2/geometry2/issues/584>`__)
* Include required header Scalar.h (`#559 <https://github.com/ros2/geometry2/issues/559>`__)
* Update maintainers (`#560 <https://github.com/ros2/geometry2/issues/560>`__)
* Contributors: Audrow Nash, Chris Lalancette, Patrick Roncagliolo, Shane Loretz, Tyler Weaver


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_bullet <https://github.com/ros2/geometry2/tree/iron/tf2_bullet/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the demos to C++17. (`#578 <https://github.com/ros2/geometry2/issues/578>`__)
* Update maintainers (`#560 <https://github.com/ros2/geometry2/issues/560>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_eigen <https://github.com/ros2/geometry2/tree/iron/tf2_eigen/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the demos to C++17. (`#578 <https://github.com/ros2/geometry2/issues/578>`__)
* Update maintainers (`#560 <https://github.com/ros2/geometry2/issues/560>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_eigen_kdl <https://github.com/ros2/geometry2/tree/iron/tf2_eigen_kdl/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update geometry2 to C++17 (`#584 <https://github.com/ros2/geometry2/issues/584>`__)
* Update maintainers (`#560 <https://github.com/ros2/geometry2/issues/560>`__)
* Use orocos_kdl_vendor and orocos-kdl target (`#534 <https://github.com/ros2/geometry2/issues/534>`__)
* Contributors: Audrow Nash, Chris Lalancette, Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_geometry_msgs <https://github.com/ros2/geometry2/tree/iron/tf2_geometry_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add do_transform_polygon_stamped (`#582 <https://github.com/ros2/geometry2/issues/582>`__)
* Update the demos to C++17. (`#578 <https://github.com/ros2/geometry2/issues/578>`__)
* Update maintainers (`#560 <https://github.com/ros2/geometry2/issues/560>`__)
* Add torque due to force offset (`#538 <https://github.com/ros2/geometry2/issues/538>`__)
* Use orocos_kdl_vendor and orocos-kdl target (`#534 <https://github.com/ros2/geometry2/issues/534>`__)
* Contributors: Audrow Nash, Chris Lalancette, Paul Gesel, Scott K Logan, Tony Najjar


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_kdl <https://github.com/ros2/geometry2/tree/iron/tf2_kdl/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the demos to C++17. (`#578 <https://github.com/ros2/geometry2/issues/578>`__)
* Update maintainers (`#560 <https://github.com/ros2/geometry2/issues/560>`__)
* Use orocos_kdl_vendor and orocos-kdl target (`#534 <https://github.com/ros2/geometry2/issues/534>`__)
* Contributors: Audrow Nash, Chris Lalancette, Scott K Logan


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_msgs <https://github.com/ros2/geometry2/tree/iron/tf2_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update geometry2 to C++17 (`#584 <https://github.com/ros2/geometry2/issues/584>`__)
* Update maintainers (`#560 <https://github.com/ros2/geometry2/issues/560>`__)
* Remove action_msgs dependency (`#547 <https://github.com/ros2/geometry2/issues/547>`__)
* Contributors: Audrow Nash, Chris Lalancette, Jacob Perron


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_py <https://github.com/ros2/geometry2/tree/iron/tf2_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update geometry2 to C++17 (`#584 <https://github.com/ros2/geometry2/issues/584>`__)
* Update maintainers (`#560 <https://github.com/ros2/geometry2/issues/560>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_ros <https://github.com/ros2/geometry2/tree/iron/tf2_ros/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Destroy callback group before node (`#595 <https://github.com/ros2/geometry2/issues/595>`__)
* Enable TransformListener node-based constructor in Intra-process enabled components (`#572 <https://github.com/ros2/geometry2/issues/572>`__)
* Fix use-after-free bug in BufferServer::cancelCB (`#579 <https://github.com/ros2/geometry2/issues/579>`__)
* Update the demos to C++17. (`#578 <https://github.com/ros2/geometry2/issues/578>`__)
* add constructor to static tf broadcaster accepting node interfaces (`#576 <https://github.com/ros2/geometry2/issues/576>`__)
* Update maintainers (`#560 <https://github.com/ros2/geometry2/issues/560>`__)
* Switching from sstream to c string formatting to fix ros arg issue (`#557 <https://github.com/ros2/geometry2/issues/557>`__)
* allow construction of tf broadcaster from node object (not a pointer) (`#555 <https://github.com/ros2/geometry2/issues/555>`__)
* Allow to construct ``TransformBroadcaster`` and ``TransformListener`` from node interfaces (`#552 <https://github.com/ros2/geometry2/issues/552>`__)
* Suppress spam from calling canTransform (`#529 <https://github.com/ros2/geometry2/issues/529>`__)
* Contributors: Alberto Soragna, Alexander Hans, Audrow Nash, Chris Lalancette, Gonzo, Michael Carroll, Patrick Roncagliolo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_ros_py <https://github.com/ros2/geometry2/tree/iron/tf2_ros_py/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update sys.path with wokring directory (`#594 <https://github.com/ros2/geometry2/issues/594>`__)
* Enable document generation using rosdoc2 for ament_python pkgs (`#587 <https://github.com/ros2/geometry2/issues/587>`__)
* Update maintainers (`#560 <https://github.com/ros2/geometry2/issues/560>`__)
* Use pytest rather than unittest to enable repeat (`#558 <https://github.com/ros2/geometry2/issues/558>`__)
* Contributors: Audrow Nash, Michael Carroll, Yadu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_sensor_msgs <https://github.com/ros2/geometry2/tree/iron/tf2_sensor_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the demos to C++17. (`#578 <https://github.com/ros2/geometry2/issues/578>`__)
* Update maintainers (`#560 <https://github.com/ros2/geometry2/issues/560>`__)
* feat: export tf2 sensor msgs target (`#536 <https://github.com/ros2/geometry2/issues/536>`__)
* tf2_sensor_msgs find the right Python executable. (`#525 <https://github.com/ros2/geometry2/issues/525>`__)
* Add missing ament_cmake_pytest package needed because of newly-enabled test (`#520 <https://github.com/ros2/geometry2/issues/520>`__)
* Port point cloud transformation to numpy (`#507 <https://github.com/ros2/geometry2/issues/507>`__)
* Contributors: Audrow Nash, Chris Lalancette, Daisuke Nishimatsu, Florian Vahl, Jorge Perez, Michael Jeronimo


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tf2_tools <https://github.com/ros2/geometry2/tree/iron/tf2_tools/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Enable document generation using rosdoc2 for ament_python pkgs (`#587 <https://github.com/ros2/geometry2/issues/587>`__)
* Update maintainers (`#560 <https://github.com/ros2/geometry2/issues/560>`__)
* Contributors: Audrow Nash, Yadu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tlsf <https://github.com/ros2/tlsf/tree/iron/tlsf/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#13 <https://github.com/ros2/tlsf/issues/13>`__)
* Update maintainers (`#12 <https://github.com/ros2/tlsf/issues/12>`__)
* Contributors: Audrow Nash, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tlsf_cpp <https://github.com/ros2/realtime_support/tree/iron/tlsf_cpp/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update realtime support to C++17. (`#122 <https://github.com/ros2/realtime_support/issues/122>`__)
* [rolling] Update maintainers - 2022-11-07 (`#121 <https://github.com/ros2/realtime_support/issues/121>`__)
* Addressing issues found in Humble testing (`#116 <https://github.com/ros2/realtime_support/issues/116>`__)
* Contributors: Audrow Nash, Chris Lalancette, Michael Carroll


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`topic_monitor <https://github.com/ros2/demos/tree/iron/topic_monitor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* update launch file name format to match documentation (`#588 <https://github.com/ros2/demos/issues/588>`__)
* [rolling] Update maintainers - 2022-11-07 (`#589 <https://github.com/ros2/demos/issues/589>`__)
* Contributors: Audrow Nash, Patrick Wspanialy


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`topic_statistics_demo <https://github.com/ros2/demos/tree/iron/topic_statistics_demo/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update the demos to C++17. (`#594 <https://github.com/ros2/demos/issues/594>`__)
* [rolling] Update maintainers - 2022-11-07 (`#589 <https://github.com/ros2/demos/issues/589>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tracetools <https://github.com/ros2/ros2_tracing/tree/iron/tracetools/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Disable tracing on Android (`#72 <https://github.com/ros2/ros2_tracing/issues/72>`__)
* Add intra-process tracepoints (`#30 <https://github.com/ros2/ros2_tracing/issues/30>`__)
* Improve tracetools rosdoc2/doxygen output (`#57 <https://github.com/ros2/ros2_tracing/issues/57>`__)
* Update README and other documentation (`#55 <https://github.com/ros2/ros2_tracing/issues/55>`__)
* Disable tracing on macOS (`#53 <https://github.com/ros2/ros2_tracing/issues/53>`__)
* Include tracepoints by default on Linux (`#31 <https://github.com/ros2/ros2_tracing/issues/31>`__)
* Explicitly link against dl for dladdr() (`#48 <https://github.com/ros2/ros2_tracing/issues/48>`__)
* Fix memory leak in tracetools::get_symbol() (`#43 <https://github.com/ros2/ros2_tracing/issues/43>`__)
* Add TRACEPOINT_ENABLED() and DO_TRACEPOINT() macros (`#46 <https://github.com/ros2/ros2_tracing/issues/46>`__)
* Update tracing to C++17. (`#33 <https://github.com/ros2/ros2_tracing/issues/33>`__)
* Add new rclcpp_subscription_init tracepoint to support new intra-process comms
* Contributors: Chris Lalancette, Christophe Bedard, Przemysław Dąbrowski, ymski


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tracetools_launch <https://github.com/ros2/ros2_tracing/tree/iron/tracetools_launch/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Error out if trace already exists unless 'append' option is used (`#58 <https://github.com/ros2/ros2_tracing/issues/58>`__)
* Improve 'ros2 trace' command error handling & add end-to-end tests (`#54 <https://github.com/ros2/ros2_tracing/issues/54>`__)
* Make subbuffer size configurable with Trace action (`#51 <https://github.com/ros2/ros2_tracing/issues/51>`__)
* Enable document generation using rosdoc2 for ament_python pkgs (`#50 <https://github.com/ros2/ros2_tracing/issues/50>`__)
* Remove deprecated context_names parameter (`#38 <https://github.com/ros2/ros2_tracing/issues/38>`__)
* Contributors: Christophe Bedard, Christopher Wecht, Yadu


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`tracetools_trace <https://github.com/ros2/ros2_tracing/tree/iron/tracetools_trace/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Error out if trace already exists unless 'append' option is used (`#58 <https://github.com/ros2/ros2_tracing/issues/58>`__)
* Improve 'ros2 trace' command error handling & add end-to-end tests (`#54 <https://github.com/ros2/ros2_tracing/issues/54>`__)
* Make subbuffer size configurable with Trace action (`#51 <https://github.com/ros2/ros2_tracing/issues/51>`__)
* Add intra-process tracepoints (`#30 <https://github.com/ros2/ros2_tracing/issues/30>`__)
* Allow requiring minimum lttng package version for is_lttng_installed (`#59 <https://github.com/ros2/ros2_tracing/issues/59>`__)
* Include tracepoints by default on Linux (`#31 <https://github.com/ros2/ros2_tracing/issues/31>`__)
* Enable document generation using rosdoc2 for ament_python pkgs (`#50 <https://github.com/ros2/ros2_tracing/issues/50>`__)
* Replace distutils.version.StrictVersion with packaging.version.Version (`#42 <https://github.com/ros2/ros2_tracing/issues/42>`__)
* Remove deprecated context_names parameter (`#38 <https://github.com/ros2/ros2_tracing/issues/38>`__)
* Contributors: Christophe Bedard, Christopher Wecht, Yadu, ymski


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`trajectory_msgs <https://github.com/ros2/common_interfaces/tree/iron/trajectory_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update common_interfaces to C++17. (`#215 <https://github.com/ros2/common_interfaces/issues/215>`__)
* [rolling] Update maintainers - 2022-11-07 (`#210 <https://github.com/ros2/common_interfaces/issues/210>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`turtlesim <https://github.com/ros/ros_tutorials/tree/iron/turtlesim/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Remove the range constraints from the holonomic parameter. (`#150 <https://github.com/ros/ros_tutorials/issues/150>`__) (`#151 <https://github.com/ros/ros_tutorials/issues/151>`__)
* Add icon (`#148 <https://github.com/ros/ros_tutorials/issues/148>`__) (`#149 <https://github.com/ros/ros_tutorials/issues/149>`__)
* Update turtlesim to C++17. (`#146 <https://github.com/ros/ros_tutorials/issues/146>`__)
* [rolling] Update maintainers - 2022-11-07 (`#145 <https://github.com/ros/ros_tutorials/issues/145>`__)
* Add parameter to enable holonomic motion (`#131 <https://github.com/ros/ros_tutorials/issues/131>`__)
* Add humble turtle (`#140 <https://github.com/ros/ros_tutorials/issues/140>`__)
* Contributors: Audrow Nash, Chris Lalancette, Daisuke Sato, mergify[bot]


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`type_description_interfaces <https://github.com/ros2/rcl_interfaces/tree/iron/type_description_interfaces/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add GetTypeDescription.srv (rep2011) (`#153 <https://github.com/ros2/rcl_interfaces/issues/153>`__)
* new package and interfaces for describing other types (`#146 <https://github.com/ros2/rcl_interfaces/issues/146>`__)
* Contributors: Emerson Knapp, William Woodall


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`unique_identifier_msgs <https://github.com/ros2/unique_identifier_msgs/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#26 <https://github.com/ros2/unique_identifier_msgs/issues/26>`__)
* Depend on rosidl_core instead of rosidl_defaults (`#24 <https://github.com/ros2/unique_identifier_msgs/issues/24>`__)
* Mirror rolling to master
* Update maintainers (`#22 <https://github.com/ros2/unique_identifier_msgs/issues/22>`__)
* Contributors: Audrow Nash, Jacob Perron, methylDragon


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`urdf <https://github.com/ros2/urdf/tree/iron/urdf/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#35 <https://github.com/ros2/urdf/issues/35>`__)
* [urdf] package.xml: add missing exec_depend to urdf_parser_plugin (`#34 <https://github.com/ros2/urdf/issues/34>`__)
* Provide copy and move constructors for ``model`` (`#33 <https://github.com/ros2/urdf/issues/33>`__)
* Add linter tests and fix errors (`#30 <https://github.com/ros2/urdf/issues/30>`__)
* fix `#30 <https://github.com/ros/robot_model/issues/30>`__
* Contributors: Audrow Nash, Daniel Reuter, Tobias Neumann


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`urdf_parser_plugin <https://github.com/ros2/urdf/tree/iron/urdf_parser_plugin/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#35 <https://github.com/ros2/urdf/issues/35>`__)
* Contributors: Audrow Nash


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`visualization_msgs <https://github.com/ros2/common_interfaces/tree/iron/visualization_msgs/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Update common_interfaces to C++17. (`#215 <https://github.com/ros2/common_interfaces/issues/215>`__)
* [rolling] Update maintainers - 2022-11-07 (`#210 <https://github.com/ros2/common_interfaces/issues/210>`__)
* Contributors: Audrow Nash, Chris Lalancette


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`yaml_cpp_vendor <https://github.com/ros2/yaml_cpp_vendor/tree/iron/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* [rolling] Update maintainers - 2022-11-07 (`#40 <https://github.com/ros2/yaml_cpp_vendor/issues/40>`__)
* Export YAML_CPP_DLL define on Windows (`#30 <https://github.com/ros2/yaml_cpp_vendor/issues/30>`__) (`#38 <https://github.com/ros2/yaml_cpp_vendor/issues/38>`__)
* Sets CMP0135 policy behavior to NEW (`#36 <https://github.com/ros2/yaml_cpp_vendor/issues/36>`__)
* Fixes policy CMP0135 warning for CMake >= 3.24 (`#35 <https://github.com/ros2/yaml_cpp_vendor/issues/35>`__)
* build shared lib only if BUILD_SHARED_LIBS is set (`#34 <https://github.com/ros2/yaml_cpp_vendor/issues/34>`__)
* Mirror rolling to master
* Contributors: Audrow Nash, Cristóbal Arroyo, Jacob Perron, hannes09


^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`zstd_vendor <https://github.com/ros2/rosbag2/tree/iron/zstd_vendor/CHANGELOG.rst>`__
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`__)
* Bump zstd to 1.4.8 in zstd_vendor package (`#1132 <https://github.com/ros2/rosbag2/issues/1132>`__)
* Fix/zstd vendor does not find system zstd (`#1111 <https://github.com/ros2/rosbag2/issues/1111>`__)
* Contributors: DasRoteSkelett, Michael Orlov


