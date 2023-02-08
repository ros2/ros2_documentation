Porting a C++ package from Catkin to Ament
==========================================

This guide will walk through how to port a C++ package from Catkin to Ament.

Pretend we have a package named ``foobar`` which provides a library ``libfoobar.so`` and an executable ``foobar_node``.

Changing the CMakeLists.txt
---------------------------

Let's look at the changes made ``foobar``'s ``CMakeLists.txt``.

.. tabs::

  .. group-tab:: ROS 1 CMakeLists.txt

    .. code-block:: CMake

      cmake_minimum_required(VERSION 3.0.2)
      project(foobar)

      add_compile_options(-std=c++14 -Wall -Werror)

      find_package(catkin REQUIRED COMPONENTS
          roscpp
          tf2
          std_msgs
      )

      catkin_package(
          LIBRARIES foobar
          INCLUDE_DIRS
              include
          CATKIN_DEPENDS
              roscpp
              tf2
              std_msgs
      )

      add_library(foobar src/foobar.cpp)
      target_include_directories(foobar PUBLIC include ${catkin_INCLUDE_DIRS})
      target_link_libraries(foobar ${catkin_LIBRARIES})

      add_executable(foobar_node src/foobar_node.cpp)
      target_include_directories(foobar_node PUBLIC include ${catkin_INCLUDE_DIRS})
      target_link_libraries(foobar_node foobar)

      install(DIRECTORY include/${PROJECT_NAME}/
          DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      )

      install(TARGETS foobar
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

      install(TARGETS foobar_node
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

      if(CATKIN_ENABLE_TESTING)
          find_package(roslint REQUIRED)
          roslint_cpp()
          roslint_add_test()

          catkin_add_gtest(test_baz
              test/test_baz.cpp
          )
          target_link_libraries(test_baz
              foobar
              ${catkin_LIBRARIES}
          )
      endif()

  .. group-tab:: ROS 2 CMakeLists.txt

    .. code-block:: CMake

      cmake_minimum_required(VERSION 3.16)
      project(foobar)

      if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
          add_compile_options(-Wall -Werror)
      endif()

      find_package(ament_cmake_ros REQUIRED)
      find_package(rclcpp REQUIRED)
      find_package(tf2 REQUIRED)
      find_package(std_msgs REQUIRED)

      add_library(foobar src/foobar.cpp)
      target_include_directories(foobar PUBLIC
          "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
          "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
      target_link_libraries(foobar PUBLIC
          rclcpp::rclcpp
          tf2::tf2
          ${std_msgs_TARGETS})
      target_compile_features(foobar PUBLIC cxx_std_17)

      add_executable(foobar_node src/foobar_node.cpp)
      target_link_libraries(foobar_node foobar)

      install(DIRECTORY include/
          DESTINATION include/${PROJECT_NAME})

      install(TARGETS foobar EXPORT foobar-export
          ARCHIVE DESTINATION lib
          LIBRARY DESTINATION lib
          RUNTIME DESTINATION bin)

      install(TARGETS foobar_node
          DESTINATION lib/${PROJECT_NAME})

      if(BUILD_TESTING)
          find_package(ament_lint_auto REQUIRED)
          ament_lint_auto_find_test_dependencies()

          find_package(ament_cmake_gtest REQUIRED)

          ament_add_gtest(test_baz
              test/test_baz.cpp)
          target_link_libraries(test_baz
              foobar)
      endif()

      ament_export_targets(foobar-export HAS_LIBRARY_TARGET)

      ament_export_dependencies(rclcpp)
      ament_export_dependencies(tf2)
      ament_export_dependencies(std_msgs)

      ament_package()



Supported CMake versions and Platforms
++++++++++++++++++++++++++++++++++++++

The first thing to note is the minimum supported CMake version increased, and there are now compiler specific checks.

.. tabs::

  .. group-tab:: ROS 1

    .. code-block:: CMake

      cmake_minimum_required(VERSION 3.0.2)
      project(foobar)

      add_compile_options(-std=c++14 -Wall -Werror)

  .. group-tab:: ROS 2

    .. code-block:: CMake 

      cmake_minimum_required(VERSION 3.14.4)
      project(foobar)

      if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
          add_compile_options(-Wall -Werror)
      endif()

This is because ROS 2 generally targets both more platforms, and newer platforms than ROS 1.

You can see which platforms a release targets by reading `REP 2000 <https://www.ros.org/reps/rep-2000.html#iron-irwini-may-2023-november-2024>`__
Note that there's a table showing required and recommended versions of dependencies, including CMake.
Only use CMake features present in the minimum version you'll target, and set your ``cmake_minimum_required()`` version accordingly.

Note that ROS 2 supports Windows.
This means compiler specific options, like ``-Wall`` and ``-Werror`` should be guarded to only take effect on platforms that support them.

You'll also notice the ``-std=c++14`` option is no longer used.
Modern CMake has better tools for choosing a C++ standard.
This example uses `target_compile_features() <https://cmake.org/cmake/help/v3.14/command/target_compile_features.html>`__.
Note that many packages use `CMAKE_CXX_STANDARD <https://cmake.org/cmake/help/v3.14/variable/CMAKE_CXX_STANDARD.html>`__ instead.
There are advantages to each.

.. tabs::

  .. group-tab:: target_compile_features

    .. code-block:: CMake 

      target_compile_features(foobar PUBLIC cxx_std_17)

  .. group-tab:: CMAKE_CXX_STANDARD

    .. code-block:: CMake 

      if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 17)
      endif()

The function ``target_compile_features()`` is target specific.
First, it says the ``foobar`` target requires at least C++ 17.
Second, because it uses the ``PUBLIC`` keyword it says any target that depends on ``foobar`` must also be compiled with at least C++ 17 support.
An advantage is the C++ version requirement propogates to downstream targets.
For example, if you use C++ 17 features in one of your library's headers then downstream targets will automatically get built with C++ 17 features enabled.
A disadvantage is you may have to specify this multiple times when you have multiple targets in your package that don't depend on each other.

The second way is ``CMAKE_CXX_STANDARD``
It applies to all targets in the project declared after it.
An advantage of this method is the C++ standard is set in only one place.
A disadvantage is it's not propogated to downstream targets.
If you use C++ 17 in one of your headers, and someone uses that header downstream then their target will fail to build unless they also declare their target requires the C++ standard your target requires.

Packages replaced in ROS 2
++++++++++++++++++++++++++

The next thing to notice is some of the dependencies have changed.
In this example ``ament_cmake_ros`` is used instead of ``catkin``, and ``rclcpp`` is used instead of roscpp.
You can find a list of packages that have been replaced at `this page <./ROS-1-Package-Equivalents>`__.

Finding the packages you need
+++++++++++++++++++++++++++++

The recommended way of finding packages has changed from ROS 1 to ROS 2.

.. tabs::

  .. group-tab:: ROS 1

    .. code-block:: CMake

      find_package(catkin REQUIRED COMPONENTS
          roscpp
          tf2
          std_msgs
      )

  .. group-tab:: ROS 2

    .. code-block:: CMake

      find_package(ament_cmake_ros REQUIRED)
      find_package(rclcpp REQUIRED)
      find_package(tf2 REQUIRED)
      find_package(std_msgs REQUIRED)

In ROS 1, it is recommended list all the packages you want to find as ``COMPONENTS`` of the ``catkin`` package.
In ROS 2, you instead find all of your packages directly.

Placement of ament_package vs catkin_package
++++++++++++++++++++++++++++++++++++++++++++

The ``catkin_package()`` function is replaced by several functions in ``ament_cmake``.
In ROS 1 the call to ``catkin_package()`` is placed near the top of the file, and given lots of arguments that have different effects.
In ROS 2 there are instead several ``ament_*()`` function calls, the last of which is ``ament_package()``.
It is very important that the ``ament_package()`` call is the last one in your ``CMakeLists.txt``.

.. tabs::

  .. group-tab:: ROS 1

    .. code-block:: CMake 

      # ... Placed near the top of the file
      catkin_package(
          LIBRARIES foobar
          INCLUDE_DIRS
              include
          CATKIN_DEPENDS
              roscpp
              tf2
              std_msgs
      )

  .. group-tab:: ROS 2

    .. code-block:: CMake

      # ... Placed at the bottom of the file
      ament_package()

Let's look closer at how each of the arguments to the ``catkin_package()`` call were replaced.

catkin_package(LIBRARIES ... INCLUDE_DIRS)
==========================================

In ROS 1 the ``LIBRARIES`` and ``INCLUDE_DIRS`` arguments setup standard CMake variables set when downstream packages ``find_package()`` your package.
In ROS 2 this should be replaced with exporting modern CMake targets.
First associate include directories and libraries with your target using ``target_include_directories()`` and ``target_link_libraries()``.
Next associate your target with ``install(TARGETS ... EXPORT export-name)`` followed by ``ament_export_targets(export-name)``.

.. code-block:: CMake

  # Set target specific include directories and libraries
  target_include_directories(foobar PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
  target_link_libraries(foobar PUBLIC
      rclcpp::rclcpp
      tf2::tf2
      ${std_msgs_TARGETS})

  # Assosiates the library target `foobar` with the export name `foobar-export`
  install(TARGETS foobar EXPORT foobar-export
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

  # ...

  # Installs the export named `foobar-export``
  ament_export_targets(foobar-export HAS_LIBRARY_TARGET)

The argument ``HAS_LIBRARY_TARGET`` means the export-name includes a shared library.
This causes ``ament_cmake`` to create a file that update paths like ``LD_LIBRARY_PATH`` (depending on the platform) when your workspace setup scripts are sourced.

Note, there are still functions for old style standard CMake variables, ``ament_export_include_directories()`` and ``ament_export_libraries()``, but these should not be used for new code.

catkin_package(CATKIN_DEPENDS and DEPENDS)
==========================================

The ``CATKIN_DEPENDS`` and ``DEPENDS`` arguments to ``catkin_package()`` are both replaced with ``ament_export_dependencies()``.
There is no destinction between types of packages depended upon.
This function makes sure downstream packages transitively ``find_package()`` your package's dependencies.

.. code-block:: CMake

      ament_export_dependencies(rclcpp)
      ament_export_dependencies(tf2)
      ament_export_dependencies(std_msgs)

Installing header files
+++++++++++++++++++++++

In ROS 1 it was common to install all headers to ``${CATKIN_PACKAGE_INCLUDE_DESTINATION}``.
In ROS 2 it is `strongly recommended to install headers to a unique directory with the same name as your package <https://colcon.readthedocs.io/en/released/user/overriding-packages.html#install-headers-to-a-unique-include-directory>`__.

.. tabs::

  .. group-tab:: ROS 1

    .. code-block:: CMake 

      catkin_package(
          # ...
          INCLUDE_DIRS
              include
          # ...
      )
      # ...
      install(DIRECTORY include/${PROJECT_NAME}/
          DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      )

  .. group-tab:: ROS 2

    .. code-block:: CMake 

      # Target specific include directories
      target_include_directories(foobar PUBLIC
        # Says where to get headers when building this package
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        # Says where to get headers after installing this package (used by downstream packages)
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

      # ...

      install(DIRECTORY include/
          DESTINATION include/${PROJECT_NAME})

BUILD_TESTING instead of CATKIN_ENABLE_TESTING
++++++++++++++++++++++++++++++++++++++++++++++

Use the variable ``BUILD_TESTING`` instead of ``CATKIN_ENABLE_TESTING`` to control when to build tests.

.. tabs::

  .. group-tab:: ROS 1

    .. code-block:: CMake 

      if(CATKIN_ENABLE_TESTING)
          # ...
      endif()

  .. group-tab:: ROS 2

    .. code-block:: CMake 

      if(BUILD_TESTING)
          # ...
      endif()

Macros to add tests
+++++++++++++++++++

The names of macros to add and create tests have changed.
For example, for ``googletest`` based tests use ``ament_add_gtest`` instead of ``catkin_add_gtest``.

.. tabs::

  .. group-tab:: ROS 1

    .. code-block:: CMake 

      catkin_add_gtest(test_baz
          test/test_baz.cpp
      )

  .. group-tab:: ROS 2

    .. code-block:: CMake 

      ament_add_gtest(test_baz
          test/test_baz.cpp)

Where to install libraries and binaries
+++++++++++++++++++++++++++++++++++++++

TODO library and binary install directory


Changing the package.xml
------------------------

Now let's look at the changes to ``foobar``'s ``package.xml``.

.. tabs::

  .. group-tab:: ROS 1 package.xml

    .. code-block:: xml

        <package>
            <name>foobar</name>
            <version>1.2.3</version>
            <description>
                Foos the bar.
            </description>
            <maintainer email="someone@example.com">Someone</maintainer>

            <license>BSD</license>
            <author>John Doe</author>
            <author email="jane.doe@example.com">Jane Doe</author>

            <buildtool_depend>catkin</buildtool_depend>

            <build_depend>roscpp</build_depend>
            <build_depend>tf2</build_depend>
            <build_depend>eigen</build_depend>

            <run_depend>roscpp</run_depend>
            <run_depend>tf2</run_depend>
            <run_depend>eigen</run_depend>

            <test_depend>gtest</test_depend>
        </package>

  .. group-tab:: ROS 1 package.xml

    .. code-block:: xml

        <package format="3">
            <name>foobar</name>
            <version>1.2.3</version>
            <description>
                Foos the bar.
            </description>
            <maintainer email="someone@example.com">Someone</maintainer>

            <license>BSD</license>
            <author>John Doe</author>
            <author email="jane.doe@example.com">Jane Doe</author>

            <buildtool_depend>ament_cmake_ros</buildtool_depend>
            <buildtool_export_depend>ament_cmake_ros</buildtool_export_depend>

            <depend>rclcpp</depend>
            <depend>tf2</depend>
            <depend>std_msgs</depend>

            <test_depend>ament_cmake_gtest</test_depend>
            <test_depend>ament_lint_auto</test_depend>
            <test_depend>ament_lint_common</test_depend>

            <export>
                <build_type>ament_cmake</build_type>
            </export>
        </package>

TODO Package format 2 or 3
TODO buildtool_export_depend
TODO export build_type