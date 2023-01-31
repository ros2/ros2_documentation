Porting a C++ package from Catkin to Ament
==========================================

This guide will walk through how to port a C++ package from Catkin to Ament.

Pretend we have a package named ``foobar`` which provides a library ``libfoobar.so`` and an executable ``foobar_node``.

It has the following CMakeLists.txt:

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

It has the following package.xml:

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

Changing the package.xml
------------------------

After porting, the ``package.xml`` should look like this:

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

Changing the CMakeLists.txt
---------------------------

After porting, the ``CMakeLists.txt`` should look like this:

.. code-block:: xml

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
        RUNTIME DESTINATION bin)

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

TODO catkin_package() -> ament_package() at end
TODO find_package(catkin REQUIRED COMPONENTS) -> find_package
TODO headers installed to unique DIRECTORY
TODO CATKIN_ENABLE_TESTING to BUILD_TESTING
TODO CATKIN_DEPENDS and DEPENDS to ament_export_dependency()

TODO library and binary install directory

TODO ament_uncrustify --reformat src/foobar/