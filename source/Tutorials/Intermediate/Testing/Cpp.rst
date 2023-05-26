.. TestingCpp:

Writing Basic Tests with C++ with GTest
=======================================

Starting point: we'll assume you have a :ref:`basic ament_cmake package<CreatePkg>` set up already and you want to add some tests to it.

In this tutorial, we'll be using `gtest <https://google.github.io/googletest/primer.html>`__.

Package Setup
-------------

Source Code
^^^^^^^^^^^
We'll start off with our code in a file called ``test/tutorial_test.cpp``

.. code-block:: c++

    #include <gtest/gtest.h>

    TEST(package_name, a_first_test)
    {
      ASSERT_EQ(4, 2 + 2);
    }

    int main(int argc, char** argv)
    {
      testing::InitGoogleTest(&argc, argv);
      return RUN_ALL_TESTS();
    }


package.xml
^^^^^^^^^^^
Add the following line to ``package.xml``

.. code-block:: c++

    <test_depend>ament_cmake_gtest</test_depend>

CMakeLists.txt
^^^^^^^^^^^^^^

.. code-block:: cmake

    if(BUILD_TESTING)
      find_package(ament_cmake_gtest REQUIRED)
      ament_add_gtest(${PROJECT_NAME}_tutorial_test test/tutorial_test.cpp)
      target_include_directories(${PROJECT_NAME}_tutorial_test PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
      )
      ament_target_dependencies(${PROJECT_NAME}_tutorial_test
        std_msgs
      )
      target_link_libraries(${PROJECT_NAME}_tutorial_test name_of_local_library)
    endif()

The testing code is wrapped in the ``if/endif`` block to avoid building tests where possible. ``ament_add_gtest`` functions much like ``add_executable`` so you'll need to call ``target_include_directories``, ``ament_target_dependencies`` and ``target_link_libraries`` as you normally would.


Running Tests
-------------

See the :doc:`tutorial on how to run tests from the command line <CLI>` for more information on running the tests and inspecting the test results.
