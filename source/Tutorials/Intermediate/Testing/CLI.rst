.. TestingCLI:

Running Tests in ROS 2 from the Command Line
============================================

Build and run your tests
^^^^^^^^^^^^^^^^^^^^^^^^

To compile and run the tests, simply run the `test <https://colcon.readthedocs.io/en/released/reference/verb/test.html>`__ verb from ``colcon``.

.. code-block:: console

  colcon test --ctest-args tests [package_selection_args]

(where ``package_selection_args`` are optional package selection arguments for ``colcon`` to limit which packages are built and run)

:ref:`Sourcing the workspace <colcon-tutorial-source-the-environment>` before testing should not be necessary.
``colcon test`` makes sure that the tests run with the right environment, have access to their dependencies, etc.

Examine Test Results
^^^^^^^^^^^^^^^^^^^^

To see the results, simply run the `test-result <https://colcon.readthedocs.io/en/released/reference/verb/test-result.html>`__ verb from ``colcon``.

.. code-block:: console

  colcon test-result --all

To see the exact test cases which fail, use the ``--verbose`` flag:

.. code-block:: console

  colcon test-result --all --verbose

Debugging tests with GDB
^^^^^^^^^^^^^^^^^^^^^^^^

If a C++ test is failing, gdb can be used directly on the test executable in the build directory.
Ensure to build the code in debug mode.
Since the previous build type may be cached by CMake, clean the cache and rebuild.

.. code-block:: console

  colcon build --cmake-clean-cache --mixin debug

Next, run the test directly through gdb.
For example:

.. code-block:: console

  gdb -ex run ./build/rcl/test/test_logging
