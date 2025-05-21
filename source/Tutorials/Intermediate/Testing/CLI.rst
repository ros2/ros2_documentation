.. TestingCLI:

Running Tests in ROS 2 from the Command Line
============================================

Prerequisites
^^^^^^^^^^^^^

You will need a workspace setup with packages that have tests in them.

Build and run your tests
^^^^^^^^^^^^^^^^^^^^^^^^

To compile and run the tests, simply run the `test <https://colcon.readthedocs.io/en/released/reference/verb/test.html>`__ verb from ``colcon`` at the root of your workspace.

.. code-block:: console

  $ colcon test --ctest-args tests [package_selection_args]

Where ``package_selection_args`` are optional package selection arguments for ``colcon`` to limit which packages are built and run.
Find more info in the `colcon documentation on Package selection arguments <https://colcon.readthedocs.io/en/released/reference/package-selection-arguments.html>`__

:ref:`Sourcing the workspace <colcon-tutorial-source-the-environment>` before testing should not be necessary.
``colcon test`` makes sure that the tests run with the right environment, have access to their dependencies, etc.

Examine Test Results
^^^^^^^^^^^^^^^^^^^^

To see the results, simply run the `test-result <https://colcon.readthedocs.io/en/released/reference/verb/test-result.html>`__ verb from ``colcon``.

.. code-block:: console

  $ colcon test-result --all

To see the exact test cases which fail, use the ``--verbose`` flag:

.. code-block:: console

  $ colcon test-result --all --verbose

Debugging tests with GDB
^^^^^^^^^^^^^^^^^^^^^^^^

For detailed guidance on debugging tests using GDB, refer to the :doc:`GDB Tutorial <../../../How-To-Guides/Getting-Backtraces-in-ROS-2>`.
