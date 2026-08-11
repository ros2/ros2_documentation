.. redirect-from::

    Tutorials/Intermediate/Testing/CLI

.. meta::
   :contentType: how-to
   :experience: intermediate
   :area: debugging, builds, tools
   :distribution: {DISTRO}
   :product: {PRODUCT}

.. _TestingCLI:

Running Tests in ROS 2 from the Command Line - how-to
=====================================================

.. short-description::
   Running tests from the command line helps verify that ROS packages build correctly and behave as expected.
   This article explains how to run package tests with ``colcon``, examine test results, and find guidance for debugging failing tests.
   After following these steps, you will be able to run tests, filter packages, view failures, and find debugging guidance.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. contents:: Contents
   :depth: 2
   :local:

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

For detailed guidance on debugging tests using GDB, refer to the :doc:`GDB Tutorial <../../Debugging/Getting-Backtraces-in-ROS-2>`.
