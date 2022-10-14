.. TestingCLI:

Running Tests in ROS 2 from the Command Line
============================================

Build and run your tests
^^^^^^^^^^^^^^^^^^^^^^^^

To compile and run the tests, simply run the `test <https://colcon.readthedocs.io/en/released/reference/verb/test.html>`__ verb from ``colcon``.

.. code-block:: console

  colcon test --cmake-args tests [package_selection_args]

(where ``package_selection_args`` are optional package selection arguments for ``colcon`` to limit which packages are built and run)

Examine Test Results
^^^^^^^^^^^^^^^^^^^^

To see the results, simply run the `test-result <https://colcon.readthedocs.io/en/released/reference/verb/test-result.html>`__ verb from ``colcon``.

.. code-block:: console

  colcon test-result --all
