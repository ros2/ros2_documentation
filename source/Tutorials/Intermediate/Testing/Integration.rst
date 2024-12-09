Writing Basic Integration Tests with launch_testing
===================================================

**Goal:** Create and run integration tests on the ROS 2 turtlesim node.

**Tutorial level:** Intermediate

**Time:** 20 minutes

.. contents:: Contents
   :depth: 2
   :local:

Prerequisites
-------------

Before starting this tutorial, it is recommended to have completed the following tutorials on launching nodes:

* :doc:`Launching Multiple Nodes <../../Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes>`
* :doc:`Creating Launch files <../../Intermediate/Launch/Creating-Launch-Files>`

Background
----------

Where unit tests focus on validating a very specific piece of functionality, integration tests focus on validating the interaction between pieces of code.
In ROS 2 this is often accomplished by launching a system of one or several nodes, for example the `Gazebo simulator <https://gazebosim.org/home>`__ and the `Nav2 navigation <https://github.com/ros-planning/navigation2.git>`__ stack.
As a result, these tests are more complex both to set up and to run.

A key aspect of ROS 2 integration testing is that nodes that are part of different tests shouldn't communicate with each other, even when run in parallel.
This will be achieved here using a specific test runner that picks unique :doc:`ROS domain IDs <../../../Concepts/Intermediate/About-Domain-ID>`.
In addition, integration tests have to fit in the overall testing workflow.
A standardized approach is to ensure each test outputs an XUnit file, which are easily parsed using common test tooling.

Overview
--------

The main tool in use here is the `launch_testing <https://docs.ros.org/en/{DISTRO}/p/launch_testing/index.html>`_ package
(`launch_testing repository <https://github.com/ros2/launch/tree/{REPOS_FILE_BRANCH}/launch_testing>`_).
This ROS-agnostic functionality can extend a Python launch file with both active tests (that run while the nodes are also running) and post-shutdown tests (which run once after all nodes have exited).
``launch_testing`` relies on the Python standard module `unittest <https://docs.python.org/3/library/unittest.html>`_ for the actual testing.
To get our integration tests run as part of ``colcon test``, we register the launch file in the ``CMakeLists.txt``.

Steps
-----

1 Describe the test in the test launch file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Both the nodes under test and the tests themselves are launched using a Python launch file, which resembles a ROS 2 Python launch file.
It is customary to make the integration test launch file names follow the pattern ``test/test_*.py``.

There are two common types of tests in integration testing: active tests, which run while the nodes under test are running, and post-shutdown tests, which are run after exiting the nodes.
We will cover both in this tutorial.

1.1 Imports
~~~~~~~~~~~

We first start by importing the Python modules we will be using.
Only two modules are specific to testing: the general-purpose ``unittest``, and ``launch_testing``.

.. code-block:: python

  import os
  import sys
  import time
  import unittest

  import launch
  import launch_ros
  import launch_testing.actions
  import rclpy
  from turtlesim.msg import Pose

1.2 Generate the test description
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The function ``generate_test_description`` describes what to launch, similar to ``generate_launch_description`` in a ROS 2 Python launch file.
In the example below, we launch the turtlesim node and half a second later our tests.

In more complex integration test setups, you will probably want to launch a system of several nodes, together with additional nodes that perform mocking or must otherwise interact with the nodes under test.

.. code-block:: python

  def generate_test_description():
      return (
          launch.LaunchDescription(
              [
                  # Nodes under test
                  launch_ros.actions.Node(
                      package='turtlesim',
                      namespace='',
                      executable='turtlesim_node',
                      name='turtle1',
                  ),
                  # Launch tests 0.5 s later
                  launch.actions.TimerAction(
                      period=0.5, actions=[launch_testing.actions.ReadyToTest()]),
              ]
          ), {},
      )

1.3 Active tests
~~~~~~~~~~~~~~~~

The active tests interact with the running nodes.
In this tutorial, we will check whether the turtlesim node publishes pose messages (by listening to the node's 'turtle1/pose' topic) and whether it logs that it spawned the turtle (by listening to stderr).

The active tests are defined as methods of a class inheriting from `unittest.TestCase <https://docs.python.org/3/library/unittest.html#unittest.TestCase>`_.
The child class, here ``TestTurtleSim``, contains the following methods:

- ``test_*``: the test methods, each performing some ROS communication with the nodes under test and/or listening to the process output (passed in through ``proc_output``).
  They are executed sequentially.
- ``setUp``, ``tearDown``: respectively run before (to prepare the test fixture) and after executing each test method.
  By creating the node in the ``setUp`` method, we use a different node instance for each test to reduce the risk of tests communicating with each other.
- ``setUpClass``, ``tearDownClass``: these class methods respectively run once before and after executing all the test methods.

It's highly recommended to go through `launch_testing's detailed documentation on this topic <https://docs.ros.org/en/{DISTRO}/p/launch_testing/index.html>`_.

.. code-block:: python

  # Active tests
  class TestTurtleSim(unittest.TestCase):
      @classmethod
      def setUpClass(cls):
          rclpy.init()

      @classmethod
      def tearDownClass(cls):
          rclpy.shutdown()

      def setUp(self):
          self.node = rclpy.create_node('test_turtlesim')

      def tearDown(self):
          self.node.destroy_node()

      def test_publishes_pose(self, proc_output):
          """Check whether pose messages published"""
          msgs_rx = []
          sub = self.node.create_subscription(
              Pose, 'turtle1/pose',
              lambda msg: msgs_rx.append(msg), 100)
          try:
              # Listen to the pose topic for 10 s
              end_time = time.time() + 10
              while time.time() < end_time:
                  # spin to get subscriber callback executed
                  rclpy.spin_once(self.node, timeout_sec=1)
              # There should have been 100 messages received
              assert len(msgs_rx) > 100
          finally:
              self.node.destroy_subscription(sub)

      def test_logs_spawning(self, proc_output):
          """Check whether logging properly"""
          proc_output.assertWaitFor(
              'Spawning turtle [turtle1] at x=',
              timeout=5, stream='stderr')

Note that the way we listen to the 'turtle1/pose' topic in ``test_publishes_pose`` differs from :doc:`the usual approach <../../Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber>`.
Instead of calling the blocking ``rclpy.spin``, we trigger the ``spin_once`` method - which executes the first available callback (our subscriber callback if a message arrived within 1 second) - until we have gathered all messages published over the last 10 seconds.
The package `launch_testing_ros <https://docs.ros.org/en/{DISTRO}/p/launch_testing_ros/index.html>`_ provides some convenience functions to achieve similar behavior,
such as `WaitForTopics <https://docs.ros.org/en/{DISTRO}/p/launch_testing_ros/launch_testing_ros.wait_for_topics.html>`_.

If you want to go further, you can implement a third test that publishes a twist message, asking the turtle to move, and subsequently checks that it moved by asserting that the pose message changed.
This effectively automates part of the `Turtlesim introduction tutorial <../../Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim>`.

1.4 Post-shutdown tests
~~~~~~~~~~~~~~~~~~~~~~~

The classes marked with the ``launch_testing.post_shutdown_test`` decorator are run after letting the nodes under test exit.
A typical test here is whether the nodes exited cleanly, for which ``launch_testing`` provides the method
`asserts.assertExitCodes <https://docs.ros.org/en/{DISTRO}/p/launch_testing/launch_testing.asserts.html#launch_testing.asserts.assertExitCodes>`_.

.. code-block:: python

  # Post-shutdown tests
  @launch_testing.post_shutdown_test()
  class TestTurtleSimShutdown(unittest.TestCase):
      def test_exit_codes(self, proc_info):
          """Check if the processes exited normally."""
          launch_testing.asserts.assertExitCodes(proc_info)

2 Register the test in the CMakeLists.txt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Registering the test in the ``CMakeLists.txt`` fulfills two functions:

- it integrates it in the ``CTest`` framework ROS 2 CMake-based packages rely on
  (and hence it will be called when running ``colcon test``).
- it allows to specify *how* the test is to be run -
  in this case, with a unique domain id to ensure test isolation.

This latter aspect is realized using the special test runner `run_test_isolated.py <https://github.com/ros2/ament_cmake_ros/blob/{REPOS_FILE_BRANCH}/ament_cmake_ros/cmake/run_test_isolated.py>`_.
To ease adding several integration tests, we define the CMake function ``add_ros_isolated_launch_test`` such that each additional test requires only a single line.

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.8)
  project(app)

  ########
  # test #
  ########

  if(BUILD_TESTING)
    # Integration tests
    find_package(ament_cmake_ros REQUIRED)
    find_package(launch_testing_ament_cmake REQUIRED)
    function(add_ros_isolated_launch_test path)
      set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
      add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
    endfunction()
    add_ros_isolated_launch_test(test/test_integration.py)
  endif()

3 Dependencies and package organization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Finally, add the following dependencies to your ``package.xml``:

.. code-block:: XML

  <test_depend>ament_cmake_ros</test_depend>
  <test_depend>launch</test_depend>
  <test_depend>launch_ros</test_depend>
  <test_depend>launch_testing</test_depend>
  <test_depend>launch_testing_ament_cmake</test_depend>
  <test_depend>rclpy</test_depend>
  <test_depend>turtlesim</test_depend>

After following the above steps, your package (here named 'app') ought to look as follows:

.. code-block::

  app/
    CMakeLists.txt
    package.xml
    tests/
        test_integration.py

Integration tests can be part of any ROS package.
One can dedicate one or more packages to just integration testing, or alternatively add them to the package of which they test the functionality.
In this tutorial, we go with the first option as we will test the existing turtlesim node.

4 Running tests and report generation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For running the integration test and examining the results, see the tutorial :doc:`Running Tests in ROS 2 from the Command Line<../../Intermediate/Testing/CLI>`.

Summary
-------

In this tutorial, we explored the process of creating and running integration tests on the ROS 2 turtlesim node.
We discussed the integration test launch file and covered writing active tests and post-shutdown tests.
To recap, the four key elements of the integration test launch file are:

* The function ``generate_test_description``: This launches our nodes under tests as well as our tests.
* ``launch_testing.actions.ReadyToTest()``: This alerts the test framework that the tests should be run, and ensures that the active tests and the nodes are run together.
* An undecorated class inheriting from ``unittest.TestCase``: This houses the active tests, including set up and teardown, and gives access to ROS logging through ``proc_output``.
* A second class inheriting from ``unittest.TestCase`` decorated with ``@launch_testing.post_shutdown_test()``: These are tests that run after all nodes have shutdown; it is common to assert that the nodes exited cleanly.

The launch test is subsequently registered in the ``CMakeLists.txt`` using the custom cmake macro ``add_ros_isolated_launch_test`` which ensures that each launch test runs with a unique ``ROS_DOMAIN_ID``,
avoiding undesired cross communication.

Related content
---------------

* :doc:`Why automatic tests? <../../Intermediate/Testing/Testing-Main>`
* :doc:`C++ unit testing with GTest <../../Intermediate/Testing/Cpp>`
  and :doc:`Python unit testing with Pytest <../../Intermediate/Testing/Python>`
* `launch_pytest documentation <https://docs.ros.org/en/{DISTRO}/p/launch_pytest/index.html>`_,
  an alternative launch integration testing package to ``launch_testing``
