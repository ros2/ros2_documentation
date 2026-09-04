.. redirect-from::

    Tutorials/Intermediate/Testing/Integration

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

* :doc:`Launching Multiple Nodes <../../../../ROS-Framework/nodes/Working-with-nodes/Launching-Multiple-Nodes/Launching-Multiple-Nodes>`
* :doc:`Creating Launch files <../../Launch/Creating-Launch-Files>`

Background
----------

Where unit tests focus on validating a very specific piece of functionality, integration tests focus on validating the interaction between pieces of code.
In ROS 2 this is often accomplished by launching a system of one or several nodes, for example the `Gazebo simulator <https://gazebosim.org/home>`__ and the `Nav2 navigation <https://github.com/ros-planning/navigation2.git>`__ stack.
As a result, these tests are more complex both to set up and to run.

A key aspect of ROS 2 integration testing is that nodes that are part of different tests shouldn't communicate with each other, even when run in parallel.
This will be achieved here using a specific test runner that picks unique :doc:`ROS domain IDs <../../../../ROS-Framework/nodes/About-Domain-ID>`.
In addition, integration tests have to fit in the overall testing workflow.
A standardized approach is to ensure each test outputs an XUnit file, which are easily parsed using common test tooling.

Overview
--------

The main tool in use here is the `launch_testing <https://docs.ros.org/en/{DISTRO}/p/launch_testing/index.html>`_ package
(`launch_testing repository <https://github.com/ros2/launch/tree/{REPOS_FILE_BRANCH}/launch_testing>`_).
This ROS-agnostic functionality can extend a Python launch file with both active tests (that run while the nodes are also running) and post-shutdown tests (which run once after all nodes have exited).
``launch_testing`` relies on the Python standard module `unittest <https://docs.python.org/3/library/unittest.html>`_ for the actual testing.
To get our integration tests run as part of ``colcon test``, we register the launch file in the ``CMakeLists.txt`` or ``setup.py`` file.

For waiting on topics and triggering actions based on publisher availability, the `launch_testing_ros <https://docs.ros.org/en/{DISTRO}/p/launch_testing_ros/index.html>`_ package provides the `WaitForTopics <https://docs.ros.org/en/{DISTRO}/p/launch_testing_ros/launch_testing_ros.wait_for_topics.html>`_ utility, which simplifies topic subscription and waiting logic in integration tests.

Steps
-----

1 Describe the test in the test launch file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Both the nodes under test and the tests themselves are launched using a Python launch file, which resembles a ROS 2 Python launch file.
It is customary to make the integration test launch file names follow the pattern ``test/test_*.py``.

There are two common types of tests in integration testing: active tests, which run while the nodes under test are running, and post-shutdown tests, which are run after exiting the nodes.
We will cover both in this tutorial.

1.1 Imports
^^^^^^^^^^^

We first start by importing the Python modules we will be using.
Key modules for testing include the general-purpose ``unittest``, ``launch_testing``, and the ``WaitForTopics`` utility from ``launch_testing_ros`` for convenient topic subscription and waiting logic.

.. code-block:: python

   import math
   import threading
   import unittest
   from typing import Any

   import launch
   import launch_ros.actions
   import launch_testing
   import launch_testing.actions
   import launch_testing.markers
   import pytest
   from geometry_msgs.msg import Twist
   from launch_testing_ros import WaitForTopics
   from launch_testing_ros.actions import EnableRmwIsolation
   from rclpy.node import Node
   from rclpy.publisher import PublisherEventCallbacks
   from turtlesim_msgs.msg import Pose

1.2 Generate the test description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The function ``generate_test_description`` describes what to launch, similar to ``generate_launch_description`` in a ROS 2 Python launch file.
In the example below, we launch the turtlesim node with immediate test execution (no arbitrary delays).

The ``EnableRmwIsolation`` action ensures that ROS communication is isolated using ``rmw_test_fixture``, preventing test interference.
The ``ReadyToTest`` action signals the test framework that the tests should begin.

.. code-block:: python

   @pytest.mark.launch_test
   @launch_testing.markers.keep_alive
   def generate_test_description() -> launch.LaunchDescription:
       """Create the launch description for turtlesim integration tests.

       Returns:
           launch.LaunchDescription: Launch actions that start turtlesim and
               signal when tests can begin.
       """
       return launch.LaunchDescription(
           [
               # Action which enables isolation of ROS communication using rmw_test_fixture
               EnableRmwIsolation(),
               # Node under test: turtlesim_node
               launch_ros.actions.Node(
                   package="turtlesim",
                   namespace="",
                   executable="turtlesim_node",
                   name="turtle1",
                   output="screen",
               ),
               launch_testing.actions.ReadyToTest(),
           ]
       )

In more complex integration test setups, you will probably want to launch a system of several nodes, together with additional nodes that perform mocking or must otherwise interact with the nodes under test.

1.3 Active tests using WaitForTopics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The active tests interact with the running nodes.
The ``WaitForTopics`` utility from ``launch_testing_ros`` provides a convenient way to:

* Subscribe to topics and wait for them to become available
* Receive messages published on those topics
* Optionally trigger actions (like publishing control messages)

1.3.1 Basic topic subscription test
"""""""""""""""""""""""""""""""""""

The simplest test verifies that a topic is published and messages are received:

.. code-block:: python

   class TestTurtleSimWithWaitForTopics(unittest.TestCase):
       def test_publishes_pose_with_wait_for_topics(self) -> None:
           """Verify pose messages are received for turtlesim."""
           with WaitForTopics([("turtle1/pose", Pose)]) as waiter:
               assert waiter.topics_received() == {"turtle1/pose"}
               assert len(waiter.received_messages("turtle1/pose")) >= 1

This test creates a waiter that subscribes to the ``turtle1/pose`` topic expecting ``Pose`` messages.
The ``WaitForTopics`` class automatically handles subscription and cleanup.
The test asserts that the topic was received and at least one message was captured.

1.3.2 Topic subscription with triggered action
"""""""""""""""""""""""""""""""""""""""""""""""

For more complex tests, you can trigger actions (such as publishing control messages) using the ``trigger`` parameter.
This allows you to verify that nodes respond appropriately to a stimulus.

First, define a trigger function that will be called once publishers are available:

.. code-block:: python

   def trigger_publish_twist(node: Node, linear_x: float, angular_z: float) -> None:
       """Publish Twist commands to trigger turtlesim pose updates.

       Args:
           node: rclpy node used to create and reuse the cmd_vel publisher.
           linear_x: Linear x velocity in m/s
           angular_z: Angular z velocity in rad/s

       Returns:
           None: Publishes messages as a side effect.
       """
       matched_event = threading.Event()

       def on_subscriber_matched(info: Any) -> None:
           if info.current_count > 0:
               matched_event.set()

       if hasattr(node, "cmd_vel_publisher"):
           node.destroy_publisher(node.cmd_vel_publisher)

       node.cmd_vel_publisher = node.create_publisher(
           Twist,
           "turtle1/cmd_vel",
           10,
           event_callbacks=PublisherEventCallbacks(matched=on_subscriber_matched),
       )

       if not matched_event.wait(timeout=5.0):
           raise RuntimeError("Timed out waiting for turtlesim cmd_vel subscriber")

       # Publish a Twist message to move the turtle
       msg = Twist()
       msg.linear.x = float(linear_x)
       msg.angular.z = float(angular_z)
       node.cmd_vel_publisher.publish(msg)

Then use this trigger function in your test:

.. code-block:: python

   def test_moves_with_triggered_twist(self) -> None:
       """Verify turtle motion after triggering Twist publication."""
       waiter = WaitForTopics([("turtle1/pose", Pose)], trigger=trigger_publish_twist)
       try:
           while True:
               # Wait for the turtle to move by publishing a Twist message with arguments
               # linear_x = 10 m/s and angular_z = 2*pi rad/s
               assert waiter.wait(linear_x=10, angular_z=2 * math.pi)
               assert waiter.topics_received() == {"turtle1/pose"}
               poses = waiter.received_messages("turtle1/pose")
               assert len(poses) >= 1
               # Check that at least one of the received poses indicates motion compatible with the
               # published Twist command
               if any(
                   math.isclose(p.linear_velocity, 10.0, rel_tol=1e-2)
                   and math.isclose(p.angular_velocity, 2 * math.pi, rel_tol=1e-2)
                   for p in poses
               ):
                   break
       finally:
           waiter.shutdown()

This test demonstrates:

* Creating a publisher and waiting for subscribers to be available
* Using ``WaitForTopics`` with a trigger function to publish messages and wait for responses
* Verifying that received messages match expected values

The while loop is necessary because the ``turtlesim`` node is constantly publishing pose messages, and
we want to wait until we receive a pose that reflects the motion commanded by our published ``Twist``
message.

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

  cmake_minimum_required(VERSION 3.20)
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
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Finally, add the following dependencies to your ``package.xml``:

.. code-block:: XML

  <test_depend>ament_cmake_ros</test_depend>
  <test_depend>launch</test_depend>
  <test_depend>launch_ros</test_depend>
  <test_depend>launch_testing</test_depend>
  <test_depend>launch_testing_ament_cmake</test_depend>
  <test_depend>rclpy</test_depend>
  <test_depend>turtlesim</test_depend>
  <test_depend>turtlesim_msgs</test_depend>

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

4 Register the test in setup.py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For Python-based packages, tests are automatically discovered when placed in a ``test`` directory.

Configure pytest discovery in ``setup.cfg``:

.. code-block:: ini

   [tool:pytest]
   testpaths = test
   python_files = test_*.py
   markers =
       launch_test: launch testing integration tests

To run the tests, you can use the command:

.. code-block:: bash

    colcon test --packages-select your_package_name --python-testing pytest



5 Running tests and report generation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For running the integration test and examining the results, see the tutorial :doc:`Running Tests in ROS 2 from the Command Line <CLI>`.

Summary
-------

In this tutorial, we explored the process of creating and running integration tests on the ROS 2 turtlesim node.
We discussed the integration test launch file and covered writing active tests and post-shutdown tests.
To recap, the four key elements of the integration test launch file are:

* The function ``generate_test_description``: This launches our nodes under test as well as our tests.
* ``launch_testing.actions.ReadyToTest()``: This alerts the test framework that the tests should be run, without arbitrary delays.
* The ``WaitForTopics`` utility: This provides a convenient way to subscribe to topics, wait for messages, and optionally trigger actions like publishing control messages based on publisher availability.
* An undecorated class inheriting from ``unittest.TestCase``: This houses the active tests, and gives access to ROS logging through ``proc_output``.
* A second class inheriting from ``unittest.TestCase`` decorated with ``@launch_testing.post_shutdown_test()``: These are tests that run after all nodes have shutdown; it is common to assert that the nodes exited cleanly.

The launch test is subsequently registered in the ``CMakeLists.txt`` using the custom cmake macro ``add_ros_isolated_launch_test`` which ensures that each launch test runs with a unique ``ROS_DOMAIN_ID``,
avoiding undesired cross communication.

Related content
---------------

* :doc:`Why automatic tests? <../About-testing>`
* :doc:`C++ unit testing with GTest <Cpp>`
  and :doc:`Python unit testing with Pytest <Python>`
* `launch_pytest documentation <https://docs.ros.org/en/{DISTRO}/p/launch_pytest/index.html>`_,
  an alternative launch integration testing package to ``launch_testing``
* `WaitForTopics documentation <https://docs.ros.org/en/{DISTRO}/p/launch_testing_ros/launch_testing_ros.wait_for_topics.html>`_ for more advanced usage patterns
