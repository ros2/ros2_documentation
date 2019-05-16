Creating an Action
==================

In this tutorial we look how to define an action in a ROS package.


Defining an Action
------------------

Identical to ROS 1, actions are be defined with ``.action`` files of the form:

.. code-block:: bash

    # Request
    ---
    # Result
    ---
    # Feedback

An action defintion is made up of three message definitions separated by ``---``.
An instance of an action is typically referred to as a *goal*.
A *request* message is sent from an action client to an action server initiating a new goal.
A *result* message is sent from an action server to an action client when a goal is done.
*Feedback* messages are periodically sent from an action server to an action client to updates about a goal.

Say we want to define a new action "Fibonacci" for computing the `Fibonacci sequence <https://en.wikipedia.org/wiki/Fibonacci_number>`__.
Let's start by creating a new package ``actions_tutorial`` in a colcon workspace:

.. code-block:: bash

    mkdir -p action_ws/src
    cd action_ws/src
    ros2 pkg create action_tutorial

Next we add the file ``action/Fibonacci.action`` with the following content:

.. code-block:: bash

    int32 order
    ---
    int32[] sequence
    ---
    int32[] sequence

The goal request is the order of the Fibonacci sequence we want to compute, the result is the final sequence, and the feedback is the sequence computed so far.

Similar to ROS messages and services, we need to generate language specific types for this action by passing our Fibonacci action defintition to the rosidl pipeline.
This is accomplished by adding the following to our ``CMakeLists.txt``:

.. code-block:: cmake

    find_package(action_msgs REQUIRED)
    find_package(rosidl_default_generators REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
      "action/Fibonacci.action"
      DEPENDENCIES action_msgs
    )

We should also add the required dependencies to our ``package.xml``:

.. code-block:: xml

    <buildtool_depend>rosidl_default_generators</buildtool_depend>

    <depend>action_msgs</depend>

    <!-- We need to use package format 3 for this tag -->
    <member_of_group>rosidl_interface_packages</member_of_group>

Note, we need to depend on ``action_msgs`` since action definitions include additional metadata (e.g. goal IDs).

We're done! We should now be able to build the package containing the "Fibonacci" action definition.

.. code-block:: bash

    # Change to the root of the workspace
    cd ..
    # Build
    colcon build

