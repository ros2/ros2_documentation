Configure action introspection
==============================

**Goal:** Configure action introspection for an action client and an action server.

**Tutorial level:** Advanced

**Time:** 15 minutes

.. contents:: Table of Contents
   :depth: 1
   :local:

Overview
--------

ROS 2 applications usually consist of actions to execute specific long-running procedures or work in remote nodes.
It is possible to introspect action data communication with action introspection.

ROS 2 actions are built on topics and services, so action introspection is based on :doc:`Service Introspection <Service-Introspection>`.

In this demo, we'll be highlighting how to configure action introspection state for an action client and an action server and monitor action communication with ``ros2 action echo``.

Installing the demo
-------------------

See the :doc:`installation instructions <../../Installation>` for details on installing ROS 2.

If you've installed ROS 2 binary packages, ensure that you have ``ros-{DISTRO}-demo-nodes-cpp`` and ``ros-{DISTRO}-demo-nodes-py`` installed.
If you downloaded the archive or built ROS 2 from source, it will already be part of the installation.

Introspection Configuration State
---------------------------------

There are three configuration states for action introspection that are the same states as service introspection.

.. list-table::  Action Introspection Configuration State
   :widths: 25 25

   * - RCL_SERVICE_INTROSPECTION_OFF
     - Disabled
   * - RCL_SERVICE_INTROSPECTION_METADATA
     - Only metadata without any user data contents
   * - RCL_SERVICE_INTROSPECTION_CONTENTS
     - User data contents with metadata

Introspection demo
------------------

This demo shows how to manage action introspection and monitor the action data communication with using ``ros2 action echo``.

Action server
^^^^^^^^^^^^^

You can find the source code here: `fibonacci_action_server.py <https://github.com/ros2/demos/blob/{REPOS_FILE_BRANCH}/action_tutorials/action_tutorials_py/action_tutorials_py/fibonacci_action_server.py>`__.
``FibonacciActionServer`` has a parameter named ``action_server_configure_introspection`` to configure the action introspection state.

.. code-block:: python

    class FibonacciActionServer(Node):
    ...
        def on_post_set_parameters_callback(self, parameter_list):
            for param in parameter_list:
                if param.name != 'action_server_configure_introspection':
                    continue

                introspection_state = ServiceIntrospectionState.OFF
                if param.value == 'disabled':
                    introspection_state = ServiceIntrospectionState.OFF
                elif param.value == 'metadata':
                    introspection_state = ServiceIntrospectionState.METADATA
                elif param.value == 'contents':
                    introspection_state = ServiceIntrospectionState.CONTENTS

                self._action_server.configure_introspection(self.get_clock(),
                                                            qos_profile_system_default,
                                                            introspection_state)
                break
    ...

If you want to try the C++ version, you can find the source code here: `fibonacci_action_server.cpp <https://github.com/ros2/demos/blob/{REPOS_FILE_BRANCH}/action_tutorials/action_tutorials_cpp/src/fibonacci_action_server.cpp>`__.

Action introspection is disabled by default, so users need to enable it with ``action_server_configure_introspection`` parameter when the server starts up.
In this demo, ``FibonacciActionServer`` enables action introspection when the value of the ``action_server_configure_introspection`` parameter is ``contents``.

.. code-block:: console

    $ ros2 run action_tutorials_py fibonacci_action_server --ros-args -p action_server_configure_introspection:=contents

To change action introspection state, we need to set the ``action_server_configure_introspection`` parameter as follows.

To change it to user data contents with metadata:

.. code-block:: console

    $ ros2 param set /fibonacci_action_server action_server_configure_introspection contents

To change it to only metadata:

.. code-block:: console

    $ ros2 param set /fibonacci_action_server action_server_configure_introspection metadata

To disable:

.. code-block:: console

    $ ros2 param set /fibonacci_action_server action_server_configure_introspection disabled

Action client
^^^^^^^^^^^^^

You can find the source code here: `fibonacci_action_client.cpp <https://github.com/ros2/demos/blob/{REPOS_FILE_BRANCH}/action_tutorials/action_tutorials_cpp/src/fibonacci_action_client.cpp>`__.
``FibonacciActionClient`` has a parameter named ``action_client_configure_introspection`` to configure the action introspection state.

.. code-block:: c++

    namespace action_tutorials_cpp
    {
    class FibonacciActionClient : public rclcpp::Node
    ...
        auto post_set_parameter_callback =
          [this](const std::vector<rclcpp::Parameter> & parameters) {
            for (const rclcpp::Parameter & param : parameters) {
              if (param.get_name() != "action_client_configure_introspection") {
                continue;
              }

              rcl_service_introspection_state_t introspection_state = RCL_SERVICE_INTROSPECTION_OFF;

              if (param.as_string() == "disabled") {
                introspection_state = RCL_SERVICE_INTROSPECTION_OFF;
              } else if (param.as_string() == "metadata") {
                introspection_state = RCL_SERVICE_INTROSPECTION_METADATA;
              } else if (param.as_string() == "contents") {
                introspection_state = RCL_SERVICE_INTROSPECTION_CONTENTS;
              }

              this->client_ptr_->configure_introspection(
                this->get_clock(), rclcpp::SystemDefaultsQoS(), introspection_state);
              break;
            }
          };
    ...

If you want to try the Python version, you can find the source code here: `fibonacci_action_client.py <https://github.com/ros2/demos/blob/{REPOS_FILE_BRANCH}/action_tutorials/action_tutorials_py/action_tutorials_py/fibonacci_action_client.py>`__.

And then, we start and configure ``FibonacciActionClient`` in the same way.

.. code-block:: console

    $ ros2 run action_tutorials_cpp fibonacci_action_client --ros-args -p action_client_configure_introspection:=contents

To change action introspection state, we need to set the ``action_client_configure_introspection`` parameter as follows.
Note that ``FibonacciActionClient`` only runs in short time, so it is recommended to set the parameter before running the client as above.

To change it to user data contents with metadata:

.. code-block:: console

    $ ros2 param set /fibonacci_action_client action_client_configure_introspection contents

To change it to only metadata:

.. code-block:: console

    $ ros2 param set /fibonacci_action_client action_client_configure_introspection metadata

To disable:

.. code-block:: console

    $ ros2 param set /fibonacci_action_client action_client_configure_introspection disabled

Introspect
^^^^^^^^^^

In this tutorial, the following is an example output with action introspection state ``contents`` on both ``FibonacciActionServer`` and ``FibonacciActionClient``.
To monitor action communication between ``FibonacciActionServer`` and ``FibonacciActionClient``, let's run it:

.. code-block:: console

    $ ros2 action echo /fibonacci example_interfaces/action/Fibonacci --flow-style
    interface: GOAL_SERVICE
    info:
      event_type: REQUEST_SENT
      stamp:
        sec: 1742070798
        nanosec: 400435819
      client_gid: [1, 15, 165, 231, 194, 197, 167, 157, 0, 0, 0, 0, 0, 0, 20, 4]
      sequence_number: 1
    request: [{goal_id: {uuid: [230, 96, 12, 6, 100, 69, 69, 70, 220, 205, 135, 251, 210, 2, 231, 110]}, goal: {order: 10}}]
    response: []
    ---
    interface: GOAL_SERVICE
    info:
      event_type: REQUEST_RECEIVED
      stamp:
        sec: 1742070798
        nanosec: 400706446
      client_gid: [1, 15, 165, 231, 194, 197, 167, 157, 0, 0, 0, 0, 0, 0, 20, 4]
      sequence_number: 1
    request: [{goal_id: {uuid: [230, 96, 12, 6, 100, 69, 69, 70, 220, 205, 135, 251, 210, 2, 231, 110]}, goal: {order: 10}}]
    response: []
    ---
    interface: RESULT_SERVICE
    info:
      event_type: REQUEST_SENT
      stamp:
        sec: 1742070798
        nanosec: 401486678
      client_gid: [1, 15, 165, 231, 194, 197, 167, 157, 0, 0, 0, 0, 0, 0, 24, 4]
      sequence_number: 1
    request: [{goal_id: {uuid: [230, 96, 12, 6, 100, 69, 69, 70, 220, 205, 135, 251, 210, 2, 231, 110]}}]
    response: []
    ---
    interface: FEEDBACK_TOPIC
    goal_id:
      uuid: [230, 96, 12, 6, 100, 69, 69, 70, 220, 205, 135, 251, 210, 2, 231, 110]
    feedback:
      sequence: [0, 1, 1]
    ---
    interface: STATUS_TOPIC
    status_list: [{goal_info: {goal_id: {uuid: [230, 96, 12, 6, 100, 69, 69, 70, 220, 205, 135, 251, 210, 2, 231, 110]}, stamp: {sec: 1742070798, nanosec: 401146752}}, status: 2}]
    ---
    interface: GOAL_SERVICE
    info:
      event_type: RESPONSE_SENT
      stamp:
        sec: 1742070798
        nanosec: 401109161
      client_gid: [1, 15, 165, 231, 194, 197, 167, 157, 0, 0, 0, 0, 0, 0, 20, 4]
      sequence_number: 1
    request: []
    response: [{accepted: true, stamp: {sec: 0, nanosec: 0}}]
    ---
    interface: RESULT_SERVICE
    info:
      event_type: REQUEST_RECEIVED
      stamp:
        sec: 1742070798
        nanosec: 401579875
      client_gid: [1, 15, 165, 231, 194, 197, 167, 157, 0, 0, 0, 0, 0, 0, 24, 4]
      sequence_number: 1
    request: [{goal_id: {uuid: [230, 96, 12, 6, 100, 69, 69, 70, 220, 205, 135, 251, 210, 2, 231, 110]}}]
    response: []
    ---
    interface: GOAL_SERVICE
    info:
      event_type: RESPONSE_RECEIVED
      stamp:
        sec: 1742070798
        nanosec: 401234269
      client_gid: [1, 15, 165, 231, 194, 197, 167, 157, 0, 0, 0, 0, 0, 0, 20, 4]
      sequence_number: 1
    request: []
    response: [{accepted: true, stamp: {sec: 0, nanosec: 0}}]
    ---
    interface: FEEDBACK_TOPIC
    goal_id:
      uuid: [230, 96, 12, 6, 100, 69, 69, 70, 220, 205, 135, 251, 210, 2, 231, 110]
    feedback:
      sequence: [0, 1, 1, 2]
    ---
    interface: FEEDBACK_TOPIC
    goal_id:
      uuid: [230, 96, 12, 6, 100, 69, 69, 70, 220, 205, 135, 251, 210, 2, 231, 110]
    feedback:
      sequence: [0, 1, 1, 2, 3]
    ---
    interface: FEEDBACK_TOPIC
    goal_id:
      uuid: [230, 96, 12, 6, 100, 69, 69, 70, 220, 205, 135, 251, 210, 2, 231, 110]
    feedback:
      sequence: [0, 1, 1, 2, 3, 5]
    ---
    interface: FEEDBACK_TOPIC
    goal_id:
      uuid: [230, 96, 12, 6, 100, 69, 69, 70, 220, 205, 135, 251, 210, 2, 231, 110]
    feedback:
      sequence: [0, 1, 1, 2, 3, 5, 8]
    ---
    interface: FEEDBACK_TOPIC
    goal_id:
      uuid: [230, 96, 12, 6, 100, 69, 69, 70, 220, 205, 135, 251, 210, 2, 231, 110]
    feedback:
      sequence: [0, 1, 1, 2, 3, 5, 8, 13]
    ---
    interface: FEEDBACK_TOPIC
    goal_id:
      uuid: [230, 96, 12, 6, 100, 69, 69, 70, 220, 205, 135, 251, 210, 2, 231, 110]
    feedback:
      sequence: [0, 1, 1, 2, 3, 5, 8, 13, 21]
    ---
    interface: FEEDBACK_TOPIC
    goal_id:
      uuid: [230, 96, 12, 6, 100, 69, 69, 70, 220, 205, 135, 251, 210, 2, 231, 110]
    feedback:
      sequence: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34]
    ---
    interface: FEEDBACK_TOPIC
    goal_id:
      uuid: [230, 96, 12, 6, 100, 69, 69, 70, 220, 205, 135, 251, 210, 2, 231, 110]
    feedback:
      sequence: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]
    ---
    interface: RESULT_SERVICE
    info:
      event_type: RESPONSE_SENT
      stamp:
        sec: 1742070807
        nanosec: 402339670
      client_gid: [1, 15, 165, 231, 194, 197, 167, 157, 0, 0, 0, 0, 0, 0, 24, 4]
      sequence_number: 1
    request: []
    response: [{status: 4, result: {sequence: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]}}]
    ---
    interface: RESULT_SERVICE
    info:
      event_type: RESPONSE_RECEIVED
      stamp:
        sec: 1742070807
        nanosec: 402698784
      client_gid: [1, 15, 165, 231, 194, 197, 167, 157, 0, 0, 0, 0, 0, 0, 24, 4]
      sequence_number: 1
    request: []
    response: [{status: 4, result: {sequence: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]}}]
    ---
    interface: STATUS_TOPIC
    status_list: [{goal_info: {goal_id: {uuid: [230, 96, 12, 6, 100, 69, 69, 70, 220, 205, 135, 251, 210, 2, 231, 110]}, stamp: {sec: 1742070798, nanosec: 401146752}}, status: 4}]
    ---
    ...

You can see the ``interface: GOAL_SERVICE`` and ``interface: RESULT_SERVICE``, those introspection service events take place in both ``FibonacciActionServer`` and ``FibonacciActionClient``.
And you can also see ``interface: FEEDBACK_TOPIC`` and ``interface: STATUS_TOPIC`` data, those topics are published by ``FibonacciActionServer`` and subscribed by ``FibonacciActionClient``.

Now you can see the action communication between ``FibonacciActionServer`` and ``FibonacciActionClient`` with ``ros2 action echo``.

Related content
---------------

- `Action Introspection REP-2018 <https://github.com/ros-infrastructure/rep/pull/405>`__.
