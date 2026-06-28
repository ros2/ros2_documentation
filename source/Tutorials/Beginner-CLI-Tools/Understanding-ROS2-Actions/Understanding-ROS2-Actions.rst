.. redirect-from::

    Tutorials/Understanding-ROS2-Actions

.. _ROS2Actions:

Learning about actions - tutorial
==================================

Actions are one of the communication types used for long-duration tasks in a ROS system.
This article walks you through using command-line tools to examine and send action goals.
A hands-on exercise with Turtlesim helps you understand how goals, feedback, and results work together.

**Area: Framework | Content-type: tutorial | Experience: beginner**

.. contents:: Contents
   :depth: 2
   :local:

Summary
-------

Actions provide a way to execute long-duration tasks with continuous feedback and the ability to cancel mid-execution.
An action client sends a goal to an action server.
The action server then acknowledges the goal, streams feedback, and returns a result when done.

Background
----------

Actions are one of the communication types in ROS, and are intended for long-duration tasks.
They consist of three parts: a goal, feedback, and a result.

Actions are built on topics and services.
Their functionality is similar to services, except actions can be cancelled.
They also provide steady feedback, as opposed to services, which return a single response.

Actions use a client-server model.
An "action client" node sends a goal to an "action server" node that acknowledges the goal and returns a stream of feedback and a result.

.. image:: images/Action-SingleActionClient.gif

Prerequisites
-------------

#. Make sure you understand the concepts of :doc:`nodes <../Understanding-ROS2-Nodes/Understanding-ROS2-Nodes>` and :doc:`topics <../Understanding-ROS2-Topics/Understanding-ROS2-Topics>`.
#. You will need the :doc:`turtlesim package <../Introducing-Turtlesim/Introducing-Turtlesim>`.

Steps
-----

.. note::
    Do not forget to source ROS in every new terminal you open.
    See :doc:`Configuring environment <../Configuring-ROS2-Environment>`.

1 Setup
^^^^^^^

Start up the two Turtlesim nodes, ``/turtlesim`` and ``/teleop_turtle``.

Open a new terminal and run:

.. code-block:: console

  $ ros2 run turtlesim turtlesim_node

Open another terminal and run:

.. code-block:: console

  $ ros2 run turtlesim turtle_teleop_key


2 Use actions
^^^^^^^^^^^^^

When you launch the ``/teleop_turtle`` node, you will see the following message in your terminal:

.. code-block:: console

    Use arrow keys to move the turtle.
    Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.

The first line corresponds to the ``cmd_vel`` topic, covered in :doc:`Learning about topics <../Understanding-ROS2-Topics/Understanding-ROS2-Topics>`.
The second line corresponds to an action.

Notice that the letter keys ``G|B|V|C|D|E|R|T`` form a "box" around the ``F`` key on a US QWERTY keyboard (if you are not using a QWERTY keyboard, see `this link <https://upload.wikimedia.org/wikipedia/commons/d/da/KB_United_States.svg>`__ to follow along).
Each key's position around ``F`` corresponds to that orientation in Turtlesim.
For example, the ``E`` key will rotate the turtle's orientation to the upper left corner.

Pay attention to the terminal where the ``/turtlesim`` node is running.
Each time you press one of these keys, you send a goal to an action server that is part of the ``/turtlesim`` node.
The goal is to rotate the turtle to face a particular direction.
Once the turtle completes its rotation, the terminal should display the result:

.. code-block:: console

    [INFO] [turtlesim]: Rotation goal completed successfully

The ``F`` key will cancel a goal mid-execution.

Try pressing the ``C`` key, and then pressing the ``F`` key before the turtle can complete its rotation.
In the terminal where the ``/turtlesim`` node is running, you should see the following message:

.. code-block:: console

  [INFO] [turtlesim]: Rotation goal canceled

Not only can the client side (your input in the teleoperation node) stop a goal, but the server side (the ``/turtlesim`` node) can as well.
When the server side chooses to stop processing a goal, this is known as "aborting" the goal.

Try hitting the ``D`` key, then the ``G`` key before the first rotation can complete.
In the terminal where the ``/turtlesim`` node is running, you should see the message:

.. code-block:: console

  [WARN] [turtlesim]: Rotation goal received before a previous goal finished. Aborting previous goal

This action server chose to abort the first goal because it got a new one.
It could have chosen something else, like reject the new goal or execute the second goal after the first one finished.
Do not assume every action server will choose to abort the current goal when it gets a new one.

3 ros2 node info
^^^^^^^^^^^^^^^^

To see the list of actions a node provides, run:

.. code-block:: console

  $ ros2 node info /turtlesim
  /turtlesim
    Subscribers:
      /parameter_events: rcl_interfaces/msg/ParameterEvent
      /turtle1/cmd_vel: geometry_msgs/msg/Twist
    Publishers:
      /parameter_events: rcl_interfaces/msg/ParameterEvent
      /rosout: rcl_interfaces/msg/Log
      /turtle1/color_sensor: turtlesim_msgs/msg/Color
      /turtle1/pose: turtlesim_msgs/msg/Pose
    Service Servers:
      /clear: std_srvs/srv/Empty
      /kill: turtlesim_msgs/srv/Kill
      /reset: std_srvs/srv/Empty
      /spawn: turtlesim_msgs/srv/Spawn
      /turtle1/set_pen: turtlesim_msgs/srv/SetPen
      /turtle1/teleport_absolute: turtlesim_msgs/srv/TeleportAbsolute
      /turtle1/teleport_relative: turtlesim_msgs/srv/TeleportRelative
      /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
      /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
      /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
      /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
      /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
      /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    Service Clients:

    Action Servers:
      /turtle1/rotate_absolute: turtlesim_msgs/action/RotateAbsolute
    Action Clients:

The command returns a list of ``/turtlesim``'s subscribers, publishers, services, action servers and action clients.

Notice that the ``/turtle1/rotate_absolute`` action for ``/turtlesim`` is under ``Action Servers``.
This means ``/turtlesim`` responds to and provides feedback for the ``/turtle1/rotate_absolute`` action.

The ``/teleop_turtle`` node has the name ``/turtle1/rotate_absolute`` under ``Action Clients``, meaning that it sends goals for that action.
To see that, run:

.. code-block:: console

  $ ros2 node info /teleop_turtle
  /teleop_turtle
    Subscribers:
      /parameter_events: rcl_interfaces/msg/ParameterEvent
    Publishers:
      /parameter_events: rcl_interfaces/msg/ParameterEvent
      /rosout: rcl_interfaces/msg/Log
      /turtle1/cmd_vel: geometry_msgs/msg/Twist
    Service Servers:
      /teleop_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
      /teleop_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
      /teleop_turtle/get_parameters: rcl_interfaces/srv/GetParameters
      /teleop_turtle/list_parameters: rcl_interfaces/srv/ListParameters
      /teleop_turtle/set_parameters: rcl_interfaces/srv/SetParameters
      /teleop_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    Service Clients:

    Action Servers:

    Action Clients:
      /turtle1/rotate_absolute: turtlesim_msgs/action/RotateAbsolute

4 ros2 action list
^^^^^^^^^^^^^^^^^^

To identify all the actions in the ROS graph, run:

.. code-block:: console

  $ ros2 action list
  /turtle1/rotate_absolute

This is the only action in the ROS graph right now.
It controls the turtle's rotation.
From the ``ros2 node info`` output, there is one action client (part of ``/teleop_turtle``) and one action server (part of ``/turtlesim``) for this action.

4.1 ros2 action list -t
~~~~~~~~~~~~~~~~~~~~~~~

Actions have types, similar to topics and services.

To find ``/turtle1/rotate_absolute``'s type, run:

.. code-block:: console

  $ ros2 action list -t
  /turtle1/rotate_absolute [turtlesim_msgs/action/RotateAbsolute]

In brackets to the right of each action name (in this case only ``/turtle1/rotate_absolute``) is the action type, ``turtlesim_msgs/action/RotateAbsolute``.
You will need this when you want to execute an action from the command line or from code.

5 ros2 action type
^^^^^^^^^^^^^^^^^^

To check the type of an action, run:

.. code-block:: console

  $ ros2 action type /turtle1/rotate_absolute
  turtlesim_msgs/action/RotateAbsolute

6 ros2 action info
^^^^^^^^^^^^^^^^^^

To further introspect the ``/turtle1/rotate_absolute`` action, run:

.. code-block:: console

  $ ros2 action info /turtle1/rotate_absolute
  Action: /turtle1/rotate_absolute
  Action clients: 1
      /teleop_turtle
  Action servers: 1
      /turtlesim

This confirms what ``ros2 node info`` showed: the ``/teleop_turtle`` node has an action client and the ``/turtlesim`` node has an action server for the ``/turtle1/rotate_absolute`` action.

7 ros2 interface show
^^^^^^^^^^^^^^^^^^^^^

Before sending an action goal, you need to know the structure of the action type.

From the ``ros2 action list -t`` output, the type of ``/turtle1/rotate_absolute`` is ``turtlesim_msgs/action/RotateAbsolute``.
To see its structure, run:

.. code-block:: console

  $ ros2 interface show turtlesim_msgs/action/RotateAbsolute

The output should look like this:

.. code-block:: text

  # The desired heading in radians
  float32 theta
  ---
  # The angular displacement in radians to the starting position
  float32 delta
  ---
  # The remaining rotation in radians
  float32 remaining

The section before the first ``---`` is the structure of the goal request.
The next section is the structure of the result.
The last section is the structure of the feedback.

8 ros2 action send_goal
^^^^^^^^^^^^^^^^^^^^^^^

To send an action goal from the command line, use:

.. code-block:: console

  $ ros2 action send_goal <action_name> <action_type> <values>

``<values>`` need to be in YAML format.

Keep an eye on the Turtlesim window, then run:

.. code-block:: console

  $ ros2 action send_goal /turtle1/rotate_absolute turtlesim_msgs/action/RotateAbsolute "{theta: 1.57}"
  Waiting for an action server to become available...
  Sending goal:
     theta: 1.57

  Goal accepted with ID: f8db8f44410849eaa93d3feb747dd444

  Result:
    delta: -1.568000316619873

  Goal finished with status: SUCCEEDED


You should see the turtle rotating.

All goals have a unique ID, shown in the return message.
You can also see the result, a field with the name ``delta``, which is the displacement to the starting position.

To see the feedback of this goal, add ``--feedback`` to the ``ros2 action send_goal`` command:

.. code-block:: console

  $ ros2 action send_goal /turtle1/rotate_absolute turtlesim_msgs/action/RotateAbsolute "{theta: -1.57}" --feedback
  Sending goal:
     theta: -1.57

  Goal accepted with ID: e6092c831f994afda92f0086f220da27

  Feedback:
    remaining: -3.1268222332000732

  Feedback:
    remaining: -3.1108222007751465

  …

  Result:
    delta: 3.1200008392333984

  Goal finished with status: SUCCEEDED

You should continue to receive feedback, the remaining radians, until the goal is complete.

.. _understanding-actions-ros2-action-echo:

9 ros2 action echo
^^^^^^^^^^^^^^^^^^

To see the data communication between an action client and an action server, use:

.. code-block:: console

  $ ros2 action echo <action_name> <optional arguments/action_type>

``ros2 action echo`` depends on action introspection, which is disabled by default.
To enable it, call ``configure_introspection`` after creating an action client or server.

Start up the ``fibonacci_action_server`` and ``fibonacci_action_client``, enabling the ``action_server_configure_introspection`` parameter for demonstration:

.. code-block:: console

  $ ros2 run action_tutorials_cpp fibonacci_action_server --ros-args -p action_server_configure_introspection:=contents

.. code-block:: console

  $ ros2 run action_tutorials_py fibonacci_action_client --ros-args -p action_client_configure_introspection:=contents

To see the action communication between ``fibonacci_action_server`` and ``fibonacci_action_client``, run:

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
   ...

.. note::

   This feature is available on ``Kilted Kaiju`` or later.

Related content
---------------

More articles:

* :doc:`Learning about topics <../Understanding-ROS2-Topics/Understanding-ROS2-Topics>`
* :doc:`Learning about services <../Understanding-ROS2-Services/Understanding-ROS2-Services>`
* :doc:`Interfaces (topics, services, actions) </Concepts/Basic/Interfaces-Topics-Services-Actions>`

External resources:

* `Actions design article <https://design.ros2.org/articles/actions.html>`_: background on the design decisions behind actions in ROS.

FAQs
----

What is the difference between an action and a service?
   Services use a single call-and-response: a client sends a request and receives one response.
   Actions are for long-duration tasks: a client sends a goal, receives a continuous stream of feedback while the task runs, and gets a final result on completion.
   Actions can also be cancelled mid-execution.

What happens if a new goal is sent before the current one finishes?
   It depends on the action server's implementation.
   The Turtlesim server aborts the current goal when it receives a new one, but a server could also reject the new goal or queue it.
   Do not assume every action server aborts the current goal automatically.

Can an action be cancelled from either side?
   Yes. The client can cancel a goal at any time.
   The server can also choose to abort a goal, for example when it receives a conflicting new goal.

What is the structure of an action type?
   An action type has three sections separated by ``---``: the goal (sent by the client), the result (returned when the action completes), and the feedback (streamed while the action is running).