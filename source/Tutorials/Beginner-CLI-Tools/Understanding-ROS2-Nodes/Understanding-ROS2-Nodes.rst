.. redirect-from::

    Tutorials/Understanding-ROS2-Nodes

.. _ROS2Nodes:

Learning about nodes - tutorial
===============================

Nodes are the fundamental building blocks of a robotic system, with each node responsible for a single task.
This article walks you through running nodes with the ``ros2`` command-line tools and inspecting how they appear on the ROS graph.
A hands-on exercise gives you practice listing active nodes and examining their connections.

**Area: ROS-tutorials | Content-type: tutorial | Experience: beginner**

.. contents:: Contents
   :depth: 3
   :local:

Summary
-------

Each node in ROS serves a single, modular purpose in a robotics system.

Nodes appear on the ROS graph and communicate with other nodes through topics, services, actions, and parameters.
A single executable can contain one or more nodes.

Background
----------

1 The ROS graph
^^^^^^^^^^^^^^^^^

The ROS graph is a network of ROS elements processing data together at the same time.
It visually represents all the executables and the connections between them.

2 Nodes in ROS
^^^^^^^^^^^^^^^^

Each node in ROS should be responsible for a single modular purpose, for example, controlling the wheel motors or publishing the sensor data from a laser range-finder.
Each node can send and receive data from other nodes via topics, services, actions, or parameters.

.. image:: images/Nodes-TopicandService.gif

A full robotic system is comprised of many nodes working together.
In ROS, a single executable (C++ program, Python program, etc.) can contain one or more nodes.

Prerequisites
-------------

:doc:`Using turtlesim, ros2, and rqt <../Introducing-Turtlesim/Introducing-Turtlesim>` shows you how to install the ``turtlesim`` package used here.

.. note::
    Make sure to source ROS in every new terminal you open.
    See :doc:`Configuring environment <../Configuring-ROS2-Environment>`.

Steps
-----

1 ros2 run
^^^^^^^^^^

The command ``ros2 run`` launches an executable from a package.

.. code-block:: console

  $ ros2 run <package_name> <executable_name>

To run Turtlesim, open a new terminal and enter the following command:

.. code-block:: console

  $ ros2 run turtlesim turtlesim_node

The Turtlesim window opens, as shown in :doc:`Using turtlesim, ros2, and rqt <../Introducing-Turtlesim/Introducing-Turtlesim>`.

In this example, the package name is ``turtlesim`` and the executable name is ``turtlesim_node``.
However, we still don't know the node name.
You can find node names by using ``ros2 node list``.

2 ros2 node list
^^^^^^^^^^^^^^^^

``ros2 node list`` shows you the names of all running nodes.
This is especially useful when you want to interact with a node, or when your system runs many nodes and you need to keep track of them.

While Turtlesim is still running in the other terminal, open a new terminal and enter the following command:

.. code-block:: console

  $ ros2 node list
  /turtlesim

The terminal returns the node name.

Open another new terminal and start the teleoperation node with the command:

.. code-block:: console

  $ ros2 run turtlesim turtle_teleop_key

Here, we are referring to the ``turtlesim`` package again, but this time we target the executable named ``turtle_teleop_key``.

Return to the terminal where you ran ``ros2 node list`` and run it again.
You will see the names of two active nodes:

.. code-block:: console

  $ ros2 node list
  /turtlesim
  /teleop_turtle

.. note::
    You can start multiple nodes at once without opening a separate terminal for each. See :ref:`Launching nodes - how-to <ROS2Launch>`.

2.1 Remapping
~~~~~~~~~~~~~

`Remapping <https://design.ros2.org/articles/ros_command_line_arguments.html#name-remapping-rules>`__ allows you to reassign default node properties, like node name, topic names, or service names, to custom values.
In :doc:`Using turtlesim, ros2, and rqt <../Introducing-Turtlesim/Introducing-Turtlesim>`, you used remapping on ``turtle_teleop_key`` to change the cmd_vel topic and target **turtle2**.

Now, let's reassign the name of our ``/turtlesim`` node.

In a new terminal, run the following command:

.. code-block:: console

  $ ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle

Because you are calling ``ros2 run`` on Turtlesim again, another Turtlesim window opens.
However, if you return to the terminal where you ran ``ros2 node list`` and run it again, you will see three node names:

.. code-block:: console

    /my_turtle
    /turtlesim
    /teleop_turtle

3 ros2 node info
^^^^^^^^^^^^^^^^

Now that you know the names of your nodes, you can access more information about them with:

.. code-block:: console

  $ ros2 node info <node_name>

To examine your latest node, ``my_turtle``, run the following command:

.. code-block:: console

  $ ros2 node info /my_turtle
  /my_turtle
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
      /my_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
      /my_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
      /my_turtle/get_parameters: rcl_interfaces/srv/GetParameters
      /my_turtle/list_parameters: rcl_interfaces/srv/ListParameters
      /my_turtle/set_parameters: rcl_interfaces/srv/SetParameters
      /my_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
      /reset: std_srvs/srv/Empty
      /spawn: turtlesim_msgs/srv/Spawn
      /turtle1/set_pen: turtlesim_msgs/srv/SetPen
      /turtle1/teleport_absolute: turtlesim_msgs/srv/TeleportAbsolute
      /turtle1/teleport_relative: turtlesim_msgs/srv/TeleportRelative
    Service Clients:

    Action Servers:
      /turtle1/rotate_absolute: turtlesim_msgs/action/RotateAbsolute
    Action Clients:

``ros2 node info`` returns a list of subscribers, publishers, services, and actions.
These are the ROS graph connections that interact with that node.

Now try running the same command on the ``/teleop_turtle`` node, and see how its connections differ from ``my_turtle``.

Related content
---------------

More articles:

* :doc:`Learning about topics <../Understanding-ROS2-Topics/Understanding-ROS2-Topics>`
* :doc:`Interfaces (topics, services, actions) </Concepts/Basic/Interfaces-Topics-Services-Actions>`
* :doc:`Concepts </Concepts>`
* :ref:`Launching nodes - how-to <ROS2Launch>`

FAQs
----

Can a single executable contain more than one node?
   Yes.
   In ROS, a single executable, such as a C++ or Python program, can contain one or more nodes.

Why does another Turtlesim window open when I remap the node name?
   Each call to ``ros2 run turtlesim turtlesim_node`` starts a new node, and Turtlesim opens a window for it.
   Remapping ``__node`` only changes the node's name; it does not reuse the existing node.

What is the difference between ``ros2 node list`` and ``ros2 node info``?
   ``ros2 node list`` shows the names of all running nodes.
   ``ros2 node info`` shows the connections of a single node, including its publishers, subscribers, services, and actions.
