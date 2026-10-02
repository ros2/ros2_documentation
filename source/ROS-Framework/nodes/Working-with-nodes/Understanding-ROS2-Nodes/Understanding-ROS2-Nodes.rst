.. redirect-from::

    Tutorials/Understanding-ROS2-Nodes
    Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes

.. _ROS2Nodes:

Learning about nodes - tutorial
===============================

Nodes are the fundamental building blocks of a robotic system, where each node is responsible for a single task.
This article walks you through how to run nodes with the ``ros2`` command-line tools and inspect how they are shown on the ROS graph.
A hands-on exercise gives you practice listing active nodes and examining their connections.

**Area: Nodes, Framework | Content-type: tutorial | Experience: beginner**

.. contents:: Contents
   :depth: 3
   :local:

Summary
-------

Each node in ROS serves a single, modular purpose in a robotics system.
For more information, see :doc:`About nodes <../../../About-Nodes>`.

Nodes communicate with other nodes through :doc:`topics, services, actions, and parameters <../../../Interfaces-Topics-Services-Actions>`.
A single executable can contain one or more nodes.

For more information, see :doc:`How ROS works <../../../How-ROS-Works>`.

Prerequisites
-------------

:doc:`Using turtlesim, ros2, and rqt <../../../../Get-Started/Introducing-Turtlesim/Introducing-Turtlesim>` shows you how to install the ``turtlesim`` package used here.

.. note::
    Make sure to source ROS in every new terminal you open.

    For more information, see :doc:`Configuring environment <../../../../Get-Started/Configuring-ROS2-Environment>`.

Steps
-----

1 Launch an executable
^^^^^^^^^^^^^^^^^^^^^^

Nodes run inside executables.
To add a node to the ROS graph, launch an executable from a package with the ``ros2 run`` command, in the following format:

.. code-block:: console

  $ ros2 run <package_name> <executable_name>

To run the ``turtlesim_node`` executable from the ``turtlesim`` package, open a new terminal and enter the following command:

.. code-block:: console

  $ ros2 run turtlesim turtlesim_node

The Turtlesim window opens, as shown in :doc:`Using turtlesim, ros2, and rqt <../../../../Get-Started/Introducing-Turtlesim/Introducing-Turtlesim>`.

In this example, the package name is ``turtlesim`` and the executable name is ``turtlesim_node``.
The executable name is not always the same as the node name on the ROS graph.
You can find node names by using ``ros2 node list``.

2 List all running nodes
^^^^^^^^^^^^^^^^^^^^^^^^

``ros2 node list`` shows you the names of all running nodes.
This is especially useful when you want to interact with a node, or when your system runs many nodes and you need to keep track of them.

While Turtlesim is still running in the other terminal, open a new terminal and enter the following command:

.. code-block:: console

  $ ros2 node list

The terminal returns the node name:

.. code-block:: console

  /turtlesim

To see how the list changes when another node starts, in another new terminal, start a teleoperation node:

.. code-block:: console

  $ ros2 run turtlesim turtle_teleop_key

Here, we are referring to the ``turtlesim`` package again, but this time we target the executable named ``turtle_teleop_key``.

Return to the terminal where you ran ``ros2 node list`` and run it again:

.. code-block:: console

  $ ros2 node list

You should see the names of two running nodes:

.. code-block:: console

  /turtlesim
  /teleop_turtle

2.1 Remap a node name
~~~~~~~~~~~~~~~~~~~~~

`Remapping <https://design.ros2.org/articles/ros_command_line_arguments.html#name-remapping-rules>`__ allows you to reassign default node properties, such as node name, topic names, or service names, to custom values.
In :doc:`Using turtlesim, ros2, and rqt <../../../../Get-Started/Introducing-Turtlesim/Introducing-Turtlesim>`, you use remapping on ``turtle_teleop_key`` to change the ``cmd_vel`` topic and target **turtle2**.

You can also remap the node name when you start a new node.
In a new terminal, run:

.. code-block:: console

  $ ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle

This starts a second Turtlesim node named ``my_turtle``.
It does not rename the ``/turtlesim`` node that is already running.
Another Turtlesim window opens for the new node.

Return to the terminal where you ran ``ros2 node list`` and run it again:

.. code-block:: console

  $ ros2 node list

You will see three node names:

.. code-block:: console

  /my_turtle
  /turtlesim
  /teleop_turtle

3 Access more information about nodes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Now that you know the names of your nodes, you can see how a node connects to the rest of the system.
The ``ros2 node info`` command shows a node's publishers, subscribers, services, and actions.
These are the ROS graph connections that interact with that node.

.. code-block:: console

  $ ros2 node info <node_name>

To examine your latest node, ``my_turtle``, run the following command:

.. code-block:: console

  $ ros2 node info /my_turtle

You should see output similar to:

.. code-block:: console

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

Now try running the same command on the ``/teleop_turtle`` node, and see how its connections differ from ``my_turtle``.

Next steps
----------

To learn how to start multiple nodes at once without opening a separate terminal for each, see :ref:`Launching nodes - how-to <ROS2Launch>`.

Related content
---------------

More articles:

* :doc:`Learning about topics <../../../interfaces/topics/Understanding-ROS2-Topics/Understanding-ROS2-Topics>`
* :doc:`Interfaces (topics, services, actions) <../../../Interfaces-Topics-Services-Actions>`
* :doc:`About nodes <../../../About-Nodes>`

FAQs
----

Can a single executable contain more than one node?
   Yes.
   In ROS, a single executable, such as a C++ or Python program, can contain one or more nodes.

Why does another Turtlesim window open when I remap the node name?
   Each call to ``ros2 run turtlesim turtlesim_node`` starts a new node, and Turtlesim opens a window for it.
   Remapping ``__node`` sets the name of that new node.
   It does not rename or replace the ``/turtlesim`` node that is already running.

What is the difference between ``ros2 node list`` and ``ros2 node info``?
   ``ros2 node list`` shows the names of all running nodes.
   ``ros2 node info`` shows the connections of a single node, including its publishers, subscribers, services, and actions.
