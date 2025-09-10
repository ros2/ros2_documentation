.. redirect-from::

    Introspection-with-command-line-tools
    Tutorials/Introspection-with-command-line-tools
    Concepts/About-Command-Line-Tools

Introspection with command line tools
=====================================

.. contents:: Table of Contents
   :local:

ROS 2 includes a suite of command-line tools for introspecting a ROS 2 system.

Usage
-----

The main entry point for the tools is the command ``ros2``, which itself has various sub-commands for introspecting and working with nodes, topics, services, and more.

To see all available sub-commands run:

.. code-block:: console

   $ ros2 --help

Examples of sub-commands that are available include:

* ``action``: Introspect/interact with ROS actions
* ``bag``: Record/play a rosbag
* ``component``: Manage component containers
* ``daemon``: Introspect/configure the ROS 2 daemon
* ``doctor``: Check ROS setup for potential issues
* ``interface``: Show information about ROS interfaces
* ``launch``: Run/introspect a launch file
* ``lifecycle``: Introspect/manage nodes with managed lifecycles
* ``multicast``: Multicast debugging commands
* ``node``: Introspect ROS nodes
* ``param``: Introspect/configure parameters on a node
* ``pkg``: Introspect ROS packages
* ``run``: Run ROS nodes
* ``security``: Configure security settings
* ``service``: Introspect/call ROS services
* ``test``: Run a ROS launch test
* ``topic``: Introspect/publish ROS topics
* ``trace``: Tracing tools to get information on ROS nodes execution (only available on Linux)

Example
-------

To produce the typical talker-listener example using command-line tools, the ``topic`` sub-command can be used to publish and echo messages on a topic.

Publish messages in one terminal with:

.. code-block:: console

   $ ros2 topic pub /chatter std_msgs/msg/String "data: Hello world"
   publisher: beginning loop
   publishing #1: std_msgs.msg.String(data='Hello world')

   publishing #2: std_msgs.msg.String(data='Hello world')

Echo messages received in another terminal with:

.. code-block:: console

   $ ros2 topic echo /chatter
   data: Hello world

   data: Hello world

ROS 2 Daemon: Background Discovery Service
------------------------------------------

ROS 2 uses a distributed discovery process for nodes to connect to each other.
As this process purposefully does not use a centralized discovery mechanism, it can take time for ROS nodes to discover all other participants in the ROS graph.
To address this, ROS 2 runs a background daemon process that maintains information about the ROS graph to provide faster responses to queries, such as the list of node names.

The ROS 2 daemon is automatically started when you first use command-line tools like ``ros2 node list``, ``ros2 topic list``, or other introspection commands.
If no daemon is running, these tools will instantiate a new daemon process in the background before executing the requested command.

The daemon communicates using the localhost network interface (127.0.0.1) and uses the :doc:`ROS_DOMAIN_ID <../Intermediate/About-Domain-ID>` environment variable as a port number offset.
This means that if you want to control a specific daemon instance (for example, using ``ros2 daemon stop``), you must ensure that your :doc:`ROS_DOMAIN_ID <../Intermediate/About-Domain-ID>` matches the domain ID used by that daemon.
Different :doc:`ROS_DOMAIN_ID <../Intermediate/About-Domain-ID>` values will result in separate daemon instances running on different ports.

You can run ``ros2 daemon --help`` for more options for interacting with the daemon, including commands to start, stop, or check the status of the daemon process.

Implementation
--------------

The source code for the ``ros2`` command is available at https://github.com/ros2/ros2cli.

The ``ros2`` tool has been implemented as a framework that can be extended via plugins.
For example, the `sros2 <https://github.com/ros2/sros2>`__ package provides a ``security`` sub-command that is automatically detected by the ``ros2`` tool if the ``sros2`` package is installed.
