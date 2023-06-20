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

.. code-block:: bash

   ros2 --help

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
* ``wtf``: An alias for ``doctor``

Example
-------

To produce the typical talker-listener example using command-line tools, the ``topic`` sub-command can be used to publish and echo messages on a topic.

Publish messages in one terminal with:

.. code-block:: bash

   $ ros2 topic pub /chatter std_msgs/msg/String "data: Hello world"
   publisher: beginning loop
   publishing #1: std_msgs.msg.String(data='Hello world')

   publishing #2: std_msgs.msg.String(data='Hello world')

Echo messages received in another terminal with:

.. code-block:: bash

   $ ros2 topic echo /chatter
   data: Hello world

   data: Hello world

Behind the scenes
-----------------

ROS 2 uses a distributed discovery process for nodes to connect to each other.
As this process purposefully does not use a centralized discovery mechanism, it can take time for ROS nodes to discover all other participants in the ROS graph.
Because of this, there is a long-running daemon in the background that stores information about the ROS graph to provide faster responses to queries, e.g. the list of node names.

The daemon is automatically started when the relevant command-line tools are used for the first time.
You can run ``ros2 daemon --help`` for more options for interacting with the daemon.

Implementation
--------------

The source code for the ``ros2`` command is available at https://github.com/ros2/ros2cli.

The ``ros2`` tool has been implemented as a framework that can be extended via plugins.
For example, the `sros2 <https://github.com/ros2/sros2>`__ package provides a ``security`` sub-command that is automatically detected by the ``ros2`` tool if the ``sros2`` package is installed.
