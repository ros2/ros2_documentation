.. redirect-from::

    Concepts/Basic/About-Nodes
    Concepts/Basic/About-Discovery

Nodes
=====

.. toctree::
   :maxdepth: 1
   :hidden:

   nodes/About-Domain-ID
   nodes/About-Logging/About-Logging
   nodes/About-Composition
   nodes/Working-with-nodes

A node is a single process that performs computation and can communicate with other nodes via interfaces.
This article describes what nodes do and how they interconnect.

**[Area: Framework | Content-type: concept | Experience: beginner]**

.. contents:: Table of Contents
   :local:

Summary
-------

A node is an independent processes that handles a specific task.
The behaviour of nodes is configured using parameters.
Nodes advertise their presence and establish connections with other compatible nodes through the discovery process.
Nodes communicate with each other using interfaces.

Nodes
-----
Nodes are the fundamental building blocks of a ROS system and represent units of computation in a ROS graph.
Each node is an independent process that handles a specific task, such as reading sensor data, processing an algorithm, or driving a motor.
Nodes can function as:

* A publisher to deliver data to other nodes.
* A subscriber to get data from other nodes.
* A service client to have another node perform a computation on their behalf.
* A service server to provide functionality to other nodes.
* An action client to have another node perform a long-running computation on their behalf.
* An action server to provide long-running functionality to other nodes.

In a ROS system, there is typically a complex network of nodes as publishers, subscribers, service servers, service clients, action servers, and action clients, each acting with a different role simultaneously.
This network is known as the ROS graph. See :doc:`How-ROS-Works`.

ROS is based on object-oriented programming principles.
Individual nodes are written as subclasses of the Node class, inheriting properties from it as defined by ROS.
Configurable parameters enable you to control node behaviour during runtime.
Learn more: :doc:`parameters <About-Parameters>`.

Communication between nodes
---------------------------

Each node runs separately in its own runtime environment.
Nodes can communicate with other nodes within the same process, in a different process, or on a different machine.

:doc:`Client libraries <About-Client-Libraries>` provide APIs that allow the node to interface with other nodes, even if other nodes are not written in the same language.

ROS nodes communicate through interfaces.
Learn more: :doc:`Interfaces-Topics-Services-Actions`

Discovery
^^^^^^^^^

Connections between nodes are established through a distributed discovery process.
Discovery of nodes happens automatically through the underlying middleware of ROS.
The discovery process can be summarized as follows:

#. When a node is started, it advertises its presence to other nodes on the network with the same ROS domain (set with the ROS_DOMAIN_ID environment variable).
   Nodes respond to this advertisement with information about themselves so that the appropriate connections can be made and the nodes can communicate.
#. Nodes periodically advertise their presence so that connections can be made with new-found entities, even after the initial discovery period.
#. Nodes advertise to other nodes when they go offline.

Nodes will only establish connections with other nodes if they have compatible *quality of service* settings.
See :doc:`interfaces/topics/Working-with-topics/Quality-of-Service`.

Node management
---------------

Individual nodes can be launched by command line.
ROS launch files allow you to start up and configure a number of executables containing ROS nodes simultaneously.

Learn more about the launch service: :doc:`../Developer-Tools/About-Launch`

Log messages
------------

Each node has a logger associated with it and, by default, log messages go out to targets including the console (on stderr), log files on disk, and the ``/rosout`` topic.
All of the targets can be individually enabled or disabled on a per-node basis.

Learn more about logging: :doc:`nodes/About-Logging/About-Logging`

Special types of nodes
----------------------
* Composable nodes
   A composable node contains separate components with shared memory.
   Learn more about how to write a composable node: :doc:`nodes/Working-with-nodes/Writing-a-Composable-Node`
* Managed nodes, also known as lifecycle nodes
   These nodes can be used to ensure that resources are correctly initialised, activated, deactivated, and cleaned up as the node moves between lifecycle states.
   A common use case is nodes that control hardware, where devices such as cameras, lidars, motor drivers, and other sensors and actuators must be started, configured, and shut down in a controlled order.
   Learn more about managed nodes: :doc:`nodes/Working-with-nodes/Managed-Nodes`

Related content
---------------
* :doc:`About-Client-Libraries`
* :doc:`About-Parameters`
* :doc:`Interfaces-Topics-Services-Actions`
* :doc:`nodes/Working-with-nodes`

FAQs
----

What is the difference between a node and an executable?
   A node is a participant on the ROS graph.
   An executable is the process you start, for example with ``ros2 run``.
   One executable can contain one or more nodes.

How do nodes find each other?
   Through discovery.
   When a node starts, it advertises its presence to other nodes on the same ROS domain.
   Compatible nodes can then form connections automatically.

Do all nodes in a system need to use the same programming language?
   No.
   Client libraries provide a common way to communicate, so a C++ node can talk to a Python node over the same interfaces.

Why might two nodes fail to connect?
   They must be on the same ROS domain, and their quality of service settings must be compatible.
   If either of those conditions is not met, the nodes do not establish a connection.

What is the difference between composable nodes and managed nodes?
   Composable nodes are components that can share a process and memory.
   Managed nodes, also called lifecycle nodes, move through defined states so resources are started and shut down in a controlled order.
