.. redirect-from::

    Concepts/Basic/About-Nodes
    Concepts/Basic/About-Discovery
    ROS-Framework/nodes/About-Discovery

.. meta::
   :contentType: about
   :experience: beginner
   :area: nodes, framework
   :distribution: {DISTRO}
   :product: {PRODUCT}

Nodes
=====

.. short-description::
   Nodes are the main units of computation in a ROS graph.
   This article explains what nodes are, how they communicate, and how they provide functionality to other parts of a ROS system.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. toctree::
   :hidden:

   nodes/About-Domain-ID
   nodes/About-Logging/About-Logging
   nodes/About-Composition
   nodes/Working-with-nodes

.. contents:: Contents
   :depth: 2
   :local:

Summary
-------

A node is a participant in the ROS graph that handles a specific task.
The behavior of nodes can be configured using parameters.
Nodes advertise their presence and establish connections with other compatible nodes through the discovery process.
Nodes communicate with each other using interfaces, like topics, services, and actions.

About nodes
-----------

Nodes are the fundamental building blocks of a ROS system and represent units of computation in a ROS graph.
Each node is an independent participant that handles one or more specific tasks, such as reading sensor data, processing an algorithm, or driving a motor.
Nodes can function as any combination of the following:

* A publisher to deliver data to other nodes.
* A subscriber to get data from other nodes.
* A service client to have another node perform a computation on its behalf.
* A service server to provide functionality to other nodes.
* An action client to have another node perform a long-running computation on its behalf.
* An action server to provide long-running functionality to other nodes.

In a ROS system, there is typically a complex network of nodes as publishers, subscribers, service servers, service clients, action servers, and action clients, each acting with a different role simultaneously.
This network of nodes and the connections between them is known as the ROS graph.
See :doc:`How-ROS-Works`.

Each client library provides a node type that you either create directly or, in object-oriented languages such as C++ and Python, subclass to add your own behavior.
Configurable parameters enable you to control node behavior during runtime.
For example, a node controlling a motor may have a parameter that controls the motor's maximum speed, while a topic might be used to set a robot's current speed.
See :doc:`parameters <About-Parameters>`.

Communication between nodes
---------------------------

Nodes can communicate with other nodes within the same process, in a different process, or on a different machine.

The ROS client libraries provide an API that allows you to write nodes in multiple programming languages which can communicate with other nodes, even if those nodes are not written in the same language.
See :doc:`About-Client-Libraries`.

ROS nodes communicate through interfaces.
See :doc:`Interfaces-Topics-Services-Actions`.

Discovery
^^^^^^^^^

Discovery of nodes happens automatically through the underlying middleware of ROS.
How discovery works depends on the :doc:`middleware (RMW) implementation <middleware/About-Middleware-Implementations>` and how it is configured.
With the default DDS-based middleware, nodes find each other through a distributed discovery process, with no central server.
Other configurations use server-client discovery instead, for example, a Fast DDS discovery server, or the router that ``rmw_zenoh`` uses by default.
See :doc:`../Developer-Tools/Launch/Discovery-Server/Discovery-Server` and :doc:`../Get-Started/Installation/RMW-Implementations/Non-DDS-Implementations/Working-with-Zenoh`.

The distributed discovery process can be summarized as follows:

#. When a node is started, it advertises its presence to other nodes on the network with the same ROS domain (set with the ``ROS_DOMAIN_ID`` environment variable).
   Nodes respond to this advertisement with information about themselves so that the appropriate connections can be made and the nodes can communicate.
#. Nodes periodically advertise their presence so that connections can be made with new-found entities, even after the initial discovery period.
#. Nodes advertise to other nodes when they shut down cleanly.
   If a node stops unexpectedly, for example because it crashed, other nodes notice it is gone when it stops sending its periodic advertisements.

Nodes must share a ROS domain ID to discover each other.
See :doc:`nodes/About-Domain-ID`.
You can also limit how far discovery reaches, for example to the local host only, with ``ROS_AUTOMATIC_DISCOVERY_RANGE`` and ``ROS_STATIC_PEERS``.
See :doc:`../Developer-Tools/Launch/Improved-Dynamic-Discovery`.

.. note::
   Discovering each other does not guarantee that two nodes exchange data.
   Quality of service (QoS) compatibility is checked separately for each pair of endpoints, for example, a publisher and a subscription on the same topic.
   If their QoS settings are incompatible, that pair does not connect, even though both nodes are visible in the ROS graph.
   See :doc:`interfaces/topics/Working-with-topics/Quality-of-Service`.

The :ref:`talker-listener demo <talker-listener>` provides an example of node discovery.
The C++ talker node in one terminal publishes messages on a topic, and the Python listener node running in another terminal subscribes to messages on the same topic.
These nodes discover each other automatically, and begin to exchange messages.

Node management
---------------

You can start individual nodes from the command line with ``ros2 run``.
The ``ros2 run`` command initiates the executable binary which can launch one or more nodes.

ROS launch files allow you to start up and configure a number of executables containing ROS nodes simultaneously.
See :doc:`launch files <../Developer-Tools/About-Launch>`.

Log messages
------------

Each node has an associated logger.
By default, log messages go out to targets including the console (on stderr), log files on disk, and the ``/rosout`` topic.
All of the targets can be individually enabled or disabled on a per-node basis.

See :doc:`nodes/About-Logging/About-Logging`.

Special types of nodes
----------------------

Composable nodes
   A composable node is written as a component which, at runtime, is loaded into a container process that can host several components at once.
   Components loaded into the same container run in a single process, and can use intra-process communication to exchange messages without copying them.
   To learn more about how to write a composable node, see :doc:`nodes/About-Composition`, and :doc:`nodes/Working-with-nodes/Writing-a-Composable-Node`.
Lifecycle nodes
   Lifecycle nodes can be used to ensure that resources are correctly initialized, activated, deactivated, and cleaned up as the node moves between lifecycle states.

   A common use case is nodes that control hardware, where devices such as cameras, lidars, motor drivers, and other sensors and actuators must be started, configured, and shut down in a controlled order.
   See :doc:`nodes/Working-with-nodes/Managed-Nodes`.
Subordinate nodes (sub-nodes)
   In C++ (``rclcpp``), a node can create secondary instances known as **subordinate nodes** (or **sub-nodes**) using ``rclcpp::Node::create_sub_node()``.
   A subordinate node shares the parent node's context, node interfaces (including logging, parameters, timers, and clock), and ROS graph identity, while specifying an additional relative *sub-namespace*.

   Subordinate nodes are useful for modular C++ software architecture.
   Internal subsystems or helper libraries running inside the same node process can use a subordinate node to publish or subscribe under scoped topic and service names without creating separate nodes, avoiding additional middleware discovery overhead and multi-node execution management.

   For a practical guide, see :doc:`nodes/Working-with-nodes/Using-Sub-Nodes`.

Related content
---------------

Related articles:

* :doc:`About-Client-Libraries`
* :doc:`About-Parameters`
* :doc:`Interfaces-Topics-Services-Actions`
* :doc:`nodes/About-Composition`
* :doc:`nodes/About-Domain-ID`
* :doc:`nodes/Working-with-nodes`

.. ros-related-articles::

FAQs
----

What is the difference between a node and an executable?
   A node is a participant on the ROS graph.
   An executable is the program you run, for example with ``ros2 run``, which starts a process.
   One process can contain one or more nodes.

How do nodes find each other?
   Through discovery.
   When a node starts, the middleware advertises its presence to other nodes on the same ROS domain.
   Their publishers, subscriptions, and other endpoints can then connect automatically.

Do all nodes in a system need to use the same programming language?
   No.
   Client libraries provide a common way to communicate, so a C++ node can talk to a Python node over the same interfaces.

Why might two nodes fail to connect?
   Common causes are:

   * The nodes use different ROS domain IDs.
   * Discovery is limited, for example, ``ROS_AUTOMATIC_DISCOVERY_RANGE`` is set to ``LOCALHOST`` and the nodes are on different machines.
   * The nodes use different, incompatible middleware (RMW) implementations.
   * The topic, service, or action names or types do not match.
   * The QoS settings of a publisher and a subscription are incompatible.
   * A firewall or network configuration blocks discovery or data traffic.

What is the difference between composable nodes and lifecycle nodes?
   Composable nodes are components that can run together in a single process.
   Lifecycle nodes move through defined states so resources are started and shut down in a controlled order.
