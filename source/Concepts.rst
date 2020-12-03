.. _ConceptsHome:

Concepts
========

The following "Concepts" pages provide an overview for key aspects of ROS 2.
Concept overviews will help you understand the "big picture" idea of ROS 2 systems.

.. toctree::
   :maxdepth: 1

   Concepts/DDS-and-ROS-middleware-implementations
   Concepts/About-Quality-of-Service-Settings
   Concepts/About-ROS-Interfaces
   Concepts/About-Topic-Statistics
   Concepts/ROS-2-Client-Libraries
   Concepts/Logging
   Concepts/About-ROS-2-Parameters

See https://docs.ros2.org/ for more high-level ROS 2 documentation.

Overview of ROS 2 Concepts
--------------------------

.. contents:: Table of Contents
   :local:

ROS 2 is a middleware based on an anonymous publish/subscribe mechanism that allows for message passing between different ROS processes.

At the heart of any ROS 2 system is the ROS graph.
The ROS graph refers to the network of nodes in a ROS system and the connections between them by which they communicate.

Quick Overview of Graph Concepts
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


* Nodes: A node is an entity that uses ROS to communicate with other nodes.
* Messages: ROS data type used when subscribing or publishing to a topic.
* Topics: Nodes can publish messages to a topic as well as subscribe to a topic to receive messages.
* Discovery: The automatic process through which nodes determine how to talk to each other.

Nodes
^^^^^

A node is a participant in the ROS graph.
ROS nodes use a ROS client library to communicate with other nodes.
Nodes can publish or subscribe to Topics.
Nodes can also provide or use Services and Actions.
There are configurable Parameters associated with a node.
Connections between nodes are established through a distributed discovery process.
Nodes may be located in the same process, in different processes, or on different machines.
These concepts will be described in more detail in the sections that follow.

Client Libraries
^^^^^^^^^^^^^^^^

ROS client libraries allow nodes written in different programming languages to communicate.
There is a core ROS client library (RCL) that implements common functionality needed for the ROS APIs of different languages.
This makes it so that language-specific client libraries are easier to write and that they have more consistent behavior.

The following client libraries are maintained by the ROS 2 team:


* rclcpp = C++ client library
* rclpy = Python client library

Additionally, other client libraries have been developed by the ROS community.
See the :ref:`ROS 2 Client Libraries <ROS-2-Client-Libraries>` article for more details.

Discovery
^^^^^^^^^

Discovery of nodes happens automatically through the underlying middleware of ROS 2.
It can be summarized as follows:

#. When a node is started, it advertises its presence to other nodes on the network with the same ROS domain (set with the ROS_DOMAIN_ID environment variable).
   Nodes respond to this advertisement with information about themselves so that the appropriate connections can be made and the nodes can communicate.
#. Nodes periodically advertise their presence so that connections can be made with new-found entities, even after the initial discovery period.
#. Nodes advertise to other nodes when they go offline.

Nodes will only establish connections with other nodes if they have compatible `Quality of Service <../Tutorials/Quality-of-Service>` settings.

Take the `talker-listener demo <talker-listener>` for example.
Running the C++ talker node in one terminal will publish messages on a topic,
and the Python listener node running in another terminal  will subscribe to messages on the same topic.

You should see that these nodes discover each other automatically, and begin to exchange messages.

Related Content
^^^^^^^^^^^^^^^

For a brief video introduction to ROS 2, see this community contributed content:

* `Getting started with ROS Part 1: Nodes, Parameters and Topics <https://youtu.be/46TPAKXBOF8>`_
* `Getting started with ROS Part 2: Services and Actions <https://youtu.be/keZAJ83eEoM>`_
