.. redirect-from::

    Concepts/Basic/About-Nodes

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
   :maxdepth: 1
   :hidden:

   nodes/About-Discovery
   nodes/About-Domain-ID
   nodes/About-Logging/About-Logging
   nodes/About-Composition
   nodes/Working-with-nodes

.. contents:: Table of Contents
   :local:

Overview of nodes
-----------------

A node is a participant in the ROS 2 graph, which uses a :doc:`client library <About-Client-Libraries>` to communicate with other nodes.
Nodes can communicate with other nodes within the same process, in a different process, or on a different machine.
Nodes are typically the unit of computation in a ROS graph; each node should do one logical thing.

Nodes can :doc:`publish <interfaces/About-Topics>` to named topics to deliver data to other nodes, or :doc:`subscribe <interfaces/About-Topics>` to named topics to get data from other nodes.
They can also act as a :doc:`service client <interfaces/About-Services>` to have another node perform a computation on their behalf, or as a :doc:`service server <interfaces/About-Services>` to provide functionality to other nodes.
For long-running computations, a node can act as an :doc:`action client <interfaces/About-Actions>` to have another node perform it on their behalf, or as an :doc:`action server <interfaces/About-Actions>` to provide functionality to other nodes.
Nodes can provide configurable :doc:`parameters <About-Parameters>` to change behavior during run-time.

Nodes are often a complex combination of publishers, subscribers, service servers, service clients, action servers, and action clients, all at the same time.

Connections between nodes are established through a distributed :doc:`discovery <nodes/About-Discovery>` process.
