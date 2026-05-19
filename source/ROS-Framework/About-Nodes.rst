Nodes
=====

.. contents:: Table of Contents
   :local:

A node is a participant in the ROS 2 graph, which uses a :doc:`client library <About-Client-Libraries>` to communicate with other nodes.
Nodes can communicate with other nodes within the same process, in a different process, or on a different machine.
Nodes are typically the unit of computation in a ROS graph; each node should do one logical thing.

Nodes can :doc:`publish <About-Topics>` to named topics to deliver data to other nodes, or :doc:`subscribe <About-Topics>` to named topics to get data from other nodes.
They can also act as a :doc:`service client <About-Services>` to have another node perform a computation on their behalf, or as a :doc:`service server <About-Services>` to provide functionality to other nodes.
For long-running computations, a node can act as an :doc:`action client <About-Actions>` to have another node perform it on their behalf, or as an :doc:`action server <About-Actions>` to provide functionality to other nodes.
Nodes can provide configurable :doc:`parameters <About-Parameters>` to change behavior during run-time.

Nodes are often a complex combination of publishers, subscribers, service servers, service clients, action servers, and action clients, all at the same time.

Connections between nodes are established through a distributed :doc:`discovery <About-Discovery>` process.
