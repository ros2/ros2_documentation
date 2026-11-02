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

Subordinate nodes (sub-nodes)
-----------------------------

In C++ (``rclcpp``), a node can create secondary instances known as **subordinate nodes** (or **sub-nodes**) using ``rclcpp::Node::create_sub_node()``.
A subordinate node shares the parent node's context, node interfaces (including logging, parameters, timers, and clock), and ROS graph identity, while specifying an additional relative *sub-namespace*.

Subordinate nodes are useful for modular C++ software architecture.
Internal subsystems or helper libraries running inside the same node process can use a subordinate node to publish or subscribe under scoped topic and service names without creating separate nodes, avoiding additional middleware discovery overhead and multi-node execution management.

For a practical guide, see the :doc:`Using subordinate nodes tutorial </Tutorials/Intermediate/Using-Sub-Nodes>`.
