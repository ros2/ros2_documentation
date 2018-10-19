
ROS is a middleware based on an anonymous publish/subscribe mechanism that allows for message passing between different ROS processes.

At the heart of any ROS 2 system is the ROS graph.
The ROS graph refers to the network of nodes in a ROS system and the connections between them by which they communicate.

Quick Overview of Graph Concepts
--------------------------------


* Nodes: A node is an entity that uses ROS to communicate with other nodes. 
* Messages: ROS data type used when subscribing or publishing to a topic. 
* Topics: Nodes can publish messages to a topic as well as subscribe to a topic to receive messages.
* Discovery: The automatic process through which nodes determine how to talk to each other.

Nodes
-----

A node is a participant in the ROS graph.
ROS nodes use a ROS client library to communicate with other nodes.
Nodes can publish or subscribe to a Topic.
Nodes can also provide or use a Service.
There are configurable Parameters associated with a node.
Connections between nodes are established through a distributed discovery process.
Nodes may be located in the same process, in different processes, or on different machines.
These concepts will be described in more detail in the sections that follow.

Client Libraries
----------------

ROS client libraries allow nodes written in different programming languages to communicate.
There is a core ROS client library (RCL) that implements common functionality needed for the ROS APIs of different languages.
This makes it so that language-specific client libraries are easier to write and that they have more consistent behavior.

The following client libraries are maintained by the ROS 2 team:


* rclcpp = C++ client library
* rclpy = Python client library

Additionally, other client libraries have been developed by the ROS community.
See the `ROS 2 Client Libraries <ROS-2-Client-Libraries>` article for more details.

Discovery
---------

Discovery of nodes happens automatically through the underlying middleware of ROS 2.
It can be summarized as follows:


#. When a node is started, it advertises its presence to other nodes on the network with the same ROS domain (set with the ROS_DOMAIN_ID environment variable).
   Nodes respond to this advertisement with information about themselves so that the appropriate connections can be made and the nodes can communicate.
#. Nodes periodically advertise their presence so that connections can be made with new-found entities, even after the initial discovery period.
#. Nodes advertise to other nodes when they go offline.

Nodes will only establish connections with other nodes if they have compatible `Quality of Service <Quality-of-Service>` settings.

Example: talker-listener
------------------------

In one terminal, start a node (written in C++) that will publish messages on a topic.

.. code-block:: bash

   ros2 run demo_nodes_cpp talker

In another terminal, start a second node (written in Python) that will subscribe to messages on the same topic.

.. code-block:: bash

   ros2 run demo_nodes_py listener

You should see that these nodes discover each other automatically, and begin to exchange messages.
