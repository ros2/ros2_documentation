.. redirect-from::

    Concepts/Basic/About-Discovery

.. meta::
   :contentType: about
   :experience: beginner
   :area: nodes, framework
   :distribution: {DISTRO}
   :product: {PRODUCT}

Discovery
=========

.. short-description::
   Discovery lets ROS nodes find each other so they can communicate without manual connection setup.
   This article describes how nodes advertise their presence, respond to matching nodes, and maintain connections within a shared ROS domain.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. contents:: Table of Contents
   :local:

Discovery of nodes happens automatically through the underlying middleware of ROS 2.
It can be summarized as follows:

#. When a node is started, it advertises its presence to other nodes on the network with the same ROS domain (set with the ROS_DOMAIN_ID environment variable).
   Nodes respond to this advertisement with information about themselves so that the appropriate connections can be made and the nodes can communicate.
#. Nodes periodically advertise their presence so that connections can be made with new-found entities, even after the initial discovery period.
#. Nodes advertise to other nodes when they go offline.

Nodes will only establish connections with other nodes if they have compatible :doc:`Quality of Service <../interfaces/topics/Working-with-topics/Quality-of-Service>` settings.

Take the :ref:`talker-listener demo <talker-listener>` for example.
Running the C++ talker node in one terminal will publish messages on a topic,
and the Python listener node running in another terminal  will subscribe to messages on the same topic.

You should see that these nodes discover each other automatically, and begin to exchange messages.
