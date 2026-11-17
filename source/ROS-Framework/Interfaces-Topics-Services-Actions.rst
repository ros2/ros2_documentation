.. redirect-from::

    About-ROS-Interfaces
    How-To-Guides/Topics-Services-Actions
    ROS-Framework/interfaces/Topics-Services-Actions

.. meta::
   :contentType: about
   :experience: beginner
   :area: interfaces, framework
   :distribution: {DISTRO}
   :product: {PRODUCT}

.. _interfaces-topics-services-actions:
.. _TopicsServicesActions:

Interfaces (topics, services, actions)
======================================

.. short-description::
   Topics, services, and actions in ROS define how nodes exchange data.
   This article explains these different types of ROS interface and the differences between them.
   With this information, you'll be able to select the right interface for your purposes.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. contents:: Contents
   :depth: 2
   :local:

.. toctree::
   :hidden:

   interfaces/About-Topics
   interfaces/About-Services
   interfaces/About-Actions
   interfaces/Working-with-interfaces

Summary
-------

ROS nodes typically communicate through the following three types of interfaces:

* :doc:`Topics <interfaces/About-Topics>`: For continuous data streams.
* :doc:`Services <interfaces/About-Services>`: For request/response interactions (short tasks that return a result quickly).
* :doc:`Actions <interfaces/About-Actions>`: For long-running tasks with feedback (tasks that may take some time to complete).

For consistent communication, each interface uses definitions provided in ``.msg``, ``.srv``, or ``.action`` files.
To learn more about the interface definitions, see :doc:`interfaces/Working-with-interfaces/Interface-specifications`.

Topics
------

The topic interface is meant for continuous data streams, such as streaming sensor data or the status of your robot.
Topic definitions are stored in ``.msg`` files.
Topics implement a publish/subscribe pattern.
A node publishes data to a topic, and other nodes subscribe to receive that data.

This interface type has the following main characteristics:

* Communication is asynchronous and one-way.
  The publisher decides when data is sent.
  Data might be published and subscribed at any time, independent of any senders/receivers.
  Callbacks receive data when it is available.
* Multiple publishers and subscribers can share the same topic.

.. mermaid::

   flowchart LR
    P[Publisher node] -->|Publishes messages| T[Topic]
    T -->|Delivers messages| S1[Subscriber node]
    T -->|Delivers messages| S2[Subscriber node]

Topic keys identify individual publishers on a topic so nodes and tools can distinguish where messages come from.
Each topic key makes it easier to track data sources when several publishers share the same topic.

Services
--------

The service interface is meant for request/response interactions, for example, when you want to send a query requesting the configuration of a specific robot.
Service definitions are stored in ``.srv`` files.
Services implement a request/response pattern.
A client sends a request, and a server replies with a response.

This interface type has the following main characteristics:

* Communication is two-way: each request receives one response.
  Client libraries let you wait for the response asynchronously, or block until it arrives.
  Blocking on a service call from inside a callback can deadlock your node.
  See :doc:`interfaces/services/Working-with-services/Sync-Vs-Async`.
* Services are ideal for remote procedure calls that terminate quickly and return a result or confirmation, such as querying the state of a node or doing a quick calculation such as IK.
* Services should not be used for longer running processes, in particular processes that might need to be preempted if exceptional situations occur.
  Use an action instead.

.. mermaid::

   sequenceDiagram
    participant Service client
    participant Service server
    Service client->>Service server: Request
    Service server-->>Service client: Response

Actions
-------

The action interface is meant for long-running tasks with feedback, for example, moving a robot to a specific position, or asking the robot to perform a complex motion.
Action definitions are stored in ``.action`` files.
Actions allow clients to send goals, receive feedback during the execution, cancel if needed, and return a result when the goal finishes.

This interface type has the following main characteristics:

* Communication is asynchronous, with feedback during execution and a final result.
* Actions are suitable for any discrete behavior that takes time and provides feedback during execution, such as moving a robot, slow perception routines that take several seconds to terminate, or initiating a lower-level control mode.
* Actions can be preempted (cancelled) and preemption should always be implemented cleanly by action servers.
* Actions can keep state for the lifetime of a goal.
  If executing two action goals in parallel on the same server, a separate state instance can be kept for each client because the goal is uniquely identified by its ID.

.. mermaid::

   sequenceDiagram
    participant c as Action client
    participant s as Action server
    c->>s: Sends a goal
    s-->>c: Provides feedback
    s-->>c: Sends a result

Key differences between ROS interfaces
--------------------------------------

All three interfaces enable communication between nodes, but each serves a different purpose.
The following table summarizes the differences between ROS interface types:

+--------------+----------------------+-----------------------+-----------------+--------------------+---------------+
|              | Pattern              | Direction             | Provided result | Typical use case   | Cancellation  |
+==============+======================+=======================+=================+====================+===============+
| **Topics**   | Publish/Subscribe    | One-way               | No              | Continuous data    | Not supported |
+--------------+----------------------+-----------------------+-----------------+--------------------+---------------+
| **Services** | Request/Response     | Two-way               | Yes             | Quick queries      | Not supported |
+--------------+----------------------+-----------------------+-----------------+--------------------+---------------+
| **Actions**  | Goal/Feedback/Result | Two-way with feedback | Yes             | Long-running tasks | Supported     |
+--------------+----------------------+-----------------------+-----------------+--------------------+---------------+

Related content
---------------

Related articles:

* :doc:`interfaces/Working-with-interfaces/Interface-specifications`
* :doc:`interfaces/About-Topics`
* :doc:`interfaces/About-Services`
* :doc:`interfaces/About-Actions`

.. ros-related-articles::

FAQs
----

What are the three primary ROS interface types?
   Topics, services, and actions.
   Topics suit continuous data streams, services suit short request/response interactions, and actions suit long-running tasks that provide feedback.

When should I use a topic?
   Use a topic for continuous, asynchronous data streams such as sensor data or robot state.
   Topics use a publish/subscribe pattern, and multiple publishers and subscribers can share the same topic.

When should I use a service?
   Use a service for a short request/response interaction that returns a result quickly, such as querying configuration or running a quick calculation.
   Do not use services for long-running processes that might need to be cancelled.

When should I use an action?
   Use an action for a longer-running task that may need feedback during execution, such as moving a robot to a pose.
   Actions let a client send a goal, receive feedback, cancel if needed, and get a final result.

Which interface types support cancellation?
   Only actions support cancellation.
   Topics and services do not.
