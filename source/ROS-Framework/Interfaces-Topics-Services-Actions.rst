.. redirect-from::

    About-ROS-Interfaces
    Concepts/Basic/About-Interfaces
    How-To-Guides/Topics-Services-Actions

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
   Interfaces in ROS define how nodes exchange data.
   This article explains the different types of ROS interface and the differences between them.
   With this information, you'll be able to select the right interfaces for your purposes.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. contents:: Table of Contents
   :depth: 2
   :local:

.. toctree::
   :maxdepth: 1
   :hidden:

   interfaces/About-Topics
   interfaces/About-Services
   interfaces/About-Actions
   interfaces/Working-with-interfaces

Summary
-------

ROS nodes typically communicate through the following three types of interfaces:

* :doc:`Topics <interfaces/About-Topics>`: For continuous data streams.
* :doc:`Services <interfaces/About-Services>`: For synchronous request/response interactions (short tasks which happen immediately).
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
* Multiple publishers and subscribers can share the same topic

.. mermaid::

   flowchart LR
    P[Publisher node] -->|Publishes messages| T[Topic]
    T -->|Delivers messages| S1[Subscriber node]
    T -->|Delivers messages| S2[Subscriber node]

Topic keys identify individual publishers on a topic so nodes and tools can distinguish where messages come from.
Each topic key makes it easier to track data sources when several publishers share the same topic.

Topic statistics
^^^^^^^^^^^^^^^^

Topic statistics are built-in measurements that help you understand how messages behave when a subscription receives them.
When enabled, they automatically track two things:

:Message age: How old a message is when it arrives, based on its timestamp.
:Message period: The time between incoming messages.

For both message age and period, ROS calculates the average, minimum, maximum, standard deviation, as well as the number of samples, using a moving window that is updated every time a new message arrives.
These calculations run in constant time and memory using the dedicated utilities.
When you enable topic statistics for a subscription, ROS publishes the collected data at regular intervals as a ``MetricsMessage`` on a statistics topic.
This gives you a clear view of timing patterns, delays, and irregularities, making it easier to assess system performance or diagnose problems related to the message flow.

.. tip::

   The default interval is 1 second.
   The default statistics topic is ``/statistics``.

Services
--------

The service interface is meant for synchronous request/response interactions, for example, when you want to send a query requesting the configuration of a specific robot.
Service definitions are stored in ``.srv`` files.
Services implement a request/response pattern.
A client sends a request, and a server replies with a response.

This interface type has the following main characteristics:

* Communication is synchronous
* Services are ideal for short-lived operations that require confirmation or provide a result in response to a request
* Services should be used for remote procedure calls that terminate quickly, such as for querying the state of a node or doing a quick calculation such as IK
* Services should never be used for longer running processes, in particular processes that might be required to preempt if exceptional situations occur. 
  They should never change or depend on state, to avoid unwanted side effects for other nodes

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

* Communication is asynchronous, with feedback and result
* Actions are suitable for operations that take time, such as slow perception routines which take several seconds to terminate, or for initiating a lower-level control mode.
* Actions should be used for any discrete behaviour that moves a robot or that runs for a longer time but provides feedback during execution.
* Actions can be preempted and preemption should always be implemented cleanly by action servers.
* Actions can keep state for the lifetime of a goal. 
  If executing two action goals in parallel on the same server, a separate state instance can be kept for each client because the goal is uniquely identified by its ID.
* Action support more complex non-blocking background processing, including longer tasks such as execution of robot actions.

.. mermaid::

   sequenceDiagram
    participant c as Action client
    participant s as Action server
    c->>s: Sends a goal
    s-->>c: Provides feedback (periodic)
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

* :doc:`interfaces/Working-with-interfaces/Interface-specifications`
* :doc:`interfaces/About-Topics`
* :doc:`interfaces/About-Services`
* :doc:`interfaces/About-Actions`

FAQs
----

What are the three primary ROS interface types?
   Topics, services, and actions.
   Topics suit continuous data streams, services suit short request/response interactions, and actions suit long-running tasks that provide feedback.

When should I use a topic?
   Use a topic for continuous, asynchronous data streams such as sensor data or robot state.
   Topics use a publish/subscribe pattern, and multiple publishers and subscribers can share the same topic.

When should I use a service?
   Use a service for a short, synchronous request/response interaction that returns a result quickly, such as querying configuration or running a quick calculation.
   Do not use services for long-running processes that might need to be cancelled.

When should I use an action?
   Use an action for a longer-running task that may need feedback during execution, such as moving a robot to a pose.
   Actions let a client send a goal, receive feedback, cancel if needed, and get a final result.

Which interface types support cancellation?
   Only actions support cancellation.
   Topics and services do not.
