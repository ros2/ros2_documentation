.. _TopicsServicesActions:

Topics vs Services vs Actions
=============================

.. contents:: Contents
   :depth: 1
   :local:

When designing a system there are three primary styles of interfaces.
The specifications for the content is in the :doc:`Interfaces Overview <../Concepts/About-ROS-Interfaces>`.
This is written to provide the reader with guidelines about when to use each type of interface.

Topics
------

* Should be used for continuous data streams (sensor data, robot state, ...).
* Are for continuous data flow. Data might be published and subscribed at any time independent of any senders/receivers. Many to many connection. Callbacks receive data once it is available. The publisher decides when data is sent.

Services
--------

* Should be used for remote procedure calls that terminate quickly, e.g. for querying the state of a node or doing a quick calculation such as IK. They should never be used for longer running processes, in particular processes that might be required to preempt if exceptional situations occur and they should never change or depend on state to avoid unwanted side effects for other nodes.
* Simple blocking call. Mostly used for comparably fast tasks as requesting specific data. Semantically for processing requests.

Actions
-------

* Should be used for any discrete behavior that moves a robot or that runs for a longer time but provides feedback during execution.
* The most important property of actions is that they can be preempted and preemption should always be implemented cleanly by action servers.
* Actions can keep state for the lifetime of a goal, i.e. if executing two action goals in parallel on the same server, for each client a separate state instance can be kept since the goal is uniquely identified by its id.
* Slow perception routines which take several seconds to terminate or initiating a lower-level control mode are good use cases for actions.
* More complex non-blocking background processing. Used for longer tasks like execution of robot actions. Semantically for real-world actions.
