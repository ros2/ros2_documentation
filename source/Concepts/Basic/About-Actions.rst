Actions
=======

.. contents:: Table of Contents
   :local:

In ROS 2, an action refers to a long-running remote procedure call with feedback and the ability to cancel or preempt the goal.
For instance, the high-level state machine running a robot may call an action to tell the navigation subsystem to travel to a waypoint, which may take several seconds (or minutes) to do.
Along the way, the navigation subsystem can provide feedback on how far along it is, and the high-level state machine has the option to cancel or preempt the travel to that waypoint.

This structure is reflected in how an action message definition looks:

.. code::

   int32 request
   ---
   int32 response
   ---
   int32 feedback

In ROS 2, actions are expected to be long running procedures, as there is overhead in setting up and monitoring the connection.
If you need a short running remote procedure call, consider using a :doc:`service <About-Services>` instead.

Actions are identified by an action name, which looks much like a topic name (but is in a different namespace).

An action consists of two parts: the action server and the action client.

Action server
-------------

The action server is the entity that will accept the remote procedure request and perform some procedure on it.
It is also responsible for sending out feedback as the action progresses and should react to cancellation/preemption requests.
For instance, consider an action to calculate the Fibonacci sequence with the following interface:

.. code::

   int32 order
   ---
   int32[] sequence
   ---
   int32[] sequence

The action server is the entity that receives this message, starts calculating the sequence up to ``order`` (providing feedback along the way), and finally returns a full result in ``sequence``.

.. note::

   There should only ever be one action server per action name.
   It is undefined which action server will receive client requests in the case of multiple action servers on the same action name.

Action client
-------------

An action client is an entity that will request a remote action server to perform a procedure on its behalf.
Following the example above, the action client is the entity that creates the initial message containing the ``order``, and waits for the action server to compute the sequence and return it (with feedback along the way).

Unlike the action server, there can be arbitrary numbers of action clients using the same action name.
