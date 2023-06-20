Services
========

.. contents:: Table of Contents
   :local:

In ROS 2, a service refers to a remote procedure call.
In other words, a node can make a remote procedure call to another node which will do a computation and return a result.

This structure is reflected in how a service message definition looks:

.. code::

   uint32 request
   ---
   uint32 response

In ROS 2, services are expected to return quickly, as the client is generally waiting on the result.
Services should never be used for longer running processes, in particular processes that might need to be preempted for exceptional situations.
If you have a service that will be doing a long-running computation, consider using an :doc:`action <About-Actions>` instead.

Services are identified by a service name, which looks much like a topic name (but is in a different namespace).

A service consists of two parts: the service server and the service client.

Service server
--------------

A service server is the entity that will accept a remote procedure request, and perform some computation on it.
For instance, suppose the ROS 2 message contains the following:

.. code::

   uint32 a
   uint32 b
   ---
   uint32 sum

The service server would be the entity that receives this message, adds ``a`` and ``b`` together, and returns the ``sum``.

.. note::

   There should only ever be one service server per service name.
   It is undefined which service server will receive client requests in the case of multiple service servers on the same service name.

Service client
--------------

A service client is an entity that will request a remote service server to perform a computation on its behalf.
Following from the example above, the service client is the entity that creates the initial message containing ``a`` and ``b``, and waits for the service server to compute the sum and return the result.

Unlike the service server, there can be arbitrary numbers of service clients using the same service name.
