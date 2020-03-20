.. _SyncAsync:

Synchronous vs. asynchronous service clients
============================================

**Level:** Intermediate

**Time:** 10 minutes

.. contents:: Contents
   :depth: 2
   :local:


Introduction
------------

This guide is intended to warn users of the risks associated with the Python synchronous service client ``call()`` API.
It is very easy to mistakenly cause deadlock when calling services synchronously, so we do not recommend using ``call()``.

We provide an example on how to use ``call()`` correctly for experienced users who wish to use synchronous calls and are aware of the pitfalls.
We also highlight possible scenarios for deadlock that accompany it.

Because we recommend avoiding sync calls, this guide will also address the features and usage of the recommended alternative, async calls (``call_async()``).

The C++ service call API is only available in async, so the comparisons and examples in this guide pertain to Python services and clients.
The definition of async given here generally applies to C++, with some exceptions.

1 Synchronous calls
-------------------

A synchronous client will block the calling thread when sending a request to a service until a response has been received; nothing else can happen on that thread during the call.
The call can take arbitrary amounts of time to complete.
Once complete, the response returns directly to the client.

The following is an example of how to correctly execute a synchronous client node, similar to the async node in the :ref:`Simple Service and Client <PySrvCli>` tutorial.

.. code-block:: python

  import sys
  from threading import Thread

  from example_interfaces.srv import AddTwoInts
  import rclpy
  from rclpy.node import Node

  class MinimalClientSync(Node):

      def __init__(self):
          super().__init__('minimal_client_sync')
          self.cli = self.create_client(AddTwoInts, 'add_two_ints')
          while not self.cli.wait_for_service(timeout_sec=1.0):
              self.get_logger().info('service not available, waiting again...')
          self.req = AddTwoInts.Request()

      def send_request(self):
          self.req.a = int(sys.argv[1])
          self.req.b = int(sys.argv[2])
          return self.cli.call(self.req)
          # This only works because rclpy.spin() is called in a separate thread below.
          # Another configuration, like spinning later in main() or calling this method from a timer callback, would result in a deadlock.

  def main():
      rclpy.init()

      minimal_client = MinimalClientSync()

      spin_thread = Thread(target=rclpy.spin, args=(minimal_client,))
      spin_thread.start()

      response = minimal_client.send_request()
      minimal_client.get_logger().info(
          'Result of add_two_ints: for %d + %d = %d' %
          (minimal_client.req.a, minimal_client.req.b, response.sum))

      minimal_client.destroy_node()
      rclpy.shutdown()


  if __name__ == '__main__':
      main()

Note inside ``main()`` that the client calls ``rclpy.spin`` in a separate thread.
Both ``send_request`` and ``rclpy.spin`` are blocking, so they need to be on separate threads.

1.1 Sync deadlock
-----------------

There are several ways that the synchronous ``call()`` API can cause deadlock.

As mentioned in the comments of the example above, failing to create a separate thread to spin ``rclpy`` is one cause of deadlock.
When a client is blocking a thread waiting for a response, but the response can only be returned on that same thread, the client will never stop waiting, and nothing else can happen.

Another cause of deadlock is blocking ``rclpy.spin`` by calling a service synchronously in a subscription, timer callback or service callback.
For example, if the synchronous client's ``send_request`` is placed in a callback:

.. code-block:: python

  def trigger_request(msg):
      response = minimal_client.send_request()  # This will cause deadlock
      minimal_client.get_logger().info(
          'Result of add_two_ints: for %d + %d = %d' %
          (minimal_client.req.a, minimal_client.req.b, response.sum))
  subscription = minimal_client.create_subscription(String, 'trigger', trigger_request, 10)

  rclpy.spin(minimal_client)

Deadlock occurs because ``rclpy.spin`` will not preempt the callback with the ``send_request`` call.
In general, callbacks should only perform light and fast operations.

.. warning::

  When deadlock occurs, you will not receive any indication that the service is blocked.
  There will be no warning or exception thrown, no indication in the stack trace, and the call will not fail.

2 Asynchronous calls
--------------------

Async calls in ``rclpy`` are entirely safe and the recommended method of calling services.
They can be made from anywhere without running the risk of blocking other ROS and non-ROS processes, unlike sync calls.

An asynchronous client will immediately return ``future``, a value that indicates whether the call and response is finished (not the value of the response itself), after sending a request to a service.
The returned ``future`` may be queried for a response at any time.

Since sending a request doesn’t block anything, a loop can be used to both spin ``rclpy`` and check ``future`` in the same thread, for example:

.. code-block:: python

    while rclpy.ok():
        rclpy.spin_once(node)
        if future.done():
            #Get response

The :ref:`Simple Service and Client <PySrvCli>` tutorial for Python illustrates how to perform an async service call and retrieve the ``future`` using a loop.

The ``future`` can also be retrieved using a timer or callback, like in `this example <https://github.com/ros2/examples/blob/master/rclpy/services/minimal_client/examples_rclpy_minimal_client/client_async_callback.py>`_, a dedicated thread, or by another method.
It is up to you, as the caller, to decide how to store ``future``, check on its status, and retrieve your response.

Summary
-------

It is not recommended to implement a synchronous service client.
They are susceptible to deadlock, but will not provide any indication of issue when deadlock occurs.
If you must use synchronous calls, the example in section `1 Synchronous calls`_ is a safe method of doing so.
You should also be aware of the conditions that cause deadlock outlined in section `1.1 Sync deadlock`_.
We recommend using async service clients instead.
