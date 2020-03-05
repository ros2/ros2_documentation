.. _SyncAsync:

Synchronous vs. asynchronous clients
====================================

**Level:** Intermediate

**Time:** 20 minutes

.. contents:: Contents
   :depth: 2
   :local:


Introduction
------------

We recommend calling services asynchronously, using the ``call_async()`` API in ``rclpy``.
It is also possible to call services synchronously, using the ``call()`` API.
However, due to the possibility of deadlock accompanying it, we do not recommend synchronous service calls.

C++ service call API are only available in async, so the comparisons and examples in this guide pertain to Python services and clients.
The definition of async given here still applies to C++ though.


1 Asynchronous calls
--------------------

Async calls can be made from anywhere without running the risk of blocking other ROS and non-ROS processes.
This is not the case for sync calls, which is why async is the more correct and safe option.

An asynchronous client will immediately return a ``future``, a value that indicates whether the call and response is finished (not the value of the response itself), after sending a request to a service.
The returned ``future`` may be queried for a response at any time.

Since sending a request doesn’t block anything, a loop can be used to both spin ``rclpy`` and check ``future`` in the same thread, for example:

.. code-block::

    while rclpy.ok():
        rclpy.spin_once(node)
        if future.done():
            #Get response

The :ref:`Simple Service and Client <PySrvCli>` tutorial for Python illustrates how to perform an async service call and retrieve the ``future`` using a loop.

The ``future`` can also be retrieved using a timer or callback, like in `this example <https://github.com/ros2/examples/blob/master/rclpy/services/minimal_client/examples_rclpy_minimal_client/client_async_callback.py>`_, a dedicated thread, or by another method.
It is up to you, as the caller, to decide how to check on and store ``future`` and retrieve your response.

2 Synchronous calls
-------------------

A synchronous client will block upon sending a request to a service until a response has been received, and then returns the response.
There is no concept of a ``future`` value for sync calls.
The response will return immediately, making synchronous nodes much simpler to code.
However, you do have to use a separate thread to spin.

The following is an example of a synchronous client node, similar to the async node in the :ref:`Simple Service and Client <PySrvCli>` tutorial.

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
The statement ``from threading import Thread`` makes it possible to create a new thread.

2 Sync deadlock
---------------

It is not possible to call a service synchronously in a subscription, timer callback or service callback.
Doing so would block ``rclpy.spin``, causing deadlock: when a client is blocking a thread waiting for a response, but the response can only be returned on that same thread, so the client will never stop waiting.

When deadlock occurs, you will not receive any indication that the service is blocked.
There will be no warning or exception thrown, no indication in the stack trace, and the call will not fail.

One example of how deadlock might occur is if the synchronous client's ``send_request`` is placed in a callback:

.. code-block:: python

  def trigger_request(msg):
      response = minimal_client.send_request()  # this will deadlock!
      minimal_client.get_logger().info(
          'Result of add_two_ints: for %d + %d = %d' %
          (minimal_client.req.a, minimal_client.req.b, response.sum))
  subscription = minimal_client.create_subscription(String, 'trigger', trigger_request, 10)

  rclpy.spin(minimal_client)

Deadlock occurs because ``rclpy.spin`` will not preempt the callback with the ``send_request`` call.
In general, callbacks should only perform light and fast operations.

.. mention multithreaded executor here, once there's documentation for that
