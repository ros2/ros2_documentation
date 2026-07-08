Writing an async node with asyncio (Python)
===========================================

**Goal:** Create and run a service and client using ``AsyncNode``, the asyncio-native node API.

**Tutorial level:** Intermediate

**Time:** 20 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

``AsyncNode`` is an asyncio-native alternative to the executor based ``Node`` you used in the beginner tutorial.
This lets you ``await`` other async operations from inside a ROS 2 callback without blocking the rest of the node.

Beyond services and clients, ``AsyncNode`` supports timers, subscriptions, and publishers as well.
Callbacks can be either ``async def`` coroutines or regular ``def`` functions.
Keep in mind that a sync callback can't ``await`` anything.

.. note::

   Actions are not yet supported.

.. note::

   Async support already exists in ``Node``: entities accept ``async def`` callbacks, and futures from ``client.call_async()`` can be awaited (see `client_async_callback example <https://github.com/ros2/examples/blob/{REPOS_FILE_BRANCH}/rclpy/services/minimal_client/examples_rclpy_minimal_client/client_async_callback.py>`__).
   Where ``AsyncNode`` differs is in the underlying runtime: callbacks run on the ``asyncio`` event loop rather than rclpy's custom executor, allowing it to natively compose with other libraries within the ``asyncio`` ecosystem.

Prerequisites
-------------

- You should have completed the :doc:`beginner service and client tutorial <../../client-libraries/Working-with-Client-Libraries/Writing-A-Simple-Py-Service-And-Client>`.
- You should be comfortable with basic :py:mod:`asyncio` concepts like `async def and await <https://docs.python.org/3/howto/a-conceptual-overview-of-asyncio.html>`_ and :py:func:`asyncio.run`.
- ``AsyncNode`` currently lives in ``rclpy.experimental`` and requires Python 3.12 or newer.

Tasks
-----

1 Create a package
^^^^^^^^^^^^^^^^^^

Open a new terminal and :doc:`source your ROS 2 installation <../../../Get-Started/Configuring-ROS2-Environment>` so that ``ros2`` commands will work.

Navigate into the ``ros2_ws/src`` directory created in a :ref:`previous tutorial <new-directory>` and create a new package:

.. code-block:: console

  $ ros2 pkg create --build-type ament_python python_async_node --dependencies rclpy example_interfaces

2 Write the service node
^^^^^^^^^^^^^^^^^^^^^^^^

Inside the ``ros2_ws/src/python_async_node/python_async_node`` directory, create a new file called ``async_service.py`` and paste the following code within:

.. code-block:: python

  import asyncio

  from example_interfaces.srv import Trigger
  import rclpy
  from rclpy.experimental import AsyncNode


  class TriggerServer(AsyncNode):

      def __init__(self):
          super().__init__('trigger_server')
          self._service = self.create_service(
              Trigger, 'trigger', self._callback
          )

      async def _callback(self, _request, response):
          self.get_logger().info('Incoming trigger request')
          await self.get_clock().sleep(2.0)
          response.success = True
          response.message = 'Slept for 2.0s'
          return response


  async def _async_main():
      with rclpy.init():
          node = TriggerServer()
          await node.run()


  def main():
      asyncio.run(_async_main())


  if __name__ == '__main__':
      main()

2.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

The first ``import`` statements pull in ``asyncio``, ``rclpy``, the ``AsyncNode`` class from ``rclpy.experimental``, and the ``Trigger`` service type.

.. code-block:: python

  import asyncio

  from example_interfaces.srv import Trigger
  import rclpy
  from rclpy.experimental import AsyncNode

The ``TriggerServer`` class inherits from ``AsyncNode`` instead of ``Node``.
Its constructor initializes the node with the name ``trigger_server`` and creates a service.

.. code-block:: python

  class TriggerServer(AsyncNode):

      def __init__(self):
          super().__init__('trigger_server')
          self._service = self.create_service(
              Trigger, 'trigger', self._callback
          )

The service callback is a coroutine.
Inside it you can ``await`` any async operation — here the node calls ``self.get_clock().sleep(2.0)``, similar to ``asyncio.sleep`` but with ROS sim time support.
While this coroutine is suspended, the node remains responsive to other callbacks.

.. code-block:: python

  async def _callback(self, _request, response):
      self.get_logger().info('Incoming trigger request')
      await self.get_clock().sleep(2.0)
      response.success = True
      response.message = 'Slept for 2.0s'
      return response

The bottom of the file defines an ``async`` helper that initializes ``rclpy``, constructs the node, and runs it:

.. code-block:: python

  async def _async_main():
      with rclpy.init():
          node = TriggerServer()
          await node.run()

To run an ``AsyncNode``, you need to ``await node.run()`` — the equivalent of ``rclpy.spin(node)``.

Finally, ``main`` itself is a regular sync function so that ``ros2 run`` can use it as an entry point — ``ros2 run`` invokes the entry point as a regular function, so it cannot be ``async def``.
It just calls ``asyncio.run`` to execute the ``_async_main`` coroutine:

.. code-block:: python

  def main():
      asyncio.run(_async_main())

3 Write the client node
^^^^^^^^^^^^^^^^^^^^^^^

Inside the ``ros2_ws/src/python_async_node/python_async_node`` directory, create a new file called ``async_client.py`` and paste the following code within:

.. code-block:: python

  import asyncio

  from example_interfaces.srv import Trigger
  import rclpy
  from rclpy.experimental import AsyncNode


  async def _async_main():
      with rclpy.init():
          async with AsyncNode('trigger_client') as node:
              client = node.create_client(Trigger, 'trigger')

              node.get_logger().info('Waiting for trigger service...')
              await client.wait_for_service()
              node.get_logger().info('Service is available, sending request')

              request = Trigger.Request()
              response = await client.call(request)

              node.get_logger().info(
                  f'Service responded: success={response.success}, '
                  f'message="{response.message}"'
              )


  def main():
      asyncio.run(_async_main())


  if __name__ == '__main__':
      main()

3.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

As with the service code, the imports pull in ``asyncio``, ``rclpy``, ``AsyncNode``, and the ``Trigger`` type.

The client uses the second ``AsyncNode`` entry point: ``async with AsyncNode(...) as node:``.
Entering the ``async with`` block is what actually runs the node — analogous to ``await node.run()`` on the server side.
At that point any subscriptions, services, and timers that were already created on the node start dispatching callbacks.
The node keeps running for the duration of the block, and when the block exits ``destroy_node()`` is called automatically and all entities are torn down.
This is the right choice for short programs like a one shot client.

.. code-block:: python

  async with AsyncNode('trigger_client') as node:
      client = node.create_client(Trigger, 'trigger')

Before sending the request, ``await client.wait_for_service()`` suspends the coroutine until a matching service is discovered (or returns immediately if one is already up) — without blocking the event loop, so other callbacks on the node continue to run.
This avoids the "client started before the server" failure mode where a request would otherwise be sent into the void.

.. code-block:: python

  node.get_logger().info('Waiting for trigger service...')
  await client.wait_for_service()
  node.get_logger().info('Service is available, sending request')

The actual service call is one line:

.. code-block:: python

  request = Trigger.Request()
  response = await client.call(request)

With ``AsyncNode`` the response is awaited directly — there is no ``Future``, and the node continues to process other work while the call is in flight.

4 Add entry points
^^^^^^^^^^^^^^^^^^

To allow the ``ros2 run`` command to run your nodes, add entry points to ``setup.py`` (located in the ``ros2_ws/src/python_async_node`` directory).

The ``entry_points`` field of your ``setup.py`` should look like this:

.. code-block:: python

  entry_points={
      'console_scripts': [
          'service = python_async_node.async_service:main',
          'client = python_async_node.async_client:main',
      ],
  },

5 Build and run
^^^^^^^^^^^^^^^

It's good practice to run ``rosdep`` in the root of your workspace (``ros2_ws``) to check for missing dependencies before building:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

            $ rosdep install -i --from-path src --rosdistro {DISTRO} -y

   .. group-tab:: macOS

      rosdep only runs on Linux, so you can skip ahead to the next step.

   .. group-tab:: Windows

      rosdep only runs on Linux, so you can skip ahead to next step.

Navigate back to the root of your workspace, ``ros2_ws``, and build your new package:

.. code-block:: console

  $ colcon build --symlink-install --packages-select python_async_node

The ``--symlink-install`` flag symlinks the Python source files into the install space instead of copying them, so later edits take effect without rebuilding.

Open a new terminal, navigate to ``ros2_ws``, and source the setup files:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      $ source install/setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      $ . install/setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      $ call install/setup.bat

Now run the service node:

.. code-block:: console

  $ ros2 run python_async_node service

The node will wait for the client's request.

Open another terminal and source the setup files from inside ``ros2_ws`` again.
Start the client node:

.. code-block:: console

  $ ros2 run python_async_node client
  [INFO] [trigger_client]: Service responded: success=True, message="Slept for 2.0s"

Return to the terminal where your service node is running.
You will see that it logged the incoming request:

.. code-block:: console

  [INFO] [trigger_server]: Incoming trigger request

Press :kbd:`Ctrl-C` in the server terminal to stop the node.

Extensions
----------

The basic ``Trigger`` server above uses ``await clock.sleep(2.0)`` as an example for async work.
Two more substantial use cases are shown below — both are straightforward modifications to ``async_service.py``.

Because the package was built with ``--symlink-install``, edits to the Python file take effect without rebuilding — stop the running service with :kbd:`Ctrl-C` and start it again to pick up the changes.

Executing a blocking function from a callback
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. note::

   Just like ``asyncio``, ``AsyncNode`` is not thread-safe.
   If you spawn your own thread, do not access ``AsyncNode`` API from it.
   Schedule that work back onto the event loop thread with ``loop.call_soon_threadsafe`` or ``asyncio.run_coroutine_threadsafe``.

Many useful libraries are still written with synchronous APIs.
Calling them directly from an async callback would block the event loop and stall every other callback on the node.
To work around this, ``asyncio.to_thread`` runs the sync function on a worker thread and returns an awaitable, allowing the node to stay responsive.

For example, we can make a synchronous HTTP request to fetch the latest ROS 2 release tag from GitHub through ``asyncio.to_thread``.
Replace the body of ``async_service.py`` with the following:

.. code-block:: python

  import asyncio
  import json
  import urllib.request

  from example_interfaces.srv import Trigger
  import rclpy
  from rclpy.experimental import AsyncNode

  ROS2_LATEST_RELEASE_URL = 'https://api.github.com/repos/ros2/ros2/releases/latest'


  class TriggerServer(AsyncNode):

      def __init__(self):
          super().__init__('trigger_server')
          self._service = self.create_service(
              Trigger, 'trigger', self._callback
          )

      def _fetch_latest_release(self):
          self.get_logger().info('Fetching latest ROS 2 release from GitHub')
          with urllib.request.urlopen(ROS2_LATEST_RELEASE_URL) as resp:
              return json.loads(resp.read())['tag_name']

      async def _callback(self, _request, response):
          tag = await asyncio.to_thread(self._fetch_latest_release)
          response.success = True
          response.message = f'Latest ROS 2 release: {tag}'
          return response


  async def _async_main():
      with rclpy.init():
          node = TriggerServer()
          await node.run()


  def main():
      asyncio.run(_async_main())


  if __name__ == '__main__':
      main()

Restart the service in one terminal, then run the client in another:

.. code-block:: console

  $ ros2 run python_async_node service

.. code-block:: console

  $ ros2 run python_async_node client
  [INFO] [trigger_client]: Service responded: success=True, message="Latest ROS 2 release: <release-tag>"

Calling another service from inside a callback
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Because callbacks are coroutines, they can ``await`` other ROS 2 service calls just like any other async operation.

Replace the body of ``async_service.py`` so that the ``Trigger`` callback delegates to ``demo_nodes_py``'s ``add_two_ints`` service:

.. code-block:: python

  import asyncio

  from example_interfaces.srv import AddTwoInts, Trigger
  import rclpy
  from rclpy.experimental import AsyncNode


  class TriggerServer(AsyncNode):

      def __init__(self):
          super().__init__('trigger_server')
          self._client = self.create_client(AddTwoInts, 'add_two_ints')
          self._service = self.create_service(
              Trigger, 'trigger', self._callback
          )

      async def _callback(self, _request, response):
          self.get_logger().info('Waiting for add_two_ints service...')
          await self._client.wait_for_service()

          self.get_logger().info('Calling add_two_ints with a=2, b=2')
          nested_request = AddTwoInts.Request(a=2, b=2)
          nested_response = await self._client.call(nested_request)

          response.success = True
          response.message = f'2+2={nested_response.sum}'
          return response


  async def _async_main():
      with rclpy.init():
          node = TriggerServer()
          await node.run()


  def main():
      asyncio.run(_async_main())


  if __name__ == '__main__':
      main()

The client is created in ``__init__`` and used inside the callback.
Because the call is awaited, the coroutine doesn't block the node — other callbacks continue to make progress while it waits for ``add_two_ints`` to respond.

.. note::

   The standard ``Node`` utilizes :doc:`callback groups <Using-callback-groups>` to manage callback execution.
   Any attempt to perform a service call from within a callback with the default ``MutuallyExclusiveCallbackGroup`` would result in deadlocking the node.
   With ``AsyncNode``, all callbacks run cooperatively on the ``asyncio`` event loop, so ``await client.call(request)`` works from any callback without additional configuration.

Run the underlying ``add_two_ints`` server in one terminal:

.. code-block:: console

  $ ros2 run demo_nodes_py add_two_ints_server

Restart your service and run the client as before:

.. code-block:: console

  $ ros2 run python_async_node service

.. code-block:: console

  $ ros2 run python_async_node client
  [INFO] [trigger_client]: Service responded: success=True, message="2+2=4"

Handling requests concurrently
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

By default, async services process one request at a time — if a callback is still running when a new request arrives, the new one waits.

Passing ``concurrent=True`` dispatches each incoming request on its own task, enabling multiple callbacks to run in parallel:

.. code-block:: python

  self._service = self.create_service(
      Trigger, 'trigger', self._callback, concurrent=True
  )

With concurrent dispatch, three clients arriving at the same time each get their response after roughly two seconds rather than queueing up.
The same flag is available on ``create_subscription``.

Summary
-------

You wrote an async service and client using ``AsyncNode``, with ``async def`` callbacks that can ``await`` long running operations.
Because ``AsyncNode`` runs on ``asyncio``, your node can directly use the rest of the Python async ecosystem — web frameworks, messaging clients, database drivers, and more.
