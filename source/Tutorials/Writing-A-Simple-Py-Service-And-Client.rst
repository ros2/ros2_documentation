.. _PySrvCli:

Writing a simple service and client (Python)
============================================

**Goal:** Create and run service and client nodes using Python.

**Tutorial level:** Beginner

**Time:** 20 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

When :ref:`nodes <ROS2Nodes>` communicate using :ref:`services <ROS2Services>`, the node that sends a request for data is called the client node, and the one that responds to the request is the service node.
The structure of the request and response is determined by a ``.srv`` file.

The example used here is a simple integer addition system; one node requests the sum of two integers, and the other responds with the result.

Prerequisites
-------------

In previous tutorials, you learned how to :ref:`create a workspace <ROS2Workspace>` and :ref:`create a package <CreatePkg>`.

Tasks
-----

1 Create a package
^^^^^^^^^^^^^^^^^^

Open a new terminal and :ref:`source your ROS 2 installation <ConfigROS2>` so that ``ros2`` commands will work.

Navigate into the ``dev_ws`` directory created in a previous tutorial.

Recall that packages should be created in the ``src`` directory, not the root of the workspace.
Navigate into ``dev_ws/src`` and create a new package:

.. code-block:: console

  ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces

Your terminal will return a message verifying the creation of your package ``py_srvcli`` and all its necessary files and folders.

The ``--dependencies`` argument will automatically add the necessary dependency lines to ``package.xml``.
``example_interfaces`` is the package that includes `the .srv file <https://github.com/ros2/example_interfaces/blob/master/srv/AddTwoInts.srv>`__ you will need to structure your requests and responses:

.. code-block:: console

    int64 a
    int64 b
    ---
    int64 sum

The first two lines are the parameters of the request, and below the dashes is the response.

1.1 Update ``package.xml``
~~~~~~~~~~~~~~~~~~~~~~~~~~

Because you used the ``--dependencies`` option during package creation, you don’t have to manually add dependencies to ``package.xml``.

As always, though, make sure to add the description, maintainer email and name, and license information to ``package.xml``.

.. code-block:: xml

  <description>Python client server tutorial</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

1.2 Update ``setup.py``
~~~~~~~~~~~~~~~~~~~~~~~

Add the same information to the ``setup.py`` file for the ``maintainer``, ``maintainer_email``, ``description`` and ``license`` fields:

.. code-block:: python

    maintainer='Your Name',
    maintainer_email='you@email.com',
    description='Python client server tutorial',
    license='Apache License 2.0',

2 Write the service node
^^^^^^^^^^^^^^^^^^^^^^^^

2.1 Add the source file for the service node
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Create a new file ``service_member_function.py`` in ``dev_ws/src/py_srvcli/py_srvcli`` with the following content:

.. code-block:: python

  from example_interfaces.srv import AddTwoInts
  import rclpy
  from rclpy.node import Node


  class MinimalService(Node):

      def __init__(self):
          super().__init__('minimal_service')
          self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

      def add_two_ints_callback(self, request, response):
          response.sum = request.a + request.b
          self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

          return response


  def main():
      rclpy.init()

      minimal_service = MinimalService()

      rclpy.spin(minimal_service)

      rclpy.shutdown()


  if __name__ == '__main__':
      main()

*Examine the code*

The first ``import`` statement imports the ``AddTwoInts`` service type from the ``example_interfaces`` package.
The following ``import`` statement imports the ROS 2 Python client library, and specifically the ``Node`` class.

.. code-block:: python

  from example_interfaces.srv import AddTwoInts

  import rclpy
  from rclpy.node import Node

The ``MinimalService`` class constructor initializes the node with the name ``minimal_service``.
Then, it creates a service and defines the type, name, and callback.

.. code-block:: python

  def __init__(self):
      super().__init__('minimal_service')
      self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

The definition of the service callback receives the request data, sums it, and returns the sum as a response.

.. code-block:: python

  def add_two_ints_callback(self, request, response):
      response.sum = request.a + request.b
      self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

      return response

Finally, the main class initializes the ROS 2 Python client library, instantiates the ``MinimalService`` class to create the service node and spins the node to handle callbacks.

2.2 Add an entry point
~~~~~~~~~~~~~~~~~~~~~~

To allow the ``ros2 run`` command to run your node, you must add the entry point to ``setup.py`` (located in the ``dev_ws/src/py_srvcli`` directory).

Add the following line between the ``'console_scripts':`` brackets:

.. code-block:: python

  'service = py_srvcli.service_member_function:main',

3 Write the client node
^^^^^^^^^^^^^^^^^^^^^^^

3.1 Add the source file for the synchronous client node
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A synchronous client will block upon sending a request to a service until a response has been received and then returns it.

Create a new file ``synchronous_client_member_function.py`` in ``dev_ws/src/py_srvcli/py_srvcli`` with the following content:

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

*Examine the code*

The synchronous client has two additional ``import`` statements.
``import sys`` is simply for `sys.argv <https://docs.python.org/3/library/sys.html#sys.argv>`__, so that the client can access the command line input arguments to set the service request based on user input.
``from threading import Thread`` is needed because both ``send_request`` and ``rclpy.spin`` are blocking, so they need to be on separate threads.

The constructor definition creates a client with the same type and name as the service node.
The type and name must match for the client and service to be able to communicate.

The ``while`` loop in the constructor checks if a service matching the type and name of the client is available once a second.

Below the constructor is the request definition, followed by ``main``.

The differences between the ``main`` of the synchronous client and the service node is that the client does ``rclpy.spin`` in a separate thread and also handles the response to a request by writing the result to a log message.

Note that it is not possible to call a service synchronously in a callback.
For example, if the synchronous client's ``send_request`` is placed in a callback:

.. code-block:: python

  def trigger_request(msg):
      response = minimal_client.send_request()  # this will deadlock!
      minimal_client.get_logger().info(
          'Result of add_two_ints: for %d + %d = %d' %
          (minimal_client.req.a, minimal_client.req.b, response.sum))
  subscription = minimal_client.create_subscription(String, 'trigger', trigger_request, 10)

  rclpy.spin(minimal_client)

There will be a deadlock because ``rclpy.spin`` will not preempt the callback with the ``send_request`` call.
In general, callbacks should only perform light and fast operations.

3.2 Add the source file for the asynchronous client node
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

An asynchronous client will immediately return a ``future`` after sending a request to a service.
The returned ``future`` may be queried for a response later.

Create a new file ``client_member_function.py`` in ``dev_ws/src/py_srvcli/py_srvcli`` with the following content:

.. code-block:: python

  import sys

  from example_interfaces.srv import AddTwoInts
  import rclpy
  from rclpy.node import Node


  class MinimalClientAsync(Node):

      def __init__(self):
          super().__init__('minimal_client_async')
          self.cli = self.create_client(AddTwoInts, 'add_two_ints')
          while not self.cli.wait_for_service(timeout_sec=1.0):
              self.get_logger().info('service not available, waiting again...')
          self.req = AddTwoInts.Request()

      def send_request(self):
          self.req.a = int(sys.argv[1])
          self.req.b = int(sys.argv[2])
          return self.cli.call_async(self.req)


  def main():
      rclpy.init()

      minimal_client = MinimalClientAsync()
      future = minimal_client.send_request()

      while rclpy.ok():
          rclpy.spin_once(minimal_client)
          if future.done():
              try:
                  response = future.result()
              except Exception as e:
                  minimal_client.get_logger().info(
                      'Service call failed %r' % (e,))
              else:
                  minimal_client.get_logger().info(
                      'Result of add_two_ints: for %d + %d = %d' %
                      (minimal_client.req.a, minimal_client.req.b, response.sum))
              break

      minimal_client.destroy_node()
      rclpy.shutdown()


  if __name__ == '__main__':
      main()

*Examine the code*

The asynchronous client also has ``import sys`` to make use of `sys.argv <https://docs.python.org/3/library/sys.html#sys.argv>`__ the same way as the synchronous client.

The only significant difference between the ``main`` of the asynchronous and synchronous clients is the ``while`` loop.
Since ``send_request`` doesn't block in this case, a loop can be used to both do ``rclpy.spin_once`` and check whether a response from the service has been received, using the same thread.
If the service has sent a response while the system is still running, the result will be written in a log message.

3.3 Add entry points
~~~~~~~~~~~~~~~~~~~~

Like the service node, you also have to add some entry points to be able to run the client nodes.

The ``entry_points`` field of your ``setup.py`` file should look like this:

.. code-block:: python

  entry_points={
      'console_scripts': [
          'service = py_srvcli.service_member_function:main',
          'client = py_srvcli.client_member_function:main',
          'synchronous_client = py_srvcli.synchronous_client_member_function:main',
      ],
  },

4 Build and run
^^^^^^^^^^^^^^^

It's good practice to run ``rosdep`` in the root of your workspace (``dev_ws``) to check for missing dependencies before building:

.. code-block:: console

  sudo rosdep install -i --from-path src --rosdistro <distro> -y

Navigate back to the root of your workspace, ``dev_ws``, and build your new package:

.. code-block:: console

  colcon build --packages-select py_srvcli

Open a new terminal, navigate to ``dev_ws``, and source the setup files:

.. code-block:: console

  . install/setup.bash

Now run the service node:

.. code-block:: console

  ros2 run py_srvcli service

The node will wait for the client’s request.

Open another terminal and source the setup files from inside ``dev_ws`` again.
Start the client node, followed by any two integers separated by a space:

.. code-block:: console

  ros2 run py_srvcli client 2 3   # or
  ros2 run py_srvcli synchronous_client 2 3

If you chose ``2`` and ``3``, for example, the client would receive a response like this:

.. code-block:: console

  [INFO] [minimal_client_async]: Result of add_two_ints: for 2 + 3 = 5

Return to the terminal where your service node is running.
You will see that it published log messages when it received the request:

.. code-block:: console

  [INFO] [minimal_service]: Incoming request
  a: 2 b: 3

Enter ``Ctrl+C`` in the server terminal to stop the node from spinning.

Summary
-------

You created two nodes to request and respond to data over a service.
You added their dependencies and executables to the package configuration files so that you could build and run them, allowing you to see a service/client system at work.

Next steps
----------

Now that you have some packages and ROS 2 systems of your own, the :ref:`next tutorial <Ros2Doctor>` will show you how to examine issues in your environment and systems in case you have problems.
