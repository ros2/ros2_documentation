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

Inside the ``dev_ws/src/py_srvcli/py_srvcli`` directory, create a new file called ``service_member_function.py`` and paste the following code within:

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


  def main(args=None):
      rclpy.init(args=args)

      minimal_service = MinimalService()

      rclpy.spin(minimal_service)

      rclpy.shutdown()


  if __name__ == '__main__':
      main()

2.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

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

Inside the ``dev_ws/src/py_srvcli/py_srvcli`` directory, create a new file called ``client_member_function.py`` and paste the following code within:

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
          self.future = self.cli.call_async(self.req)


  def main(args=None):
      rclpy.init(args=args)

      minimal_client = MinimalClientAsync()
      minimal_client.send_request()

      while rclpy.ok():
          rclpy.spin_once(minimal_client)
          if minimal_client.future.done():
              try:
                  response = minimal_client.future.result()
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


3.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

The only different ``import`` statement for the client is ``import sys``.
The client node code uses `sys.argv <https://docs.python.org/3/library/sys.html#sys.argv>`__ to get access to command line input arguments for the request.

The constructor definition creates a client with the same type and name as the service node.
The type and name must match for the client and service to be able to communicate.

The ``while`` loop in the constructor checks if a service matching the type and name of the client is available once a second.

Below the constructor is the request definition, followed by ``main``.

The only significant difference in the client’s ``main`` is the ``while`` loop.
The loop checks the ``future`` to see if there is a response from the service, as long as the system is running.
If the service has sent a response, the result will be written in a log message.


3.2 Add an entry point
~~~~~~~~~~~~~~~~~~~~~~

Like the service node, you also have to add an entry point to be able to run the client node.

The ``entry_points`` field of your ``setup.py`` file should look like this:

.. code-block:: python

  entry_points={
      'console_scripts': [
          'service = py_srvcli.service_member_function:main',
          'client = py_srvcli.client_member_function:main',
      ],
  },

4 Build and run
^^^^^^^^^^^^^^^

It's good practice to run ``rosdep`` in the root of your workspace (``dev_ws``) to check for missing dependencies before building:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

            rosdep install -i --from-path src --rosdistro <distro> -y

   .. group-tab:: macOS

      rosdep only runs on Linux, so you can skip ahead to next step.

   .. group-tab:: Windows

      rosdep only runs on Linux, so you can skip ahead to next step.


Navigate back to the root of your workspace, ``dev_ws``, and build your new package:

.. code-block:: console

  colcon build --packages-select py_srvcli

Open a new terminal, navigate to ``dev_ws``, and source the setup files:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      call install/setup.bat

Now run the service node:

.. code-block:: console

  ros2 run py_srvcli service

The node will wait for the client’s request.

Open another terminal and source the setup files from inside ``dev_ws`` again.
Start the client node, followed by any two integers separated by a space:

.. code-block:: console

  ros2 run py_srvcli client 2 3

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

In the last few tutorials you've been utilizing interfaces to pass data across topics and services.
Next, you'll learn how to :ref:`create custom interfaces <CustomInterfaces>`.

Related content
---------------

* There are several ways you could write a service and client in Python; check out the ``minimal_client`` and ``minimal_service`` packages in the `ros2/examples <https://github.com/ros2/examples/tree/master/rclpy/services>`_ repo.

* In this tutorial, you used the ``call_async()`` API in your client node to call the service.
  There is another service call API available for Python called synchronous calls.
  We do not recommend using synchronous calls, but if you'd like to learn more about them, read the guide to :ref:`Synchronous vs. asynchronous clients <SyncAsync>`.
