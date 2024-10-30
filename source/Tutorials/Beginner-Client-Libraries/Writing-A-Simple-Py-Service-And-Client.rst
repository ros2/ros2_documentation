.. redirect-from::

    Tutorials/Writing-A-Simple-Py-Service-And-Client

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

When :doc:`nodes <../Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes>` communicate using :doc:`services <../Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services>`, the node that sends a request for data is called the client node, and the one that responds to the request is the service node.
The structure of the request and response is determined by a ``.srv`` file.

The example used here is a simple integer addition system; one node requests the sum of two integers, and the other responds with the result.

Prerequisites
-------------

In previous tutorials, you learned how to :doc:`create a workspace <./Creating-A-Workspace/Creating-A-Workspace>` and :doc:`create a package <./Creating-Your-First-ROS2-Package>`.

Tasks
-----

1 Create a package
^^^^^^^^^^^^^^^^^^

Open a new terminal and :doc:`source your ROS 2 installation <../Beginner-CLI-Tools/Configuring-ROS2-Environment>` so that ``ros2`` commands will work.

Navigate into the ``ros2_ws`` directory created in a :ref:`previous tutorial <new-directory>`.

Recall that packages should be created in the ``src`` directory, not the root of the workspace.
Navigate into ``ros2_ws/src`` and create a new package:

.. code-block:: console

  ros2 pkg create --build-type ament_python --license Apache-2.0 py_srvcli --dependencies rclpy example_interfaces

Your terminal will return a message verifying the creation of your package ``py_srvcli`` and all its necessary files and folders.

The ``--dependencies`` argument will automatically add the necessary dependency lines to ``package.xml``.
``example_interfaces`` is the package that includes `the .srv file <https://github.com/ros2/example_interfaces/blob/{REPOS_FILE_BRANCH}/srv/AddTwoInts.srv>`__ you will need to structure your requests and responses:

.. code-block:: console

    int64 a
    int64 b
    ---
    int64 sum

The first two lines are the parameters of the request, and below the dashes is the response.

1.1 Update ``package.xml``
~~~~~~~~~~~~~~~~~~~~~~~~~~

Because you used the ``--dependencies`` option during package creation, you don't have to manually add dependencies to ``package.xml``.

As always, though, make sure to add the description, maintainer email and name, and license information to ``package.xml``.

.. code-block:: xml

  <description>Python client server tutorial</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>Apache-2.0</license>

1.2 Update ``setup.py``
~~~~~~~~~~~~~~~~~~~~~~~

Add the same information to the ``setup.py`` file for the ``maintainer``, ``maintainer_email``, ``description`` and ``license`` fields:

.. code-block:: python

    maintainer='Your Name',
    maintainer_email='you@email.com',
    description='Python client server tutorial',
    license='Apache-2.0',

2 Write the service node
^^^^^^^^^^^^^^^^^^^^^^^^

Inside the ``ros2_ws/src/py_srvcli/py_srvcli`` directory, create a new file called ``service_member_function.py`` and paste the following code within:

.. code-block:: python

  from example_interfaces.srv import AddTwoInts

  import rclpy
  from rclpy.executors import ExternalShutdownException
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
      try:
          with rclpy.init():
              minimal_service = MinimalService()

              rclpy.spin(minimal_service)
      except (KeyboardInterrupt, ExternalShutdownException):
          pass


  if __name__ == '__main__':
      main()

2.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

The first ``import`` statement imports the ``AddTwoInts`` service type from the ``example_interfaces`` package.
The following ``import`` statements import the necessary ROS 2 Python client library interfaces.

.. code-block:: python

  from example_interfaces.srv import AddTwoInts

  import rclpy
  from rclpy.executors import ExternalShutdownException
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

To allow the ``ros2 run`` command to run your node, you must add the entry point to ``setup.py`` (located in the ``ros2_ws/src/py_srvcli`` directory).

Add the following line between the ``'console_scripts':`` brackets:

.. code-block:: python

  'service = py_srvcli.service_member_function:main',

3 Write the client node
^^^^^^^^^^^^^^^^^^^^^^^

Inside the ``ros2_ws/src/py_srvcli/py_srvcli`` directory, create a new file called ``client_member_function.py`` and paste the following code within:

.. code-block:: python

  from example_interfaces.srv import AddTwoInts

  import rclpy
  from rclpy.executors import ExternalShutdownException
  from rclpy.node import Node


  class MinimalClientAsync(Node):

      def __init__(self):
          super().__init__('minimal_client_async')
          self.cli = self.create_client(AddTwoInts, 'add_two_ints')
          while not self.cli.wait_for_service(timeout_sec=1.0):
              self.get_logger().info('service not available, waiting again...')
          self.req = AddTwoInts.Request()

      def send_request(self):
          self.req.a = 41
          self.req.b = 1
          return self.cli.call_async(self.req)


  def main(args=None):
      try:
          with rclpy.init(args=args):
              minimal_client = MinimalClientAsync()
              future = minimal_client.send_request()
              rclpy.spin_until_future_complete(minimal_client, future)
              response = future.result()
              minimal_client.get_logger().info(
                  'Result of add_two_ints: for %d + %d = %d' %
                  (minimal_client.req.a, minimal_client.req.b, response.sum))
      except (KeyboardInterrupt, ExternalShutdownException):
          pass


  if __name__ == '__main__':
      main()


3.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

As with the service code, we first ``import`` the necessary libraries.

.. code-block:: python

  from example_interfaces.srv import AddTwoInts

  import rclpy
  from rclpy.executors import ExternalShutdownException
  from rclpy.node import Node

The ``MinimalClientAsync`` class constructor initializes the node with the name ``minimal_client_async``.
The constructor definition creates a client with the same type and name as the service node.
The type and name must match for the client and service to be able to communicate.
The ``while`` loop in the constructor checks if a service matching the type and name of the client is available once a second.
Finally it creates a new ``AddTwoInts`` request object.

.. code-block:: python

  def __init__(self):
      super().__init__('minimal_client_async')
      self.cli = self.create_client(AddTwoInts, 'add_two_ints')
      while not self.cli.wait_for_service(timeout_sec=1.0):
          self.get_logger().info('service not available, waiting again...')
      self.req = AddTwoInts.Request()

Below the constructor is the ``send_request`` method, which will send the request and spin until it receives the response or fails.

.. code-block:: python

  def send_request(self):
      self.req.a = 41
      self.req.b = 1
      return self.cli.call_async(self.req)

Finally we have the ``main`` method, which constructs a ``MinimalClientAsync`` object, sends the request using the passed-in command-line arguments, calls ``rclpy.spin_until_future_complete`` to wait for the result, and logs the results.

.. code-block:: python

  def main(args=None):
      try:
          with rclpy.init(args=args):
              minimal_client = MinimalClientAsync()
              future = minimal_client.send_request()
              rclpy.spin_until_future_complete(minimal_client, future)
              response = future.result()
              minimal_client.get_logger().info(
                  'Result of add_two_ints: for %d + %d = %d' %
                  (minimal_client.req.a, minimal_client.req.b, response.sum))
      except (KeyboardInterrupt, ExternalShutdownException):
          pass

.. warning::

  Do not use ``rclpy.spin_until_future_complete`` in a ROS 2 callback.
  For more details see the :doc:`sync deadlock article <../../../How-To-Guides/Sync-Vs-Async>`.

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

It's good practice to run ``rosdep`` in the root of your workspace (``ros2_ws``) to check for missing dependencies before building:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

            rosdep install -i --from-path src --rosdistro {DISTRO} -y

   .. group-tab:: macOS

      rosdep only runs on Linux, so you can skip ahead to next step.

   .. group-tab:: Windows

      rosdep only runs on Linux, so you can skip ahead to next step.


Navigate back to the root of your workspace, ``ros2_ws``, and build your new package:

.. code-block:: console

  colcon build --packages-select py_srvcli

Open a new terminal, navigate to ``ros2_ws``, and source the setup files:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      source install/setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      call install/setup.bat

Now run the service node:

.. code-block:: console

  ros2 run py_srvcli service

The node will wait for the client's request.

Open another terminal and source the setup files from inside ``ros2_ws`` again.
Start the client node:

.. code-block:: console

  ros2 run py_srvcli client

The client sends the request to the service, which computes the sum and returns the result.
The client should receive the following response:

.. code-block:: console

  [INFO] [minimal_client_async]: Result of add_two_ints: for 41 + 1 = 42

Return to the terminal where your service node is running.
You will see that it published log messages when it received the request:

.. code-block:: console

  [INFO] [minimal_service]: Incoming request
  a: 41 b: 1

Enter ``Ctrl+C`` in the server terminal to stop the node from spinning.


Summary
-------

You created two nodes to request and respond to data over a service.
You added their dependencies and executables to the package configuration files so that you could build and run them, allowing you to see a service/client system at work.

Next steps
----------

In the last few tutorials you've been utilizing interfaces to pass data across topics and services.
Next, you'll learn how to :doc:`create custom interfaces <./Custom-ROS2-Interfaces>`.

Related content
---------------

* There are several ways you could write a service and client in Python; check out the ``minimal_client`` and ``minimal_service`` packages in the `ros2/examples <https://github.com/ros2/examples/tree/{REPOS_FILE_BRANCH}/rclpy/services>`_ repo.

* In this tutorial, you used the ``call_async()`` API in your client node to call the service.
  There is another service call API available for Python called synchronous calls.
  We do not recommend using synchronous calls, but if you'd like to learn more about them, read the guide to :doc:`Synchronous vs. asynchronous clients <../../How-To-Guides/Sync-Vs-Async>`.
