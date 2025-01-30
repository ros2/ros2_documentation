Monitoring for parameter changes (Python)
=========================================

**Goal:** Learn to use the ParameterEventHandler class to monitor and respond to parameter changes.

**Tutorial level:** Intermediate

**Time:** 20 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

Often a node needs to respond to changes to its own parameters or another node's parameters.
The ParameterEventHandler class makes it easy to listen for parameter changes so that your code can respond to them.
This tutorial will show you how to use the Python version of the ParameterEventHandler class to monitor for changes to a node's own parameters as well as changes to another node's parameters.

Prerequisites
-------------

Before starting this tutorial, you should first complete the following tutorials:

- :doc:`../Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters`
- :doc:`../Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python`

Tasks
-----

In this tutorial, you will create a new package to contain some sample code, write some Python code to use the ParameterEventHandler class, and test the resulting code.


1 Create a package
^^^^^^^^^^^^^^^^^^

First, open a new terminal and :doc:`source your ROS 2 installation <../Beginner-CLI-Tools/Configuring-ROS2-Environment>` so that ``ros2`` commands will work.

Follow :ref:`these instructions <new-directory>` to create a new workspace named ``ros2_ws``.

Recall that packages should be created in the ``src`` directory, not the root of the workspace.
So, navigate into ``ros2_ws/src`` and then create a new package there:

.. code-block:: console

  ros2 pkg create --build-type ament_python --license Apache-2.0 python_parameter_event_handler --dependencies rclpy

Your terminal will return a message verifying the creation of your package ``python_parameter_event_handler`` and all its necessary files and folders.

The ``--dependencies`` argument will automatically add the necessary dependency lines to ``package.xml`` and ``CMakeLists.txt``.

1.1 Update ``package.xml``
~~~~~~~~~~~~~~~~~~~~~~~~~~

Because you used the ``--dependencies`` option during package creation, you don’t have to manually add dependencies to ``package.xml``.
As always, though, make sure to add the description, maintainer email and name, and license information to ``package.xml``.

.. code-block:: xml

  <description>Python parameter events client tutorial</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>Apache-2.0</license>

2 Write the Python node
^^^^^^^^^^^^^^^^^^^^^^^

Inside the ``ros2_ws/src/python_parameter_event_handler/python_parameter_event_handler`` directory, create a new file called ``parameter_event_handler.py`` and paste the following code within:

.. code-block:: Python

    import rclpy
    from rclpy.executors import ExternalShutdownException
    import rclpy.node
    import rclpy.parameter

    from rclpy.parameter_event_handler import ParameterEventHandler


    class SampleNodeWithParameters(rclpy.node.Node):
        def __init__(self):
            super().__init__('node_with_parameters')

            self.declare_parameter('an_int_param', 0)

            self.handler = ParameterEventHandler(self)

            self.callback_handle = self.handler.add_parameter_callback(
                parameter_name="an_int_param",
                node_name="node_with_parameters",
                callback=self.callback,
            )

        def callback(self, p: rclpy.parameter.Parameter) -> None:
            self.get_logger().info(f"Received an update to parameter: {p.name}: {rclpy.parameter.parameter_value_to_python(p.value)}")


    def main():
        try:
            with rclpy.init():
                node = SampleNodeWithParameters()
                rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass

2.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

The ``import`` statements at the top are used to import the package dependencies.

.. code-block:: Python

    import rclpy
    from rclpy.executors import ExternalShutdownException
    import rclpy.node
    import rclpy.parameter

    from rclpy.parameter_event_handler import ParameterEventHandler

The next piece of code creates the class ``SampleNodeWithParameters`` and the constructor.
The constructor for the class declares an integer parameter ``an_int_param``,  with a default value of 0.
Next, the code creates a ``ParameterEventHandler`` that will be used to monitor changes to parameters.

.. code-block:: Python

    class SampleNodeWithParameters(rclpy.node.Node):
        def __init__(self):
            super().__init__('node_with_parameters')

            self.declare_parameter('an_int_param', 0)

            self.handler = ParameterEventHandler(self)


Finally, we add parameter callback and get callback handler for the new callback.

.. note::

   It is very important to save the handle that is returned by ``add_parameter_callback``; otherwise, the callback will not be properly registered.

.. code-block:: Python

            self.callback_handle = self.handler.add_parameter_callback(
                parameter_name="an_int_param",
                node_name="node_with_parameters",
                callback=self.callback,
            )

For the callback function, we use the ``callback`` method of the ``SampleNodeWithParameters`` class.

.. code-block:: Python

        def callback(self, p: rclpy.parameter.Parameter) -> None:
            self.get_logger().info(f"Received an update to parameter: {p.name}: {rclpy.parameter.parameter_value_to_python(p.value)}")


Following the ``SampleNodeWithParameters`` is a typical ``main`` function which initializes ROS, spins the sample node so that it can send and receive messages, and then shuts down after the user enters ^C at the console.

.. code-block:: Python

    def main():
        try:
            with rclpy.init():
                node = SampleNodeWithParameters()
                rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass


2.2 Add an entry point
~~~~~~~~~~~~~~~~~~~~~~

Open the ``setup.py`` file.
Again, match the ``maintainer``, ``maintainer_email``, ``description`` and ``license`` fields to your ``package.xml``:

.. code-block:: Python

    maintainer='YourName',
    maintainer_email='you@email.com',
    description='Python parameter tutorial',
    license='Apache-2.0',

Add the following line within the ``console_scripts`` brackets of the ``entry_points`` field:

.. code-block:: Python

  entry_points={
      'console_scripts': [
          'node_with_parameters = python_parameter_event_handler.parameter_event_handler:main',
      ],
  },


3 Build and run
^^^^^^^^^^^^^^^

It's good practice to run ``rosdep`` in the root of your workspace (``ros2_ws``) to check for missing dependencies before building:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y

   .. group-tab:: macOS

      rosdep only runs on Linux, so you can skip ahead to next step.

   .. group-tab:: Windows

      rosdep only runs on Linux, so you can skip ahead to next step.

Navigate back to the root of your workspace, ``ros2_ws``, and build your new package:

.. code-block:: console

    colcon build --packages-select python_parameter_event_handler

Open a new terminal, navigate to ``ros2_ws``, and source the setup files:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      call install\setup.bat

Now run the node:

.. code-block:: console

     ros2 run python_parameter_event_handler node_with_parameters

The node is now active and has a single parameter and will print a message whenever this parameter is updated.
To test this, open up another terminal and source the ROS setup file as before and execute the following command:

.. code-block:: console

    ros2 param set node_with_parameters an_int_param 43

The terminal running the node will display a message similar to the following:

.. code-block:: console

    [INFO] [1698483083.315084660] [node_with_parameters]: Received an update to parameter: an_int_param: 43

The callback we set previously in the node has been invoked and has displayed the new updated value.
You can now terminate the running parameter_event_handler sample using ^C in the terminal.

3.1 Monitor changes to another node's parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can also use the ParameterEventHandler to monitor parameter changes to another node's parameters.
Let's update the SampleNodeWithParameters class to monitor for changes to a parameter in another node.
We will use the parameter_blackboard demo application to host a double parameter that we will monitor for updates.

First update the constructor to add the following code after the existing code:

.. code-block:: Python

    def __init__(...):
        ...
        self.callback_handle = self.handler.add_parameter_callback(
            parameter_name="a_double_param",
            node_name="parameter_blackboard",
            callback=self.callback,
        )


In a terminal, navigate back to the root of your workspace, ``ros2_ws``, and build your updated package as before:

.. code-block:: console

    colcon build --packages-select python_parameter_event_handler

Then source the setup files:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      call install\setup.bat

Now, to test monitoring of remote parameters, first run the newly-built parameter_event_handler code:

.. code-block:: console

     ros2 run python_parameter_event_handler node_with_parameters

Next, from another terminal (with ROS initialized), run the parameter_blackboard demo application, as follows:

.. code-block:: console

     ros2 run demo_nodes_cpp parameter_blackboard

Finally, from a third terminal (with ROS initialized), let's set a parameter on the parameter_blackboard node:

.. code-block:: console

     ros2 param set parameter_blackboard a_double_param 3.45

Upon executing this command, you should see output in the parameter_event_handler window, indicating that the callback function was invoked upon the parameter update:

.. code-block:: console

      [INFO] [1699821958.757770223] [node_with_parameters]: Received an update to parameter: a_double_param: 3.45

Summary
-------

You created a node with a parameter and used the ParameterEventHandler class to set a callback to monitor changes to that parameter.
You also used the same class to monitor changes to a remote node.
The ParameterEventHandler is a convenient way to monitor for parameter changes so that you can then respond to the updated values.

Related content
---------------

To learn how to adapt ROS 1 parameter files for ROS 2, see the :doc:`Migrating YAML parameter files from ROS 1 to ROS2 <../../How-To-Guides/Migrating-from-ROS1/Migrating-Parameters>` tutorial.
