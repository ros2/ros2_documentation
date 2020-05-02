.. _PythonParamNode:

Using parameters in a class (Python)
=================================

**Goal:** Create and run a class with ROS parameters using Python (rclpy).

**Tutorial level:** Beginner

**Time:** 20 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

When making your own :ref:`nodes <ROS2Nodes>` you will sometimes need to add parameters that can be set from the launch file.

This tutorial will show you how to create those parameters in a Python class, and how to set them in a launch file.

Prerequisites
-------------

In previous tutorials, you learned how to :ref:`create a workspace <ROS2Workspace>` and :ref:`create a package <CreatePkg>`.
You have also learned about :ref:`parameters <ROS2Params>` and their function in a ROS 2 system.

Tasks
-----
1 Create a package
^^^^^^^^^^^^^^^^^^

Open a new terminal and :ref:`source your ROS 2 installation <ConfigROS2>` so that ``ros2`` commands will work.

Navigate into the ``dev_ws`` directory created in a previous tutorial.

Recall that packages should be created in the ``src`` directory, not the root of the workspace.
Navigate into ``dev_ws/src`` and create a new package:

.. code-block:: console

  ros2 pkg create --build-type ament_python python_parameters --dependencies rclpy

Your terminal will return a message verifying the creation of your package ``python_parameters`` and all its necessary files and folders.

The ``--dependencies`` argument will automatically add the necessary dependency lines to ``package.xml`` and ``CMakeLists.txt``.

1.1 Update ``package.xml``
~~~~~~~~~~~~~~~~~~~~~~~~~~

Because you used the ``--dependencies`` option during package creation, you don’t have to manually add dependencies to ``package.xml`` or ``CMakeLists.txt``.

As always, though, make sure to add the description, maintainer email and name, and license information to ``package.xml``.

.. code-block:: xml

  <description>Python parameter tutorial</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

2 Write the Python node
^^^^^^^^^^^^^^^^^^^^

Inside the ``dev_ws/src/python_parameters/src`` directory, create a new file called ``python_parameters_node.py`` and paste the following code within:

.. code-block:: Python
    import rclpy
    from rclpy.node import Node
    from rclpy.exceptions import ParameterNotDeclaredException

    class MinimalParam(Node):
        def __init__(self):
            super().__init__('minimal_param_node')
            timer_period = 2  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.declare_parameter("my_parameter")

        def timer_callback(self):
            # First get the value parameter "my_parameter" and get its string value
            my_param = self.get_parameter("my_parameter").get_parameter_value().string_value
            
            # Send back a hello with the name
            self.get_logger().info('Hello %s!' % my_param)

            # Then set the parameter "my_parameter" back to string value "nobody"
            my_new_param = rclpy.parameter.Parameter("my_parameter",
                                                    rclpy.Parameter.Type.STRING,
                                                    "nobody")
            all_new_parameters = [my_new_param]
            self.set_parameters(all_new_parameters)

    def main():
        rclpy.init()
        node = MinimalParam()
        rclpy.spin(node)

    if __name__ == '__main__':
        main()



2.1 Examine the code
~~~~~~~~~~~~~~~~~~~~
Declaring a parameter before getting or setting it is compulsory, or you will raise a `ParameterNotDeclaredException` exception.


2.2 Add an entry point
~~~~~~~~~~~~~~~~~~~~~~

Open the ``setup.py`` file.
Again, match the ``maintainer``, ``maintainer_email``, ``description`` and ``license`` fields to your ``package.xml``:

.. code-block:: python

  maintainer='YourName',
  maintainer_email='you@email.com',
  description='Examples of minimal parameter getter/setter using rclpy',
  license='Apache License 2.0',

Add the following line within the ``console_scripts`` brackets of the ``entry_points`` field:

.. code-block:: python

  entry_points={
          'console_scripts': [
                  'param_talker = python_parameters.python_parameters_node:main',
          ],
  },

Don’t forget to save.


3 Build and run
^^^^^^^^^^^^^^^

It's good practice to run ``rosdep`` in the root of your workspace (``dev_ws``) to check for missing dependencies before building:

.. code-block:: console

  sudo rosdep install -i --from-path src --rosdistro <distro> -y

Navigate back to the root of your workspace, ``dev_ws``, and build your new package:

.. code-block:: console

    colcon build --packages-select python_parameters

Open a new terminal, navigate to ``dev_ws``, and source the setup files:

.. code-block:: console

    . install/setup.bash

Now run the node:

.. code-block:: console

     ros2 run python_parameters param_talker

Except the first message where the parameter had a default value (an empty string), the terminal should return the following message every 2 seconds:

.. code-block:: console

    [INFO] [parameter_node]: Hello nobody!

There are two ways to change the parameter:

3.1 Change via the console
~~~~~~~~~~~~~~~~~~~~~~~~~~

This part will use the knowledge you have gained from the :ref:`tutoral about parameters <ROS2Params>` and apply it to the node you have just created.

Make sure the node is running:

.. code-block:: console

     ros2 run python_parameters parameter_node

Open another terminal, source the setup files from inside ``dev_ws`` again, and enter the following line:

.. code-block:: console

    ros2 param list

There you will see the custom parameter ``my_parameter``.
To change it simply run the following line in the console:

.. code-block:: console

    ros2 param set /minimal_param_node my_parameter Bob

You know it went well if you get the output ``Set parameter successful``.
If you look at the other terminal, you should see the output change to ``[INFO] [minimal_param_node]: Hello Bob!``

Since the Python talker then set the parameter back to `nobody`, further outputs show  ``[INFO] [minimal_param_node]: Hello nobody!``

3.2 Change via a launch file
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For this, please refer to the :ref:`C++ equivalent tutorial section 3.2 <CppParamNode>`.

Summary
-------

You created a node with a custom parameter, that can be set either from the launch file or the command line.
You wrote the code of a parameter talker: a Python node that declares, and then loops getting and setting a string parameter. You added the entry point so that you could build and run it, and used `ros2 param` to interact with the parameter talker. 

Next steps
----------

Now that you have some packages and ROS 2 systems of your own, the :ref:`next tutorial <Ros2Doctor>` will show you how to examine issues in your environment and systems in case you have problems.
