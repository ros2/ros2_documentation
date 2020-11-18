.. _PythonParamNode:

Using parameters in a class (Python)
====================================

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
^^^^^^^^^^^^^^^^^^^^^^^

Inside the ``dev_ws/src/python_parameters/python_parameters`` directory, create a new file called ``python_parameters_node.py`` and paste the following code within:

.. code-block:: Python

    import rclpy
    import rclpy.node
    from rclpy.exceptions import ParameterNotDeclaredException
    from rcl_interfaces.msg import ParameterType

    class MinimalParam(rclpy.node.Node):
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

            # Then set the parameter "my_parameter" back to string value "world"
            my_new_param = rclpy.parameter.Parameter(
                "my_parameter",
                rclpy.Parameter.Type.STRING,
                "world"
            )
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
Declaring a parameter before getting or setting it is compulsory, or you will raise a ``ParameterNotDeclaredException`` exception.

2.1.1 (Optional) Add ParameterDescriptor
""""""""""""""""""""""""""""""""""""""""
Optionally, you can set a descriptor for the parameter.
Descriptors allow you to specify the type of the parameter and some description text.
For that to work, the ``__init__`` code has to be changed to:

.. code-block:: Python

    # ...

    class MinimalParam(rclpy.node.Node):
        def __init__(self):
            super().__init__('minimal_param_node')
            timer_period = 2  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)

            from rcl_interfaces.msg import ParameterDescriptor
            my_parameter_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                                          description='This parameter is mine!')

            self.declare_parameter("my_parameter",
                                   "default value for my_parameter",
                                   my_parameter_descriptor)

The rest of the code remains the same.
Once you run the node, you can then run ``ros2 param describe /minimal_param_node my_parameter`` to see the type and description.

2.2 Add an entry point
~~~~~~~~~~~~~~~~~~~~~~

Open the ``setup.py`` file.
Again, match the ``maintainer``, ``maintainer_email``, ``description`` and ``license`` fields to your ``package.xml``:

.. code-block:: python

  maintainer='YourName',
  maintainer_email='you@email.com',
  description='Python parameter tutorial',
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

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        rosdep install -i --from-path src --rosdistro rolling -y

   .. group-tab:: macOS

      rosdep only runs on Linux, so you can skip ahead to next step.

   .. group-tab:: Windows

      rosdep only runs on Linux, so you can skip ahead to next step.

Navigate back to the root of your workspace, ``dev_ws``, and build your new package:

.. code-block:: console

    colcon build --packages-select python_parameters

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

Now run the node:

.. code-block:: console

     ros2 run python_parameters param_talker

Except the first message where the parameter had a default value (an empty string), the terminal should return the following message every 2 seconds:

.. code-block:: console

    [INFO] [parameter_node]: Hello world!

There are two ways to change the parameter:

3.1 Change via the console
~~~~~~~~~~~~~~~~~~~~~~~~~~

This part will use the knowledge you have gained from the :ref:`tutoral about parameters <ROS2Params>` and apply it to the node you have just created.

Make sure the node is running:

.. code-block:: console

     ros2 run python_parameters param_talker

Open another terminal, source the setup files from inside ``dev_ws`` again, and enter the following line:

.. code-block:: console

    ros2 param list

There you will see the custom parameter ``my_parameter``.
To change it simply run the following line in the console:

.. code-block:: console

    ros2 param set /minimal_param_node my_parameter earth

You know it went well if you get the output ``Set parameter successful``.
If you look at the other terminal, you should see the output change to ``[INFO] [minimal_param_node]: Hello earth!``

Since the Python talker then set the parameter back to ``world``, further outputs show  ``[INFO] [minimal_param_node]: Hello world!``

3.2 Change via a launch file
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can also set parameters in a launch file, but first you will need to add a launch directory.
Inside the ``dev_ws/src/python_parameters/`` directory, create a new directory called ``launch``.
In there, create a new file called ``python_parameters_launch.py``



.. code-block:: Python

  from launch import LaunchDescription
  from launch_ros.actions import Node

  def generate_launch_description():
      return LaunchDescription([
          Node(
              package="python_parameters",
              executable="param_talker",
              name="custom_parameter_node",
              output="screen",
              emulate_tty=True,
              parameters=[
                  {"my_parameter": "earth"}
              ]
          )
      ])


Here you can see that we set ``my_parameter`` to ``earth`` when we launch our node ``parameter_node``.
By adding the two lines below, we ensure our output is printed in our console.

.. code-block:: console

          output="screen",
          emulate_tty=True,

Now open the ``setup.py`` file.
Add the ``import`` statements to the top of the file, and the other new statement to the ``data_files`` parameter to include all launch files:


.. code-block:: Python

    import os
    from glob import glob
    # ...

    setup(
      # ...
      data_files=[
          # ...
          (os.path.join('share', package_name), glob('launch/*_launch.py')),
        ]
      )

Open a console and navigate to the root of your workspace, ``dev_ws``, and build your new package:

.. code-block:: console

    colcon build --packages-select python_parameters

Then source the setup files in a new terminal:

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

Now run the node using the launch file we have just created:

.. code-block:: console

     ros2 launch python_parameters python_parameters_launch.py

The terminal should return the following message:

.. code-block:: console

    [parameter_node-1] [INFO] [custom_parameter_node]: Hello earth!


Summary
-------

You created a node with a custom parameter, that can be set either from the launch file or the command line.
You wrote the code of a parameter talker: a Python node that declares, and then loops getting and setting a string parameter.
You added the entry point so that you could build and run it, and used ``ros2 param`` to interact with the parameter talker.

Next steps
----------

Now that you have some packages and ROS 2 systems of your own, the :ref:`next tutorial <Ros2Doctor>` will show you how to examine issues in your environment and systems in case you have problems.
