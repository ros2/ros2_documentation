.. _WritingATf2StaticBroadcasterPy:

Writing a tf2 static broadcaster (Python)
=========================================

**Goal:** Learn how to broadcast static coordinate frames to tf2.

**Tutorial level:** Intermediate

**Time:** 15 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

Publishing static transforms is useful to define the relationship between a robot base and its sensors or non-moving parts.
For example, it is easiest to reason about laser scan measurements in a frame at the center of the laser scanner.

This is a standalone tutorial covering the basics of static transforms, which consists of two parts.
In the first part we will write code to publish static transforms to tf2.
In the second part we will explain how to use the commandline ``static_transform_publisher`` executable tool in ``tf2_ros``.

In the next two tutorials we will write the code to reproduce the demo from the :ref:`Introduction to tf2 <IntroToTf2>` tutorial.
After that, the following tutorials focus on extending the demo with more advanced tf2 features.

Prerequisites
-------------

In previous tutorials, you learned how to :ref:`create a workspace <ROS2Workspace>` and :ref:`create a package <CreatePkg>`.

Tasks
-----

1 Create a package
^^^^^^^^^^^^^^^^^^

First we will create a package that will be used for this tutorial and the following ones.
The package called ``learning_tf2_py`` will depend on ``rclpy``, ``tf2_ros``, ``geometry_msgs``, and ``turtlesim``.
Code for this tutorial is stored `here <https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/static_turtle_tf2_broadcaster.py>`_.

Open a new terminal and :ref:`source your ROS 2 installation <ConfigROS2>` so that ``ros2`` commands will work.
Navigate to workspace's ``src`` folder and create a new package:

.. code-block:: console

   ros2 pkg create --build-type ament_python learning_tf2_py

Your terminal will return a message verifying the creation of your package ``learning_tf2_py`` and all its necessary files and folders.

2 Write the static broadcaster node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Let's first create the source files.
Inside the ``src/learning_tf2_py/learning_tf2_py`` directory download the example static broadcaster code by entering the following command:

.. tabs::

    .. group-tab:: Linux

        .. code-block:: console

            wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/static_turtle_tf2_broadcaster.py

    .. group-tab:: macOS

        .. code-block:: console

            wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/static_turtle_tf2_broadcaster.py

    .. group-tab:: Windows

        In a Windows command line prompt:

        .. code-block:: console

                curl -sk https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/static_turtle_tf2_broadcaster.py -o static_turtle_tf2_broadcaster.py

        Or in powershell:

        .. code-block:: console

                curl https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/static_turtle_tf2_broadcaster.py -o static_turtle_tf2_broadcaster.py

Open the file using your preferred text editor.

.. code-block:: python

   import sys

   from geometry_msgs.msg import TransformStamped

   import rclpy
   from rclpy.node import Node

   from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

   import tf_transformations


   class StaticFramePublisher(Node):
      """
      Broadcast transforms that never change.

      This example publishes transforms from `world` to a static turtle frame.
      The transforms are only published once at startup, and are constant for all
      time.
      """

      def __init__(self, transformation):
         super().__init__('static_turtle_tf2_broadcaster')

         self._tf_publisher = StaticTransformBroadcaster(self)

         # Publish static transforms once at startup
         self.make_transforms(transformation)

      def make_transforms(self, transformation):
         static_transformStamped = TransformStamped()
         static_transformStamped.header.stamp = self.get_clock().now().to_msg()
         static_transformStamped.header.frame_id = 'world'
         static_transformStamped.child_frame_id = sys.argv[1]
         static_transformStamped.transform.translation.x = float(sys.argv[2])
         static_transformStamped.transform.translation.y = float(sys.argv[3])
         static_transformStamped.transform.translation.z = float(sys.argv[4])
         quat = tf_transformations.quaternion_from_euler(
               float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7]))
         static_transformStamped.transform.rotation.x = quat[0]
         static_transformStamped.transform.rotation.y = quat[1]
         static_transformStamped.transform.rotation.z = quat[2]
         static_transformStamped.transform.rotation.w = quat[3]

         self._tf_publisher.sendTransform(static_transformStamped)


   def main():
      logger = rclpy.logging.get_logger('logger')

      # obtain parameters from command line arguments
      if len(sys.argv) < 8:
         logger.info('Invalid number of parameters. Usage: \n'
                     '$ ros2 run learning_tf2_py static_turtle_tf2_broadcaster'
                     'child_frame_name x y z roll pitch yaw')
         sys.exit(0)
      else:
         if sys.argv[1] == 'world':
               logger.info('Your static turtle name cannot be "world"')
               sys.exit(0)

      # pass parameters and initialize node
      rclpy.init()
      node = StaticFramePublisher(sys.argv)
      try:
         rclpy.spin(node)
      except KeyboardInterrupt:
         pass

      rclpy.shutdown()

2.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

Now let's look at the code that is relevant to publishing the static turtle pose to tf2.
The first lines import required packages.
First we import the ``TransformStamped`` from the ``geometry_msgs``, which provides us a template for the message that we will publish to the transformation tree.

.. code-block:: python

   from geometry_msgs.msg import TransformStamped

Afterward, ``rclpy`` is imported so its ``Node`` class can be used.

.. code-block:: python

   import rclpy
   from rclpy.node import Node

The ``tf2_ros`` package provides a ``StaticTransformBroadcaster`` to make the publishing of static transforms easy.
To use the ``StaticTransformBroadcaster``, we need to import it from the ``tf2_ros`` module.
``tf_transformations`` provides functions for converting Euler angles to quaternions and vice versa.

.. code-block:: python

   from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

   import tf_transformations

The ``StaticFramePublisher`` class constructor initializes the node with the name ``static_turtle_tf2_broadcaster``.
Then, ``StaticTransformBroadcaster`` is created, which will send one static transformation upon the startup.

.. code-block:: python

   self._tf_publisher = StaticTransformBroadcaster(self)
   self.make_transforms(transformation)

Here we create a ``TransformStamped`` object, which will be the message we will send over once populated.
Before passing the actual transform values we need to give it the appropriate metadata.

#. We need to give the transform being published a timestamp and we'll just stamp it with the current time, ``self.get_clock().now()``

#. Then we need to set the name of the parent frame of the link we're creating, in this case ``world``

#. Finally, we need to set the name of the child frame of the link we're creating

.. code-block:: python

   static_transformStamped = TransformStamped()
   static_transformStamped.header.stamp = self.get_clock().now().to_msg()
   static_transformStamped.header.frame_id = 'world'
   static_transformStamped.child_frame_id = sys.argv[1]

Here we populate the 6D pose (translation and rotation) of the turtle.

.. code-block:: python

   static_transformStamped.transform.translation.x = float(sys.argv[2])
   static_transformStamped.transform.translation.y = float(sys.argv[3])
   static_transformStamped.transform.translation.z = float(sys.argv[4])
   quat = tf_transformations.quaternion_from_euler(
      float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7]))
   static_transformStamped.transform.rotation.x = quat[0]
   static_transformStamped.transform.rotation.y = quat[1]
   static_transformStamped.transform.rotation.z = quat[2]
   static_transformStamped.transform.rotation.w = quat[3]

Finally, we broadcast static transform using the ``sendTransform()`` function.

.. code-block:: python

   self._tf_publisher.sendTransform(static_transformStamped)

2.2 Add dependencies
~~~~~~~~~~~~~~~~~~~~

Navigate one level back to the ``src/learning_tf2_py`` directory, where the ``setup.py``, ``setup.cfg``, and ``package.xml`` files have been created for you.

Open ``package.xml`` with your text editor.

As mentioned in the :ref:`Creating your first ROS 2 package tutorial <CreatePkg>`, make sure to fill in the ``<description>``, ``<maintainer>`` and ``<license>`` tags:

.. code-block:: xml

  <description>Learning tf2 with rclpy</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

After the lines above, add the following dependencies corresponding to your node’s import statements:

.. code-block:: xml

   <exec_depend>geometry_msgs</exec_depend>
   <exec_depend>rclpy</exec_depend>
   <exec_depend>tf_transformations</exec_depend>
   <exec_depend>tf2_ros</exec_depend>
   <exec_depend>turtlesim</exec_depend>

This declares the required ``geometry_msgs``, ``tf_transformations``, ``rclpy``, ``tf2_ros``, and ``turtlesim`` dependencies when its code is executed.

Make sure to save the file.

2.3 Add an entry point
~~~~~~~~~~~~~~~~~~~~~~

To allow the ``ros2 run`` command to run your node, you must add the entry point to ``setup.py`` (located in the ``src/learning_tf2_py`` directory).

Finally, add the following line between the ``'console_scripts':`` brackets:

.. code-block:: python

   'static_turtle_tf2_broadcaster = learning_tf2_py.static_turtle_tf2_broadcaster:main',

3 Build and run
^^^^^^^^^^^^^^^

It's good practice to run ``rosdep`` in the root of your workspace to check for missing dependencies before building:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        rosdep install -i --from-path src --rosdistro {DISTRO} -y

   .. group-tab:: macOS

      rosdep only runs on Linux, so you will need to install ``geometry_msgs`` and ``turtlesim`` dependencies yourself

   .. group-tab:: Windows

      rosdep only runs on Linux, so you will need to install ``geometry_msgs`` and ``turtlesim`` dependencies yourself

Still in the root of your workspace, build your new package:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      colcon build --packages-select learning_tf2_py

  .. group-tab:: macOS

    .. code-block:: console

      colcon build --packages-select learning_tf2_py

  .. group-tab:: Windows

    .. code-block:: console

      colcon build --merge-install --packages-select learning_tf2_py

Open a new terminal, navigate to the root of your workspace, and source the setup files:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      # CMD
      call install\setup.bat

      # Powershell
      .\install\setup.ps1

Now run the ``static_turtle_tf2_broadcaster`` node:

.. code-block:: console

   ros2 run learning_tf2_py static_turtle_tf2_broadcaster mystaticturtle 0 0 1 0 0 0

This sets a turtle pose broadcast for ``mystaticturtle`` to float 1 meter above the ground.

We can now check that the static transform has been published by echoing the ``tf_static`` topic

.. code-block:: console

   ros2 topic echo /tf_static

If everything went well you should see a single static transform

.. code-block:: console

   transforms:
   - header:
      stamp:
         sec: 1622908754
         nanosec: 208515730
      frame_id: world
   child_frame_id: mystaticturtle
   transform:
      translation:
         x: 0.0
         y: 0.0
         z: 1.0
      rotation:
         x: 0.0
         y: 0.0
         z: 0.0
         w: 1.0

The proper way to publish static transforms
-------------------------------------------

This tutorial aimed to show how ``StaticTransformBroadcaster`` can be used to publish static transforms.
In your real development process you shouldn't have to write this code yourself and should use the dedicated ``tf2_ros`` tool to do so.
``tf2_ros`` provides an executable named ``static_transform_publisher`` that can be used either as a commandline tool or a node that you can add to your launchfiles.

Publish a static coordinate transform to tf2 using an x/y/z offset in meters and roll/pitch/yaw in radians.
In our case, roll/pitch/yaw refers to rotation about the x/y/z-axis, respectively.

.. code-block:: console

   ros2 run tf2_ros static_transform_publisher x y z yaw pitch roll frame_id child_frame_id

Publish a static coordinate transform to tf2 using an x/y/z offset in meters and quaternion.

.. code-block:: console

   ros2 run tf2_ros static_transform_publisher x y z qx qy qz qw frame_id child_frame_id

``static_transform_publisher`` is designed both as a command-line tool for manual use, as well as for use within ``launch`` files for setting static transforms. For example:

.. code-block:: console

   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
      return LaunchDescription([
         Node(
               package='tf2_ros',
               executable='static_transform_publisher',
               arguments = ['0', '0', '1', '0', '0', '0', 'world', 'mystaticturtle']
         ),
      ])

Summary
-------

In this tutorial you learned how static transforms are useful to define static relationships between frames, like ``mystaticturtle`` in relation to the ``world`` frame.
In addition, you learned how static transforms can be useful for understanding sensor data, such as from laser scanners, by relating the data to a common coordinate frame.
Finally, you wrote your own node to publish static transforms to tf2 and learned how to publish required static transformations using ``static_transform_publisher`` executable and launch files.
