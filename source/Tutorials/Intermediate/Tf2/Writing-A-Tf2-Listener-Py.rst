.. redirect-from::

    Tutorials/Tf2/Writing-A-Tf2-Listener-Py

.. _WritingATf2ListenerPy:

Writing a listener (Python)
===========================

**Goal:** Learn how to use tf2 to get access to frame transformations.

**Tutorial level:** Intermediate

**Time:** 10 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

In previous tutorials we created a tf2 broadcaster to publish the pose of a turtle to tf2.

In this tutorial we'll create a tf2 listener to start using tf2.

Prerequisites
-------------

This tutorial assumes you have completed the :doc:`tf2 broadcaster tutorial (Python) <./Writing-A-Tf2-Broadcaster-Py>`.
In the previous tutorial, we created a ``learning_tf2_py`` package, which is where we will continue working from.

Tasks
-----

1 Write the listener node
^^^^^^^^^^^^^^^^^^^^^^^^^

Let's first create the source files. Go to the ``learning_tf2_py`` package we created in the previous tutorial.
Inside the ``src/learning_tf2_py/learning_tf2_py`` directory download the example listener code by entering the following command:

.. tabs::

    .. group-tab:: Linux

        .. code-block:: console

            wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/turtle_tf2_listener.py

    .. group-tab:: macOS

        .. code-block:: console

            wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/turtle_tf2_listener.py

    .. group-tab:: Windows

        In a Windows command line prompt:

        .. code-block:: console

            curl -sk https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/turtle_tf2_listener.py -o turtle_tf2_listener.py

        Or in powershell:

        .. code-block:: console

            curl https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/turtle_tf2_listener.py -o turtle_tf2_listener.py

Open the file using your preferred text editor.

.. code-block:: python

    import math

    from geometry_msgs.msg import Twist

    import rclpy
    from rclpy.node import Node

    from tf2_ros import TransformException
    from tf2_ros.buffer import Buffer
    from tf2_ros.transform_listener import TransformListener

    from turtlesim.srv import Spawn


    class FrameListener(Node):

        def __init__(self):
            super().__init__('turtle_tf2_frame_listener')

            # Declare and acquire `target_frame` parameter
            self.target_frame = self.declare_parameter(
              'target_frame', 'turtle1').get_parameter_value().string_value

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            # Create a client to spawn a turtle
            self.spawner = self.create_client(Spawn, 'spawn')
            # Boolean values to store the information
            # if the service for spawning turtle is available
            self.turtle_spawning_service_ready = False
            # if the turtle was successfully spawned
            self.turtle_spawned = False

            # Create turtle2 velocity publisher
            self.publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 1)

            # Call on_timer function every second
            self.timer = self.create_timer(1.0, self.on_timer)

        def on_timer(self):
            # Store frame names in variables that will be used to
            # compute transformations
            from_frame_rel = self.target_frame
            to_frame_rel = 'turtle2'

            if self.turtle_spawning_service_ready:
                if self.turtle_spawned:
                    # Look up for the transformation between target_frame and turtle2 frames
                    # and send velocity commands for turtle2 to reach target_frame
                    try:
                        t = self.tf_buffer.lookup_transform(
                            to_frame_rel,
                            from_frame_rel,
                            rclpy.time.Time())
                    except TransformException as ex:
                        self.get_logger().info(
                            f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                        return

                    msg = Twist()
                    scale_rotation_rate = 1.0
                    msg.angular.z = scale_rotation_rate * math.atan2(
                        t.transform.translation.y,
                        t.transform.translation.x)

                    scale_forward_speed = 0.5
                    msg.linear.x = scale_forward_speed * math.sqrt(
                        t.transform.translation.x ** 2 +
                        t.transform.translation.y ** 2)

                    self.publisher.publish(msg)
                else:
                    if self.result.done():
                        self.get_logger().info(
                            f'Successfully spawned {self.result.result().name}')
                        self.turtle_spawned = True
                    else:
                        self.get_logger().info('Spawn is not finished')
            else:
                if self.spawner.service_is_ready():
                    # Initialize request with turtle name and coordinates
                    # Note that x, y and theta are defined as floats in turtlesim/srv/Spawn
                    request = Spawn.Request()
                    request.name = 'turtle2'
                    request.x = float(4)
                    request.y = float(2)
                    request.theta = float(0)
                    # Call request
                    self.result = self.spawner.call_async(request)
                    self.turtle_spawning_service_ready = True
                else:
                    # Check if the service is ready
                    self.get_logger().info('Service is not ready')


    def main():
        rclpy.init()
        node = FrameListener()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass

        rclpy.shutdown()

1.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

To understand how the service behind spawning turtle works, please refer to :doc:`writing a simple service and client (Python) <../../Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client>` tutorial.

Now, let's take a look at the code that is relevant to get access to frame transformations.
The ``tf2_ros`` package provides an implementation of a ``TransformListener`` to help make the task of receiving transforms easier.

.. code-block:: python

    from tf2_ros.transform_listener import TransformListener

Here, we create a ``TransformListener`` object. Once the listener is created, it starts receiving tf2 transformations over the wire, and buffers them for up to 10 seconds.

.. code-block:: python

    self.tf_listener = TransformListener(self.tf_buffer, self)

Finally, we query the listener for a specific transformation. We call ``lookup_transform`` method with following arguments:

#. Target frame

#. Source frame

#. The time at which we want to transform

Providing ``rclpy.time.Time()`` will just get us the latest available transform.
All this is wrapped in a try-except block to handle possible exceptions.

.. code-block:: python

    t = self.tf_buffer.lookup_transform(
        to_frame_rel,
        from_frame_rel,
        rclpy.time.Time())

1.2 Add an entry point
~~~~~~~~~~~~~~~~~~~~~~

To allow the ``ros2 run`` command to run your node, you must add the entry point
to ``setup.py`` (located in the ``src/learning_tf2_py`` directory).

.. code-block:: python

    'turtle_tf2_listener = learning_tf2_py.turtle_tf2_listener:main',

2 Update the launch file
^^^^^^^^^^^^^^^^^^^^^^^^

Open the launch file called ``turtle_tf2_demo.launch.py`` with your text editor, add two new nodes to the launch description, add a launch argument, and add the imports. The resulting file should look like:

.. code-block:: python

    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration

    from launch_ros.actions import Node


    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='turtlesim',
                executable='turtlesim_node',
                name='sim'
            ),
            Node(
                package='learning_tf2_py',
                executable='turtle_tf2_broadcaster',
                name='broadcaster1',
                parameters=[
                    {'turtlename': 'turtle1'}
                ]
            ),
            DeclareLaunchArgument(
                'target_frame', default_value='turtle1',
                description='Target frame name.'
            ),
            Node(
                package='learning_tf2_py',
                executable='turtle_tf2_broadcaster',
                name='broadcaster2',
                parameters=[
                    {'turtlename': 'turtle2'}
                ]
            ),
            Node(
                package='learning_tf2_py',
                executable='turtle_tf2_listener',
                name='listener',
                parameters=[
                    {'target_frame': LaunchConfiguration('target_frame')}
                ]
            ),
        ])

This will declare a ``target_frame`` launch argument, start a broadcaster for second turtle that we will spawn and listener that will subscribe to those transformations.


3 Build
^^^^^^^

Run ``rosdep`` in the root of your workspace to check for missing dependencies.

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

          rosdep install -i --from-path src --rosdistro {DISTRO} -y

   .. group-tab:: macOS

        rosdep only runs on Linux, so you will need to install ``geometry_msgs`` and ``turtlesim`` dependencies yourself

   .. group-tab:: Windows

        rosdep only runs on Linux, so you will need to install ``geometry_msgs`` and ``turtlesim`` dependencies yourself

Still in the root of your workspace, build your package:

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

4 Run
^^^^^

Now you're ready to start your full turtle demo:

.. code-block:: console

    ros2 launch learning_tf2_py turtle_tf2_demo.launch.py

You should see the turtle sim with two turtles.
In the second terminal window type the following command:

.. code-block:: console

    ros2 run turtlesim turtle_teleop_key

To see if things work, simply drive around the first turtle using the arrow keys (make sure your terminal window is active, not your simulator window), and you'll see the second turtle following the first one!

Summary
-------

In this tutorial you learned how to use tf2 to get access to frame transformations.
You also have finished writing your own turtlesim demo that you first tried in :doc:`Introduction to tf2 <./Introduction-To-Tf2>` tutorial.
