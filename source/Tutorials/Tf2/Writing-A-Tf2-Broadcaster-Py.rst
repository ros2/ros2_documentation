.. _WritingATf2BroadcasterPy:

Writing a tf2 broadcaster (Python)
==================================

**Goal:** Learn how to broadcast the state of a robot to tf2.

**Tutorial level:** Intermediate

**Time:** 15 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

In the next two tutorials we will write the code to reproduce the demo from the :ref:`Introduction to tf2 <IntroToTf2>` tutorial.
After that, the following tutorials focus on extending the demo with more advanced tf2 features.

Prerequisites
-------------

In previous tutorials, you learned how to :ref:`create a workspace <ROS2Workspace>` and :ref:`create a package <CreatePkg>`.
You also have created the ``learning_tf2_py`` package where we will continue working on.

Tasks
-----

1 Write the broadcaster node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Let's first create the source files.
Go to the ``learning_tf2_py`` package we created in the previous tutorial.
Inside the ``src/learning_tf2_py/learning_tf2_py`` directory, create a new file called
``turtle_tf2_broadcaster.py`` and paste the following code within:

.. code-block:: python

    from geometry_msgs.msg import TransformStamped

    import rclpy
    from rclpy.node import Node

    from tf2_ros import TransformBroadcaster

    import tf_transformations

    from turtlesim.msg import Pose


    class FramePublisher(Node):

        def __init__(self):
            super().__init__('turtle_tf2_frame_publisher')

            # Declare and acquire `turtlename` parameter
            self.declare_parameter('turtlename', 'turtle')
            self.turtlename = self.get_parameter('turtlename').get_parameter_value().string_value

            # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
            # callback function on each message
            self.subscription = self.create_subscription(
                Pose,
                f'/{self.turtlename}/pose',
                self.handle_turtle_pose,
                1)
            self.subscription

        def handle_turtle_pose(self, msg):
            # Initialize the transform broadcaster
            br = TransformBroadcaster(self)
            t = TransformStamped()

            # Read message content and assign it to
            # corresponding tf variables
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = self.turtlename
            t.transform.translation.x = msg.x
            t.transform.translation.y = msg.y
            t.transform.translation.z = 0.0
            q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            # Send the transformation
            br.sendTransform(t)


    def main():
        rclpy.init()
        node = FramePublisher()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass

        rclpy.shutdown()

1.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

Now, let's take a look at the code that is relevant to publishing the turtle pose to tf2.
Firstly, we define and acquire a single parameter ``turtlename``, which specifies a turtle name, e.g. ``turtle1`` or ``turtle2``.

.. code-block:: python

    self.declare_parameter('turtlename', 'turtle')
    self.turtlename = self.get_parameter('turtlename').get_parameter_value().string_value

Afterward, the node subscribes to topic ``turtleX/pose`` and runs function ``handle_turtle_pose`` on every incoming message.

.. code-block:: python

    self .subscription = self.create_subscription(
        Pose,
        f'/{self.turtlename}/pose',
        self.handle_turtle_pose,
        1)

Now, we create a Transform object and give it the appropriate metadata.

#. We need to give the transform being published a timestamp, we'll just stamp it with the current time, ``ros::Time::now()``.

#. Then, we need to set the name of the parent frame of the link we're creating, in this case ``world``

#. Finally, we need to set the name of the child node of the link we're creating, in this case this is the name of the turtle itself.

The handler function for the turtle pose message broadcasts this turtle's translation and rotation, and publishes it as a transform from frame ``world`` to frame ``turtleX``.

.. code-block:: python

    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = 'world'
    t.child_frame_id = self.turtlename

Here we copy the information from the 3D turtle pose into the 3D transform.

.. code-block:: python

    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

This is where the real work is done. Sending a transform with a ``TransformBroadcaster`` requires passing in just the transform itself.

.. code-block:: python

    br.sendTransform(t)

.. note::

    You can also publish static transforms on the same pattern by instantiating a ``tf2_ros.StaticTransformBroadcaster`` instead of a ``tf2_ros.TransformBroadcaster``.
    The static transforms will be published on the ``/tf_static`` topic and will be sent only when required (latched topic) and not periodically.
    For more details see :ref:`here <WritingATf2StaticBroadcasterPy>`.

1.2 Add dependencies
~~~~~~~~~~~~~~~~~~~~

Navigate one level back to the ``src/learning_tf2_py`` directory, where the ``setup.py``, ``setup.cfg``, and ``package.xml`` files are located.

Open ``package.xml`` with your text editor.
Add the following dependencies corresponding to your node's import statements:

.. code-block:: xml

    <exec_depend>launch</exec_depend>
    <exec_depend>launch_ros</exec_depend>

This declares the additional required ``launch`` and ``launch_ros`` dependencies when its code is executed.

Make sure to save the file.

1.3 Add an entry point
~~~~~~~~~~~~~~~~~~~~~~

To allow the ``ros2 run`` command to run your node, you must add the entry point
to ``setup.py`` (located in the ``src/learning_tf2_py`` directory).

Add the following line between the ``'console_scripts':`` brackets:

.. code-block:: python

    'turtle_tf2_broadcaster = learning_tf2_py.turtle_tf2_broadcaster:main',

2 Write the launch file
^^^^^^^^^^^^^^^^^^^^^^^

Now create a launch file for this demo. With your text editor, create a new
file called ``turtle_tf2_demo.launch.py``, and add the following lines:

.. code-block:: python

    from launch import LaunchDescription
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
        ])

2.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

First we import required modules from the ``launch`` and ``launch_ros`` packages.

.. code-block:: python

    from launch import LaunchDescription
    from launch_ros.actions import Node

Now we run our nodes that start the turtlesim simulation and broadcast ``turtle1`` state to the tf2 using our ``turtle_tf2_broadcaster`` node.

.. code-block:: python

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

2.2 Update setup.py
~~~~~~~~~~~~~~~~~~~

Reopen ``setup.py`` and add the line so that the launch files from the ``launch/`` folder would be installed.
The ``data_files`` field should now look like this:

.. code-block:: python

    data_files=[
        ...
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],

3 Build and run
^^^^^^^^^^^^^^^

Run ``rosdep`` in the root of your workspace to check for missing dependencies, build your updated package, and source the setup files.

Now run the launch file that will start the turtlesim simulation node and ``turtle_tf2_broadcaster`` node:

.. code-block:: console

    ros2 launch learning_tf2_py turtle_tf2_demo.launch.py

In the second terminal window type the following command:

.. code-block:: console

    ros2 run turtlesim turtle_teleop_key

You will now see that the turtlesim simulation have started with one turtle in the middle.
Now, use the ``tf_echo`` tool to check if the turtle pose is actually getting broadcast to tf2:

.. code-block:: console

    ros2 run tf tf_echo /world /turtle1

This should show you the pose of the first turtle.
Drive around the turtle using the arrow keys (make sure your terminal window is active, not your simulator window).
If you run ``tf_echo`` for the transform between the ``world`` and ``turtle2``, you should not see a transform, because the second turtle is not there yet.
However, as soon as we add the second turtle in the next tutorial, the pose of ``turtle2`` will be broadcast to tf2.
