.. _IntroToTF2:

Introduction to tf2
===================

**Goal:** Run a turtlesim demo and see some of the tf2 power in a multi-robot example using turtlesim.

**Tutorial level:** Beginner

**Time:** 10 minutes

.. contents:: Contents
   :depth: 2
   :local:

Installing the Demo
-------------------

Let's start by getting any dependencies and compiling the demo package.

.. code-block:: console

   sudo apt-get install ros-rolling-turtle-tf2-py ros-rolling-tf2-tools

Running the Demo
----------------

Now that we're done compiling the turtle_tf2_py tutorial package let's run the demo.

.. code-block:: console

   ros2 launch turtle_tf2_py turtle_tf2_demo.launch.py

You will see the turtlesim start with two turtles.

.. image:: turtlesim_follow1.png

In the second terminal window type the following command:

.. code-block:: console

   ros2 run turtlesim turtle_teleop_key

Once the turtlesim is started you can drive the center turtle around in the turtlesim using the keyboard arrow keys,
select the second ros2 run terminal window so that your keystrokes will be captured to drive the turtle.

.. image:: turtlesim_follow2.png

As you can see that one turtle will continuously move to follow the turtle you are driving around.

What is Happening
-----------------

This demo is using the tf2 library to create three coordinate frames: a world frame, a turtle1 frame, and a turtle2 frame.
This tutorial uses a tf2 broadcaster to publish the turtle coordinate frames and a tf2 listener to compute the difference in the turtle frames and move one turtle to follow the other.

tf2 Tools
---------

Now let's look at how tf2 is being used to create this demo.
We can use tf2 tools to look at what tf2 is doing behind the scenes.

1 Using view_frames
^^^^^^^^^^^^^^^^^^^

``view_frames`` creates a diagram of the frames being broadcast by tf2 over ROS.

.. code-block:: console

   ros2 run tf2_tools view_frames

You will see:

.. code-block:: console

   Listening to tf data during 5 seconds...
   Generating graph in frames.pdf file...

Here a tf2 listener is listening to the frames that are being broadcast over
ROS and drawing a tree of how the frames are connected. To view the tree:

.. code-block:: console

   evince frames.pdf

.. image:: turtlesim_frames.png

Here we can see three frames that are broadcasted by tf2: world, turtle1, and turtle2.
The world here is the parent of the turtle1 and turtle2 frames.
``view_frames`` also report some diagnostic information about when the oldest and most
recent frame transforms were received and how fast the tf2 frame is published to tf2 for debugging purposes.

2 Using tf_echo
^^^^^^^^^^^^^^^

tf_echo reports the transform between any two frames broadcast over ROS.

Usage:

.. code-block:: console

   ros2 run tf2_ros tf2_echo [reference_frame] [target_frame]

Let's look at the transform of the turtle2 frame with respect to turtle1 frame which is equivalent to

.. code-block:: console

   ros2 run tf2_ros tf2_echo turtle2 turtle1

You will see the transform displayed as the tf_echo listener receives the frames broadcast over ROS.

.. code-block:: console

   At time 1622031731.625364060
   - Translation: [2.796, 1.039, 0.000]
   - Rotation: in Quaternion [0.000, 0.000, 0.202, 0.979]
   At time 1622031732.614745114
   - Translation: [1.608, 0.250, 0.000]
   - Rotation: in Quaternion [0.000, 0.000, 0.032, 0.999]

As you drive your turtle around you will see the transform change as the two turtles move relative to each other.

rviz and tf2
------------

``rviz`` is a visualization tool that is useful for examining tf2 frames.
Let's look at our turtle frames using rviz.
Let's start rviz with the turtle_tf2 configuration file using the -d option for rviz:

.. code-block:: console

   ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share turtle_tf2_py)/rviz/turtle_rviz.rviz

.. image:: turtlesim_rviz.png

In the side bar you will see the frames broadcast by tf2. As you drive the turtle around you will see the frames move in rviz.

Next steps
----------

Now that we have examined the turtle_tf2_demo, let's look at how to write the broadcaster (Python) (C++) for this demo.
