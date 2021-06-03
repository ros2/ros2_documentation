.. _Tf2Main:

tf2 Tutorials
=============

Many of the tf2 tutorials are available for both C++ and Python.
The tutorials are streamlined to complete either the C++ track or the Python track.
If you want to learn both C++ and Python, you should run through the tutorials
once for C++ and once for Python.

.. contents:: Contents
   :depth: 2
   :local:

Workspace Setup
---------------

If you have not yet created a workspace in which to complete the tutorials,
:ref:`follow this tutorial <ROS2Workspace>`.

Learning tf2
------------

#. :ref:`Introduction to tf2 <IntroToTf2>`.

   This tutorial will give you a good idea of what tf2 can do for you. It
   shows off some of the tf2 power in a multi-robot example using turtlesim.
   This also introduces using tf2_echo, view_frames, and rviz.

.. Following sections are coming soon.

.. #. Writing a tf2 static broadcaster (C++)(Python).

..    This tutorial teaches you how to broadcast static coordinate frames to tf2.

.. #. Writing a tf2 broadcaster (C++)(Python).

..    This tutorial teaches you how to broadcast the state of a robot to tf2.

.. #. Writing a tf2 listener (C++)(Python)

..    This tutorial teaches you how to use tf2 to get access to frame transformations.

.. #. Adding a frame (C++)(Python)
   
..    This tutorial teaches you how to add an extra fixed frame to tf2.

.. #. Learning about tf2 and time (C++)(Python)

..    This tutorial teaches you to use the timeout in lookup_transform function to
..    wait for a transform to be available on the tf2 tree.

.. #. Time travel with tf2 (C++)(Python)

..    This tutorial teaches you about advanced time travel features of tf2.

.. Debugging tf2
.. -------------

.. #. Quaterion Fundamentals
   
.. #. Basic Debugging tf2
   
.. Using sensor messages with tf2
.. ------------------------------

.. #. Using Stamped datatypes with tf2_ros::MessageFilter

.. Setting up your robot with tf2
.. ------------------------------

.. #. Setting up your robot using tf

..    This tutorial provides a guide to set up your robot to start using tf.

.. #. Using the robot state publisher on your own robot

..    This tutorial explains how you can publish the state of your robot to tf,
..    using the robot state publisher.

.. #. Using urdf with robot_state_publisher

..    This tutorial gives a full example of a robot model with URDF that uses
..    robot_state_publisher. First, we create the URDF model with all the
..    necessary parts. Then we write a node which publishes the JointState
..    and transforms. Finally, we run all the parts together.