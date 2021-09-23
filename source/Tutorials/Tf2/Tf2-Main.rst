.. _Tf2Main:

tf2 Tutorials
=============

Many of the tf2 tutorials are available for both C++ and Python.
The tutorials are streamlined to complete either the C++ track or the Python track.
If you want to learn both C++ and Python, you should go through the tutorials once for C++ and once for Python.

.. contents:: Contents
   :depth: 2
   :local:

.. toctree::
   :hidden:

   Introduction-To-Tf2
   Writing-A-Tf2-Static-Broadcaster-Py
   Writing-A-Tf2-Static-Broadcaster-Cpp
   Writing-A-Tf2-Broadcaster-Py
   Writing-A-Tf2-Broadcaster-Cpp
   Writing-A-Tf2-Listener-Py
   Writing-A-Tf2-Listener-Cpp
   Adding-A-Frame-Py
   Adding-A-Frame-Cpp
   Learning-About-Tf2-And-Time-Py
   Learning-About-Tf2-And-Time-Cpp
   Time-Travel-With-Tf2-Py
   Time-Travel-With-Tf2-Cpp
   Debugging-Tf2-Problems
   Quaternion-Fundamentals
   Using-Stamped-Datatypes-With-Tf2-Ros-MessageFilter

Workspace setup
---------------

If you have not yet created a workspace in which to complete the tutorials, :ref:`follow this tutorial <ROS2Workspace>`.

Learning tf2
------------

#. :ref:`Introduction to tf2 <IntroToTf2>`.

   This tutorial will give you a good idea of what tf2 can do for you.
   It shows off some of the tf2 power in a multi-robot example using turtlesim.
   This also introduces using ``tf2_echo``, ``view_frames``, and ``rviz``.

#. Writing a tf2 static broadcaster :ref:`(Python) <WritingATf2StaticBroadcasterPy>` :ref:`(C++) <WritingATf2StaticBroadcasterCpp>`.

   This tutorial teaches you how to broadcast static coordinate frames to tf2.

#. Writing a tf2 broadcaster :ref:`(Python) <WritingATf2BroadcasterPy>` :ref:`(C++) <WritingATf2BroadcasterCpp>`.

   This tutorial teaches you how to broadcast the state of a robot to tf2.

#. Writing a tf2 listener :ref:`(Python) <WritingATf2ListenerPy>` :ref:`(C++) <WritingATf2ListenerCpp>`.

   This tutorial teaches you how to use tf2 to get access to frame transformations.

#. Adding a frame :ref:`(Python) <AddingAFramePy>` :ref:`(C++) <AddingAFrameCpp>`.

   This tutorial teaches you how to add an extra fixed frame to tf2.

#. Learning about tf2 and time :ref:`(Python) <LearningAboutTf2AndTimePy>` :ref:`(C++) <LearningAboutTf2AndTimeCpp>`.

   This tutorial teaches you to use the timeout in ``lookup_transform`` function to
   wait for a transform to be available on the tf2 tree.

#. Time travel with tf2 :ref:`(Python) <TimeTravelWithTf2Py>` :ref:`(C++) <TimeTravelWithTf2Cpp>`.

   This tutorial teaches you about advanced time travel features of tf2.

Debugging tf2
-------------

#. :ref:`Quaternion fundamentals <QuaternionFundamentals>`.

   This tutorial teaches you basics of quaternion usage in ROS 2.

#. :ref:`Debugging tf2 problems <DebuggingTf2Problems>`.

   This tutorial teaches you about a systematic approach for debugging tf2 related problems.

Using sensor messages with tf2
------------------------------

#. :ref:`Using stamped datatypes with tf2_ros::MessageFilter <UsingStampedDatatypesWithTf2RosMessageFilter>`.

   This tutorial teaches you how to use ``tf2_ros::MessageFilter`` to process stamped datatypes.
