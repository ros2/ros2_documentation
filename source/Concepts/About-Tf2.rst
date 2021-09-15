.. _AboutTf2:

About tf2
=========

.. contents:: Table of Contents
   :depth: 2
   :local:

Overview
--------

tf2 is the transform library, which lets the user keep track of multiple coordinate frames over time.
tf2 maintains the relationship between coordinate frames in a tree structure buffered in time and lets the user transform points, vectors, etc. between any two coordinate frames at any desired point in time.

.. image:: images/ros2_tf2_frames.png

What does tf2 do? Why should I use tf2?
---------------------------------------

You want to see what tf2 can do instead of just reading about it? Check out the :ref:`tf2 introduction demo <IntroToTf2>`.

A robotic system typically has many 3D coordinate frames that change over time, such as a world frame, base frame, gripper frame, head frame, etc. tf2 keeps track of all these frames over time, and allows you to ask questions like:

* Where was the head frame relative to the world frame, 5 seconds ago?
* What is the pose of the object in my gripper relative to my base?
* What is the current pose of the base frame in the map frame? 

tf2 can operate in a distributed system. This means all the information about the coordinate frames of a robot is available to all ROS components on any computer in the system. Tf2 can operate with a central server that contains all transform information, or you can have every component in your distributed system build its own transform information database.

Paper
-----

There is a paper on tf2 presented at TePRA 2013 Papers/TePRA2013_Foote 

Tutorials
---------

We created a set of :ref:`tutorials <Tf2Main>` that walks you through using tf2, step by step.
You can get started on the :ref:`introduction to tf2 <IntroToTf2>` tutorial.
For a complete list of all tf2 and tf2-related tutorials check out the :ref:`tutorials <Tf2Main>` page.

There are essentially two tasks that any user would use tf2 for, listening for transforms and broadcasting transforms.

Anyone using tf2 will need to listen for transforms:

   Listening for transforms - Receive and buffer all coordinate frames that are broadcasted in the system, and query for specific transforms between frames.
   Check out the writing a tf2 listener tutorial :ref:`(Python) <WritingATf2ListenerPy>` :ref:`(C++) <WritingATf2ListenerCpp>`. 

To extend the capabilities of a robot you will need to start broadcasting transforms.

   Broadcasting transforms - Send out the relative pose of coordinate frames to the rest of the system.
   A system can have many broadcasters that each provide information about a different part of the robot.
   Check out the writing a tf2 broadcaster tutorial :ref:`(Python) <WritingATf2BroadcasterPy>` :ref:`(C++) <WritingATf2BroadcasterCpp>`. 

Once you are finished with the basic tutorials, you can move on to learn about tf2 and time.
The tf2 and time tutorial :ref:`(Python) <LearningAboutTf2AndTimePy>` :ref:`(C++) <LearningAboutTf2AndTimeCpp>` teaches the basic principles of tf2 and time.
The advanced tutorial about tf2 and time :ref:`(Python) <TimeTravelWithTf2Py>` :ref:`(C++) <TimeTravelWithTf2Cpp>` teaches the principles of time traveling with tf2.

If you are looking for an easy tool to manually tweak tf2 transforms, such as for quick calibration-by-eye tuning, try Manual TF2 Calibration Tools 

Further Reading
---------------

* ROS 2 Discourse `announcment of porting to ROS 2 <https://discourse.ros.org/t/rqt-in-ros2/6428>`__).
* `RQt for ROS 1 documentation <https://wiki.ros.org/rqt>`__.
* Brief overview of RQt (from `a Willow Garage intern blog post <http://web.archive.org/web/20130518142837/http://www.willowgarage.com/blog/2012/10/21/ros-gui>`__).

  .. raw:: html

     <iframe width="560" height="315" src="https://www.youtube-nocookie.com/embed/CyP9wHu2PpY" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
