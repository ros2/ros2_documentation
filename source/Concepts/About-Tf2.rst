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

Properties of tf2
-----------------

A robotic system typically has many 3D coordinate frames that change over time, such as a world frame, base frame, gripper frame, head frame, etc.
tf2 keeps track of all these frames over time, and allows you to ask questions like:

* Where was the head frame relative to the world frame 5 seconds ago?
* What is the pose of the object in my gripper relative to my base?
* What is the current pose of the base frame in the map frame?

tf2 can operate in a distributed system.
This means all the information about the coordinate frames of a robot is available to all ROS 2 components on any computer in the system.
tf2 can have every component in your distributed system build its own transform information database or have a central node that gathers and stores all transform information.

Tutorials
---------

We created a set of :doc:`tutorials <../Tutorials/Intermediate/Tf2/Tf2-Main>` that walks you through using tf2, step by step.
You can get started on the :doc:`introduction to tf2 <../Tutorials/Intermediate/Tf2/Introduction-To-Tf2>` tutorial.
For a complete list of all tf2 and tf2-related tutorials check out the :doc:`tutorials <../Tutorials/Intermediate/Tf2/Tf2-Main>` page.

There are essentially two main tasks that any user would use tf2 for, listening for transforms and broadcasting transforms.

If you want to use tf2 to transform between coordinate frames, your nodes will need to listen for transforms.
What you will do is receive and buffer all coordinate frames that are broadcasted in the system, and query for specific transforms between frames.
Check out the "Writing a listener" tutorial :doc:`(Python) <../Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py>` :doc:`(C++) <../Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp>` to learn more.

To extend the capabilities of a robot, you will need to start broadcasting transforms.
Broadcasting transforms means to send out the relative pose of coordinate frames to the rest of the system.
A system can have many broadcasters that each provide information about a different part of the robot.
Check out the "Writing a broadcaster" tutorial :doc:`(Python) <../Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py>` :doc:`(C++) <../Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Cpp>` to learn more.

In addition to that, tf2 can broadcast static transforms that do not change over time.
This mainly saves storage and lookup time, but also reduces the publishing overhead.
You should note that static transforms are published once and assumed to not change, so no history is stored.
If you want to define static transforms in your tf2 tree, take a look at the "Writing a static broadcaster" :doc:`(Python) <../Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py>` :doc:`(C++) <../Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Cpp>` tutorial.

You can also learn how to add fixed and dynamic frames to your tf2 tree in the "Adding a frame" :doc:`(Python) <../Tutorials/Intermediate/Tf2/Adding-A-Frame-Py>` :doc:`(C++) <../Tutorials/Intermediate/Tf2/Adding-A-Frame-Cpp>` tutorial.

Once you are finished with the basic tutorials, you can move on to learn about tf2 and time.
The tf2 and time tutorial :doc:`(Python) <../Tutorials/Intermediate/Tf2/Learning-About-Tf2-And-Time-Py>` :doc:`(C++) <../Tutorials/Intermediate/Tf2/Learning-About-Tf2-And-Time-Cpp>` teaches the basic principles of tf2 and time.
The advanced tutorial about tf2 and time :doc:`(Python) <../Tutorials/Intermediate/Tf2/Time-Travel-With-Tf2-Py>` :doc:`(C++) <../Tutorials/Intermediate/Tf2/Time-Travel-With-Tf2-Cpp>` teaches the principles of time traveling with tf2.

Paper
-----

There is a paper on tf2 presented at TePRA 2013: `tf: The transform library <https://ieeexplore.ieee.org/abstract/document/6556373>`_.
