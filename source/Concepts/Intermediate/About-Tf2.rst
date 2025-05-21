.. redirect-from::

   Concepts/About-Tf2

Tf2
===

.. contents:: Table of Contents
   :local:

Overview
--------

tf2 is the transform library, which lets the user keep track of multiple coordinate frames over time.
tf2 maintains the relationship between coordinate frames in a tree structure buffered in time and lets the user transform points, vectors, etc. between any two coordinate frames at any desired point in time.

.. image:: ../images/ros2_tf2_frames.png

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

.. mermaid::

   flowchart LR
      E((Earth))
      E --> A[[Car A]]
      E --> B[[Car B]]
      E --> C{{Satellite C}}
      E --> D((Moon D))

Publishing transforms
^^^^^^^^^^^^^^^^^^^^^

When publishing transforms we typically think of the transforms as the transform from one frame to the other.
The semantic difference is whether you are transforming data represented in a frame or transforming the frame itself.
These values are directly inverse.
Transforms published in the ``geometry_msgs/msg/Transform`` message represent the frame formulation.
Keep this in mind when debugging published transforms they are the inverse of what you will lookup depending on what direction you're traversing the transform tree.

.. math::


   _{B}T^{data}_{A} = (_{B}T^{frame}_{A})^{-1}

The TF library handles inverting these elements for you depending on which way you're traversing the transform tree.
For the rest of this document we will just use :math:`T^{data}` but the ``data`` is unwritten.

Position
^^^^^^^^

If the driver in car :math:`A` observes something and a person on the ground wants to know where it is relative to it's position, you transform the observation from the source frame to the target frame.

.. math::

   _{E}T_{A} * P_{A}^{Obs} = P_{E}^{Obs}


Now if a person in car B wants to know where it is too you can compute the net transform.


.. math::

   _{B}T_{E} * _{E}T_{A} * P_{A}^{Obs} = _{B}T_{A} * P_{A}^{Obs} = P_{B}^{Obs}


This is exactly what ``lookupTransform`` provides where ``A`` is the *source* ``frame_id`` and ``B`` is the *target* ``frame_id``.

It is recommended to use the ``transform<T>(target_frame, ...)`` methods when possible because they will read the *source* ``frame_id`` from the datatype and write out the *target* ``frame_id`` in the resulting datatype and the math will be taken care of internally.

If :math:`P` is a ``Stamped`` datatype then :math:`_A` is it's ``frame_id``.

As an example, if a root frame ``A`` is one meter below frame ``B`` the transform from ``A`` to ``B`` is positive.

However when converting data from coordinate frame ``B`` to coordinate frame ``A`` you have to use the inverse of this value.
This can be seen as you'll be adding value to the height when you change to the lower reference frame.
However if you are transforming data from coordinate frame ``A`` into coordinate frame ``B`` the height is reduced because the new reference is higher.

.. math::


   _{B}T_{A} = (_{B}{Tf}_{A})^{-1}


Velocity
^^^^^^^^

For representing ``Velocity`` we have three pieces of information.
:math:`V^{moving\_frame - reference\_frame}_{observing\_frame}`
This velocity represents the velocity between the moving frame and the reference frame.
And it is represented in the observing frame.

For example a driver in Car A can report that they're driving forward (observed in A) at 1m/s (relative to earth) so that would be :math:`V_{A}^{A - E} = (1,0,0)`
Whereas that same velocity could be observed from the view point of the earth (lets assume the car is driving east and Earth is NED), it would be :math:`V_{E}^{A - E} = (0, 1, 0)`

However transforms can show that these are actually the same with:

.. math::

   _{E}T_{A} * V_{A}^{A - E} = V_{E}^{A - E}


Velocities can be added or subtracted if they're represented in the same frame, in this case ``Obs``.

.. math::

   V_{Obs}^{A - C} = V_{Obs}^{A - B} + V_{Obs}^{D - C}

Velocities can be "reversed" by inverting.

.. math::

   V_{Obs}^{A - C} = -(V_{Obs}^{C - A})

If you want to compare two velocities you must first transform them into the same observational frame first.


Tutorials
---------

We created a set of :doc:`tutorials <../../Tutorials/Intermediate/Tf2/Tf2-Main>` that walks you through using tf2, step by step.
You can get started on the :doc:`introduction to tf2 <../../Tutorials/Intermediate/Tf2/Introduction-To-Tf2>` tutorial.
For a complete list of all tf2 and tf2-related tutorials check out the :doc:`tutorials <../../Tutorials/Intermediate/Tf2/Tf2-Main>` page.

There are essentially two main tasks that any user would use tf2 for, listening for transforms and broadcasting transforms.

If you want to use tf2 to transform between coordinate frames, your nodes will need to listen for transforms.
What you will do is receive and buffer all coordinate frames that are broadcasted in the system, and query for specific transforms between frames.
Check out the "Writing a listener" tutorial :doc:`(Python) <../../Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py>` :doc:`(C++) <../../Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp>` to learn more.

To extend the capabilities of a robot, you will need to start broadcasting transforms.
Broadcasting transforms means to send out the relative pose of coordinate frames to the rest of the system.
A system can have many broadcasters that each provide information about a different part of the robot.
Check out the "Writing a broadcaster" tutorial :doc:`(Python) <../../Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py>` :doc:`(C++) <../../Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Cpp>` to learn more.

In addition to that, tf2 can broadcast static transforms that do not change over time.
This mainly saves storage and lookup time, but also reduces the publishing overhead.
You should note that static transforms are published once and assumed to not change, so no history is stored.
If you want to define static transforms in your tf2 tree, take a look at the "Writing a static broadcaster" :doc:`(Python) <../../Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py>` :doc:`(C++) <../../Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Cpp>` tutorial.

You can also learn how to add fixed and dynamic frames to your tf2 tree in the "Adding a frame" :doc:`(Python) <../../Tutorials/Intermediate/Tf2/Adding-A-Frame-Py>` :doc:`(C++) <../../Tutorials/Intermediate/Tf2/Adding-A-Frame-Cpp>` tutorial.

Once you are finished with the basic tutorials, you can move on to learn about tf2 and time.
The tf2 and time tutorial :doc:`(C++) <../../Tutorials/Intermediate/Tf2/Learning-About-Tf2-And-Time-Cpp>` teaches the basic principles of tf2 and time.
The advanced tutorial about tf2 and time :doc:`(C++) <../../Tutorials/Intermediate/Tf2/Time-Travel-With-Tf2-Cpp>` teaches the principles of time traveling with tf2.

Paper
-----

There is a paper on tf2 presented at TePRA 2013: `tf: The transform library <https://ieeexplore.ieee.org/abstract/document/6556373>`_.
