.. _QuaternionFundamentals:

Quaternion fundamentals
=======================

**Goal:** Learn the basics of quaternion usage in ROS 2.

**Tutorial level:** Intermediate

**Time:** 10 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

A quaternion is a 4-tuple representation of orientation, which is more concise than a rotation matrix.
Quaternions are very efficient for analyzing situations where rotations in three dimensions are involved.
Quaternions are used widely in robotics, quantum mechanics, computer vision, and 3D animation.

You can learn more about the underlying mathematical concept on `Wikipedia <https://en.wikipedia.org/wiki/Quaternion>`_.
You can also take a look at an explorable video series `Visualizing quaternions <https://eater.net/quaternions>`_ made by `3blue1brown <https://www.youtube.com/3blue1brown>`_.

In this tutorial, you will learn how quaternions and conversion methods work in ROS 2.

Prerequisites
-------------

For this tutorial you may need to install the ``tf_transformations`` ROS 2 package to follow the Python code parts.
You can find the source code of the ``tf_transformations`` package `here <https://github.com/DLu/tf_transformations>`_.

However, this is not a hard requirement and you can stick to any other geometric transfromation library that suit you best.
You can take a look at libraries like `transforms3d <https://github.com/matthew-brett/transforms3d>`_, `scipy.spatial.transform <https://github.com/scipy/scipy/tree/master/scipy/spatial/transform>`_, `pytransform3d <https://github.com/rock-learning/pytransform3d>`_, `numpy-quaternion <https://github.com/moble/quaternion>`_ or `blender.mathutils <https://docs.blender.org/api/master/mathutils.html>`_.

Components of a quaternion
--------------------------

ROS 2 uses quaternions to track and apply rotations.
A quaternion has 4 components ``(x, y, z, w)``.
In ROS 2, ``w`` is last, but in some libraries like Eigen, ``w`` can be placed at the first position.
The commonly-used unit quaternion that yields no rotation about the x/y/z axes is ``(0, 0, 0, 1)``, and can be created in a following way:

.. code-block:: C++

   #include <tf2/LinearMath/Quaternion.h>
   ...

   tf2::Quaternion q;
   // Create a quaternion from roll/pitch/yaw in radians (0, 0, 0)
   q.setRPY(0, 0, 0);
   // Print the quaternion components (0, 0, 0, 1)
   RCLCPP_INFO(this->get_logger(), "%f %f %f %f",
               q.x(), q.y(), q.z(), q.w());

The magnitude of a quaternion should always be one.
If numerical errors cause a quaternion magnitude other than one, ROS 2 will print warnings.
To avoid these warnings, normalize the quaternion:

.. code-block:: C++

   q.normalize();

Quaternion types in ROS 2
-------------------------

ROS 2 uses two quaternion datatypes: ``tf2::Quaternion`` and its equivalent ``geometry_msgs::msg::Quaternion``.
To convert between them in C++, use the methods of ``tf2_geometry_msgs``.

C++

.. code-block:: C++

   #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
   ...

   tf2::Quaternion tf2_quat, tf2_quat_from_msg;
   tf2_quat.setRPY(roll, pitch, yaw);
   // Convert tf2::Quaternion to geometry_msgs::msg::Quaternion
   geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);

   // Convert geometry_msgs::msg::Quaternion to tf2::Quaternion
   tf2::convert(msg_quat, tf2_quat_from_msg);
   // or
   tf2::fromMsg(msg_quat, tf2_quat_from_msg);


Python

.. code-block:: python

   from geometry_msgs.msg import Quaternion
   ...

   # Create a list of floats, which is compatible with tf2
   # Quaternion methods
   quat_tf = [0.0, 1.0, 0.0, 0.0]

   # Convert a list to geometry_msgs.msg.Quaternion
   msg_quat = Quaternion(x=quat_tf[0], y=quat_tf[1], z=quat_tf[2], w=quat_tf[3])

Quaternion operations
---------------------

1 Think in RPY then convert to quaternion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It's easy for us to think of rotations about axes, but hard to think in terms of quaternions.
A suggestion is to calculate target rotations in terms of roll (about an X-axis), pitch (about the Y-axis), and yaw (about the Z-axis), and then convert to a quaternion.

.. code-block:: python

   import tf_transformations
   ...

   q = tf_transformations.quaternion_from_euler(1.5707, 0, -1.5707)
   print(f'The quaternion representation is x: {q[0]} y: {q[1]} z: {q[2]} w: {q[3]}.')


2 Applying a quaternion rotation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To apply the rotation of one quaternion to a pose, simply multiply the previous quaternion of the pose by the quaternion representing the desired rotation.
The order of this multiplication matters.

C++

.. code-block:: C++

   #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
   ...

   tf2::Quaternion q_orig, q_rot, q_new;

   q_orig.setRPY(0.0, 0.0, 0.0);
   // Rotate the previous pose by 180* about X
   q_rot.setRPY(3.14159, 0.0, 0.0);
   q_new = q_rot * q_orig;
   q_new.normalize();

Python

.. code-block:: python

   import tf_transformations
   ...

   q_orig = tf_transformations.quaternion_from_euler(0, 0, 0)
   # Rotate the previous pose by 180* about X
   q_rot = tf_transformations.quaternion_from_euler(3.14159, 0, 0)
   q_new = tf_transformations.quaternion_multiply(q_rot, q_orig)


3 Inverting a quaternion
^^^^^^^^^^^^^^^^^^^^^^^^

An easy way to invert a quaternion is to negate the w-component:

.. code-block:: python

   q[3] = -q[3]

4 Relative rotations
^^^^^^^^^^^^^^^^^^^^

Say you have two quaternions from the same frame, ``q_1`` and ``q_2``.
You want to find the relative rotation, ``q_r``, that converts ``q_1`` to ``q_2`` in a following manner:

.. code-block:: C++

   q_2 = q_r * q_1

You can solve for ``q_r`` similarly to solving a matrix equation.
Invert ``q_1`` and right-multiply both sides. Again, the order of multiplication is important:

.. code-block:: C++

   q_r = q_2 * q_1_inverse

Here's an example to get the relative rotation from the previous robot pose to the current robot pose in python:

.. code-block:: python

   q1_inv[0] = prev_pose.pose.orientation.x
   q1_inv[1] = prev_pose.pose.orientation.y
   q1_inv[2] = prev_pose.pose.orientation.z
   q1_inv[3] = -prev_pose.pose.orientation.w # Negate for inverse

   q2[0] = current_pose.pose.orientation.x
   q2[1] = current_pose.pose.orientation.y
   q2[2] = current_pose.pose.orientation.z
   q2[3] = current_pose.pose.orientation.w

   qr = tf_transformations.quaternion_multiply(q2, q1_inv)

Summary
-------

In this tutorial, you learned about the fundamental concepts of a quaternion and its related mathematical operations, like inversion and rotation.
You also learned about its usage examples in ROS 2 and conversion methods between two separate Quaternion classes.
