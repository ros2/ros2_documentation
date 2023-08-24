Migrating from ROS 1 to ROS 2
=============================

These guides show how to convert existing ROS 1 packages to ROS 2.
If you are new to porting between ROS 1 and ROS 2, it is recommended to read through the guides in order.

.. toctree::
   :maxdepth: 1

   Migrating-from-ROS1/Migrating-Packages
   Migrating-from-ROS1/Migrating-Interfaces
   Migrating-from-ROS1/Migrating-CPP-Packages
   Migrating-from-ROS1/Migrating-Python-Packages
   Migrating-from-ROS1/Migrating-Launch-Files
   Migrating-from-ROS1/Migrating-Parameters
   Migrating-from-ROS1/Migrating-Scripts

Automatic tools
---------------

There are also some automatic conversion tools that exist, though they are not exhaustive:

* `Magical ROS 2 Conversion Tool <https://github.com/DLu/roscompile/tree/main/magical_ros2_conversion_tool>`_
* Launch File migrator that converts a ROS 1 XML launch file to a ROS 2 Python launch file: https://github.com/aws-robotics/ros2-launch-file-migrator
* Amazon has made their tools for porting from ROS 1 to ROS 2 available at: https://github.com/awslabs/ros2-migration-tools/tree/master/porting\_tools
* `rospy2 <https://github.com/dheera/rospy2>`_ Python project to automatically convert rospy calls to rclpy calls
