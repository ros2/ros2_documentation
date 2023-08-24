Migrating Packages from ROS 1 to ROS 2
======================================

.. contents:: Table of Contents
   :depth: 2
   :local:

There are two different kinds of package migrations:

* Migrating the source code of an existing package from ROS 1 to ROS 2 with the intent that a significant part of the source code will stay the same or at least similar.
  An example for this is `pluginlib <https://github.com/ros/pluginlib>`_ where the source code is maintained in different branches within the same repository and common patches can be ported between those branches when necessary.
* Implementing the same or similar functionality of a ROS 1 package for ROS 2 but with the assumption that the source code will be significantly different.
  An example for this is `roscpp <https://github.com/ros/ros_comm/tree/melodic-devel/clients/roscpp>`_ in ROS 1 and `rclcpp <https://github.com/ros2/rclcpp/tree/rolling/rclcpp>`_ in ROS 2 which are separate repositories and don't share any code.

Prerequisites
-------------

Before being able to migrate a ROS 1 package to ROS 2 all of its dependencies must be available in ROS 2.

Package manifests
^^^^^^^^^^^^^^^^^

ROS 2 doesn't support format 1 of the package specification but only newer format versions (2 and higher).
Therefore the ``package.xml`` file must be updated to at least format 2 if it uses format 1.
Since ROS 1 supports all formats it is safe to perform that conversion in the ROS 1 package.

Some packages might have different names in ROS 2 so the dependencies might need to be updated accordingly.

Metapackages
^^^^^^^^^^^^

ROS 2 doesn't have a special package type for metapackages.
Metapackages can still exist as regular packages that only contain runtime dependencies.
When migrating metapackages from ROS 1, simply remove the ``<metapackage />`` tag in your package manifest.
