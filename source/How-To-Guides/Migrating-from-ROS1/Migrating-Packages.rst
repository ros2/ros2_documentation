Migrating Packages
==================

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

Package.xml format version
--------------------------

ROS 2 only supports ``package.xml`` format versions 2 and higher.
If your package's ``package.xml`` uses format 1, then update it using the :doc:`Package.xml format 1 to 2 migration guide <./Migrating-Package-XML>`.

Dependency names
----------------

Dependency names that come from :doc:`rosdep <../../Tutorials/Intermediate/Rosdep>` should not need to change, as those are shared across ROS 1 and ROS 2.

Some packages released into ROS might have different names in ROS 2 so the dependencies might need to be updated accordingly.

Metapackages
------------

ROS 2 doesn't have a special package type for metapackages.
Metapackages can still exist as regular packages that only contain runtime dependencies.
When migrating metapackages from ROS 1, simply remove the ``<metapackage />`` tag in your package manifest.
See :doc:`Using variants <../Using-Variants>` for more information on metapackages/variants.

Licensing
---------

In ROS 1 our recommended license was the `3-Clause BSD License <https://opensource.org/licenses/BSD-3-Clause>`__.
In ROS 2 our recommended license is the `Apache 2.0 License <https://www.apache.org/licenses/LICENSE-2.0>`__.

For any new project we recommend using the Apache 2.0 License, whether ROS 1 or ROS 2.

However, when migrating code from ROS 1 to ROS 2 we cannot simply change the license.
The existing license must be preserved for any preexisting contributions.

To that end if a package is being migrated we recommend keeping the existing license and continuing to contribute to that package under the existing OSI license, which we expect to be the BSD license for core elements.

This will keep things clear and easy to understand.

Changing the License
^^^^^^^^^^^^^^^^^^^^

It is possible to change the license, however you will need to contact all the contributors and get permission.
For most packages this is likely to be a significant effort and not worth considering.
If the package has a small set of contributors then this may be feasible.
