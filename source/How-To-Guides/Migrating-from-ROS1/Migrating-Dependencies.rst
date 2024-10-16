Migrating Dependencies
======================

This page shows you how to tell if all of your ROS 1 package's dependencies are available in ROS 2.

.. contents:: Table of Contents
   :depth: 2
   :local:

Prerequisites
-------------

This page assumes you have a Linux machine with :doc:`rosdep </Tutorials/Intermediate/Rosdep>` installed.

Why do dependencies matter?
---------------------------

Virtually all ROS packages depend on something.
A package that needs the transform between two points probably depends on ``tf2``.
A package that installs URDF files probably needs ``xacro``.
No package can work without its dependencies, so when you want to migrate any package to ROS 2 you must make sure all of its dependencies are available first.

What ROS distro are you targeting?
----------------------------------

TODO this guide assumes you're targeting ROS {DISTRO}.

Determine your package's dependencies
-------------------------------------

If you want to know what your package depends on, then read it's ``package.xml``.
Every package's ``package.xml`` file must list that package's dependencies.

However, there are two kinds of dependencies in ROS, and the difference is important when deciding if your package is ready to be migrated to ROS 2.

* a ROS package
* a rosdep key

A ROS package is exactly what it sounds like; your ROS package may depend on one or more other ROS packages.
A rosdep key is different.
It describes a system dependency.
For example, `tf2 <https://index.ros.org/p/tf2/>`__ is a ROS package, while `eigen <https://index.ros.org/d/eigen/>`__ is a rosdep key.
Read :doc:`the tutorial on rosdep </Tutorials/Intermediate/Rosdep>` to learn more.

Is it a rosdep key or a package?
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

TODO use the rosdep keys command followed by the rosdep resolve command

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

        cd /tmp
        git clone https://github.com/ros/geometry2.git
        rosdep keys --from-paths /tmp/geometry2/tf2_kdl | xargs rosdep resolve --os=ubuntu:focal --rosdistro=noetic


TODO if the package to be installed starts with `ros-distro`, then it's a ROS package. Otherwise it's a ROSDep key

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

        $ rosdep keys --from-paths /tmp/geometry2/tf2_kdl | xargs rosdep resolve --os=ubuntu:focal --rosdistro=noetic
        WARNING: ROS_PYTHON_VERSION is unset. Defaulting to 3
        #ROSDEP[eigen]
        #apt
        libeigen3-dev
        #ROSDEP[cmake_modules]
        #apt
        ros-noetic-cmake-modules
        #ROSDEP[tf2_ros]
        #apt
        ros-noetic-tf2-ros
        #ROSDEP[catkin]
        #apt
        ros-noetic-catkin
        #ROSDEP[rostest]
        #apt
        ros-noetic-rostest
        #ROSDEP[liborocos-kdl-dev]
        #apt
        liborocos-kdl-dev
        #ROSDEP[tf2]
        #apt
        ros-noetic-tf2
        #ROSDEP[ros_environment]
        #apt
        ros-noetic-ros-environment


.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

        cd /tmp
        git clone https://github.com/ros/geometry2.git
        rosdep keys --from-paths /tmp/geometry2/tf2_kdl | xargs rosdep resolve --os=ubuntu:noble --rosdistro=rolling


.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

        $ rosdep keys --from-paths /tmp/geometry2/tf2_kdl | xargs rosdep resolve --os=ubuntu:noble --rosdistro=rolling
        WARNING: ROS_PYTHON_VERSION is unset. Defaulting to 3
        #ROSDEP[tf2]
        #apt
        ros-rolling-tf2
        #ROSDEP[catkin]
        #ROSDEP[rostest]
        #ROSDEP[liborocos-kdl-dev]
        #apt
        liborocos-kdl-dev
        #ROSDEP[ros_environment]
        #apt
        ros-rolling-ros-environment
        #ROSDEP[cmake_modules]
        #ROSDEP[eigen]
        #apt
        libeigen3-dev
        #ROSDEP[tf2_ros]
        #apt
        ros-rolling-tf2-ros
        ERROR: no rosdep rule for 'catkin'
        ERROR: no rosdep rule for 'rostest'
        ERROR: no rosdep rule for 'cmake_modules'


Check if a rosdep key is available
----------------------------------

TODO It matters what OS you're using. We're deling with ssytem deps after all

Check if a ROS package is available
-----------------------------------

TODO Searching ROS Index for the given ROS distro


Has this ROS package been replaced?
-----------------------------------

Some packages haven't been migrated to ROS 2 because they were replaced with something better.
If you can't find a package in the ROS Index, then check the table below to see if it has a replacement.

TODO move_base -> nav2, ... what else?

.. list-table:: Equivalent packages in ROS 1 and ROS 2
   :widths: 25 25
   :header-rows: 1

   * - ROS 1
     - ROS 2
   * - catkin
     - ament_cmake_ros
   * - cmake_modules
     - tinyxml_vendor, tinyxml2_vendor, eigen3_cmake_module
   * - roscpp
     - rclcpp
   * - roslaunch
     - launch_ros
   * - rospy
     - rclpy
   * - rostest
     - launch_testing_ros

Conclusion
----------

You now know if all of your package's dependencies are available in ROS 2.
If any dependency is not available, you must migrate it first.
Head back to :doc:`Migrating Packages <./Migrating-Packages>` to learn how to migrate it.