Migrating Dependencies
======================

This page shows you how to tell if all of your ROS 1 package's dependencies are available in ROS 2.

.. contents:: Table of Contents
   :depth: 2
   :local:

Prerequisites
-------------

This page assumes you have Linux machine with :doc:`rosdep </Tutorials/Intermediate/Rosdep>` installed.

Why do dependencies matter?
---------------------------

Virtually all ROS packages depend on something.
A package that needs the transform between two points probably depends on ``tf2``.
A package that installs URDF files probably needs ``xacro``.
No package can work without its dependencies, so when you want to migrate any package to ROS 2 you must make sure all of its dependencies are available first.

Ask ``rosdep`` if your dependencies are available
-------------------------------------------------

Use :doc:`rosdep </Tutorials/Intermediate/Rosdep>` to determine if your ROS 1 package's dependencies are available in ROS 2.

First, decide what ROS 2 distro you want your package to work on.
Next, look up the Ubuntu version supported by that ROS distro in `REP 2000 <https://www.ros.org/reps/rep-2000.html>`__.
Finally run these two commands:

1. Run ``rosdep keys --from-paths PATH_TO_YOUR_ROS1_PACKAGE`` to get a list of your package's dependencies.
2. Run ``rosdep resolve --os=ubuntu:LOWERCASE_UBUNTU_DISTRO --rosdistro=ROS_DISTRO DEPENDENCY1 DEPENDENCY2 ...`` to see which of those dependencies are available in ROS 2.

Example
^^^^^^^

Create a folder called ``my_package`` and create a file ``package.xml`` inside it with the following content:

.. code-block:: xml

    <package format="2">
        <name>my_package</name>
        <version>1.0.0</version>
        <description>
            My ROS 1 package
        </description>
        <maintainer email="gerky@example.com">Brian Gerky</maintainer>
        <license>BSD</license>

        <buildtool_depend>catkin</buildtool_depend>

        <build_depend>cmake_modules</build_depend>
        <build_depend>eigen</build_depend>

        <build_export_depend>eigen</build_export_depend>

        <depend>liborocos-kdl-dev</depend>
        <depend>tf2</depend>
        <depend>tf2_ros</depend>

        <test_depend>ros_environment</test_depend>
        <test_depend>rostest</test_depend>
    </package>

Run ``rosdep keys --from-paths my_package`` to get a list of its dependencies:

.. code-block:: bash

    $ rosdep keys --from-paths my_package/
    WARNING: ROS_PYTHON_VERSION is unset. Defaulting to 3
    liborocos-kdl-dev
    catkin
    cmake_modules
    ros_environment
    eigen
    tf2
    rostest
    tf2_ros

Pretend you've chosen ROS Jazzy.
Look it up in `REP 2000 to discover that it supports Ubuntu Noble <https://www.ros.org/reps/rep-2000.html#jazzy-jalisco-may-2024-may-2029>`__.
Run ``rosdep resolve --os=ubuntu:noble --rosdistro=jazzy ...`` to check if this package's dependencies are available in ROS Jazzy.

.. code-block:: bash

    $ rosdep resolve --os=ubuntu:noble --rosdistro=jazzy eigen liborocos-kdl-dev rostest cmake_modules ros_environment tf2_ros catkin tf2
    #ROSDEP[eigen]
    #apt
    libeigen3-dev
    #ROSDEP[liborocos-kdl-dev]
    #apt
    liborocos-kdl-dev
    #ROSDEP[rostest]
    #ROSDEP[cmake_modules]
    #ROSDEP[ros_environment]
    #apt
    ros-jazzy-ros-environment
    #ROSDEP[tf2_ros]
    #apt
    ros-jazzy-tf2-ros
    #ROSDEP[catkin]
    #ROSDEP[tf2]
    #apt
    ros-jazzy-tf2
    ERROR: no rosdep rule for 'rostest'
    ERROR: no rosdep rule for 'cmake_modules'
    ERROR: no rosdep rule for 'catkin'

Focus on the ``ERROR`` messages.
They say that these three dependencies are not available:

* rostest
* cmake_modules
* catkin

However, these packages might not be in ROS Jazzy because they have been replaced.
Read the next section to learn how to determine that.

Determine if a package has been replaced.

There are three er

Every package's ``package.xml`` file must list that package's dependencies.
There are two kinds of dependencies in ROS:

* a ROS package (`example: tf2 <https://index.ros.org/p/tf2/>`__)
* a rosdep key (`example: eigen <https://index.ros.org/d/eigen/>`__ )

The difference is important when deciding if your package is ready to be migrated.
If your ROS 1 package depends on another ROS package and that ROS package is not available in ROS 2, then you must migrate your dependency to ROS 2 before you can migrate yours.
If your ROS 1 package depends on a rosdep key, and that rosdep key is not available on the platforms that


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