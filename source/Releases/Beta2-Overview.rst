.. redirect-from::

    Beta2-Overview

Beta 2 (codename 'r2b2'; July 2017)
===================================

.. contents:: Table of Contents
   :depth: 2
   :local:

Supported Platforms
-------------------

We support ROS 2 Beta 2 on three platforms: Ubuntu 16.04 (Xenial), macOS 10.12 (Sierra), and Windows 10.
We provide both binary packages and instructions for how to compile from source for all 3 platforms (see `install instructions <../Installation>` as well as `documentation <https://docs.ros2.org/beta2/>`__).

Features
--------

Improvements since Beta 1 release
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* DDS_Security support (aka SROS2, see `sros2 <https://github.com/ros2/sros2>`__)
* Debian packages for Ubuntu Xenial
* Typesupport has been redesigned so that you only build a single executable and can choose one of the available RMW implementations by setting an environment variable (see `documentation <../Tutorials/Working-with-multiple-RMW-implementations>`).
* Namespace support for nodes and topics (see `design article <https://design.ros2.org/articles/topic_and_service_names.html>`__, see known issues below).
* A set of command-line tools using the extensible ``ros2`` command (see `tutorial <../Tutorials/Introspection-with-command-line-tools>`).
* A set of macros for logging messages in C / C++ (see API docs of `rcutils <https://docs.ros2.org/beta2/api/rcutils/index.html>`__).

New demo application
^^^^^^^^^^^^^^^^^^^^

* `Turtlebot 2 demos <https://github.com/ros2/turtlebot2_demo>`__ using the following repositories that have been (partially) converted to ROS 2 (Linux only):

  * `ros_astra_camera <https://github.com/ros2/ros_astra_camera.git>`__
  * `depthimage_to_laserscan <https://github.com/ros2/depthimage_to_laserscan.git>`__
  * `pcl_conversions <https://github.com/ros2/pcl_conversions.git>`__
  * `cartographer <https://github.com/ros2/cartographer.git>`__
  * `cartographer_ros <https://github.com/ros2/cartographer_ros.git>`__
  * `ceres-solver <https://github.com/ros2/ceres-solver.git>`__
  * `navigation <https://github.com/ros2/navigation.git>`__
  * `teleop_twist_keyboard <https://github.com/ros2/teleop_twist_keyboard.git>`__
  * `joystick_drivers <https://github.com/ros2/joystick_drivers.git>`__
  * `teleop_twist_joy <https://github.com/ros2/teleop_twist_joy.git>`__

* `Dummy_robot demo <../Tutorials/dummy-robot-demo>`:

  * `robot_model <https://github.com/ros2/robot_model>`__
  * `robot_state_publisher <https://github.com/ros2/robot_state_publisher>`__

Selected features from previous Alpha/Beta releases
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For the complete list, see `earlier release notes <../Releases>`.


* C++ and Python implementations of ROS 2 client libraries including APIs for:

  * Publishing and subscribing to ROS topics
  * Requesting and replying ROS services (synchronous (C++ only) and asynchronous)
  * Getting and setting ROS parameters (C++ only, synchronous and asynchronous)
  * Timer callbacks

* Support for interoperability between multiple DDS/RTPS implementations

  * eProsima Fast RTPS is our default implementation, and is included in the binary packages
  * RTI Connext is supported: build from source to try it out
  * We initially supported PrismTech OpenSplice but support for it is currently on hold

* A graph API for network events
* Distributed discovery
* Realtime safe code paths for publish and subscribe with compatible DDS implementation (only Connext at the moment)

  * Support for custom allocators

* ROS 1 <-> ROS 2 dynamic bridge node
* Executor threading model (C++ only)
* Component model to compose nodes at compile / link / runtime
* Managed component using a standard lifecycle
* Extended ``.msg`` format with new features:

  * Bounded arrays
  * Default values

Known issues
^^^^^^^^^^^^

* We’re tracking issues in various repositories, but the main entry point is the `ros2/ros2 issue tracker <https://github.com/ros2/ros2/issues>`__
* We’d like to highlight a `known issue <https://github.com/ros2/rmw_connext/issues/234>`__ that we are looking into which doesn't allow two topics with the same base name but different namespaces to have a different type when using ``rmw_connext_cpp``.
* Services with long responses are not working with Fast-RTPS. The fix, while not being part of beta2, is available upstream so you can work around this issue by building from source using Fast-RTPS master branch.
