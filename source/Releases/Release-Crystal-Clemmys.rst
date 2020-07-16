.. redirect-from::

    Release-Crystal-Clemmys

ROS 2 Crystal Clemmys (codename 'crystal'; December 2018)
=========================================================

.. contents:: Table of Contents
   :depth: 2
   :local:

*Crystal Clemmys* is the third release of ROS 2.

Supported Platforms
-------------------

Crystal Clemmys is primarily supported on the following platforms (see `REP 2000 <https://www.ros.org/reps/rep-2000.html#crystal-clemmys-december-2018-december-2019>`__ for full details):

Tier 1 platforms:

* Ubuntu 18.04 (Bionic)
* Mac macOS 10.12 (Sierra)
* Windows 10

Tier 2 platforms:

* Ubuntu 16.04 (Xenial)


New features in this ROS 2 release
----------------------------------

* Actions in C / C++ (`server <https://github.com/ros2/examples/tree/af08e6f7ac50f7808dbe6165f1adfd8e6cd3a79c/rclcpp/minimal_action_server>`__ / `client <https://github.com/ros2/examples/tree/af08e6f7ac50f7808dbe6165f1adfd8e6cd3a79c/rclcpp/minimal_action_client>`__ examples)
* `gazebo_ros_pkgs <http://gazebosim.org/tutorials?tut=ros2_overview>`__
* `image_transport <https://github.com/ros-perception/image_common/wiki/ROS2-Migration>`__
* `navigation2 <https://github.com/ros-planning/navigation2/blob/master/README.md>`__
* `rosbag2 <https://index.ros.org/r/rosbag2/github-ros2-rosbag2/#crystal>`__
* `rqt <../Tutorials/RQt-Overview-Usage>`
* Improvement in memory management
* Introspection information about nodes
* Launch system improvements

  * `Arguments <https://github.com/ros2/launch/pull/123>`__
  * `Nested launch files <https://github.com/ros2/launch/issues/116>`__
  * `Conditions <https://github.com/ros2/launch/issues/105>`__
  * `Pass params to Nodes <https://github.com/ros2/launch/issues/117>`__

* Laid the groundwork for `file-based logging and /rosout publishing <https://github.com/ros2/rcl/pull/327>`__
* `Time and Duration API in Python <https://github.com/ros2/rclpy/issues/186>`__
* `Parameters work with Python nodes <https://github.com/ros2/rclpy/issues/202>`__


Changes since the Bouncy release
--------------------------------

Changes since the `Bouncy Bolson <Release-Bouncy-Bolson>` release:

* geometry2 - ``tf2_ros::Buffer`` API Change

  ``tf2_ros::Buffer`` now uses ``rclcpp::Time``, with the constructor requiring a ``shared_ptr`` to a ``rclcpp::Clock`` instance.
  See https://github.com/ros2/geometry2/pull/67 for details, with example usage:

  .. code-block:: c++

    #include <tf2_ros/transform_listener.h>
    #include <rclcpp/rclcpp.hpp>
    ...
    # Assuming you have a rclcpp::Node my_node
    tf2_ros::Buffer buffer(my_node.get_clock());
    tf2_ros::TransformListener tf_listener(buffer);

* All ``rclcpp`` and ``rcutils`` logging macros require semicolons.

  See https://github.com/ros2/rcutils/issues/113 for details.

* ``rcutils_get_error_string_safe()`` and ``rcl_get_error_string_safe()`` have been replaced with ``rcutils_get_error_string().str`` and ``rcl_get_error_string().str``.

  See https://github.com/ros2/rcutils/pull/121 for details.

* rmw - ``rmw_init`` API Change

  There are two new structs, the ``rcl_context_t`` and the ``rcl_init_options_t``, which are used with ``rmw_init``.
  The init options struct is used to pass options down to the middleware and is an input to ``rmw_init``.
  The context is a handle which is an output of ``rmw_init`` function is used to identify which init-shutdown cycle each entity is associated with, where an "entity" is anything created like a node, guard condition, etc.

  This is listed here because maintainers of alternative rmw implementations will need to implement these new functions to have their rmw implementation work in Crystal.

  This is the function that had a signature change:

  * `rmw_init <https://github.com/ros2/rmw/blob/b7234243588a70fce105ea20b073f5ef6c1b685c/rmw/include/rmw/init.h#L54-L82>`__

  Additionally, there are these new functions which need to be implemented by each rmw implementation:

  * `rmw_shutdown <https://github.com/ros2/rmw/blob/b7234243588a70fce105ea20b073f5ef6c1b685c/rmw/include/rmw/init.h#L84-L109>`__
  * `rmw_init_options_init <https://github.com/ros2/rmw/blob/b7234243588a70fce105ea20b073f5ef6c1b685c/rmw/include/rmw/init_options.h#L62-L92>`__
  * `rmw_init_options_copy <https://github.com/ros2/rmw/blob/b7234243588a70fce105ea20b073f5ef6c1b685c/rmw/include/rmw/init_options.h#L94-L128>`__
  * `rmw_init_options_fini <https://github.com/ros2/rmw/blob/b7234243588a70fce105ea20b073f5ef6c1b685c/rmw/include/rmw/init_options.h#L130-L153>`__

  Here's an example of what minimally needs to be changed in an rmw implementation to adhere to this API change:

  * `rmw_fastrtps pr <https://github.com/ros2/rmw_fastrtps/pull/237/files>`_

* rcl - ``rcl_init`` API Change

  Like the ``rmw`` change above, there's two new structs in ``rcl`` called ``rcl_context_t`` and ``rcl_init_options_t``.
  The init options are passed into ``rcl_init`` as an input and the context is passed in as an output.
  The context is used to associate all other rcl entities to a specific init-shutdown cycle, effectively making init and shutdown no longer global functions, or rather those functions no longer use an global state and instead encapsulate all state within the context type.

  Any maintainers of a client library implementation (that also uses ``rcl`` under the hood) will need to make changes to work with Crystal.

  These functions were removed:

  * ``rcl_get_global_arguments``
  * ``rcl_get_instance_id``
  * ``rcl_ok``

  These functions had signature changes:

  * `rcl_init <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/init.h#L30-L82>`__
  * `rcl_shutdown <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/init.h#L84-L111>`__
  * `rcl_guard_condition_init <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/guard_condition.h#L54-L99>`__
  * `rcl_guard_condition_init_from_rmw <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/guard_condition.h#L101-L140>`__
  * `rcl_node_init <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/node.h#L100-L194>`__
  * `rcl_timer_init <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/timer.h#L64-L159>`__

  These are the new functions and types:

  * `rcl_context_t <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/context.h#L36-L136>`__
  * `rcl_get_zero_initialized_context <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/context.h#L138-L142>`__
  * `rcl_context_fini <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/context.h#L146-L171>`__
  * `rcl_context_get_init_options <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/context.h#L175-L205>`__
  * `rcl_context_get_instance_id <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/context.h#L207-L233>`__
  * `rcl_context_is_valid <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/context.h#L235-L255>`__
  * `rcl_init_options_t <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/init_options.h#L32-L37>`__
  * `rcl_get_zero_initialized_init_options <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/init_options.h#L39-L43>`__
  * `rcl_init_options_init <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/init_options.h#L45-L73>`__
  * `rcl_init_options_copy <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/init_options.h#L75-L105>`__
  * `rcl_init_options_fini <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/init_options.h#L107-L128>`__
  * `rcl_init_options_get_rmw_init_options <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/init_options.h#L130-L153>`__
  * `rcl_node_is_valid_except_context <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/node.h#L288-L299>`__
  * `rcl_publisher_get_context <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/publisher.h#L378-L404>`__
  * `rcl_publisher_is_valid_except_context <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/publisher.h#L428-L439>`__

  These new and changed functions will impact how you handle init and shutdown in your client library.
  For examples, look at the following ``rclcpp`` and ``rclpy`` PR's:

  * `rclcpp <https://github.com/ros2/rclcpp/pull/587>`__
  * `rclpy <https://github.com/ros2/rclpy/pull/249>`__

  However, you may just continue to offer a single, global init and shutdown in your client library, and just store a single global context object.

Known Issues
------------

* A race condition in Fast-RTPS 1.7.0 may cause messages to drop under stress (`Issue <https://github.com/ros2/rmw_fastrtps/issues/258>`__).
* Using the TRANSIENT_LOCAL QoS setting with rmw_fastrtps_cpp can crash applications with large messages (`Issue <https://github.com/ros2/rmw_fastrtps/issues/257>`__).
* Cross-vendor communication between rmw_fastrtps_cpp and other implementations is not functioning on Windows (`Issue <https://github.com/ros2/rmw_fastrtps/issues/246>`__).
* When using OpenSplice (version < 6.9.190227) on macOS and Windows you might experience naming conflicts when when referencing field types with names from other packages if the same name also exist in the current package (`Issue <https://github.com/ros2/rmw_opensplice/issues/259>`__).
  By updating to a newer OpenSplice version as well as at least the third patch release of Crystal the problem should be resolved.
  On Linux updating to the latest Debian packages will include the newest OpenSplice version.
