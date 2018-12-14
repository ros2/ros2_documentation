ROS 2 Crystal Clemmys (codename 'crystal'; December 2018)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

*Crystal Clemmys* is the third release of ROS 2.

Supported Platforms
^^^^^^^^^^^^^^^^^^^

Crystal Clemmys is primarily supported on the following platforms.
See `REP 2000 <http://www.ros.org/reps/rep-2000.html#crystal-clemmys-december-2018-december-2019>`__ for full details.

Tier 1 platforms:
* Ubuntu 18.04 (Bionic)
* Mac OS X 10.12 (Sierra)
* Windows 10

Tier 2 platforms:
* Ubuntu 16.04 (Xenial)


New features in this ROS 2 release
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* first item goes here

Changes since the Bouncy release
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Changes since the `Bouncy Bolson <Release-Bouncy-Bolson>` release:

* geometry2 - ``tf2_ros::Buffer`` API Change

  ``tf2_ros::Buffer`` now uses ``rclcpp::Time``, with the constructor requiring a ``shared_ptr`` to a ``rclcpp::Clock`` instance.
  See https://github.com/ros2/geometry2/pull/67 for details, with example usage::

    #include <tf2_ros/transform_listener.h>
    #include <rclcpp/rclcpp.hpp>
    ...
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf2_ros::Buffer buffer(clock);
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

  * `rmw_init <https://github.com/ros2/rmw/blob/b7234243588a70fce105ea20b073f5ef6c1b685c/rmw/include/rmw/init.h#L54-L82>`_

  Additionally, there are these new functions which need to be implemented by each rmw implementation:

  * `rmw_shutdown <https://github.com/ros2/rmw/blob/b7234243588a70fce105ea20b073f5ef6c1b685c/rmw/include/rmw/init.h#L84-L109>`_
  * `rmw_init_options_init <https://github.com/ros2/rmw/blob/b7234243588a70fce105ea20b073f5ef6c1b685c/rmw/include/rmw/init_options.h#L62-L92>`_
  * `rmw_init_options_copy <https://github.com/ros2/rmw/blob/b7234243588a70fce105ea20b073f5ef6c1b685c/rmw/include/rmw/init_options.h#L94-L128>`_
  * `rmw_init_options_fini <https://github.com/ros2/rmw/blob/b7234243588a70fce105ea20b073f5ef6c1b685c/rmw/include/rmw/init_options.h#L130-L153>`_

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

  * `rcl_init <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/init.h#L30-L82>`_
  * `rcl_shutdown <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/init.h#L84-L111>`_
  * `rcl_guard_condition_init <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/guard_condition.h#L54-L99>`_
  * `rcl_guard_condition_init_from_rmw <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/guard_condition.h#L101-L140>`_
  * `rcl_node_init <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/node.h#L100-L194>`_
  * `rcl_timer_init <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/timer.h#L64-L159>`_

  These are the new functions and types:

  * `rcl_context_t <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/context.h#L36-L136>`_
  * `rcl_get_zero_initialized_context <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/context.h#L138-L142>`_
  * `rcl_context_fini <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/context.h#L146-L171>`_
  * `rcl_context_get_init_options <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/context.h#L175-L205>`_
  * `rcl_context_get_instance_id <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/context.h#L207-L233>`_
  * `rcl_context_is_valid <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/context.h#L235-L255>`_
  * `rcl_init_options_t <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/init_options.h#L32-L37>`_
  * `rcl_get_zero_initialized_init_options <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/init_options.h#L39-L43>`_
  * `rcl_init_options_init <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/init_options.h#L45-L73>`_
  * `rcl_init_options_copy <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/init_options.h#L75-L105>`_
  * `rcl_init_options_fini <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/init_options.h#L107-L128>`_
  * `rcl_init_options_get_rmw_init_options <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/init_options.h#L130-L153>`_
  * `rcl_node_is_valid_except_context <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/node.h#L288-L299>`_
  * `rcl_publisher_get_context <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/publisher.h#L378-L404>`_
  * `rcl_publisher_is_valid_except_context <https://github.com/ros2/rcl/blob/657d9e84c73e4268176efd163e96fda73c1a76d9/rcl/include/rcl/publisher.h#L428-L439>`_

  These new and changed functions will impact how you handle init and shutdown in your client library.
  For examples, look at the ``rclcpp`` and ``rclpy`` pr's:

  * `rclcpp <https://github.com/ros2/rclcpp/pull/587>`_
  * `rclpy <https://github.com/ros2/rclpy/pull/249>`_

  However, you may just continue to offer a single, global init and shutdown in your client library, and just store a single global context object.

Known Issues
^^^^^^^^^^^^

* first item goes here
