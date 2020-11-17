
ROS 2 Eloquent Elusor (codename 'eloquent'; November 22nd, 2019)
================================================================

.. contents:: Table of Contents
   :depth: 2
   :local:

*Eloquent Elusor* is the fifth release of ROS 2.

Supported Platforms
-------------------

Eloquent Elusor is primarily supported on the following platforms:

Tier 1 platforms:

* Ubuntu 18.04 (Bionic): ``amd64`` and ``arm64``
* Mac macOS 10.14 (Mojave)
* Windows 10 (Visual Studio 2019)

Tier 2 platforms:

* Ubuntu 18.04 (Bionic): ``arm32``

Tier 3 platforms:

* Debian Stretch (9): ``amd64``, ``arm64`` and ``arm32``
* OpenEmbedded Thud (2.6) / webOS OSE: ``arm32`` and ``x86``

For more information about RMW implementations, compiler / interpreter versions, and system dependency versions see `REP 2000 <https://www.ros.org/reps/rep-2000.html>`__.

Installation
------------

`Install Eloquent Elusor <../eloquent/Installation/Summary.html>`__

New features in this ROS 2 release
----------------------------------

A few features and improvements we would like to highlight:

* `Support for markup-based launch files (XML/YAML) <https://github.com/ros2/launch/pull/226>`__
* `Improved launch-based testing <https://github.com/ros2/ros2/issues/739#issuecomment-555743540>`__
* `Passing key-value parameters on CLI <https://github.com/ros2/design/pull/245>`__
* `Support stream logging macros <https://github.com/ros2/rclcpp/pull/926>`__
* `Per-node logging <https://github.com/ros2/ros2/issues/789>`__ - All stdout/stderr output from nodes are logged in ~/.ros
* `ros2doctor <https://index.ros.org/doc/ros2/Tutorials/Getting-Started-With-Ros2doctor/>`__
* `Improved performance of sourcing setup files <https://github.com/ros2/ros2/issues/764>`__
* rviz: `interactive markers <https://github.com/ros2/rviz/pull/457>`__, `torque ring <https://github.com/ros2/rviz/pull/396>`__, `tf message filters <https://github.com/ros2/rviz/pull/375>`__
* rqt: `parameter plugin <https://github.com/ros-visualization/rqt_reconfigure/pull/31>`__, `tf tree plugin <https://github.com/ros-visualization/rqt_tf_tree/pull/13>`__, `robot steering plugin <https://github.com/ros-visualization/rqt_robot_steering/pull/7>`__ (also backported to Dashing)
* `turtlesim <https://github.com/ros/ros_tutorials/pull/53>`__ (also backported to Dashing)
* RMW implementations:

  * `API to loan message for zero copy <https://github.com/ros2/design/pull/256>`__, used by `rmw_iceoryx <https://github.com/ros2/rmw_iceoryx>`__
  * `Fast RTPS 1.9.3 <https://github.com/ros2/ros2/issues/734#issuecomment-518018479>`__
  * New Tier-2 implementation: `rmw_cyclonedds <https://github.com/ros2/rmw_cyclonedds>`__ (also backported to Dashing)

* Environment variable `ROS_LOCALHOST_ONLY <https://github.com/ros2/ros2/issues/798>`__ to limit communication to localhost
* MacOS Mojave Support
* `Tracing instrumentation <https://github.com/ros2/ros2/pull/748>`__ for rcl and rclcpp


During the development the `Eloquent meta ticket <https://github.com/ros2/ros2/issues/734>`__ on GitHub contains an up-to-date state of the ongoing high level tasks as well as references specific tickets with more details.

Changes since the Dashing release
---------------------------------

geometry_msgs
^^^^^^^^^^^^^

The ``geometry_msgs/msg/Quaternion.msg`` interface now default initializes to a valid quaternion, with the following values:

.. math::

    x = 0 \\
    y = 0 \\
    z = 0 \\
    w = 1

Here is the pull request for more detail: `https://github.com/ros2/common_interfaces/pull/74 <https://github.com/ros2/common_interfaces/pull/74>`_

Static transform broadcasters and listeners now use QoS durability ``transient_local`` on the ``/tf_static`` topic.
Similar to the latched setting in ROS 1, static transforms only need to be published once.
New listeners will receive transforms from all static broadcasters that are alive and have published before.
All publishers must be updated to use this durability setting or their messages won't be received by transform listeners.
See this pull request for more detail: `https://github.com/ros2/geometry2/pull/160 <https://github.com/ros2/geometry2/pull/160>`_

rclcpp
^^^^^^

API Break with ``get_actual_qos()``
"""""""""""""""""""""""""""""""""""

Introduced in Dashing, the ``get_actual_qos()`` method on the ``PublisherBase`` and ``SubscriptionBase`` previously returned an rmw type, ``rmw_qos_profile_t``, but that made it awkward to reuse with the creation of other entities.
Therefore it was updated to return a ``rclcpp::QoS`` instead.

Existing code will need to use the ``rclcpp::QoS::get_rmw_qos_profile()`` method if the rmw profile is still required.
For example:

.. code-block:: cpp

    void my_func(const rmw_qos_profile_t & rmw_qos);

    /* Previously: */
    // my_func(some_pub->get_actual_qos());
    /* Now: */
    my_func(some_pub->get_actual_qos()->get_rmw_qos_profile());

The rationale for breaking this directly rather than doing a tick-tock is that it is a new function and is expected to be used infrequently by users.
Also, since only the return type is changing, adding a new function with a different would be to only way to do a deprecation cycle and ``get_actual_qos()`` is the most appropriate name, so we would be forced to pick a less obvious name for the method.

API Break with Publisher and Subscription Classes
"""""""""""""""""""""""""""""""""""""""""""""""""

In an effort to streamline the construction of Publishers and Subscriptions, the API of the constructors were changed.

It would be impossible to support a deprecation cycle, because the old signature takes an rcl type and the new one takes the ``NodeBaseInterface`` type so that it can get additional information it now needs, and there's no way to get the additional information needed from just the rcl type.
The new signature could possibly be backported if that would help contributors, but since the publishers and subscriptions are almost always created using the factory functions or some other higher level API, we do not expect this to be a problem for most users.

Please see the original pr for more detail and comment there if this causes issues:

`https://github.com/ros2/rclcpp/pull/867 <https://github.com/ros2/rclcpp/pull/867>`_

Compiler warning about unused result of ``add_on_set_parameters_callback``
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

*Since Eloquent Patch Release 2 (2020-12-04)*

Users should retain the  handle returned by ``rclcpp::Node::add_on_set_parameters_callback``, otherwise their callback may be unregistered.
A warning has been added to help identify bugs where the returned handle is not used.

`https://github.com/ros2/rclcpp/pull/1243 <https://github.com/ros2/rclcpp/pull/1243>`_

rmw
^^^

API Break Due to Addition of Publisher and Subscription Options
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

The ``rmw_create_publisher()`` method had a new argument added of type ``const rmw_publisher_options_t *``.
This new structure holds options (beyond the typesupport, topic name, and QoS) for new publishers.

The ``rmw_create_subscription()`` method had one argument removed, ``bool ignore_local_publications``, and replaced by the new options of type ``const rmw_subscription_options_t *``.
The ``ignore_local_publications`` option was moved into the new ``rmw_subscription_options_t`` type.

In both cases the new argument, which are pointers, may never be null, and so the rmw implementations should check to make sure the options are not null.
Additionally, the options should be copied into the corresponding rmw structure.

See this pull request, and the associated pull requests for more details:

`https://github.com/ros2/rmw/pull/187 <https://github.com/ros2/rmw/pull/187>`_

ros2cli
^^^^^^^

ros2msg and ros2srv deprecated
""""""""""""""""""""""""""""""

The CLI tools ``ros2msg`` and ``ros2srv`` are deprecated.
They have been replaced by the tool ``ros2interface``, which also supports action and IDL interfaces.
You can run ``ros2 interface --help`` for usage.

ros2node
""""""""

Service clients have been added to ros2node info.
As part of that change the Python function ``ros2node.api.get_service_info``
has been renamed to ``ros2node.api.get_service_server_info``.

rviz
^^^^

Renamed '2D Nav Goal' tool
""""""""""""""""""""""""""

The tool was renamed to '2D Goal Pose' and the default topic was changed from ``/move_base_simple/goal`` to ``/goal_pose``.

Here is the related pull request:

`https://github.com/ros2/rviz/pull/455 <https://github.com/ros2/rviz/pull/455>`_

TF2 Buffer
^^^^^^^^^^

TF2 buffers now have to be given a timer interface.

If a timer interface is not given, an exception will be thrown.

For example:

.. code-block:: cpp

    tf = std::make_shared<tf2_ros::Buffer>(get_clock());
    // The next two lines are new in Eloquent
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface());
    tf->setCreateTimerInterface(timer_interface);
    // Pass the Buffer to the TransformListener as before
    transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf);

rcl
^^^

ROS command line argument changes
"""""""""""""""""""""""""""""""""

To cope with an increasingly complex interface, with a now extended set of configuration options, ROS CLI syntax has been changed.
As an example, a command line using Dashing syntax like:

.. code-block:: console

    ros2 run some_package some_node foo:=bar __params:=/path/to/params.yaml __log_level:=WARN --user-flag

is written using Eloquent (and onwards) syntax as:

.. code-block:: console

    ros2 run some_package some_node --ros-args --remap foo:=bar --params-file /path/to/params.yaml --log-level WARN -- --user-flag

This explicit syntax affords new features, like single parameter assignment ``--param name:=value``.
For further reference and rationale, check the `ROS command line arguments design document <https://design.ros2.org/articles/ros_command_line_arguments.html>`__.

.. warning::

   Former syntax has been deprecated and is due for removal in the next release.

Known Issues
------------

* `[ros2/rosidl#402] <https://github.com/ros2/rosidl/issues/402>`_ ``find_package(PCL)`` interferes with ROS interface generation.
  Workaround: invoke ``find_package(PCL)`` *after* ``rosidl_generate_interfaces()``.
* `[ros2/rclcpp#893] <https://github.com/ros2/rclcpp/issues/893>`_ ``rclcpp::Context`` is not destroyed because of a reference cycle with ``rclcpp::GraphListener``. This causes a memory leak. A fix has not been backported because of the risk of breaking ABI.

Timeline before the release
---------------------------

A few milestones leading up to the release:

    Mon. Sep 30th (alpha)
        First releases of core packages available.
        Testing can happen from now on (some features might not have landed yet).

    Fri. Oct 18th
        API and feature freeze for core packages
        Only bug fix releases should be made after this point.
        New packages can be released independently.

    Thu. Oct 24th (beta)
        Updated releases of core packages available.
        Additional testing of the latest features.

    Wed. Nov 13th (release candidate)
        Updated releases of core packages available.

    Tue. Nov 19th
        Freeze rosdistro.
        No PRs for Eloquent on the `rosdistro` repo will be merged (reopens after the release announcement).
