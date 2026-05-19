Eloquent Elusor (``eloquent``)
==============================

.. contents:: Table of Contents
   :depth: 2
   :local:

*Eloquent Elusor* is the fifth release of ROS 2.

Supported Platforms
-------------------

Eloquent Elusor supports the following platforms according to `the platform support tiers <../The-ROS2-Project/Platform-Support-Tiers>`:

Tier 1 platforms:

* Ubuntu 18.04 (Bionic): ``amd64`` and ``arm64``
* Mac macOS 10.14 (Mojave)
* Windows 10 (Visual Studio 2019)

Tier 2 platforms:

* Ubuntu 18.04 (Bionic): ``arm32``

Tier 3 platforms:

* Debian Stretch (9): ``amd64``, ``arm64`` and ``arm32``
* OpenEmbedded Thud (2.6) / webOS OSE: ``arm32`` and ``x86``

Targeted platforms:

+--------------+----------------------+----------------------+----------------------+-------------------+----------------+
| Architecture | Ubuntu Bionic (18.04)| MacOS Mojave (10.14) | Windows 10 (VS2019)  | Debian Buster (10)| OpenEmbedded / |
|              |                      |                      |                      |                   | webOS OSE      |
+==============+======================+======================+======================+===================+================+
| amd64        | Tier 1 [d][a][s]     | Tier 1 [a][s]        | Tier 1 [a][s]        | Tier 3 [s]        |                |
+--------------+----------------------+----------------------+----------------------+-------------------+----------------+
| arm64        | Tier 1 [d][a][s]     |                      |                      | Tier 3 [s]        | Tier 3 [s]     |
+--------------+----------------------+----------------------+----------------------+-------------------+----------------+
| arm32        | Tier 2 [a][s]        |                      |                      | Tier 3 [s]        | Tier 3 [s]     |
+--------------+----------------------+----------------------+----------------------+-------------------+----------------+


The following indicators show what delivery mechanisms are available for
each platform.

\" \[d\] \" Debian packages will be provided for this platform for
packages submitted to the rosdistro.

\" \[a\] \" Binary releases are provided as a single archive per
platform containing all packages in the Eloquent ROS 2 repos file[^7].

\" \[s\] \" Compilation from source.

Middleware Implementation Support:

+--------------------------+---------------------+---------------+-----------------------------------+-----------------------------------+
| Middleware Library       | Middleware Provider | Support Level | Platforms                         | Architectures                     |
+==========================+=====================+===============+===================================+===================================+
| rmw_fastrtps_cpp*        | eProsima Fast-RTPS  | Tier 1        | All Platforms                     | All Architectures                 |
+--------------------------+---------------------+---------------+-----------------------------------+-----------------------------------+
| rmw_connext_cpp          | RTI Connext         | Tier 1        | All Platforms except Debian and   | All Architectures except          |
|                          |                     |               | OpenEmbedded                      | arm64/arm32                       |
+--------------------------+---------------------+---------------+-----------------------------------+-----------------------------------+
| rmw_cyclonedds_cpp       | Eclipse Cyclone DDS | Tier 2        | All Platforms                     | All Architectures                 |
+--------------------------+---------------------+---------------+-----------------------------------+-----------------------------------+
| rmw_opensplice_cpp       | ADLINK OpenSplice   | Tier 2        | All Platforms except Debian and   | All Architectures                 |
|                          |                     |               | OpenEmbedded                      |                                   |
+--------------------------+---------------------+---------------+-----------------------------------+-----------------------------------+
| rmw_fastrtps_dynamic_cpp | eProsima Fast-RTPS  | Tier 2        | All Platforms                     | All Architectures                 |
+--------------------------+---------------------+---------------+-----------------------------------+-----------------------------------+

\" \* \" means default RMW implementation.

Middleware implementation support is dependent upon the platform support
tier. For example a Tier 1 middleware implementation on a Tier 2
platform can only receive Tier 2 support.

Minimum language requirements:

- C++14
- Python 3.6

Dependency Requirements:

+--------------+-------------------+----------------+----------------+----------------+-----------------------+
|              | Required Support                                    | Recommended Support                    |
+--------------+-------------------+----------------+----------------+----------------+-----------------------+
| Package      | Ubuntu Bionic     | MacOS**        | Windows 10**   | Debian Buster  | OpenEmbedded**        |
+==============+===================+================+================+================+=======================+
| CMake        | 3.10.2            | 3.14.4         | 3.14.4         | 3.13.4         | 3.16.1 / 3.12.2****   |
+--------------+-------------------+----------------+----------------+----------------+-----------------------+
| EmPY         | 3.3.2                                                                                        |
+--------------+-------------------+----------------+----------------+----------------+-----------------------+
| Gazebo       | 9.0.0             | 9.9.0          | N/A            | 9.8.0*         | N/A                   |
+--------------+-------------------+----------------+----------------+----------------+-----------------------+
| Ogre         | 1.10*                                                                | N/A                   |
+--------------+-------------------+----------------+----------------+----------------+-----------------------+
| OpenCV       | 3.2.0             | 4.1.0          | 3.4.6*         | 3.2.0          | 4.1.0 / 3.2.0****     |
+--------------+-------------------+----------------+----------------+----------------+-----------------------+
| OpenSSL      | 1.1.0g            | 1.0.2r         | 1.0.2r         | 1.1.1c         | 1.1.1d / 1.1.1b****   |
+--------------+-------------------+----------------+----------------+----------------+-----------------------+
| Poco         | 1.8.0             | 1.9.0          | 1.8.0*         | 1.9.0          | 1.9.4                 |
+--------------+-------------------+----------------+----------------+----------------+-----------------------+
| Python       | 3.6.5             | 3.7.3          | 3.7.3          | 3.7.3          | 3.8.2 / 3.7.5****     |
+--------------+-------------------+----------------+----------------+----------------+-----------------------+
| Qt           | 5.9.5             | 5.12.3         | 5.10.0         | 5.11.3         | 5.14.1 / 5.12.5****   |
+--------------+-------------------+----------------+----------------+----------------+-----------------------+
|                                  | **Linux only**                                                           |
+--------------+-------------------+----------------+----------------+----------------+-----------------------+
| PCL          | 1.8.1             | N/A            | N/A            | 1.9.1          | 1.8.1                 |
+--------------+-------------------+----------------+----------------+----------------+-----------------------+
| **RMW DDS Middleware Providers**                                                                            |
+--------------+-------------------+----------------+----------------+----------------+-----------------------+
| Connext DDS  | 5.3.1***                                            | N/A                                    |
+--------------+-------------------+----------------+----------------+----------------+-----------------------+
| Cyclone DDS  | 0.7.x (Coquette)                                                                             |
+--------------+-------------------+----------------+----------------+----------------+-----------------------+
| Fast-RTPS    | 1.9.0                                                                                        |
+--------------+-------------------+----------------+----------------+----------------+-----------------------+
| OpenSplice   | 6.9.190705OSS                                                        | N/A                   |
+--------------+-------------------+----------------+----------------+----------------+-----------------------+


\" \* \" means that this is not the upstream version (available on the
official Operating System repositories) but a package distributed by
OSRF or the community (package built and distributed on custom
repositories).

\" \*\* \" Rolling distributions will see multiple version changes of
these dependencies during their lifetime. The versions shown for
OpenEmbedded are those provided by the 3.1 Dunfell release series; the
versions provided by the other supported release series are listed here:
<https://github.com/ros/meta-ros/wiki/Package-Version-Differences> .
Note that the OpenEmbedded releases series for which a ROS distro has
support will change during its support time frame, as per the
OpenEmbedded support policy shown here:
<https://github.com/ros/meta-ros/wiki/Policies#openembedded-release-series-support>
. However, it will always be supported by least one stable OpenEmbedded
release series.

\" \*\*\* \" It is anticipated that this will be increased to Connext
DDS 6.0.0 pending migration patches[^8].

\" \*\*\*\* \" webOS OSE provides this different version.

This document only captures the version at the first release of a ROS
distribution and will not be updated as the dependencies move forward.
These versions are thus a low watermark.

Package manager use for dependencies:

- Ubuntu, Debian: apt
- MacOS: Homebrew, pip
- Windows: Chocolatey, pip
- OpenEmbedded: opkg

Build System Support:

- ament_cmake
- cmake
- setuptools

Installation
------------

`Install Eloquent Elusor <../../eloquent/Installation.html>`__

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

    $ ros2 run some_package some_node foo:=bar __params:=/path/to/params.yaml __log_level:=WARN --user-flag

is written using Eloquent (and onwards) syntax as:

.. code-block:: console

    $ ros2 run some_package some_node --ros-args --remap foo:=bar --params-file /path/to/params.yaml --log-level WARN -- --user-flag

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
