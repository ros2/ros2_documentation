.. _upcoming-release:

.. _lyrical-release:

Lyrical Luth (codename 'lyrical'; May, 2026)
============================================

.. contents:: Table of Contents
   :depth: 2
   :local:

*Lyrical Luth* is the twelfth release of ROS 2.

Release Timeline
----------------

**As soon as possible** - Migrate ROS Rolling to ROS Lyrical's target platforms
    * RHEL 10 + Ubuntu 26.04: Migrate as soon as core packages successfully build on both platforms.
    * Windows 11: Migrate as soon as we have a green build

**Mon. April 13, 2026** - Alpha + RMW freeze (*Delayed; was originally April 6th*)
    * Preliminary testing of ROS Base packages
    * API and feature freeze for RMW provider packages.

**Mon. April 20, 2026** - Freeze (*Delayed; was originally April 13th*)
    * API and feature freeze for ROS Base packages in Rolling Ridley.
    * Only bug fix releases should be made after this point.
    * New packages can be released.

**Mon. April 21, 2026** - Branch (*Delayed; was originally April 20th*)
    * Branch from Rolling Ridley
    * ``rosdistro`` is reopened for Rolling PRs for ROS Base packages.
    * Lyrical development shifts from ``ros-rolling-*`` packages to ``ros-lyrical-*`` packages.

**Mon. April 27, 2026** - Beta
    * Updated releases of ROS Desktop packages available.
    * Call for general testing.

**Thu, April 30, 2026** - Kick off Tutorial Party
    * Open up tutorials for community testing.

**Mon. May 11, 2026** - Release Candidate
    * Build release candidate packages up to ROS Desktop

**Mon. May 18, 2026** - Distro Freeze
    * Freeze all Lyrical branches on all ROS desktop packages
    * No pull requests for any Lyrical branch or targeting ``lyrical/distribution.yaml`` in ``rosdistro`` repo will be merged.

**Friday May 22nd, 2026** - General Availability
    * Release announcement.
    * ROS desktop packages source freeze is lifted and ``rosdistro`` is reopened for Lyrical pull requests.

For progress on the development of Lyrical Luth, see `this project board <https://github.com/orgs/ros2/projects/70>`__.
For the broad process followed by Lyrical Luth, see the :doc:`process description page <Release-Process>`.

Supported Platforms
-------------------

ROS Lyrical supports the following platforms according to `the platform support tiers <../The-ROS2-Project/Platform-Support-Tiers>`:

+--------------+-------------------+-------------------+---------------+-------------------+-----------+-----------------+----------------+
| Architecture | Ubuntu Resolute   | Ubuntu Noble*     | Windows 11    | RHEL 10           | macOS     | Debian Trixie*  | OpenEmbedded / |
|              | (26.04)           | (24.04)           | (VS2022)      |                   |           | (13)            | Yocto Project  |
+==============+===================+===================+===============+===================+===========+=================+================+
| amd64        | Tier 1 [d][a]     | Tier 3            | Tier 1 [a]    | Tier 2 [d][a]     | Tier 3    | Tier 3          | Tier 3         |
+--------------+-------------------+-------------------+---------------+-------------------+-----------+-----------------+----------------+
| arm64        | Tier 1 [d][a]     | Tier 3            |               |                   |           | Tier 3          | Tier 3         |
+--------------+-------------------+-------------------+---------------+-------------------+-----------+-----------------+----------------+
| arm32        | Tier 3            | Tier 3            |               |                   |           | Tier 3          | Tier 3         |
+--------------+-------------------+-------------------+---------------+-------------------+-----------+-----------------+----------------+

* ``*`` Early EOL per `the platform EOL policy <../The-ROS2-Project/Platform-EOL-Policy>`
    * Ubuntu Noble is supported until ``2029-06-01``
    * Debian Trixie is supported until ``2028-08-09``
* ``[d]`` You may install ROS Lyrical on this platform using Distribution-specific packaegs (Debian, RPM, etc.).
* ``[a]`` You may install ROS Lyrical by downloading an archive containing pre-built packages for all packages in the `ROS Lyrical ros2.repos file <https://github.com/ros2/ros2/blob/lyrical/ros2.repos>`__

To use ROS Lyrical on any Tier 3 platform, you must build ROS Lyrical from source.

Minimum Language Requirements
-----------------------------

* `C++20 <https://discourse.openrobotics.org/t/ros-2-lyrical-c-version/52551>`__
* C17
* Python 3.12 - 3.14

Dependency Requirements
-----------------------

TODO - this section will show a table of important system package versions across supported platforms.

Middleware Implementation support
---------------------------------

The default middleware in ROS Lyrical is **rmw_fastrtps_cpp**.

+---------------------------+-------------------------+---------------+----------------------------+-------------------------------+
| Middleware Library        | Middleware Provider     | Support Level | Platforms                  | Architectures                 |
+===========================+=========================+===============+============================+===============================+
| rmw_fastrtps_cpp          | eProsima Fast-DDS       | Tier 1        | All Platforms              | All Architectures             |
+---------------------------+-------------------------+---------------+----------------------------+-------------------------------+
| rmw_connextdds            | RTI Connext             | Tier 1        | Ubuntu, Windows, and macOS | All Architectures except arm64|
+---------------------------+-------------------------+---------------+----------------------------+-------------------------------+
| rmw_cyclonedds_cpp        | Eclipse Cyclone DDS     | Tier 1        | All Platforms              | All Architectures             |
+---------------------------+-------------------------+---------------+----------------------------+-------------------------------+
| rmw_zenoh_cpp             | Eclipse Zenoh           | Tier 1        | All Platforms              | All Architectures             |
+---------------------------+-------------------------+---------------+----------------------------+-------------------------------+
| rmw_fastrtps_dynamic_cpp  | eProsima Fast-DDS       | Tier 2        | All Platforms              | All Architectures             |
+---------------------------+-------------------------+---------------+----------------------------+-------------------------------+
| rmw_gurumdds_cpp          | GurumNetworks GurumDDS  | Tier 3        | Ubuntu and Windows         | All Architectures except arm32|
+---------------------------+-------------------------+---------------+----------------------------+-------------------------------+

Middleware implementation support is dependent upon the platform support tier.
For example, a Tier 1 middleware implementation on a Tier 2 platform will only receive Tier 2 support.

Installation
------------

TODO

New features in this ROS 2 release
----------------------------------

``ament_cmake``
^^^^^^^^^^^^^^^

Allow multiple ``ament_python_install_package()`` calls per package
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
This enables shipping a single package with both python code and generated interfaces (from ``rosidl_generate_interfaces()``).
Source directories from each call are merged at install time, with the last call winning on file conflicts.

See https://github.com/ament/ament_cmake/pull/587 for more details.

``class_loader``
^^^^^^^^^^^^^^^^

Add support for passing arguments to constructors.
As a result, users can now create plugins that are not default constructible, removing the need for initialize method.

See https://github.com/ros/class_loader/pull/223 for more details.

``plugin_lib``
^^^^^^^^^^^^^^

Add support for passing arguments to constructors.

See https://github.com/ros/pluginlib/pull/291 for more details.

``image_transport``
^^^^^^^^^^^^^^^^^^^

``image_transport`` now supports lifecycle nodes.

See https://github.com/ros-perception/image_common/pull/352 for more details.

``point_cloud_transport``
^^^^^^^^^^^^^^^^^^^^^^^^^

``point_cloud_transport`` now supports lifecycle nodes.

See https://github.com/ros-perception/point_cloud_transport/pull/109 for more details.

``rcl_logging_implementation``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A new ``rcl_logging_implementation`` package has been introduced to allow users to select the logging backend implementation at runtime without rebuilding ``rcl``.
Users can set the ``RCL_LOGGING_IMPLEMENTATION`` environment variable to switch between available logging backends (e.g., ``rcl_logging_spdlog``, ``rcl_logging_noop``, or custom implementations).
If not specified, ``rcl_logging_spdlog`` is used by default.

See https://github.com/ros2/rcl/issues/1178, https://github.com/ros2/rcl/pull/1276, and https://github.com/ros2/rcl_logging/pull/135 for more details.

``rclcpp``
^^^^^^^^^^

* Added new `Callback Group Events Executor <https://github.com/ros2/rclcpp/pull/3097>`__.
  Like its predecessor the experimental ``EventsExecutor``, the ``EventsCBGExecutor`` uses
  an events queue to process ready entities.
  Builds on the experimental ``EventsExecutor`` by adding support for multiple sources of
  ROS time and multiple threads.
  Compared to the Single and Multithreaded Executors, the ``EventsCBGExecutor`` exhibits
  around 10 to 15% less CPU usage.
  Note: The experimental ``EventsExecutor`` is now deprecated. For similar performance, use
  the ``EventsCBGExecutor`` with one thread.

* `Unified component container interface <https://github.com/ros2/rclcpp/pull/3134>`__ -
  ``component_container`` is now the single entrypoint for launching both regular and
  isolated component containers with all types of executors.

* Added ``disable_callbacks()`` and ``enable_callbacks()`` APIs to ``SubscriptionBase``.
  By design the subscription itself is a shared pointer and its callbacks are propagated to
  the executor, making it difficult to manage subscription lifecycle from the application
  side.
  When a subscription is deleted at the application layer, it may still be referenced by
  the node and executor, causing callbacks to be invoked unexpectedly.
  This can lead to undefined behavior when the callback accesses resources that have been
  cleaned up.
  The new APIs allow users to disable subscription callbacks at runtime without destroying
  the subscription, preventing unexpected callback invocations.

  See `ros2/rclcpp#2984 <https://github.com/ros2/rclcpp/issues/2984>`__ and
  `ros2/rclcpp#2985 <https://github.com/ros2/rclcpp/pull/2985>`__ for more details.

``rclpy``
^^^^^^^^^

Native asyncio support with ``AsyncNode`` (experimental)
""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Added ``AsyncNode``, a new node type that runs on the ``asyncio`` event loop.
Subscription, service, and timer callbacks can now ``await`` any asyncio operation.
The new async ``client.call(request)`` and sim-time aware ``clock.sleep(...)`` are awaitable from any asyncio task.
CPU usage is significantly reduced compared to the ``SingleThreadedExecutor``.

See the :doc:`Writing an async node with asyncio <../Tutorials/Intermediate/Writing-An-Async-Node-With-Asyncio-Python>` tutorial and https://github.com/ros2/rclpy/pull/1620 for more details.

``rosbag2``
^^^^^^^^^^^

``Rosbag2`` received several new recording, playback, observability, and API improvements.

* Added support for repeating transient-local messages when recording bags.
  Users can configure selected transient-local topics with ``--repeat-transient-local`` and
  an optional per-topic depth, or use ``--repeat-all-transient-local`` to automatically apply
  the behavior to topics whose publishers offer ``TRANSIENT_LOCAL`` durability QoS.
  This helps preserve latched/static state, such as maps or static transforms, across bag
  splits and snapshots.

  See `ros2/rosbag2#2385 <https://github.com/ros2/rosbag2/pull/2385>`__,
  `ros2/rosbag2#2386 <https://github.com/ros2/rosbag2/pull/2386>`__,
  `ros2/rosbag2#2387 <https://github.com/ros2/rosbag2/pull/2387>`__, and
  `ros2/rosbag2#2391 <https://github.com/ros2/rosbag2/pull/2391>`__ for more details.

* Added recorder service APIs for starting and stopping recording, starting and stopping topic
  discovery, and querying whether discovery is running.
  This makes it possible to control a recorder process remotely through ROS services instead
  of only through the command-line process lifecycle.

  See `ros2/rosbag2#2248 <https://github.com/ros2/rosbag2/pull/2248>`__ for more details.

* Added scheduled service operations for coordinated recording and playback workflows.
  Record, play, resume, and split operations can now be scheduled by time, with support for
  node time, publish time, and receive time modes where applicable.
  Service responses were also extended with explicit return codes and error strings.

  See `ros2/rosbag2#2330 <https://github.com/ros2/rosbag2/pull/2330>`__ and
  `ros2/rosbag2#2357 <https://github.com/ros2/rosbag2/pull/2357>`__ for more details.

* Added circular logging by split count.
  When recording with bag splitting enabled, ``--max-bag-files`` limits the maximum number of
  bag files stored on disk by automatically deleting the oldest split files as new ones are
  created.

  See `ros2/rosbag2#2218 <https://github.com/ros2/rosbag2/pull/2218>`__ for more details.

* Added a time-bounded snapshot cache option, ``--max-cache-duration``.
  This allows snapshot-mode recording to bound the cached data by duration in addition to
  existing cache controls.

  See `ros2/rosbag2#2289 <https://github.com/ros2/rosbag2/pull/2289>`__ for more details.

* Added message-loss observability during recording.
  ``Rosbag2`` can now collect message-loss statistics from the transport layer and recorder
  internals, and publish incremental per-topic loss events on the predefined
  ``events/rosbag2_messages_lost`` topic.
  The publishing rate can be configured with ``--stats_max_publishing_rate``.

  See `ros2/rosbag2#2039 <https://github.com/ros2/rosbag2/pull/2039>`__,
  `ros2/rosbag2#2144 <https://github.com/ros2/rosbag2/pull/2144>`__, and
  `ros2/rosbag2#2150 <https://github.com/ros2/rosbag2/pull/2150>`__ for more details.

* Expanded the ``rosbag2_py`` player and recorder APIs.
  Python users can now programmatically control playback and recording with APIs such as
  pause, resume, stop, seek, play-next, spin control, and wait helpers, instead of relying
  only on blocking command-line-style helpers.

  See `ros2/rosbag2#2047 <https://github.com/ros2/rosbag2/pull/2047>`__ and
  `ros2/rosbag2#2062 <https://github.com/ros2/rosbag2/pull/2062>`__ for more details.

* Added APIs for querying player timing metadata.
  Users can now query the player's starting time and playback duration, and Python readers
  can access the send timestamp when reading serialized messages.

  See `ros2/rosbag2#2061 <https://github.com/ros2/rosbag2/pull/2061>`__ and
  `ros2/rosbag2#2095 <https://github.com/ros2/rosbag2/pull/2095>`__ for more details.

* Improved message definition resolution for recorded data.
  ``Rosbag2`` can now find message definitions in nested subdirectories, better resolve inner
  message definitions for service and action introspection data, and support relative includes
  in local IDL message definitions.

  See `ros2/rosbag2#2041 <https://github.com/ros2/rosbag2/pull/2041>`__,
  `ros2/rosbag2#2052 <https://github.com/ros2/rosbag2/pull/2052>`__,
  `ros2/rosbag2#2055 <https://github.com/ros2/rosbag2/pull/2055>`__, and
  `ros2/rosbag2#2241 <https://github.com/ros2/rosbag2/pull/2241>`__ for more details.

* Updated split bag file naming to include the split index, bag name, and timestamp, making
  split files easier to identify and sort.

  See `ros2/rosbag2#2265 <https://github.com/ros2/rosbag2/pull/2265>`__ for more details.

* Enabled the ``rosbag2_performance_benchmarking`` package to be built by default, making the
  benchmarking tools easier to use for recorder and player performance evaluation.

  See `ros2/rosbag2#2093 <https://github.com/ros2/rosbag2/pull/2093>`__ for more details.

``rosidl_python``
^^^^^^^^^^^^^^^^^

Passing in Python ``set`` objects into array or sequence fields is now deprecated.
Instead pass in something that implements ``collections.abc.Sequence`` most commonly a ``list``, ``tuple``, or a ``numpy.ndarray``. To be removed in ROS M.
