.. _jazzy-release:

Jazzy Jalisco (``jazzy``)
=========================

.. toctree::
   :hidden:

   Jazzy-Jalisco-Complete-Changelog

.. contents:: Table of Contents
   :depth: 2
   :local:

*Jazzy Jalisco* is the tenth release of ROS 2.
What follows is highlights of the important changes and features in Jazzy Jalisco since the last release.
For a list of all of the changes since Iron, see the :doc:`long form changelog <Jazzy-Jalisco-Complete-Changelog>`

Supported Platforms
-------------------

Jazzy Jalisco is primarily supported on the following platforms:

Tier 1 platforms:

* Ubuntu 24.04 (Noble): ``amd64`` and ``arm64``
* Windows 10 (Visual Studio 2019): ``amd64``

Tier 2 platforms:

* RHEL 9: ``amd64``

Tier 3 platforms:

* macOS: ``amd64``
* Debian Bookworm: ``amd64``

For more information about RMW implementations, compiler / interpreter versions, and system dependency versions see `REP 2000 <https://www.ros.org/reps/rep-2000.html>`__.

Installation
------------

`Install Jazzy Jalisco <../../jazzy/Installation.html>`__

Changes to how ROS 2 and Gazebo integrate
-----------------------------------------

Starting with Jazzy Jalisco, we are streamlining how ROS 2 and `Gazebo <https://gazebosim.org>`__ integrate.
For every ROS 2 release, there will be a recommended, supported Gazebo release that goes along with that release.
For Jazzy Jalisco, the recommended Gazebo release will be Harmonic.

To make it easier for ROS 2 packages to consume Gazebo packages, there are now ``gz_*_vendor`` packages.
Those packages are:

* gz_common_vendor: https://github.com/gazebo-release/gz_common_vendor
* gz_cmake_vendor: https://github.com/gazebo-release/gz_cmake_vendor
* gz_math_vendor: https://github.com/gazebo-release/gz_math_vendor
* gz_transport_vendor: https://github.com/gazebo-release/gz_transport_vendor
* gz_sensor_vendor: https://github.com/gazebo-release/gz_sensor_vendor
* gz_sim_vendor: https://github.com/gazebo-release/gz_sim_vendor
* gz_tools_vendor: https://github.com/gazebo-release/gz_tools_vendor
* gz_utils_vendor: https://github.com/gazebo-release/gz_utils_vendor
* sdformat_vendor: https://github.com/gazebo-release/sdformat_vendor

ROS 2 packages can use the functionality in these packages by adding dependencies in ``package.xml``, e.g.:

.. code::

   <depend>gz_math_vendor</depend>

And then using them in ``CMakeLists.txt``, e.g.:

.. code::

   find_package(gz_math_vendor REQUIRED)
   find_package(gz-math)

   add_executable(my_executable src/exe.cpp)
   target_link_libraries(my_executable gz-math::core)

.. note::

   It will still be possible to use alternate Gazebo versions with Jazzy Jalisco.  But those will not be as well tested or integrated with ROS 2.  See https://gazebosim.org/docs/harmonic/ros_installation for more information.

New features in this ROS 2 release
----------------------------------

``common_interfaces``
^^^^^^^^^^^^^^^^^^^^^

New VelocityStamped message
"""""""""""""""""""""""""""

Added a new message with all fields needed to define a velocity and transform it.

See https://github.com/ros2/common_interfaces/pull/240 for more details.

Adds ARROW_STRIP to Marker.msg
""""""""""""""""""""""""""""""

Added new type of Marker, ``ARROW_STRIP``, to Marker.msg.

See https://github.com/ros2/common_interfaces/pull/242 for more details.

``image_transport``
^^^^^^^^^^^^^^^^^^^

Support lazy subscribers
""""""""""""""""""""""""

See https://github.com/ros-perception/image_common/issues/272 for more details.

Expose option to set callback groups
""""""""""""""""""""""""""""""""""""

See https://github.com/ros-perception/image_common/issues/274 for more details.

Enable allow list
"""""""""""""""""

Added parameter so users can selectively disable ``image_transport`` plugins at runtime.

See https://github.com/ros-perception/image_common/issues/264 for more details.

Advertise and subscribe with custom QoS
"""""""""""""""""""""""""""""""""""""""

Allow users to pass in a custom quality-of-service when creating ``image_transport`` publishers and subscribers.

See https://github.com/ros-perception/image_common/issues/288 for more detatils.

Added rclcpp component to Republish
"""""""""""""""""""""""""""""""""""

Users can now start the ``image_transport`` republisher node as an rclcpp_component.

See https://github.com/ros-perception/image_common/issues/275 for more details.


``message_filters``
^^^^^^^^^^^^^^^^^^^

TypeAdapters support
""""""""""""""""""""

Allows users to use Type Adaptation within message_filters.

See https://github.com/ros2/message_filters/pull/96 for more information.

``rcl``
^^^^^^^

Add get type description service
""""""""""""""""""""""""""""""""

Implements the ``~/get_type_description`` service which allows external users to get descriptions of each type that a node offers.
This is offered by each node according to `REP 2016 <https://github.com/ros-infrastructure/rep/pull/381>`__.

See https://github.com/ros2/rcl/pull/1052 for more details.

``rclcpp``
^^^^^^^^^^

Type support helper for services
""""""""""""""""""""""""""""""""

New type support helper for services ``rclcpp::get_service_typesupport_handle`` is added to extract service type support handle.

See https://github.com/ros2/rclcpp/pull/2209 for more details.

``rclpy``
^^^^^^^^^^

ParameterEventHandler
"""""""""""""""""""""

New class ``ParameterEventHandler`` allows us to monitor and respond changes to parameters via parameter events.

See https://github.com/ros2/rclpy/pull/1135 for more details.

``ros2cli``
^^^^^^^^^^^

Added a ``--log-file-name`` command line argument
"""""""""""""""""""""""""""""""""""""""""""""""""

It is now possible to use ``--log-file-name`` command line argument to specify the log file name prefix.

.. code-block:: console

   $ ros2 run demo_nodes_cpp talker --ros-args --log-file-name filename

See https://github.com/ros2/ros2cli/issues/856 for more information.

Added QoS to subscription options
"""""""""""""""""""""""""""""""""

A user-settable QoS parameter was added to the ``TopicStatisticsOptions``, which allows the statistics to have a different QoS from the subscription itself.

See https://github.com/ros2/rclcpp/pull/2323 for more details.

Add clients and services count
""""""""""""""""""""""""""""""

It is now possible to get the number of clients created by a service.

``ros2action``
^^^^^^^^^^^^^^

``type`` sub-command supported
""""""""""""""""""""""""""""""

It is now possible to use the ``type`` sub-command to check the action type.

.. code-block:: console

   $ ros2 action type /fibonacci
   action_tutorials_interfaces/action/Fibonacci

See https://github.com/ros2/ros2cli/pull/894 for more information.

``rosbag2``
^^^^^^^^^^^

Service recording and playback
""""""""""""""""""""""""""""""

It is now possible to record and play service data with the ``ros2bag`` command line interface.

This features builds on `Service Introspection <https://github.com/ros2/ros2/issues/1285>`__, which has been available since Iron Irwini.
`Service recording and display <https://github.com/ros2/rosbag2/pull/1480>`__ adds the ability to record service data into a bag file.
And `Service playback <https://github.com/ros2/rosbag2/pull/1481>`__ can play that service data from the bag file.

Record all services data:

.. code-block:: console

   $ ros2 bag record --all-services

Record all services and all topic data:

.. code-block:: console

   $ ros2 bag record --all

Play service data from bag file:

.. code-block:: console

   $ ros2 bag play --publish-service-requests bag_path

See the `design document <https://github.com/ros2/rosbag2/blob/rolling/docs/design/rosbag2_record_replay_service.md>`__ for more information.

New filter modes
""""""""""""""""

It is now possible to filter by topic type.

.. code-block:: console

    $ ros2 bag record --topic_types sensor_msgs/msg/Image sensor_msgs/msg/CameraInfo

.. code-block:: console

    $ ros2 bag record --topic_types sensor_msgs/msg/Image

See more details https://github.com/ros2/rosbag2/pull/1577 and https://github.com/ros2/rosbag2/pull/1582.

Player and Recorder are now exposed as rclcpp components
""""""""""""""""""""""""""""""""""""""""""""""""""""""""

This allows a "zero-copy" when using intra-process communication during data record or reply.
This can significantly reduce CPU load during recording or reply when dealing with high-bandwidth data streams and will help to avoid data loss in the transport layer.
It also provides the ability to use YAML configuration files for ``rosbag2_transport::Player`` and ``rosbag2_transport::Recorder`` composable nodes.

See https://github.com/ros2/rosbag2/tree/jazzy?tab=readme-ov-file#using-with-composition for more details.

Added option to disable recorder keyboard controls
""""""""""""""""""""""""""""""""""""""""""""""""""

See https://github.com/ros2/rosbag2/pull/1607 for more details.

Use middleware send and receive timestamps from ``message_info`` during recording
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Where available, ``rosbag2`` now uses the send and receive timestamps as provided by the middleware.
These timestamps are more indicative of when the data was actually sent and received, respectively.
Note that saving the timestamp into a bag is currently only supported for MCAP files (the default).

See https://github.com/ros2/rosbag2/pull/1531 for more details.

Added compression threads priority to record options
""""""""""""""""""""""""""""""""""""""""""""""""""""

It is now possible to specify the priority of the thread that performs compression.

See https://github.com/ros2/rosbag2/pull/1457 for more details.

Added ability to split already existing ros2 bags by time
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Added ``start_time_ns`` and ``end_time_ns`` to the ``StorageOptions`` to exclude messages not in
``[start_time;end_time]`` during the ``ros2 bag convert`` operation.

See https://github.com/ros2/rosbag2/pull/1455 for more details.

Store serialized metadata in bag files directly
"""""""""""""""""""""""""""""""""""""""""""""""

``rosbag2`` has always stored metadata in the ``metadata.yaml`` file associated with a bag file.
Now the metadata is also stored in each bag file, once when opening the file and a second time when closing the written bag file.
This allows bag files to be self-contained, and used without the ``metadata.yaml`` file in the rosbag2 player or third-party applications.
``ros2 bag reindex`` can still be used to restore the ``metadata.yaml`` file, if desired.

Store ROS_DISTRO name in the metadata
"""""""""""""""""""""""""""""""""""""

See https://github.com/ros2/rosbag2/pull/1241 for more details.

Added introspection QoS methods to Python bindings
""""""""""""""""""""""""""""""""""""""""""""""""""

It is now possible to instrospect QoS setting from Python bindings.

See https://github.com/ros2/rosbag2/pull/1648 for more details.

``rosidl``
^^^^^^^^^^

Added interfaces to support key annotation
""""""""""""""""""""""""""""""""""""""""""

The ``key`` annotation allows indicating that a data member is part of the key, which can have zero or more key fields and can be applied to structure fields of various types.

See https://github.com/ros2/rosidl/pull/796 and https://github.com/ros2/rosidl_typesupport_fastrtps/pull/116 for more details.

``rviz2``
^^^^^^^^^

Added regex filter field for TF display
"""""""""""""""""""""""""""""""""""""""

When there are many frames on ``/tf`` it can be hard to properly visualize them in RViz, especially if frames overlap.
The usual solution to this is to enable and disable desired frames in Frames field of the TF display.
Now it is possible to filter frames using regular expressions.

See https://github.com/ros2/rviz/pull/1032 for more details.

Append measured subscription frequency to topic status
""""""""""""""""""""""""""""""""""""""""""""""""""""""

It is possible to visualize Hz in the topic status widget.

See https://github.com/ros2/rviz/issues/1113 for more details.

Reset functionality
"""""""""""""""""""

It is possible to reset Time using a new service or using the keyboard shortcut ``R``.

See https://github.com/ros2/rviz/issues/1109 and https://github.com/ros2/rviz/issues/1088 for more details.

Added support for point_cloud_transport
"""""""""""""""""""""""""""""""""""""""

It is possible to subscribe to point clouds using the ``point_cloud_transport`` package.

See https://github.com/ros2/rviz/pull/1008 for more details.

Feature parity with RViz for ROS
""""""""""""""""""""""""""""""""

It is possible to use the same plugins available in the ROS 1 version.

* DepthCloud
* AccelStamped
* TwistStamped
* WrenchStamped
* Effort

Camera info display
"""""""""""""""""""

It is possible to visualize CameraInfo messages in the 3D scene.

See https://github.com/ros2/rviz/pull/1166 for more details.

``rcpputils``
^^^^^^^^^^^^^

Added tl_expected
"""""""""""""""""

`std::expected <https://en.cppreference.com/w/cpp/utility/expected>`__ is C++23 feature, which is not yet supported in ROS 2.
However, it is possible to use ``tl::expected`` from rcpputils via a backported implementation.

See https://github.com/ros2/rcpputils/pull/185 for more details.

``rcutils``
^^^^^^^^^^^

Add human readable date to logging formats
""""""""""""""""""""""""""""""""""""""""""

It is now possible to output dates in a human readable format when using console logging by using the ``{date_time_with_ms}`` token in the ``RCUTILS_CONSOLE_OUTPUT_FORMAT`` environment variable.

See https://github.com/ros2/rcutils/pull/441 for more details.

Changes since the Iron release
------------------------------

``common_interfaces``
^^^^^^^^^^^^^^^^^^^^^

Added IDs to geometry_msgs/Polygon and PolygonStamped
"""""""""""""""""""""""""""""""""""""""""""""""""""""
Polygons are often used to represent specific objects but are difficult to rectify currently without any kind of specific identification.
This feature adds an ID field to disambiguate polygons.

See https://github.com/ros2/common_interfaces/pull/232 for more details.


``geometry2``
^^^^^^^^^^^^^

Removed deprecated headers
""""""""""""""""""""""""""

In Humble, the headers: ``tf2_bullet/tf2_bullet.h``, ``tf2_eigen/tf2_eigen.h``, ``tf2_geometry_msgs/tf2_geometry_msgs.h``,
``tf2_kdl/tf2_kdl.h``, ``tf2_sensor_msgs/tf2_sensor_msgs.h``  were deprecated in favor of: ``tf2_bullet/tf2_bullet.hpp``,
``tf2_eigen/tf2_eigen.hpp``, ``tf2_geometry_msgs/tf2_geometry_msgs.hpp``, ``tf2_kdl/tf2_kdl.hpp``, ``tf2_sensor_msgs/tf2_sensor_msgs.hpp``
In Jazzy, the ``tf2_bullet/tf2_bullet.h``, ``tf2_eigen/tf2_eigen.h``, ``tf2_geometry_msgs/tf2_geometry_msgs.h``,
``tf2_kdl/tf2_kdl.h``, ``tf2_sensor_msgs/tf2_sensor_msgs.h`` headers have been completely removed.

Changed return types of ``wait_for_transform_async`` and ``wait_for_transform_full_async``
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Previously ``wait_for_transform_async`` and ``wait_for_transform_full_async`` of the ``Buffer`` class returned a future containing true or false
In Jazzy, the future will contain the information of the transform being waited on.

Enabled Twist interpolator
""""""""""""""""""""""""""

Included new API to lookup the velocity of the moving frame in the reference frame.

See https://github.com/ros2/geometry2/pull/646 for more information.

``rcl``
^^^^^^^

Actual and expected call time when timer is called
""""""""""""""""""""""""""""""""""""""""""""""""""

New timer API ``rcl_timer_call_with_info`` is added to collect actual and expected call time when the timer is called.
This allows users to get the timer information when the timer is expected to be called and actual time that timer is called.

See https://github.com/ros2/rcl/pull/1113 for more details.

Improved rcl_wait in the area of timeout computation and spurious wakeups
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Added special handling for timers with a clock that has time override enabled.
For these timer we should not compute a timeout, as the waitset is woken up by the associated guard condition.

See https://github.com/ros2/rcl/issues/1146 for more details.

``rclcpp``
^^^^^^^^^^

Fixed data race conditions
""""""""""""""""""""""""""

Fixed data race conditions in executors.

See https://github.com/ros2/rclcpp/issues/2500 for more details.

Utilize ``rclcpp::WaitSet`` as part of the executors
""""""""""""""""""""""""""""""""""""""""""""""""""""

Improve the number of ``rcl_wait_set`` creations and deletions by making the default Single/Multithreaded executors work like the static single threaded executor
in terms of entity collection rebuilding.

See https://github.com/ros2/rclcpp/pull/2142 for more details.

Due to this change, callbacks in the executor are no longer ordered consistently, even within the same entity.

See https://github.com/ros2/rclcpp/issues/2532 for more details.

``rclcpp::get_typesupport_handle`` is deprecated
""""""""""""""""""""""""""""""""""""""""""""""""

The ``rclcpp::get_typesupport_handle`` that extracts message type support handle is deprecated, and will be removed in a future release.
Instead, ``rclcpp::get_message_typesupport_handle`` should be used.

See https://github.com/ros2/rclcpp/pull/2209 for more details.

Deprecated ``rclcpp/qos_event.hpp`` header was removed
""""""""""""""""""""""""""""""""""""""""""""""""""""""

In Iron, the header ``rclcpp/qos_event.hpp`` was deprecated in favor of ``rclcpp/event_handler.hpp``.
In Jazzy, the ``rclcpp/qos_event.hpp`` header has been completely removed.

Deprecated subscription callback signatures were removed
""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Back in Humble, subscription signatures of the form ``void callback(std::shared_ptr<MessageT>)`` and ``void callback(std::shared_ptr<MessageT>, const rclcpp::MessageInfo &)`` were deprecated.

In Jazzy, these subscription signatures have been removed.
Users should switch to using ``void callback(std::shared_ptr<const MessageT>)`` or ``void callback(std::shared_ptr<const MessageT>, const rclcpp MessageInfo &)``.

Actual and expected call time when timer is called
""""""""""""""""""""""""""""""""""""""""""""""""""

``rclcpp::TimerInfo`` argument is added to the timer callback to collect actual and expected call time when the timer is called.
This allows users to get the timer information when the timer is expected to be called and actual time that timer is called.

See https://github.com/ros2/rclcpp/pull/2343 for more details.

``rclcpp_action``
^^^^^^^^^^^^^^^^^

Callback after cancel
"""""""""""""""""""""

Added a function to stop callbacks of a goal handle after it has gone out of scope.
This function allows us to drop the handle in a locked context.

See https://github.com/ros2/rclcpp/pull/2281 for more details.

``rclcpp_lifecycle``
^^^^^^^^^^^^^^^^^^^^

Add new node interface TypeDescriptionsInterface
""""""""""""""""""""""""""""""""""""""""""""""""

Add new node interface ``TypeDescriptionsInterface`` to provide the ``GetTypeDescription`` service.

See https://github.com/ros2/rclcpp/pull/2224 for more details.

``rclpy``
^^^^^^^^^

``rclpy.node.Node.declare_parameter``
"""""""""""""""""""""""""""""""""""""

The ``rclpy.node.Node.declare_parameter`` does not allow statically typing parameter without a default value.

See https://github.com/ros2/rclpy/pull/1216 for more details.

Added types to method arguments
"""""""""""""""""""""""""""""""

Added type checking to improve the experience for anyone using static type checking.

See https://github.com/ros2/rclcpp/pull/2224, https://github.com/ros2/rclpy/issues/1240, https://github.com/ros2/rclpy/issues/1237, https://github.com/ros2/rclpy/issues/1231, https://github.com/ros2/rclpy/issues/1241, and https://github.com/ros2/rclpy/issues/1233.

``rosbag2``
^^^^^^^^^^^

Rename of the ``--exclude`` CLI option
""""""""""""""""""""""""""""""""""""""

The ``--exclude`` CLI option was renamed to the ``--exclude-regex`` to better reflect what it does.

See https://github.com/ros2/rosbag2/pull/1480 for more information.

Changes in representation of the ``offered_qos_profiles``
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Enum values are now used for ``offered_qos_profiles`` in the code, in human-readable string values for QoS settings in the metadata, and in the overriding QoS profile YAML files.

See https://github.com/ros2/rosbag2/tree/jazzy?tab=readme-ov-file#overriding-qos-profiles for an example.

Added node name to the read and write bag split event messages
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

See https://github.com/ros2/rosbag2/pull/1609 for more details.

Added ``BagSplitInfo`` service call on bag close
""""""""""""""""""""""""""""""""""""""""""""""""

See https://github.com/ros2/rosbag2/pull/1422 for more details.

Resolved multiple issues related to the handling SIGINT and SIGTERM signals in rosbag2
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

See https://github.com/ros2/rosbag2/pull/1557, https://github.com/ros2/rosbag2/pull/1301 and
https://github.com/ros2/rosbag2/pull/1464 for more details.

Added ``topic_id`` returned by storage to the ``TopicMetadata``
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

See https://github.com/ros2/rosbag2/pull/1538 for more details.

Added Python bindings for CompressionOptions and CompressionMode structures
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

See https://github.com/ros2/rosbag2/pull/1425 for more details.

Improve performance in ``SqliteStorage::get_bagfile_size()``
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

This minimizes the probability of losing messages during bag split operation when recording with the SQLite3 storage plugin.

See https://github.com/ros2/rosbag2/pull/1516 for more details.

``rqt_bag``
^^^^^^^^^^^

Improved performance and updated rosbag API
"""""""""""""""""""""""""""""""""""""""""""

There are some breaking changes in the rosbag2 API and Ubuntu Noble library versions that required some changes to ``rqt_bag``.

See https://github.com/ros-visualization/rqt_bag/pull/156 for more details.

Development progress
--------------------

For progress on the development of Jazzy Jalisco, see `this project board <https://github.com/orgs/ros2/projects/52>`__.

For the broad process followed by Jazzy Jalisco, see the :doc:`process description page <Release-Process>`.

Known Issues
------------

To come.

Release Timeline
----------------

    November, 2023 - Platform decisions
        REP 2000 is updated with the target platforms and major dependency versions.

    By January, 2024 - Rolling platform shift
        Build farm is updated with the new platform versions and dependency versions for Jazzy Jalisco.

    Mon. April 8, 2024 - Alpha + RMW freeze
        Preliminary testing and stabilization of ROS Base [1]_ packages, and API and feature freeze for RMW provider packages.

    Mon. April 15, 2024 - Freeze
        API and feature freeze for ROS Base [1]_ packages in Rolling Ridley.
        Only bug fix releases should be made after this point.
        New packages can be released independently.

    Mon. April 22, 2024 - Branch
        Branch from Rolling Ridley.
        ``rosdistro`` is reopened for Rolling PRs for ROS Base [1]_ packages.
        Jazzy development shifts from ``ros-rolling-*`` packages to ``ros-jazzy-*`` packages.

    Mon. April 29, 2024 - Beta
        Updated releases of ROS Desktop [2]_ packages available.
        Call for general testing.

    Wed, May 1, 2024 - Kick off of Tutorial Party
        Tutorials hosted at https://github.com/osrf/ros2_test_cases are open for community testing.

    Mon. May 13, 2024 - Release Candidate
        Release Candidate packages are built.
        Updated releases of ROS Desktop [2]_ packages available.

    Mon. May 20, 2024 - Distro Freeze
        Freeze all Jazzy branches on all `ROS 2 desktop packages <https://www.ros.org/reps/rep-2001.html#jazzy-jalisco-may-2024-may-2029>`__ and ``rosdistro``.
        No pull requests for any ``jazzy`` branch or targeting ``jazzy/distribution.yaml`` in ``rosdistro`` repo will be merged.

    Thu. May 23, 2024 - General Availability
        Release announcement.
        `ROS 2 desktop packages <https://www.ros.org/reps/rep-2001.html#jazzy-jalisco-may-2024-may-2029>`__ source freeze is lifted and ``rosdistro`` is reopened for Jazzy pull requests.

.. [1] The ``ros_base`` variant is described in `REP 2001 (ros-base) <https://www.ros.org/reps/rep-2001.html#ros-base>`_.
.. [2] The ``desktop`` variant is described in `REP 2001 (desktop-variants) <https://www.ros.org/reps/rep-2001.html#desktop-variants>`_.
