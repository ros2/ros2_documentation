.. _upcoming-release:

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

`Install Iron Irwini <../../jazzy/Installation.html>`__

New features in this ROS 2 release
----------------------------------

``common_interfaces``
^^^^^^^^^^^^^^^^^^^^^

New VelocityStamped message
"""""""""""""""""""""""""""
Added a new message with all fields needed to define a velocity and transform it

See https://github.com/ros2/common_interfaces/pull/240 for more details.

``rclcpp``
^^^^^^^^^^

Type support helper for services
""""""""""""""""""""""""""""""""

New type support helper for services ``rclcpp::get_service_typesupport_handle`` is added to extract service type support handle.

See https://github.com/ros2/rclcpp/pull/2209 for more details.

``ros2cli``
^^^^^^^^^^^

``--log-file-name`` command line argument
"""""""""""""""""""""""""""""""""""""""""

It is now possible to use ``--log-file-name`` command line argument to specify the log file name prefix.

.. code-block:: bash

   ros2 run demo_nodes_cpp talker --ros-args --log-file-name filename

See https://github.com/ros2/ros2cli/issues/856 for more information.

``ros2action``
^^^^^^^^^^^^^^

``type`` sub-command supported
""""""""""""""""""""""""""""""

It is now possible to use ``type`` sub-command to check the action type.

.. code-block:: bash

   ros2 action type /fibonacci
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

.. code-block:: bash

   ros2 bag record --all-services

Record all services data with all topic data:

.. code-block:: bash

   ros2 bag record --all

Play service data from bag file:

.. code-block:: bash

   ros2 bag play --publish-service-requests bag_path

See the `design document <https://github.com/ros2/rosbag2/blob/rolling/docs/design/rosbag2_record_replay_service.md>`__ for more information.

Changes since the Iron release
------------------------------

``geometry2``
^^^^^^^^^^^^^

Deprecated headers were removed
"""""""""""""""""""""""""""""""

In Humble, the headers: ``tf2_bullet/tf2_bullet.h``, ``tf2_eigen/tf2_eigen.h``, ``tf2_geometry_msgs/tf2_geometry_msgs.h``,
``tf2_kdl/tf2_kdl.h``, ``tf2_sensor_msgs/tf2_sensor_msgs.h``  were deprecated in favor of: ``tf2_bullet/tf2_bullet.hpp``,
``tf2_eigen/tf2_eigen.hpp``, ``tf2_geometry_msgs/tf2_geometry_msgs.hpp``, ``tf2_kdl/tf2_kdl.hpp``, ``tf2_sensor_msgs/tf2_sensor_msgs.hpp``
In Jazzy, the ``tf2_bullet/tf2_bullet.h``, ``tf2_eigen/tf2_eigen.h``, ``tf2_geometry_msgs/tf2_geometry_msgs.h``,
``tf2_kdl/tf2_kdl.h``, ``tf2_sensor_msgs/tf2_sensor_msgs.h`` headers have been completely removed.

Return types of ``wait_for_transform_async`` and ``wait_for_transform_full_async`` changed
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
Previously ``wait_for_transform_async`` and ``wait_for_transform_full_async`` of the ``Buffer`` class returned a future containing true or false
In Jazzy, the future will contain the information of the transform being waited on.

Enable Twist interpolator
"""""""""""""""""""""""""
Included new API to lookup the velocity of the moving frame in the reference frame.

See https://github.com/ros2/geometry2/pull/646 for more information.

``rclcpp``
^^^^^^^^^^

``rclcpp::get_typesupport_handle`` is deprecated
""""""""""""""""""""""""""""""""""""""""""""""""

The ``rclcpp::get_typesupport_handle`` that extracts message type support handle is deprecated, and will be removed in a future release.
Instead, ``rclcpp::get_message_typesupport_handle`` should be used.

See https://github.com/ros2/rclcpp/pull/2209 for more details.

Deprecated ``rclcpp/qos_event.hpp`` header was removed
""""""""""""""""""""""""""""""""""""""""""""""""""""""

In Iron, the header ``rclcpp/qos_event.hpp`` was deprecated in favor of ``rclcpp/event_handler.hpp``.
In Jazzy, the ``rclcpp/qos_event.hpp`` header been completely removed.

Deprecated subscription callback signatures were removed
""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Back in Humble, subscription signatures of the form ``void callback(std::shared_ptr<MessageT>)`` and ``void callback(std::shared_ptr<MessageT>, const rclcpp::MessageInfo &)`` were deprecated.

In Jazzy, these subscription signatures have been removed.
Users should switch to using ``void callback(std::shared_ptr<const MessageT>)`` or ``void callback(std::shared_ptr<const MessageT>, const rclcpp MessageInfo &)``.

``rclpy``
^^^^^^^^^^

``rclpy.node.Node.declare_parameter``
"""""""""""""""""""""""""""""""""""""

The ``rclpy.node.Node.declare_parameter`` does not allow statically typing parameter without default value.

See https://github.com/ros2/rclpy/pull/1216 for more details.


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
        Freeze rosdistro.
        No PRs for Jazzy on the ``rosdistro`` repo will be merged (reopens after the release announcement).

    Thu. May 23, 2024 - General Availability
        Release announcement.
        ``rosdistro`` is reopened for Jazzy PRs.

.. [1] The ``ros_base`` variant is described in `REP 2001 (ros-base) <https://www.ros.org/reps/rep-2001.html#ros-base>`_.
.. [2] The ``desktop`` variant is described in `REP 2001 (desktop-variants) <https://www.ros.org/reps/rep-2001.html#desktop-variants>`_.
