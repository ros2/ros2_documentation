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

``rosidl_python``
^^^^^^^^^^^^^^^^^

Passing in Python ``set`` objects into array or sequence fields is now deprecated.
Instead pass in something that implements ``collections.abc.Sequence`` most commonly a ``list``, ``tuple``, or a ``numpy.ndarray``. To be removed in ROS M.
