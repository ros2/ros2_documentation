Lyrical Luth Supported Platforms
================================

ROS Lyrical supports the following platforms according to `the platform support tiers <../../The-ROS2-Project/Platform-Support-Tiers>`:

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

* ``*`` Early EOL per `the platform EOL policy <../../The-ROS2-Project/Platform-EOL-Policy>`
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
