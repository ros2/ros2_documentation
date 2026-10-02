Lyrical Luth Supported Platforms
================================

ROS Lyrical supports the following platforms according to :doc:`the platform support tiers <../../../The-ROS2-Project/Platform-Support-Tiers>`:

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

* ``*`` Early EOL per :doc:`the platform EOL policy <../../../The-ROS2-Project/Platform-EOL-Policy>`
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

+---------------+-----------------+---------------+---------------+----------+----------+------------------+--------------------+
|               | Required Support                |                             Recommended Support                             |
+===============+=================+===============+===============+==========+==========+==================+====================+
| Package       | Ubuntu Resolute | Windows 11    | Ubuntu Noble  | RHEL 10  | macOS*** | Debian Trixie    | OpenEmbedded       |
+---------------+-----------------+---------------+---------------+----------+----------+------------------+--------------------+
| CMake         | 4.2.3           | 3.28.3        | 3.28.3        | 3.30.5   | 4.3.2    | 3.31.6           | 4.3.2              |
+---------------+-----------------+---------------+---------------+----------+----------+------------------+--------------------+
| EmPY          | 4.2.1           | 3.3.4         | 3.3.4         | 4.2.1    |          | 3.3.4            | 3.3.2              |
+---------------+-----------------+---------------+---------------+----------+----------+------------------+--------------------+
| Gazebo        | Jetty*          | N/A           | N/A           | N/A      | Jetty*   | Jetty*           | N/A                |
+---------------+-----------------+---------------+---------------+----------+----------+------------------+--------------------+
| NumPy         | 2.3.4           | 1.26.4        | 1.26.4        | 1.26.4   | 2.4.5    | 2.2.4            | N/A                |
+---------------+-----------------+---------------+---------------+----------+----------+------------------+--------------------+
| Ogre          | 1.12.10         | 1.12.10       | 1.12.10       | N/A      |          | 1.12.10          | N/A                |
+---------------+-----------------+---------------+---------------+----------+----------+------------------+--------------------+
| OpenCV        | 4.10.0          | 4.10.0        | 4.6.0         | 4.10.0   | 4.13.10  | 4.10.0           | 4.13.10            |
+---------------+-----------------+---------------+---------------+----------+----------+------------------+--------------------+
| OpenSSL       | 3.5.5           | ``>=3.4``     | 3.0.13        | 3.5.1    | 3.6.2    | 3.5.6            | 3.5.6              |
+---------------+-----------------+---------------+---------------+----------+----------+------------------+--------------------+
| Python        | 3.14.3          | 3.12.3        | 3.12.3        | 3.12.12  | 3.14.5   | 3.13.5           | 3.14.4             |
+---------------+-----------------+---------------+---------------+----------+----------+------------------+--------------------+
| Qt            | 6.10.2          | ``>=6``       | 5.15.13       | 6.10.1   | 6.11.1   | 6.8.2            | N/A                |
+---------------+-----------------+---------------+---------------+----------+----------+------------------+--------------------+
| PCL           | 1.15.1          | N/A           | 1.14.0        | 1.15.0*  | 1.15.1   | 1.15.0           | 6.12.0             |
+---------------+-----------------+---------------+---------------+----------+----------+------------------+--------------------+

\" \* \" means that this is not the upstream version (available on the
official Operating System repositories) but a package distributed by
OSRF or the community (package built and distributed on custom
repositories).

\" \*\* \" means that the dependency may see multiple version changes,
because the dependency uses a package manager that continually updates
the dependency without a stable API.

This document only captures the version at the first release of a ROS
distribution and will not be updated as the dependencies move forward.
These versions are thus a low watermark.

Middleware Implementation support
---------------------------------

The default middleware in ROS Lyrical is **rmw_fastrtps_cpp**.

+---------------+-----------------+---------------+---------------+----------+----------+------------------+--------------------+
| Middleware    | Ubuntu Resolute | Windows 11    | Ubuntu Noble  | RHEL 10  | macOS    | Debian Trixie    | OpenEmbedded       |
+===============+=================+===============+===============+==========+==========+==================+====================+
| Connext DDS   | 7.7.0                           | N/A                      | 7.7.0    | N/A                                   |
+---------------+---------------------------------+--------------------------+----------+---------------------------------------+
| Cyclone DDS   | 11.0.x                                                                                                        |
+---------------+---------------------------------------------------------------------------------------------------------------+
| Fast-DDS      | 3.6.x                                                                                                         |
+---------------+---------------------------------------------------------------------------------------------------------------+
| Zenoh         | 1.8.0                                                                                                         |
+---------------+---------------------------------------------------------------------------------------------------------------+


+---------------------------+-------------------------+---------------+-------------------------------+
| Middleware Library        | Middleware Provider     | Support Level | Architectures                 |
+===========================+=========================+===============+===============================+
| rmw_fastrtps_cpp          | eProsima Fast-DDS       | Tier 1        | All Architectures             |
+---------------------------+-------------------------+---------------+-------------------------------+
| rmw_connextdds            | RTI Connext             | Tier 1        | All Architectures except arm64|
+---------------------------+-------------------------+---------------+-------------------------------+
| rmw_cyclonedds_cpp        | Eclipse Cyclone DDS     | Tier 1        | All Architectures             |
+---------------------------+-------------------------+---------------+-------------------------------+
| rmw_zenoh_cpp             | Eclipse Zenoh           | Tier 1        | All Architectures             |
+---------------------------+-------------------------+---------------+-------------------------------+
| rmw_fastrtps_dynamic_cpp  | eProsima Fast-DDS       | Tier 2        | All Architectures             |
+---------------------------+-------------------------+---------------+-------------------------------+

Middleware implementation support is dependent upon the platform support tier.
For example, a Tier 1 middleware implementation on a Tier 2 platform will only receive Tier 2 support.
