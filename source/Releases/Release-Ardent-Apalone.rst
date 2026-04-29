.. redirect-from::

  Release-Ardent-Apalone

Ardent Apalone (``ardent``)
===========================

.. contents:: Table of Contents
   :depth: 2
   :local:

Welcome to the first non-beta release of ROS 2 software named *Ardent Apalone*!

Supported Platforms
-------------------

This version of ROS 2 is supported on three platforms:


* Ubuntu 16.04 (Xenial)
* Mac macOS 10.12 (Sierra)
* Windows 10

Binary packages as well as instructions for how to compile from source are provided for all 3 platforms (see `install instructions <../../Installation>` as well as `documentation <https://docs.ros2.org/ardent/>`__).

+--------------+-----------------------+----------------------+--------------------+
|                                  Required support                                |
+--------------+-----------------------+----------------------+--------------------+
| Architecture | Ubuntu Xenial (16.04) | MacOS Sierra (10.12) | Windows 10 (VS2015)|
+==============+=======================+======================+====================+
| amd64        | X                     | X                    | X                  |
+--------------+-----------------------+----------------------+--------------------+
| arm64        | X                     |                      |                    |
+--------------+-----------------------+----------------------+--------------------+

Minimum language requirements:

- C11[^2]
- C++14
- Python 3.5

[^2]: C11 is required, but support for some non-compliant systems is
    also provided, e.g. MSVC.

Dependency Requirements:

+--------------+----------------+------------+--------------+
| Package      | Ubuntu Xenial  | MacOS**    | Windows 10** |
+==============+================+============+==============+
| CMake        | 3.5.1          | 3.11.0     | 3.10.2       |
+--------------+----------------+------------+--------------+
| EmPY         | 3.3.2          | 3.6.5      | 3.3.2        |
+--------------+----------------+------------+--------------+
| Ogre         | 1.10*          | 1.10*      | 1.10*        |
+--------------+----------------+------------+--------------+
| OpenCV       | 2.4.9          | 3.4.1      | 2.4.13.2*    |
+--------------+----------------+------------+--------------+
| Poco         | 1.7.7*         | 1.7.7*     | 1.7.7*       |
+--------------+----------------+------------+--------------+
| Python       | 3.5.1          | 3.6.5      | 3.6.4        |
+--------------+----------------+------------+--------------+
| Qt           | 5.5.1          | 5.10.0     | 5.10.0       |
+--------------+----------------+------------+--------------+
| **Linux only (used for turtlebot demo)**                  |
+--------------+----------------+------------+--------------+
| PCL          | 1.7.2          | N/A        | N/A          |
+--------------+----------------+------------+--------------+

\" \* \" means that this is not the upstream version (available on the
official Operating System repositories) but a package distributed by
OSRF or the community (package built and distributed on custom
repositories).

\" \*\* \" Rolling distributions will see multiple version changes of
these dependencies during their lifetime.

This document only captures the version at the first release of a ROS
distribution and will not be updated as the dependencies move forward.
These versions are thus a low watermark.

Package manager use for dependencies:

- Ubuntu Xenial: apt
- MacOS: Homebrew, pip
- Windows: Chocolatey, pip

Build System Support:

- ament_cmake
- cmake
- setuptools

Middleware Implementation Support:

- eProsima Fast-RTPS
- RTI Connext
- ADLINK OpenSplice

Features
--------

New features in this ROS 2 release
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


* Distributed discovery, publish / subscribe, request / response communication

  * Provided by a C API
  * Implemented using different vendors:

    * eProsima's Fast RTPS as well as ADLINK's OpenSplice (from binary and source)
    * RTI's Connext (only from source)

  * Numerous quality of service settings for handling non-ideal networks
  * DDS Security support (with Connext and Fast RTPS)

* C++ and Python 3 client libraries

  * Sharing common code in C to unify the implementation
  * Execution model separated from the nodes, composable nodes
  * Node-specific parameters (only in C++ atm)
  * Life cycle (only in C++ atm)
  * Optionally intra-process communication using the same API (only in C++)

* Message definitions (with bounded arrays and strings as well as default values)
* Command line tools (e.g. ``ros2 run``)
* ``rviz`` with a few display types (the Windows version will likely follow in a few weeks)
* File system-based resource index (querying information without recursive crawling)
* Realtime safe code paths for pub / sub (with compatible DDS implementations only)
* Bridge between ROS 1 and ROS 2
* HSR demo `see Beta 3 <Beta3-Overview>`
* Turtlebot demo `see Beta 2 <Beta2-Overview>`

For a more detailed description please see the `Features <../../The-ROS2-Project/Features>` page.

Changes since Beta 3 release
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Improvements since the Beta 3 release:


* ``rviz``
* Different initialization options for message data structures in C++ (see `design doc <https://design.ros2.org/articles/generated_interfaces_cpp.html#constructors>`__)
* Logging API improvements, now also used in the demos
* Time support in C++ with different clocks
* wait-for-service support in the Python client library
* Draft implementation of `REP 149 <https://reps.openrobotics.org/rep-0149/>`__ specifying format 3 of the package manifest files

Known Issues
------------


* Fast RTPS performance with larger data like the image demo
* Using Connext it is currently not allowed for two topics with the same base name but different namespaces to have a different type (see `issue <https://github.com/ros2/rmw_connext/issues/234>`__).
* Listing of node names (e.g. using ``ros2 node list``) does not work across some rmw implementations.
* On Windows Python launch files might hang when trying to abort using ``Ctrl-C`` (see `issue <https://github.com/ros2/launch/issues/64>`__). In order to continue using the shell which is blocked by the hanging command you might want to end the hanging Python process using the process monitor.
