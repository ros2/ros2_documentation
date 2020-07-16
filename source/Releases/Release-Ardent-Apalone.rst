.. redirect-from::

    Release-Ardent-Apalone

ROS 2 Ardent Apalone (codename 'ardent'; December 2017)
=======================================================

Welcome to the first non-beta release of ROS 2 software named *Ardent Apalone*!

Supported Platforms
-------------------

This version of ROS 2 is supported on three platforms:


* Ubuntu 16.04 (Xenial)
* Mac macOS 10.12 (Sierra)
* Windows 10

Binary packages as well as instructions for how to compile from source are provided for all 3 platforms (see `install instructions <../Installation>` as well as `documentation <https://docs.ros2.org/ardent/>`__).

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

For a more detailed description please see the `Features <../Features>` page.

Changes since Beta 3 release
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Improvements since the Beta 3 release:


* ``rviz``
* Different initialization options for message data structures in C++ (see `design doc <https://design.ros2.org/articles/generated_interfaces_cpp.html#constructors>`__)
* Logging API improvements, now also used in the demos
* Time support in C++ with different clocks
* wait-for-service support in the Python client library
* Draft implementation of `REP 149 <https://www.ros.org/reps/rep-0149.html>`__ specifying format 3 of the package manifest files

Known Issues
------------


* Fast RTPS performance with larger data like the image demo
* Using Connext it is currently not allowed for two topics with the same base name but different namespaces to have a different type (see `issue <https://github.com/ros2/rmw_connext/issues/234>`__).
* Listing of node names (e.g. using ``ros2 node list``) does not work across some rmw implementations.
* On Windows Python launch files might hang when trying to abort using ``Ctrl-C`` (see `issue <https://github.com/ros2/launch/issues/64>`__). In order to continue using the shell which is blocked by the hanging command you might want to end the hanging Python process using the process monitor.
