.. redirect-from::

    Beta1-Overview

Beta 1 (codename 'Asphalt'; December 2016)
==========================================

.. contents:: Table of Contents
   :depth: 2
   :local:

Supported Platforms
-------------------

We support ROS 2 Beta 1 on three platforms: Ubuntu 16.04 (Xenial), Mac OS X 10.11 (El Capitan), and Windows 8.1 and 10. We provide both binary packages and instructions for how to compile from source for all 3 platforms.

Features
--------

Improvements since Alpha 8 release
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Support for node composition at compile, link, or runtime.
* A standard lifecycle for managed nodes.
* Improved support for Quality of Service tuning and tests.
* `New and updated design documents <https://design.ros2.org/>`__
* More `tutorials <../Tutorials>` and `examples <https://github.com/ros2/examples>`__
* Bridging services to / from ROS 1 (in addition to topics)

Selected features from previous Alpha releases
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For the complete list, see `earlier release notes <../Releases>`.


* C++ and Python implementations of ROS 2 client libraries including APIs for:

  * Publishing and subscribing to ROS topics
  * Requesting and replying ROS services (synchronous (C++ only) and asynchronous)
  * Getting and setting ROS parameters (C++ only, synchronous and asynchronous)
  * Timer callbacks
  * Support for interoperability between multiple DDS/RTPS implementations
  * eProsima Fast RTPS is our default implementation, and is included in the binary packages
  * RTI Connext is supported: build from source to try it out
  * We initially supported PrismTech OpenSplice but eventually decided to drop it

* A graph API for network events
* Distributed discovery
* Realtime safe code paths for publish and subscribe with compatible DDS implementation (only Connext at the moment)

  * Support for custom allocators

* ROS 1 <-> ROS 2 dynamic bridge node
* Executor threading model in C++
* Extended ``.msg`` format with new features:

  * Bounded arrays
  * Default values

Known issues
^^^^^^^^^^^^

* We’re tracking issues in various repositories, but the main entry point is the `ros2/ros2 issue tracker <https://github.com/ros2/ros2/issues>`__
* We’d like to highlight a `known issue <https://github.com/ros2/rmw_fastrtps/issues/81>`__ that we are working with eProsima to fix that results in significantly degrated performance for large messages under FastRTPS.
  This will be observed when running some of the demos with larger image resolutions.
