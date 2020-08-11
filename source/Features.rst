.. _Features:

Features Status
===============

The features listed below are available in the current ROS 2 release.
Unless otherwise specified, the features are available for all supported platforms (Ubuntu 18.04, macOS 10.12.x, Windows 10), DDS implementations (eProsima Fast RTPS, RTI Connext and ADLINK Opensplice) and programming language client libraries (C++ and Python).
For planned future development, see the :ref:`Roadmap <Roadmap>`.

.. list-table::
   :header-rows: 1

   * - Functionality
     - Link
     - Fine print
   * - Discovery, transport and serialization over DDS
     - `Article <https://design.ros2.org/articles/ros_on_dds.html>`__
     -
   * - Support for `multiple DDS implementations <Concepts/DDS-and-ROS-middleware-implementations>`, chosen at runtime
     - `Tutorials </Tutorials/Working-with-multiple-RMW-implementations>`
     - Currently eProsima Fast RTPS, RTI Connext and ADLINK OpenSplice are fully supported.
   * - Common core client library that is wrapped by language-specific libraries
     - `Details <Concepts/ROS-2-Client-Libraries>`
     -
   * - Publish/subscribe over topics
     - `Sample code <https://github.com/ros2/examples>`__\ , `Article <https://design.ros2.org/articles/topic_and_service_names.html>`__
     -
   * - Clients and services
     - `Sample code <https://github.com/ros2/examples>`__
     -
   * - Set/retrieve parameters
     - `Sample code <https://github.com/ros2/demos/tree/0.5.1/demo_nodes_cpp/src/parameters>`__
     -
   * - ROS 1 - ROS 2 communication bridge
     - `Tutorial <https://github.com/ros2/ros1_bridge/blob/master/README.md>`__
     - Available for topics and services, not yet available for actions.
   * - Quality of service settings for handling non-ideal networks
     - `Demo <Tutorials/Quality-of-Service>`
     -
   * - Inter- and intra-process communication using the same API
     - `Demo <Tutorials/Intra-Process-Communication>`
     - Currently only in C++.
   * - Composition of node components at compile-, link- or ``dlopen``\ -time
     - `Demo <Tutorials/Composition>`
     - Currently only in C++.
   * - Support for nodes with managed lifecycles
     - `Demo <Tutorials/Managed-Nodes>`
     - Currently only in C++.
   * - DDS-Security support
     - `Demo <https://github.com/ros2/sros2>`__
     -
   * - Command-line introspection tools using an extensible framework
     - `Tutorial <Tutorials/Introspection-with-command-line-tools>`
     -
   * - Launch system  for coordinating multiple nodes
     - `Tutorial <Tutorials/Launch-system>`
     -
   * - Namespace support for nodes and topics
     - `Article <https://design.ros2.org/articles/topic_and_service_names.html>`__
     -
   * - Static remapping of ROS names
     - `Tutorial <Tutorials/Node-arguments>`
     -
   * - Demos of an all-ROS 2 mobile robot
     - `Demo <https://github.com/ros2/turtlebot2_demo>`__
     -
   * - Preliminary support for real-time code
     - `Demo <Tutorials/Real-Time-Programming>`, `demo <Tutorials/Allocator-Template-Tutorial>`
     - Linux only. Not available for Fast RTPS.
   * - Preliminary support for "bare-metal" microcontrollers
     - `Wiki <https://github.com/ros2/freertps/wiki>`__
     -

Besides core features of the platform, the biggest impact of ROS comes from its available packages.
The following are a few high-profile packages which are available in the latest release:

* `gazebo_ros_pkgs <https://index.ros.org/r/gazebo_ros_pkgs/>`__
* `image_transport <https://index.ros.org/r/image_common>`__
* `navigation2 <https://index.ros.org/r/navigation2/>`__
* `rosbag2 <https://index.ros.org/r/rosbag2/>`__
* `RQt <https://index.ros.org/r/rqt/>`__
* `RViz2 <https://index.ros.org/r/rviz/>`__
