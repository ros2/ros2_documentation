# ROS 2 Feature Status

The features listed below are available in the current ROS 2 release.
Unless otherwise specified, the features are available for all supported platforms (Ubuntu 18.04, OS X 10.12.x, Windows 10), DDS implementations (eProsima Fast RTPS, RTI Connext and PrismTech Opensplice) and programming language client libraries (C++ and Python).
For planned future development, see the [Roadmap](Roadmap.md).

| Functionality | Link | Fine print |
| --- | --- | --- |
| Discovery, transport and serialization over DDS | [Article](http://design.ros2.org/articles/ros_on_dds.html) | |
| Support for multiple DDS implementations, chosen at runtime | [Tutorial\](DDS-and-ROS-middleware-implementations.md) | Currently eProsima Fast RTPS, RTI Connext and PrismTech Opensplice are fully supported. |
| Common core client library that is wrapped by language-specific libraries | [Tutorial\](ROS 2 Client Libraries.md) | |
| Publish/subscribe over topics | [Sample code](https://github.com/ros2/examples), [Article](http://design.ros2.org/articles/topic_and_service_names.html) | |
| Clients and services | [Sample code](https://github.com/ros2/examples) | |
| Set/retrieve parameters | [Sample code](https://github.com/ros2/demos/tree/0.5.1/demo_nodes_cpp/src/parameters) | Parameters not yet available in `rcl`/Python. |
| ROS 1 - ROS 2 communication bridge | [Tutorial](https://github.com/ros2/ros1_bridge/blob/master/README.md) | Available for topics and services, not yet available for actions. |
| Quality of service settings for handling non-ideal networks | [Demo\](Quality-Of-Service.md) | |
| Inter- and intra-process communication using the same API | [Demo\](Intra-Process-Communication.md) | Currently only in C++. |
| Composition of node components at compile-, link- or `dlopen`-time | [Demo\](Composition.md) | Currently only in C++. |
| Support for nodes with managed lifecycles | [Demo](https://github.com/ros2/ros2/wiki/Managed-Nodes) | Currently only in C++. |
| DDS-Security support | [Demo](https://github.com/ros2/sros2) | |
| Command-line introspection tools using an extensible framework | [Tutorial\](Introspection-with-command-line-tools.md) | |
| Launch system  for coordinating multiple nodes | [Tutorial](https://github.com/ros2/ros2/wiki/Launch-system) | |
| Namespace support for nodes and topics | [Article](http://design.ros2.org/articles/topic_and_service_names.html) | |
| Static remapping of ROS names | [Tutorial](https://github.com/ros2/ros2/wiki/Node-arguments) | |
| Demos of an all-ROS 2 mobile robot | [Demo](https://github.com/ros2/turtlebot2_demo) | |
| Preliminary support for real-time code | [Demo\](Real-Time-Programming.md), [demo\](Allocator-Template-Tutorial.md) | Linux only. Not available for Fast RTPS. |
| Preliminary support for "bare-metal" microcontrollers | [Wiki](https://github.com/ros2/freertps/wiki)| |
