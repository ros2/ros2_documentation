
Tutorials
=========

About ROS 2
-----------


* `Overview of ROS 2 concepts <Overview-of-ROS-2-concepts>`
* `DDS and ROS middleware implementations <DDS-and-ROS-middleware-implementations>`
* `ROS 2 Client Libraries <ROS-2-Client-Libraries>`
* `About ROS interfaces <About-ROS-Interfaces>`
* `About Quality of Service Settings <About-Quality-of-Service-Settings>`
* `Releasing a ROS 2 package with bloom <Releasing-a-ROS-2-package-with-bloom>`

ROS 2 Tutorials
---------------


* `Installation from binary and source, all platforms <Installation>`
* `Using colcon to build a custom package <Colcon-Tutorial>`
* `Introspection with command-line tools <Introspection-with-command-line-tools>`
* `User Interfaces with RQt <RQt-Overview-Usage>`
* `Passing ROS arguments to nodes via the command-line <Node-arguments>`
* `Launching/monitoring multiple nodes with Launch <Launch-system>`
* `Working with multiple RMW implementations <Working-with-multiple-RMW-implementations>`
* `Composing multiple nodes in a single process <Composition>`
* `Defining custom interfaces (msg/srv) <Defining-custom-interfaces-(msg-srv)>`
* `New features in ROS 2 interfaces (msg srv) <New-features-in-ROS-2-interfaces-(msg-srv)>`
* `Eclipse Oxygen with ROS 2 and rviz2 <Eclipse-Oxygen-with-ROS-2-and-rviz2>` [community-contributed]
* `Building ROS 2 on Linux with Eclipse Oxygen <Building-ROS-2-on-Linux-with-Eclipse-Oxygen>` [community-contributed]
* `Building Realtime rt_preempt kernel for ROS 2 <Building-Realtime-rt_preempt-kernel-for-ROS-2>` [community-contributed]

Advanced
^^^^^^^^


* `Implement a custom memory allocator <Allocator-Template-Tutorial>`

Docker
^^^^^^


* `Run 2 nodes in a single docker container <Run-2-nodes-in-a-single-docker-container>` [community-contributed]
* `Run 2 nodes in two separate docker containers <Run-2-nodes-in-two-separate-docker-containers>` [community-contributed]

ROS 2 Demos
-----------


* `Use quality-of-service settings to handle lossy networks <Quality-of-Service>`
* `Management of nodes with managed lifecycles <Managed-Nodes>`
* `Efficient intra-process communication <Intra-Process-Communication>`
* `Bridge communication between ROS 1 and ROS 2 <https://github.com/ros2/ros1_bridge/blob/master/README.md>`__
* `Recording and playback of topic data with rosbag using the ROS 1 bridge <Rosbag-with-ROS1-Bridge>`
* `Turtlebot 2 demo using ROS 2 <https://github.com/ros2/turtlebot2_demo>`__
* `TurtleBot 3 demo using ROS 2 <http://emanual.robotis.com/docs/en/platform/turtlebot3/applications/#ros2>`__ [community-contributed]
* `Using tf2 with ROS 2 <tf2>`
* `Write real-time safe code that uses the ROS 2 APIs <Real-Time-Programming>`
* `Use the rclpy API to write ROS 2 programs in Python <Python-Programming>`
* `Use the robot state publisher to publish joint states and TF <dummy-robot-demo>`
* `Use DDS-Security <https://github.com/ros2/sros2/blob/master/README.md>`__
* `Logging and logger configuration <Logging-and-logger-configuration>`

ROS 2 Examples
--------------


* `Python and C++ minimal examples <https://github.com/ros2/examples>`__

Troubleshooting
---------------

Enable Multicast
^^^^^^^^^^^^^^^^

In order to communicate successfully via DDS, the used network interface has to be multicast enabled.
We've seen in past experiences that this might not necessarily be enabled by default (on Ubuntu or OSX) when using the loopback adapter.
See the `original issue <https://github.com/ros2/ros2/issues/552>`__ or a `conversation on ros-answers <https://answers.ros.org/question/300370/ros2-talker-cannot-communicate-with-listener/>`__.
You can verify that your current setup allows multicast with the ROS 2 tool:

In Terminal 1:

.. code-block:: bash

   ros2 multicast receive

In Terminal 2:

.. code-block:: bash

   ros2 multicast send
