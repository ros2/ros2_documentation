
Tutorials
=========

Beginner level
--------------

The beginner level tutorials are a collection of step-by-step instructions meant to introduce newcomers to ROS 2.
We recommend following the tutorials in order, from the "Users" set to the "Developers" set to gain basic foundational knowledge of ROS 2.

You're welcome to pick tutorials out of the set if you're looking for something specific, just understand that the tutorials build off of each other and are not meant to be comprehensive documentation.
These tutorials are under construction, so please share your `feedback <https://github.com/ros2/ros2_documentation/issues/new>`__.

.. Feedback is placeholder for actual feedback solution in development

Users
^^^^^

.. toctree::
   :maxdepth: 1

   Tutorials/Configuring-ROS2-Environment
   Tutorials/Turtlesim/Introducing-Turtlesim
   Tutorials/Understanding-ROS2-Nodes
   Tutorials/Topics/Understanding-ROS2-Topics
   Tutorials/Services/Understanding-ROS2-Services
   Tutorials/Parameters/Understanding-ROS2-Parameters
   Tutorials/Understanding-ROS2-Actions
   Tutorials/Rqt-Console/Using-Rqt-Console
   Tutorials/Launch-Files/Creating-Launch-Files
   Tutorials/Ros2bag/Recording-And-Playing-Back-Data

Developers
^^^^^^^^^^

.. toctree::
   :maxdepth: 1

   Tutorials/Workspace/Creating-A-Workspace
   Tutorials/Creating-A-ROS2-Package
   Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber
   Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber
   Tutorials/Writing-A-Simple-Cpp-Service-And-Client
   Tutorials/Writing-A-Simple-Py-Service-And-Client
   Tutorials/Getting-Started-With-Ros2doctor


Miscellaneous
-------------

.. toctree::
   :maxdepth: 1

   Tutorials/Developing-a-ROS-2-Package
   Tutorials/Colcon-Tutorial
   Tutorials/Ament-Tutorial
   Tutorials/Ament-CMake-Documentation
   Tutorials/Cross-compilation
   Tutorials/catment
   Tutorials/Introspection-with-command-line-tools
   Tutorials/RQt-Overview-Usage
   Tutorials/RQt-Port-Plugin-Windows
   Tutorials/Node-arguments
   Tutorials/Launch-system
   Tutorials/Launch-files-migration-guide
   Tutorials/Working-with-multiple-RMW-implementations
   Tutorials/Composition
   Tutorials/Rosidl-Tutorial.rst
   Tutorials/New-features-in-ROS-2-interfaces-(msg-srv)
   Tutorials/Defining-custom-interfaces-(msg-srv)
   Tutorials/Actions
   Tutorials/Eclipse-Oxygen-with-ROS-2-and-rviz2
   Tutorials/Building-ROS-2-on-Linux-with-Eclipse-Oxygen
   Tutorials/Building-Realtime-rt_preempt-kernel-for-ROS-2
   Tutorials/Working-with-Eclipse-CycloneDDS
   Tutorials/Releasing-a-ROS-2-package-with-bloom


Advanced
--------

.. toctree::
   :maxdepth: 1

   Tutorials/Allocator-Template-Tutorial


Using Docker
------------

.. toctree::
   :maxdepth: 1

   Tutorials/Run-2-nodes-in-a-single-docker-container
   Tutorials/Run-2-nodes-in-two-separate-docker-containers

Demos
-----

.. toctree::
   :hidden:

   Tutorials/Quality-of-Service
   Tutorials/Managed-Nodes
   Tutorials/Intra-Process-Communication
   Tutorials/Rosbag-with-ROS1-Bridge
   Tutorials/tf2
   Tutorials/Real-Time-Programming
   Tutorials/Python-Programming
   Tutorials/dummy-robot-demo
   Tutorials/Logging-and-logger-configuration

* `Use quality-of-service settings to handle lossy networks <Tutorials/Quality-of-Service>`.
* `Management of nodes with managed lifecycles <Tutorials/Managed-Nodes>`.
* `Efficient intra-process communication <Tutorials/Intra-Process-Communication>`.
* `Bridge communication between ROS 1 and ROS 2 <https://github.com/ros2/ros1_bridge/blob/master/README.md>`__.
* `Recording and playback of topic data with rosbag using the ROS 1 bridge <Tutorials/Rosbag-with-ROS1-Bridge>`.
* `Turtlebot 2 demo using ROS 2 <https://github.com/ros2/turtlebot2_demo>`__.
* `TurtleBot 3 demo using ROS 2 <http://emanual.robotis.com/docs/en/platform/turtlebot3/applications/#ros2>`__. [community-contributed]
* `Using tf2 with ROS 2 <Tutorials/tf2>`.
* `Write real-time safe code that uses the ROS 2 APIs <Tutorials/Real-Time-Programming>`.
* `Use the rclpy API to write ROS 2 programs in Python <Tutorials/Python-Programming>`.
* `Use the robot state publisher to publish joint states and TF <Tutorials/dummy-robot-demo>`.
* `Use DDS-Security <https://github.com/ros2/sros2/blob/master/README.md>`__.
* `Logging and logger configuration <Tutorials/Logging-and-logger-configuration>`.

Examples
--------

* `Python and C++ minimal examples <https://github.com/ros2/examples>`__.
