Introspection
=============

Sometimes you need to understand what is happening inside your robot's software.
This article summarizes the ROS developer tools and guidance available to help with introspection.

**Area: introspection, tools | Content-type: about | Experience: beginner, intermediate, expert**

.. contents:: Table of Contents
   :local:

Summary
-------

Understanding what is happening inside your robot's software is key to understanding how to improve it.
Introspection tools in ROS allow you to understand and modify, at run-time, node state, parameters, flowing data, and more.

Core ROS packages:

* `ros2doctor <https://docs.ros.org/en/kilted/p/ros2doctor/>`_: Command line tool to check for common misconfigurations in a ROS system.

* `ros2launch <https://github.com/ros2/launch>`_:  Command line tool for launching multiple processes and for writing tests involving multiple processes.

* `rosbag2 <https://docs.ros.org/en/kilted/p/ros2bag/>`_: Tool for recording and playing back data moving across topics, services and actions in a ROS application.

* `ros2cli <https://github.com/ros2/ros2cli>`_: A set of command line tools for ROS, used to manage nodes, topics, services, and other ROS components, including introspection.

* `RViz <https://docs.ros.org/en/kilted/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html>`_: Provides a 3D visualization of the robot's perspective, including sensor data and intentions.

* `rosdoc2 <https://docs.ros.org/en/rolling/How-To-Guides/Documenting-a-ROS-2-Package.html>`_: Tool for documenting a ROS2 package.


Community-contributed packages:

* `colcon <https://docs.ros.org/en/lyrical/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html>`_: Tool for creating and building a ROS workspace.

* `ament_cmake <https://docs.ros.org/en/lyrical/How-To-Guides/Ament-CMake-Documentation.html>`_: Build system for CMake-based packages in ROS.

* `tracetools <https://docs.ros.org/en/kilted/p/tracetools/>`_: Trace points for creating and debugging execution traces.

Related content (placeholder)
-----------------------------

More articles about introspection:

* Example

* Example

FAQs (placeholder)
------------------

* Example

* Example
