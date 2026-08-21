Introspection and analysis
==========================

.. toctree::
   :maxdepth: 1
   :hidden:

   Introspection-and-analysis/About-Command-Line-Tools
   Introspection-and-analysis/About-Security
   Introspection-and-analysis/Publishing-Messages-Using-YAML-Files
   Introspection-and-analysis/Topic-Statistics-Tutorial/Topic-Statistics-Tutorial
   Introspection-and-analysis/Discovery-Server/Discovery-Server
   Introspection-and-analysis/FastDDS-Configuration
   Introspection-and-analysis/Improved-Dynamic-Discovery
   Introspection-and-analysis/Tracing/ROS2-Tracing-Trace-and-Analyze
   Introspection-and-analysis/Security/Security-Main
   Introspection-and-analysis/Logging-and-logger-configuration
   Introspection-and-analysis/Service-Introspection
   Introspection-and-analysis/Node-arguments

Sometimes you need to understand what is happening inside your robot's software.
This article summarizes the ROS developer tools and guidance available to help with introspection and analysis.

**Area: introspection, analysis, tools | Content-type: about | Experience: beginner, intermediate, expert**

.. contents:: Table of Contents
   :local:
   :depth: 2

Summary
-------

Understanding what is happening inside your robot's software is key to understanding how to improve it.
Introspection tools in ROS allow you to understand and modify, at run-time, node state, parameters, flowing data, and more.
ROS analysis tools build further on this ability by enabling you to analyze the data gathered by introspection.

Core ROS packages
-----------------

* `ros2 <https://github.com/ros2>`_: A set of software libraries and tools for building robot applications.

* `rqt <https://github.com/ros-visualization/rqt>`_: A set of graphical tools for visualizing and interacting with ROS data.

* `rosbag2 <https://docs.ros.org/en/kilted/p/ros2bag/>`_: Tool for recording and playing back data moving across topics, services and actions in a ROS application.

* `rosconsole <https://github.com/ros/rosconsole/>`_: A logging tool for monitoring, debugging, and diagnosing running applications.

* `tracetools <https://docs.ros.org/en/kilted/p/tracetools/>`_: Trace points for creating and debugging execution traces.

Community-contributed packages
------------------------------

* :doc:`colcon <../ROS-Framework/client-libraries/Working-with-Client-Libraries/Colcon-Tutorial>`: Tool for creating and building a ROS workspace.

* :doc:`ament_cmake <Build/Ament-CMake-Documentation>`: Build system for CMake-based packages in ROS.

* `tracetools <https://docs.ros.org/en/kilted/p/tracetools/>`_: Trace points for creating and debugging execution traces.

* `Plotjugler <https://index.ros.org/p/plotjuggler/>`_: Visualization and analysis tool for inspecting, monitoring, and interpreting time-series data.

.. Related content (placeholder)
   -----------------------------

   More articles about introspection:

   * Example

   * Example

   FAQs (placeholder)
   ------------------

   * Example

   * Example
