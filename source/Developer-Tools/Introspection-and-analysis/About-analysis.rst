Analysis
========

Analyzing introspection data can help with improving it's performance.
This article summarizes the ROS developer tools and guidance available to help with analysis.

**Area: analysis, tools | Content-type: about | Experience: beginner, intermediate, expert**

.. contents:: Table of Contents
   :local:

Summary
-------

Understanding what is happening inside your robot's software starts with introspection, which allows you to collect data.
Analysis tools in ROS help you go further by making sense of that data and revealing insights at the right level of detail.

Core ROS packages:

* `ros2doctor <https://docs.ros.org/en/kilted/p/ros2doctor/>`_: Command line tool to check for common misconfigurations in a ROS system.

* `tracetools <https://docs.ros.org/en/kilted/p/tracetools/>`_: Trace points for creating and debugging execution traces.

* `rosbag2 <https://docs.ros.org/en/kilted/p/rosbag2/>`_: Record and play back data moving across topics, services and actions in a ROS application, for later visualization and debugging.

* `ros2cli <https://github.com/ros2/ros2cli>`_: ROS command line interface tools for debugging such as ros2action, ros2node, ros2param, ros2pkg,  ros2service, and so on.

* `rviz <https://github.com/ros2/rviz>`_ and `rviz default plugins <https://github.com/ros2/rviz/tree/rolling/rviz_default_plugins>`_: 3D robot visualization tool for the ROS framework

* `rqt_gui and dependent packages <https://index.ros.org/p/rqt_gui/#rolling-deps>`_: Tool for starting an instance of the ROS integrated graphical user interface.

Community-contributed packages:

*  `Plotjuggler <https://index.ros.org/p/plotjuggler/>`_: Visualization and analysis tool for inspecting, monitoring, and interpreting time-series data.

* `Foxglove <https://foxglove.dev/>`_: A collection of tools visualizing and debugging robot data throughout the development lifecycle.

* `Rerun <https://github.com/rerun-io/rerun>`_: Tool to visualize, query, and debug data from robotics, computer vision, and physical AI 

.. Related content (placeholder)
   -----------------------------

   More articles about analysis:

   * Example

   * Example

   FAQs (placeholder)
   ------------------

   * Example

   * Example
