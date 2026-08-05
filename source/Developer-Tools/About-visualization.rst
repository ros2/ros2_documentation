Visualization
=============

.. toctree::
   :maxdepth: 1

   Visualization/RViz/RViz-Main
   Visualization/About-RQt
   Visualization/Rqtbag/Create-An-Rqtbag-Plugin

Sometimes you need to visualize what your robot is sensing and planning to better develop, test, and debug your system.
This article summarizes the ROS developer tools and guidance available to help with visualization.


**Area: visualization, tools | Content-type: about | Experience: beginner, intermediate, expert**

.. contents:: Table of Contents
   :local:

Summary
-------

Understanding what a robot is sensing, experiencing, and planning makes development and debugging much easier.
Visualization tools in ROS help you convert raw robot data into clear, human-readable visual formats.

Core ROS packages:

* `RViz <https://docs.ros.org/en/kilted/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html>`_: Provides a 3D visualization of the robot's perspective, including sensor data and intentions.

* `ros2cli <https://github.com/ros2/ros2cli>`_: ROS command line interface tools for debugging such as ros2action, ros2node, ros2param, ros2pkg,  ros2service, and so on.

* `rviz <https://github.com/ros2/rviz>`_ and `rviz default plugins <https://github.com/ros2/rviz/tree/rolling/rviz_default_plugins>`_: 3D robot visualization tool for the ROS framework

* `rqt_gui and dependent packages <https://index.ros.org/p/rqt_gui/#rolling-deps>`_: Tool for starting an instance of the ROS integrated graphical user interface.

Community-contributed packages:

* `Foxglove <https://foxglove.dev/>`_: A collection of tools visualizing and debugging robot data throughout the development lifecycle.

* `Rerun <https://github.com/rerun-io/rerun>`_: Tool to visualize, query, and debug data from robotics, computer vision, and physical AI systems.

.. Related content (placeholder)
   -----------------------------

   More articles about visualization:

   * Example

   * Example

   FAQs (placeholder)
   ------------------

   * Example

   * Example
