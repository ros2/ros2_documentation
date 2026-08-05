Debugging
=========

.. toctree::
   :maxdepth: 1

   Debugging/Debugging-Tf2-Problems/Debugging-Tf2-Problems
   Debugging/ROS-2-IDEs
   Debugging/Getting-Backtraces-in-ROS-2
   Debugging/Building-ROS-2-with-Tracing

Sometimes your robot won't behave as you expected.
This article summarizes the ROS developer tools and guidance available to help with debugging.

.. contents:: Table of Contents
   :local:

**Area: debugging, tools | Content-type: about | Experience: beginner, intermediate, expert**

Summary
-------

ROS provides a range of tools to help developers build, test, and monitor robotic systems.
Debugging is an essential part of development, and many of these tools can be used to identify and fix issues.
Some of the tools are also useful for other aspects of robotic system development.
For example, debugging overlaps with introspection, analysis, node management, visualization, and simulation, as these activities are often used to investigate and diagnose issues in a system.

Core ROS packages:

* `ros2doctor <https://docs.ros.org/en/kilted/p/ros2doctor/>`_: Command line tool to check for common misconfigurations in a ROS system.

* `tracetools <https://docs.ros.org/en/kilted/p/tracetools/>`_: Trace points for creating and debugging execution traces.

* `rosbag2 <https://docs.ros.org/en/kilted/p/rosbag2/>`_: Record and play back data moving across topics, services and actions in a ROS application, for later visualization and debugging.

* `ros2cli <https://github.com/ros2/ros2cli>`_: ROS command line interface tools for debugging such as ros2action, ros2node, ros2param, ros2pkg,  ros2service, and so on.

* `rviz <https://github.com/ros2/rviz>`_ and `rviz default plugins <https://github.com/ros2/rviz/tree/rolling/rviz_default_plugins>`_: 3D robot visualization tool for the ROS framework

* `rqt_gui and dependent packages <https://index.ros.org/p/rqt_gui/#rolling-deps>`_: Tool for starting an instance of the ROS integrated graphical user interface.

Community-contributed packages:

* `PlotJuggler <https://docs.ros.org/en/kilted/p/plotjuggler/>`_: A tool for visualizing and analyzing data from ROS applications.

* `Foxglove <https://foxglove.dev/>`_: A collection of tools visualizing and debugging robot data throughout the development lifecycle.

* `Rerun <https://github.com/rerun-io/rerun>`_: Tool to visualize, query, and debug data from robotics, computer vision, and physical AI systems.

Related content
---------------

More articles about debugging:

..
 * `Debugging a ROS application - overview  <https://docs.google.com/document/d/1fz0O2xkaS8u3aoW7eEnA79DThredxxm7O6Cw_UggYu8/edit?usp=sharing>`_
 * `Using ros2doctor to identify issues - how-to <https://docs.ros.org/en/kilted/t/ros2doctor.html>`_
 * `Building ROS 2 with tracing - how-to <https://docs.ros.org/en/kilted/t/tracetools.html>`_

.. FAQs  (placeholder)
   -------------------

   * Example

   * Example
