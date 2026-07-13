Debugging
=========

Sometimes your robot won't behave as you expected. This article summarizes the ROS developer tools and guidance available to help with debugging.

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
QUESTION FOR THE REVIEWER: ARE THERE ANY OTHER CORE ROS PACKAGES THAT WE CAN LIST HERE?

Community-contributed packages:

* `PlotJuggler <https://docs.ros.org/en/kilted/p/plotjuggler/>`_: A tool for visualizing and analyzing data from ROS applications.
QUESTION FOR THE REVIEWER: ARE THERE ANY OTHER COMMUNITY-CONTRIBUTED PACKAGES THAT WE CAN LIST HERE?

Related content
---------------

More articles about debugging:

..
 * `Debugging a ROS application - overview  <https://docs.google.com/document/d/1fz0O2xkaS8u3aoW7eEnA79DThredxxm7O6Cw_UggYu8/edit?usp=sharing>`_
 * `Using ros2doctor to identify issues - how-to <https://docs.ros.org/en/kilted/t/ros2doctor.html>`_
 * `Building ROS 2 with tracing - how-to <https://docs.ros.org/en/kilted/t/tracetools.html>`_

FAQs  (placeholder)
-------------------

* Example

* Example
