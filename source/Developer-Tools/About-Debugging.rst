.. meta::
   :contentType: about
   :experience: intermediate
   :area: debugging
   :distribution: {DISTRO}
   :product: {PRODUCT}

Debugging
=========

.. short-description::
   Sometimes your robot won't behave as you expected.
   This article summarizes the ROS developer tools and guidance available to help with debugging.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. toctree::
   :hidden:

   Debugging/Debugging-Tf2-Problems/Debugging-Tf2-Problems
   Debugging/ROS-2-IDEs
   Debugging/Getting-Backtraces-in-ROS-2
   Debugging/Building-ROS-2-with-Tracing
   Debugging/About-testing

.. contents:: Contents
   :depth: 2
   :local:

Summary
-------

ROS provides a range of tools to help developers build, test, and monitor robotic systems.
Debugging is an essential part of development, and many of these tools can be used to identify and fix issues.
Some of the tools are also useful for other aspects of robotic system development.
For example, debugging overlaps with introspection, analysis, node management, visualization, and simulation, as these activities are often used to investigate and diagnose issues in a system.

Core ROS packages
-----------------

* `ros2doctor <{package_link(ros2doctor)}>`__: Command-line tool to check for common misconfigurations in a ROS system.

* `tracetools <{package_link(tracetools)}>`__: Trace points for creating and debugging execution traces.

* `rosbag2 <{package_link(rosbag2)}>`__: Record and play back data moving across topics, services, and actions in a ROS application, for later visualization and debugging.

* `ros2cli <https://github.com/ros2/ros2cli>`__: ROS command-line tools for debugging, such as ``ros2 action``, ``ros2 node``, ``ros2 param``, ``ros2 pkg``, and ``ros2 service``.

* `rviz <https://github.com/ros2/rviz>`__ and `rviz default plugins <https://github.com/ros2/rviz/tree/{REPOS_FILE_BRANCH}/rviz_default_plugins>`__: 3D robot visualization tool for the ROS framework.

* `rqt_gui and dependent packages <https://index.ros.org/p/rqt_gui/#{DISTRO}-deps>`__: Tool for starting an instance of the ROS integrated graphical user interface.

Community-contributed packages
------------------------------

* `PlotJuggler <{package_link(plotjuggler)}>`__: A tool for visualizing and analyzing data from ROS applications.

* `Foxglove <https://foxglove.dev/>`__: A collection of tools for visualizing and debugging robot data throughout the development lifecycle.

* `Rerun <https://github.com/rerun-io/rerun>`__: Tool to visualize, query, and debug data from robotics, computer vision, and physical AI systems.
