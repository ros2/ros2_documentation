.. meta::
   :contentType: about
   :experience: intermediate
   :area: visualization, tools
   :distribution: {DISTRO}
   :product: {PRODUCT}

Visualization
=============

.. short-description::
   Sometimes you need to visualize what your robot is sensing and planning to better develop, test, and debug your system.
   This article summarizes the ROS developer tools and guidance available to help with visualization.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. toctree::
   :hidden:

   Visualization/RViz/RViz-Main
   Visualization/About-RQt
   Visualization/Rqtbag/Create-An-Rqtbag-Plugin

.. contents:: Contents
   :depth: 2
   :local:

Summary
-------

Understanding what a robot is sensing, experiencing, and planning makes development and debugging much easier.
Visualization tools in ROS help you convert raw robot data into clear, human-readable visual formats.

Core ROS packages
-----------------

* :doc:`RViz <Visualization/RViz/RViz-User-Guide/RViz-User-Guide>`: Provides a 3D visualization of the robot's perspective, including sensor data and intentions.
  See `rviz <https://github.com/ros2/rviz>`__ and `rviz default plugins <https://github.com/ros2/rviz/tree/{REPOS_FILE_BRANCH}/rviz_default_plugins>`__.

* :doc:`rqt <Visualization/About-RQt>`: Plugin-based framework for the ROS integrated graphical user interface.
  See `rqt_gui and dependent packages <https://index.ros.org/p/rqt_gui/#{DISTRO}-deps>`__.

Community-contributed packages
------------------------------

* `Foxglove <https://foxglove.dev/>`__: A collection of tools for visualizing and debugging robot data throughout the development lifecycle.

* `Rerun <https://github.com/rerun-io/rerun>`__: Tool to visualize, query, and debug data from robotics, computer vision, and physical AI systems.
