.. redirect-from::

    Concepts/Basic/About-Launch

.. meta::
   :contentType: about
   :experience: beginner
   :area: node-management, tools
   :distribution: {DISTRO}
   :product: {PRODUCT}

Node management
===============

.. short-description::
   Launch files help you start and configure multiple nodes in a ROS system without running each process manually.
   This article introduces the ROS launch system and explains how it uses launch files to describe, run, and manage system processes.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. toctree::
   :maxdepth: 2
   :hidden:

   Launch/Launching-composable-nodes
   Launch/Creating-Launch-Files
   Launch/Launch-Main
   Launch/Using-Event-Handlers
   Launch/Using-ROS2-Launch-For-Large-Projects
   Launch/Using-Substitutions
   Launch/Launch-file-different-formats
   Launch/Launch-system
   Launch/FastDDS-Configuration
   Launch/Improved-Dynamic-Discovery
   Launch/Node-arguments
   Launch/Discovery-Server/Discovery-Server
   Launch/Security/About-Security

Sometimes you need to manage many interconnected nodes at once to reliably start, configure, and shut down your system.
Managing multiple interconnected nodes is essential for complex robotic systems effective runtime node management.
This article summarizes the ROS developer tools and guidance available to help with node management, also referred to as launch or runtime node management.

.. contents:: Table of Contents
   :depth: 2
   :local:

Summary
-------

Complex robotic systems can include many interconnected nodes, each with different settings and interfaces.
Runtime node management tools in ROS help you start, configure, and stop multiple nodes together using launch files.

Core ROS packages
-----------------

* `ros2launch <https://github.com/ros2/launch_ros/tree/{REPOS_FILE_BRANCH}/ros2launch>`_: Command line tool for launching multiple processes and for writing tests involving multiple processes.

* `launch <{package_link(launch)}>`_: The launch system itself, which describes a system configuration and then executes and monitors it.

* `launch_ros <{package_link(launch_ros)}>`_: ROS-specific extensions to the launch system, such as launching nodes and composable nodes.

.. Community-contributed packages:

.. Related content (placeholder)
   -----------------------------

   More articles about node management:

   * Example

   * Example

   FAQs (placeholder)
   ------------------

   * Example

   * Example
