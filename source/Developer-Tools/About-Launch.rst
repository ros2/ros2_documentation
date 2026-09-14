.. redirect-from::

    Concepts/Basic/About-Launch

Node management
===============
.. toctree::
   :maxdepth: 1
   :hidden:

   Launch/Launch-Main
   Launch/Launching-composable-nodes
   Launch/Launch-file-different-formats

Sometimes you need to manage many interconnected nodes at once to reliably start, configure, and shut down your system.
Managing multiple interconnected nodes is essential for complex robotic systems effective runtime node management.
This article summarizes the ROS developer tools and guidance available to help with node management, also referred to as launch or runtime node management.

**Area: node management, tools | Content-type: about | Experience: beginner, intermediate, expert**

.. contents:: Table of Contents
   :local:
   :depth: 2

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
