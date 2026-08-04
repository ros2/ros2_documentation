Builds
======

The ability to work with multiple packages together is useful for developing complex robotics systems.
This article summarizes the ROS developer tools and guidance available to help with builds.


**Area: builds, tools | Content-type: about | Experience: beginner, intermediate, expert**

.. contents:: Table of Contents
   :local:

Summary
-------

Most ROS systems use multiple packages working together.
Build tools in ROS enable you to work with multiple packages in a single project and to share your packages with other members of the ROS community.

Core ROS packages:

* `colcon <https://docs.ros.org/en/lyrical/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html>`_: A tool for building packages in a ROS workspace.

* `CMake <https://cmake.org/>`_: A supported tool for building complex C++ projects.

* `ament_cmake <https://docs.ros.org/en/lyrical/How-To-Guides/Ament-CMake-Documentation.html>`_: A CMake library to facilitate inter package dependencies in ROS.

Community-contributed packages:

* `Cargo <https://github.com/rust-lang/cargo>`_: A build and package management tool for Rust-based robotics software.

* `Rust <https://github.com/rust-lang>`_: Programming language used to develop high-performance, reliable robotics software and tools.


ROS build farm location
-----------------------

The ROS build farm is critical infrastructure maintained by Open Robotics.
It supports the building of source and binary packages, as well as continuous integration, testing, and analysis for ROS packages:

* `<https://build.ros2.org/>`_

Read more about ROS build farms: `ROS Build Farms — ROS 2 Documentation: Lyrical  documentation <https://docs.ros.org/en/lyrical/The-ROS2-Project/Contributing/Build-Farms.html>`_

.. Related content (placeholder)
   -----------------------------

   More articles about builds:

   * Example

   * Example

   FAQs (placeholder)
   ------------------

   * Example

   * Example
