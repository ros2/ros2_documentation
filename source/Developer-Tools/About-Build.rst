Builds
======

.. toctree::
   :maxdepth: 1

   Build/About-Cross-Compilation
   Build/About-Build-System
   Build/Deployment-Guidelines/Deployment-Guidelines
   Build/The-Keystore
   Build/Using-Xacro-to-Clean-Up-a-URDF-File/Using-Xacro-to-Clean-Up-a-URDF-File
   Build/Supplementing-Custom-Rosdep-Keys
   Build/Ament-Lint-For-Clean-Code
   Build/Eclipse-Oxygen-with-ROS-2-and-rviz2/Eclipse-Oxygen-with-ROS-2-and-rviz2
   Build/Ament-CMake-Documentation
   Build/Ament-CMake-Python-Documentation
   Build/Building-a-Custom-Deb-Package
   Build/Cross-compilation
   Build/Run-2-nodes-in-single-or-separate-docker-containers
   Build/Setup-ROS-2-with-VSCode-and-Docker-Container
   Build/Core-maintainer-guide
   Build/DDS-tuning
   Build/Using-Variants
   Build/Package-Docs
   Build/Using-Python-Packages
   Build/Developing-a-ROS-2-Package
   Build/Releasing/Releasing-a-Package
   Build/Building-Realtime-rt_preempt-kernel-for-ROS-2/Building-Realtime-rt_preempt-kernel-for-ROS-2
   Build/Building-ROS2-Package-with-eclipse-2021-06/Building-ROS2-Package-with-eclipse-2021-06
   Build/Single-Package-Define-And-Use-Interface

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
