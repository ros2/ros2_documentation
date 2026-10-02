Motion planning and manipulation
================================

.. toctree::
   :maxdepth: 1
   :hidden:

   Motion-planning/Real-Time-Programming

Motion planning enables you to control a robot's movements to achieve an intended goal.
Manipulation capabilities enable robots to analyze their environment and perform tasks like picking and placing objects.
This article summarizes ROS developer tools and guidance available to help with motion planning and manipulation.

**Area: motion-planning, manipulation, capabilities | Content-type: about | Experience: beginner, intermediate, expert**

.. contents:: Table of Contents
   :local:

Summary
-------

Every robot needs to interact with its environment.
Motion planning enables your robot to plan movements, including responding to real-world effects such as wheel slip or things in the environment moving.
This can involve controlling complex chains of actuators and links, such as those found in industrial manipulators.
ROS motion planning capabilities let you focus on where you want the end effector to be, rather than focusing on controlling every actuator individually.
Manipulation capabilities include analysis of the environment around a manipulator, planning the motion of the manipulator to get the end effector where it needs to be, and controlling end effects to do things like pick and place objects.

For mobile robots, ROS also supports navigation capabilities.
See :doc:`About-Navigation`.

ROS capabilities are enabled by community-contributed packages.

Community-contributed packages
------------------------------

* `MoveIt <https://moveit.ai/>`_: A motion planning, manipulation, and kinematics framework for ROS.

* `Tesseract <https://github.com/tesseract-robotics>`_: A lightweight motion planning framework designed for the needs of industrial workcells.

* `Open Motion Planning Library (OMPL) <https://ompl.kavrakilab.org>`_: A C++ library providing sampling-based algorithms for motion planning in robotics and automated systems.

* `Search Based Planning Library (SBPL) <https://github.com/sbpl/sbpl>`_: A C++ library providing search-based motion planning algorithms for robotics and automated systems.

* `RoboPlan <https://github.com/open-planning/roboplan>`_: A Rust and C++ library providing a planning framework for robotics and automated systems.

* `pyroboplan <https://pyroboplan.readthedocs.io/en/latest/>`_: A Python library for modeling robot kinematics and dynamics.

.. Related content (placeholder)
   ------------------------------
   * Example

   * Example

   FAQs (placeholder)
   ------------------
   * Example

   * Example
