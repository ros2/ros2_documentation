.. redirect-from::

  Roadmap

.. _Roadmap:

Roadmap
=======

.. contents:: Table of Contents
   :depth: 2
   :local:

This page describes planned work for ROS 2.
The set of planned features and development efforts should provide insight into the overall direction of ROS 2.
If you would like to see other features on the roadmap, please get in touch with us at info@openrobotics.org.

Iron Roadmap
------------

Iron Irwini is the ROS 2 release expected in May 2023.
See :ref:`release <iron-release>` for a detailed timeline.

The items in the roadmap below are the major features being worked on by the ROS 2 community.
The "Size" is an estimated size of the task, where *Small* means person-days to complete, *Medium* means person-weeks to complete, and *Large* means person-months to complete.

If you are working on a feature for ROS 2 Iron and would like to have it listed, please open a pull request to `ROS 2 Documentation <https://github.com/ros2/ros2_documentation>`__.
If you'd like to take on one of these tasks, please :doc:`get in touch with us <../Contact>`.

.. raw:: html

   <style>
     .wy-table-responsive table td, .wy-table-responsive table th {
       white-space: normal;
     }
   </style>

.. list-table::
   :widths: 40 10 25 25
   :header-rows: 1

   * - Task
     - Size
     - Owner
     - Expected Completion
   * - Various improvements and port of further functionality for Diagnostics
     - Medium
     - Bosch
     - Q1 2023
   * - License linter and copyright file generator for binary packages
     - Medium
     - Bosch
     - Q2 2023
   * - rclc Dispatcher Executor for non-POSIX OS
     - Medium
     - Bosch
     - Q1 2023
   * - Improve rclcpp executor performance
     - Large
     - Open Robotics
     - Q4 2022
   * - DDS User Experience - Improve the out-of-the-box experience
     - Large
     - Open Robotics
     - Q1 2023
   * - DDS User Experience - Configuration of Initial Peers for Discovery
     - Small
     - Open Robotics
     - Q4 2022
   * - DDS User Experience - Develop a configuration tool
     - Medium
     - Open Robotics
     - Q4 2022
   * - Python per-package documentation generation
     - Small
     - Open Robotics
     - Q4 2022
   * - rclpy performance with large messages
     - Medium
     - Open Robotics
     - Q1 2023
   * - ROS 1 to ROS 2 migration documentation and tools
     - Medium
     - Open Robotics
     - Q2 2023
   * - SDF integration
     - Medium
     - Open Robotics
     - Q1 2023
   * - Better error message for launch (stretch goal)
     - Medium
     - Open Robotics
     - Q2 2023
   * - Relaunch of individual nodes in a complex system (stretch goal)
     - Small
     - Open Robotics
     - Q2 2023
   * - Logging configuration and features (stretch goal)
     - Medium
     - Open Robotics
     - Q2 2023
   * - Iron release
     - Large
     - Open Robotics
     - Q2 2023
   * - `ContentFiltering fallback in rcl <https://github.com/ros2/design/pull/282>`__
     - Large
     - Sony Group Corporation
     - Q1 2023
   * - `on_pub/sub_matched callback support <https://github.com/ros2/rmw/issues/330>`__
     - Medium
     - Sony Group Corporation
     - Q1 2023
   * - ROS 2 core ContentFiltering Enhancement
     - Medium
     - Sony Group Corporation
     - Q2 2023

Additional project-specific roadmaps can be found in the links below:

- MoveIt2: https://moveit.ros.org/documentation/contributing/roadmap/
- Nav2: https://navigation.ros.org/roadmap/roadmap.html


Planned releases
----------------

Please see the :doc:`Distributions page <../Releases>` for the timeline of and information about future distributions.

Contributing to ROS 2
---------------------

Looking for something to work on, or just want to help out? Here are a few resources to get you going.

1. The :doc:`Contributing <Contributing>` guide describes how to make a contribution to ROS 2.
2. Check out the list of :doc:`Feature Ideas <Feature-Ideas>` for inspiration.
3. For more information on the design of ROS 2 please see `design.ros2.org <https://design.ros2.org>`__.
4. The core code for ROS 2 is in the `ros2 GitHub organization <https://github.com/ros2>`__.
5. The Discourse forum/mailing list for discussing ROS 2 design is `ng-ros <https://discourse.ros.org/c/ng-ros>`__.
6. Questions should be asked on `ROS answers <https://answers.ros.org>`__\ , make sure to include at least the ``ros2`` tag and the rosdistro version you are running, e.g. ``{DISTRO}``.
