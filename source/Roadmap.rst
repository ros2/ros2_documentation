.. _Roadmap:

Roadmap
=======

.. contents:: Table of Contents
   :depth: 2
   :local:

This page describes planned work for ROS 2.
The set of planned features and development efforts should provide insight into the overall direction of ROS 2.
If you would like to see other features on the roadmap, please get in touch with us at info@openrobotics.org.

Humble Roadmap
----------------

Humble Hawksbill is the ROS 2 release expected in May 2022.
See :ref:`release <humble-release>` for a detailed timeline.

The items in the roadmap below are the major features being worked on by the ROS 2 community.
The "Size" is an estimated size of the task, where *Small* means person-days to complete, *Medium* means person-weeks to complete, and *Large* means person-months to complete.

If you are working on a feature for ROS 2 Humble and would like to have it listed, please open a pull request to `ROS 2 Documentation <https://github.com/ros2/ros2_documentation>`__.
If you'd like to take on one of these tasks, please :ref:`get in touch with us <Help>`.

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
   * - Identify if messages have changed over time
     - Large
     - Apex
     -
   * - Rosbag2 playback controlled by /clock sim time
     - Small
     - AWS
     - Q3 2021
   * - Rosbag2 "Snapshot mode"
     - Medium
     - AWS
     - Q3 2021
   * - Interface definitions: get exact msg/srv/action definitions on the wire at runtime
     - Large
     - AWS / Foxglove
     - Q1 2022
   * - Performance improvements: Implement events executor in rclcpp
     - Large
     - iRobot
     - 4th quarter 2021
   * - Performance improvements: Improve intra-process optimization and loaned message APIs
     - Medium
     - iRobot
     - 4th quarter 2021
   * - Developer experience: Better integration of ROS 2 repos with package managers and documentation for building/running ROS 2 applications.
     - Large
     - iRobot
     - 2th quarter 2022
   * - Develop a system level QoS/network debugging tool
     - Large
     - Open Robotics
     -
   * - Documentation: Add Python API support to rosdoc2
     - Medium
     - Open Robotics
     -
   * - Documentation: Support inter-package linking for rosdoc2 buildfarm jobs
     - Small
     - Open Robotics
     -
   * - Documentation: Upload C++ API documentation for core packages to docs site
     - Medium
     - Open Robotics
     -
   * - Humble Release
     - Large
     - Open Robotics
     - 2nd quarter 2022
   * - Improve service and action reliability for frequencies greater than 10Hz
     - Medium
     - Open Robotics
     -
   * - Improve RViz2 stability
     - Medium
     - Open Robotics
     -
   * - Middleware: Default middleware selection
     - Medium
     - Open Robotics
     - 4th quarter 2021
   * - Middleware: Report on feasibility of using Zenoh as an RMW
     - Medium
     - Open Robotics
     -
   * - Overlays work to override any package in a dependency tree
     - Large
     - Open Robotics
     -
   * - Revamp launch testing
     - Large
     - Open Robotics
     -
   * - Filter topic messages by content in rclcpp
     - Large
     - Sony
     - 1st quarter 2022
   * - Wait for an acknowledgment from a reliable publisher in rclcpp and rclpy
     - Small
     - Sony
     - 4th quarter 2022


Additional project-specific roadmaps can be found in the links below:

- MoveIt2: https://moveit.ros.org/documentation/contributing/roadmap/
- Nav2: https://navigation.ros.org/roadmap/roadmap.html


Planned releases
----------------

Please see the :ref:`Distributions page <Releases>` for the timeline of and information about future distributions.

Contributing to ROS 2
---------------------

Looking for something to work on, or just want to help out? Here are a few resources to get you going.

1. The :ref:`Contributing <Contributing>` guide describes how to make a contribution to ROS 2.
2. Check out the list of :ref:`Feature Ideas <FeatureIdeas>` for inspiration.
3. For more information on the design of ROS 2 please see `design.ros2.org <https://design.ros2.org>`__.
4. The core code for ROS 2 is in the `ros2 GitHub organization <https://github.com/ros2>`__.
5. The Discourse forum/mailing list for discussing ROS 2 design is `ng-ros <https://discourse.ros.org/c/ng-ros>`__.
6. Questions should be asked on `ROS answers <https://answers.ros.org>`__\ , make sure to include at least the ``ros2`` tag and the rosdistro version you are running, e.g. ``galactic``.
