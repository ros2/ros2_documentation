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

The items in the roadmap below are the major features planned for Humble Hawksbill.
The "Size" is an estimated size of the task, where *Small* means person-days to complete, *Medium* means person-weeks to complete, and *Large* means person-months to complete.

Open Robotics Contributions
^^^^^^^^^^^^^^^^^^^^^^^^^^^

These are the features that are being worked on by Open Robotics and are prioritized: the higher they are in the list, the higher priority they are.

.. raw:: html

   <style>
     .wy-table-responsive table td, .wy-table-responsive table th {
       white-space: normal;
     }
   </style>

.. list-table::
   :widths: 60 15 25
   :header-rows: 1

   * - Task
     - Size
     - Expected Completion
   * - Humble Release
     - Large
     - 2nd quarter 2022
   * - Middleware: Default middleware selection
     - Medium
     -
   * - Middleware: Report on feasibility of using Zenoh as an RMW
     - Medium
     -
   * - Documentation: Upload C++ API documentation for core packages to docs site
     - Medium
     -
   * - Overlays work to override any package in a dependency tree
     - Large
     -
   * - Documentation: Add Python API support to rosdoc2
     - Medium
     -
   * - Improve service and action reliability > 10Hz
     - Medium
     -
   * - Documentation: Support inter-package linking for rosdoc2 buildfarm jobs
     - Small
     -
   * - Improve RViz2 stability
     - Medium
     -
   * - Develop a system level QoS/network debugging tool
     - Large
     -
   * - Revamp launch testing
     - Large
     -

Community Contributions
^^^^^^^^^^^^^^^^^^^^^^^

These are the features that are being worked on by the community.
This list is not prioritized.
If you are working on a feature for ROS 2 Humble and would like to have it listed, please open a pull request to `ROS 2 Documentation <https://github.com/ros2/ros2_documentation>`__.

.. list-table::
   :widths: 40 15 20 25
   :header-rows: 1

   * - Task
     - Size
     - Owner
     - Expected Completion
   * -
     -
     -
     -



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
