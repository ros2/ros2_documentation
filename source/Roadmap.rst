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

The items in the roadmap below are the major features being worked on by Open Robotics.
Note that the table is in priority order; the higher in the list they are, the more important they are.
The "Size" is an estimated size of the task, where *Small* means person-days to complete, *Medium* means person-weeks to complete, and *Large* means person-months to complete.

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
   * - Middleware: Investigate using Zenoh as an RMW
     - Medium
     -
   * - Documentation: C++ API documentation for core packages
     - Medium
     -
   * - Documentation: Make rosdoc2 buildfarm jobs support inter-package linking
     - Small
     -
   * - Documentation: Python API documentation
     - Medium
     -
   * - RViz2: Fix outstanding crashes, memory leaks, problems
     - Medium
     -
   * - Tooling: Improve system-level debugging tools
     - Large
     -
   * - Launch: Revamp launch testing
     - Large
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
