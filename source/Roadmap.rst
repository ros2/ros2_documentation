.. _Roadmap:

Roadmap
=======

.. contents:: Table of Contents
   :depth: 2
   :local:

This page describes planned work for ROS 2.
The set of planned features and development efforts should provide insight into the overall direction of ROS 2.
If you would like to see other features on the roadmap, please get in touch with us at info@openrobotics.org.

Galactic Roadmap
----------------

Galactic Geochelone is the ROS 2 release expected in May 2021.
See :ref:`release <galactic-release>` for a detailed timeline.

The items in the roadmap below are the major features being worked on by the ROS 2 community.
Note that the table is in priority order; the higher in the list they are, the more important they are.
The "Size" is an estimated size of the task, where S means person-days to complete, M means person-weeks to complete, and L means person-months to complete.

If you are working on a feature for ROS 2 Galactic and would like to have it listed, please open a pull request to `ROS 2 Documentation <https://github.com/ros2/ros2_documentation>`__.
If you'd like to take on one of these tasks, please :ref:`get in touch with us <Help>`.

.. list-table::
   :header-rows: 1

   * - Task
     - Size
     - Owner
     - Expected Completion
   * - Middleware: Improve DDS service reliability
     - Medium
     - ADLINK, eProsima, and Open Robotics
     - 1st quarter 2021
   * - Middleware: Default middleware selection
     - Large
     - Open Robotics
     - Complete
   * - Middleware: Improve DDS fully-connected overhead
     - Small
     - ADLINK, eProsima, and Open Robotics
     - 1st quarter 2021
   * - Documentation: Consolidate ROS 2 documentation in an easy to find/search place
     - Large
     - Open Robotics
     - 4th quarter 2020
   * - Documentation: Automatically generate and host per-package documentation
     - Large
     - Open Robotics
     - 4th quarter 2020
   * - Middleware: Switch to CycloneDDS as default RMW vendor
     - Small
     - ADLINK and Open Robotics
     - 1st quarter 2021
   * - Galactic Release
     - Large
     - Open Robotics
     - 2nd quarter 2021
   * - Quality: Add code coverage checks to CI for packages that are QL 1
     - Medium
     - Open Robotics
     - 2nd quarter 2021
   * - Quality: Keep builds on ci.ros2.org and build.ros2.org green
     - Medium
     - Open Robotics and everyone
     - ongoing
   * - rosbag2: Improve sqlite3 backend performance
     - Small
     - Apex and others
     - Complete
   * - rosbag2: Separate threads for queueing messages and writing to disk
     - Small
     - Apex and others
     - Complete
   * - rosbag2: Record /clock topic
     - Medium
     -
     -
   * - Performance: Reduce the performance overhead of executors
     - Large
     -
     -
   * - Quality: Turn on more compiler warnings
     - Small
     - Open Robotics
     - 2nd quarter 2021
   * - Quality: Increase testing coverage of C/C++ packages
     - Large
     -
     -
   * - Documentation: More intermediate/advanced tutorials
     - Medium
     - Open Robotics
     - 2nd quarter 2021
   * - Quality: Quality Level 1 declaration up to rclcpp
     - Large
     - Open Robotics
     - Complete
   * - Features: Specific demo to show public adoption of ROS 2
     - Medium
     - PickNik Robotics (with Hello Robot 'Stretch')
     -
   * - ROS 1 -> ROS 2 Porting: Enhance documentation to ease porting from ROS 1
     - Small
     -
     -
   * - Quality: Setup configuration so packages can opt-in to clang-tidy for build.ros2.org PR builds
     - Small
     -
     -
   * - Quality: Setup configuration so packages can opt-in to scan-build for build.ros2.org PR builds
     - Small
     - Open Robotics
     - 2nd quarter 2021
   * - Quality: Enable scan-build for core package PR builds
     - Medium
     - Open Robotics
     - 2nd quarter 2021
   * - Launch: Rewrite launch_testing to be a pytest extension
     - Medium
     -
     -
   * - Launch: Re-enable previously disabled tests in launch
     - Medium
     -
     -
   * - Features: Make multi-robot configurations easier to setup and use with ROS 2
     - Medium
     - Open Robotics
     - 1st quarter 2021
   * - Quality: Increase testing coverage of Python packages
     - Large
     -
     -
   * - Quality: Create and maintain an asan job on ci.ros2.org for the ROS 2 core packages
     - Large
     -
     -
   * - Quality: Create and maintain a tsan job on ci.ros2.org for the ROS 2 core packages
     - Large
     -
     -
   * - Quality: Create and maintain a ubsan job on ci.ros2.org for the ROS 2 core packages
     - Large
     -
     -
   * - Quality: Create and maintain a valgrind job on ci.ros2.org for the ROS 2 core packages
     - Large
     -
     -
   * - Middleware: Evaluate middlewares other than DDS
     - Large
     -
     -
   * - Middleware: Documentation for implementing new RMWs
     - Medium
     -
     -
   * - Tech Debt: Rewrite rclpy to use pybind11
     - Large
     -
     -

Quarterly Roadmap
-----------------

**Product Readiness** is the current focus topic for ROS 2.
Over the next two quarters we will concentrate development efforts on topics that make ROS 2 more suitable for use in production scenarios.
This includes improving the out-of-box experience for common use cases, documentation improvements, and addressing disparities between ROS 1 and ROS 2.

## 2020 Q3(Jul - Sep)

* **Transport Documentation and Configurations**: Describe and document the ROS 2 transport system.
  Provide default configurations for common uses cases along with documentation.
  `Tracking ticket <https://github.com/ros2/ros2/issues/1006>`__.

## 2020 Q4(Oct - Dec)

* **Performance Improvements**: Analyze ``rcl*-level`` performance and resource usage.
  Develop a strategy to improve performance and reduce resource usage based on data from the analysis.
  `Tracking ticket <https://github.com/ros2/ros2/issues/1007>`__.

* **Launch**: Address current shortcoming in launch, and improve launch testing.
  `Tracking ticket <https://github.com/ros2/ros2/issues/1008>`__.

* **Documentation Infrastructure**: Develop package-level documentation generation infrastructure, deploy documentation, and consolidate existing documentation.
  `Tracking ticket <https://github.com/ros2/ros2/issues/1009>`__.

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
6. Questions should be asked on `ROS answers <https://answers.ros.org>`__\ , make sure to include at least the ``ros2`` tag and the rosdistro version you are running, e.g. ``ardent``.
