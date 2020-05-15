.. redirect-from::

   ROS-2-On-boarding-Guide

ROS 2 on-boarding guide
=======================

The purpose of this guide is supplement the on-boarding of new developers when they join the ROS 2 team.
It is mostly used by the ROS 2 team, but it might be useful for others as well.

Request access to the GitHub organizations
------------------------------------------

Our code is federated across a few GitHub organizations, you'll want access to them so you can make pull requests with branches rather than forks:


* https://github.com/ros2
* https://github.com/ament
* https://github.com/osrf (optional, as-needed)

Request access to the buildfarm
-------------------------------

The build farm is hosted at: ci.ros2.org

To request access send an email to ros@osrfoundation.org.

How to give access?
^^^^^^^^^^^^^^^^^^^

Your GitHub username must be added with the same permissions as existing users to Jenkins (http://ci.ros2.org/configureSecurity/\ ).
This can be done by any existing user.

How to access the machines running the ci.ros2.org?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Only do this if you're working at OSRF or if you're asked to log into the machines.
To be able to ssh into the node hosted on AWS, you need give request access from Tully Foote (tfoote@osrfoundation.org).

Request access to the Google drive ROS 2 folder
-----------------------------------------------

Only do this if you're working at OSRF or need access to a particular document.
To request access send an email to ros@osrfoundation.org (anybody on the mailing list can share it).

Choose a DDS domain ID
----------------------

ROS 2 uses DDS as the underlying transport and DDS supports a physical segmentation of the network based on the "domain ID" (it is used to calculate the multicast port).
We use a unique value for this on each machine (or group of machines) to keep each group's ROS 2 nodes from interfering with other developers' testing.
We expose this setting via the ``ROS_DOMAIN_ID`` environment variable and use a document to ensure we don't accidentally choose the same one as someone else.
This is, however, only important for people who will be working on the OSRF network, but it isn't a bad idea to set up at any organization with multiple ROS 2 users on the same network.

Get a Personal ROS_DOMAIN_ID
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Go to the `ROS 2 Assigned Domain ID's Spreadsheet <https://docs.google.com/spreadsheets/d/1YuDSH1CeySBP4DaCX4KoCDW_lZY4PuFWUu4MW6Vsp1s/edit>`__ and reserve an ID, or email ros@osrfoundation.org and ask for one to be allocated to you.
Numbers under 128 are preferred.

To ensure it is always set, add this line to your ``~/.bashrc`` or equivalent:

.. code-block:: bash

   export ROS_DOMAIN_ID=<your_domain_id>

Watching ROS 2 Repositories
---------------------------

We try to spread our responsibilities out across the team and so we ask everyone to watch the main repositories for ROS 2.


* What am I currently watching?

  * https://github.com/watching

* How do I watch a repository?

  * https://help.github.com/articles/watching-repositories/

* Which repositories should I watch?

  * All the repositories listed in the `ros2.repos file <https://github.com/ros2/ros2/blob/master/ros2.repos>`__, included the commented out ones,
  * Also all of these extra repositories from the ROS 2 organization:

    * https://github.com/ros2/ci
    * https://github.com/ros2/design
    * https://github.com/ros2/ros_astra_camera
    * https://github.com/ros2/ros_core_documentation
    * https://github.com/ros2/ros2
    * https://github.com/ros2/sros2
    * https://github.com/ros2/turtlebot2_demo
    * https://github.com/ros2/joystick_drivers

Developer Workflow
------------------

We track open tickets and active PRs related to upcoming releases and larger projects using `GitHub project boards <https://github.com/orgs/ros2/projects>`_.

Higher level tasks are tracked on the internal (private) Jira: https://osrfoundation.atlassian.net/projects/ROS2

The usual workflow is (this list is a work in progress):

* Discuss design (GitHub ticket, and a meeting if needed)
* Assign implementation to a team member
* Write implementation on a feature branch

  * Please check out the `developer guide <Developer-Guide>` for guidelines and best practices

* Write tests
* Enable and run linters
* Run tests locally using ``colcon test`` (see `colcon tutorial <../Tutorials/Colcon-Tutorial>`)
* Once everything builds locally without warnings and all tests are passing, run CI on your feature branch:

  * Go to ci.ros2.org
  * Log in (top right corner)
  * Click on the ``ci_launcher`` job
  * Click "Build with Parameters" (left column)
  * In the first box "CI_BRANCH_TO_TEST" enter your feature branch name
  * Hit the ``build`` button

* If your use case require to run code coverage:

  * Go to ci.ros2.org
  * Log in (top right corner)
  * Click on the ``ci_linux_coverage`` job
  * Click "Build with Parameters" (left column)
  * Check instructions to provide parameters to the coverage run detailed at the end of this document
  * Hit the ``build`` button
  * At the end of the document there are instructions about how to interpret the result of report

* If built without warnings, errors and test failures, post the links of your jobs on your PR or high level ticket aggregating all your PRs (see example `here <https://github.com/ros2/rcl/pull/106#issuecomment-271119200>`__)

  * Note that the markdown for these badges is in the console output of the ``ci_launcher`` job

* To get the PR reviewed, you need to put the label "in review":

  * Through GitHub interface:

    * Click on "" next to labels
    * Remove "in progress" label if applicable
    * Add "in review" label

  * If the PR is part of a project board:

    * Drag the card from "In progress" to "In review"

* When the PR has been approved:

  * the person who submitted the PR merges it using "Squash and Merge" option so that we keep a clean history

    * If the commits deserve to keep separated: squash all the nitpick/linters/typo ones together and merge the remaining set

      * Note: each PR should target a specific feature so Squash and Merge should make sense 99% of the time

* Delete the branch once merged

GitHub tips
^^^^^^^^^^^

Link PRs to the issues they address using `keywords <https://help.github.com/en/github/managing-your-work-on-github/linking-a-pull-request-to-an-issue#linking-a-pull-request-to-an-issue-using-a-keyword>`_ and the ticket number.
This will close the issue once the pull request is merged.

* In the same repo: "fixes #216"
* In another repo: "fixes ros2/rosidl#216"

Build Farm Introduction
-----------------------

The build farm is located at `ci.ros2.org <http://ci.ros2.org/>`__.

Every night we run nightly jobs which build and run all the tests in various scenarios on various platforms.
Additionally, we test all pull requests against these platforms before merging.

This is the current set of target platforms and architectures, though it evolves overtime:


* Ubuntu 16.04 Xenial

  * amd64
  * aarch64

* macOS 10.12 Sierra

  * amd64

* Windows 10

  * amd64

There several categories of jobs on the buildfarm:


* manual jobs (triggered manually by developers):

  * ci_linux: build + test the code on Ubuntu Xenial
  * ci_linux-aarch64: build + test the code on Ubuntu Xenial on an ARM 64-bit machine (aarch64)
  * ci_linux_coverage: build + test + generation of test coverage
  * ci_osx: build + test the code on MacOS 10.12
  * ci_windows: build + test the code on Windows 10
  * ci_launcher: trigger all the jobs listed above

* nightly (run every night):

  * Debug: build + test the code with CMAKE_BUILD_TYPE=Debug

    * nightly_linux_debug
    * nightly_linux-aarch64_debug
    * nightly_osx_debug
    * nightly_win_deb

  * Release: build + test the code with CMAKE_BUILD_TYPE=Release

    * nightly_linux_release
    * nightly_linux-aarch64_release
    * nightly_osx_release
    * nightly_win_rel

  * Repeated: build then run each test up to 20 times or until failed (aka flakyness hunter)

    * nightly_linux_repeated
    * nightly_linux-aarch64_repeated
    * nightly_osx_repeated
    * nightly_win_rep

  * Coverage:

    * nightly_linux_coverage: build + test the code + analyses coverage for c/c++ and python

      * results are exported as a cobertura report


* packaging (run every night, against fastrtps; result is bundled into an archive):

  * packaging_linux
  * packaging_osx
  * Packaging_windows

Note on Coverage runs
^^^^^^^^^^^^^^^^^^^^^

ROS2 packages are organized in a way that the testing code for a given package
is not only contained within the Package, but could also be present in a
different package. In other words: packages can exercice during the testing
phase code beloging to other packages.

To achieve the coverage rate reached by all code available in the ROS2 core
packages it is recommended to run builds including the following set of
packages, plus the package under testing:

::
        actionlib_msgs action_msgs action_tutorials_interfaces ament_index_cpp assimp builtin_interfaces bullet class_loader composition composition_interfaces connext_cmake_module console_bridge_vendor curl cyclonedds demo_nodes_cpp demo_nodes_py diagnostic_msgs dummy_map_server dummy_sensors eigen eigen3_cmake_module example_interfaces fastcdr fastrtps fastrtps_cmake_module foonathan_memory_vendor geometry_msgs gmock_vendor interactive_markers kdl_parser laser_geometry launch launch_ros launch_testing launch_testing_ros launch_xml launch_yaml libcurl_vendor libstatistics_collector libyaml_vendor lifecycle lifecycle_msgs log4cxx logging_demo map_msgs message_filters move_base_msgs nav_msgs orocos_kdl osrf_pycommon osrf_testing_tools_cpp pendulum_msgs pluginlib python_cmake_module python_qt_binding qt_dotgraph qt_gui qt_gui_app qt_gui_cpp qt_gui_py_common rcl rcl_action rclcpp rclcpp_action rclcpp_components rclcpp_lifecycle rcl_interfaces rcl_lifecycle rcl_logging_spdlog rclpy rcl_yaml_param_parser rcpputils rcutils resource_retriever rmw rmw_connext_shared_cpp rmw_dds_common rmw_fastrtps_cpp rmw_fastrtps_shared_cpp rmw_implementation rmw_implementation_cmake robot_state_publisher ros2bag ros2cli ros2lifecycle_test_fixtures ros2node ros2param ros2pkg ros2run ros2service ros2test rosbag2_compression rosbag2_converter_default_plugins rosbag2_cpp rosbag2_storage rosbag2_storage_default_plugins rosbag2_test_common rosbag2_tests rosbag2_transport rosgraph_msgs rosidl_adapter rosidl_cmake rosidl_default_generators rosidl_default_runtime rosidl_generator_c rosidl_generator_cpp rosidl_generator_dds_idl rosidl_generator_py rosidl_parser rosidl_runtime_c rosidl_runtime_cpp rosidl_runtime_py rosidl_typesupport_c rosidl_typesupport_connext_c rosidl_typesupport_connext_cpp rosidl_typesupport_cpp rosidl_typesupport_fastrtps_c rosidl_typesupport_fastrtps_cpp rosidl_typesupport_interface rosidl_typesupport_introspection_c rosidl_typesupport_introspection_cpp ros_testing rqt_console rqt_gui rqt_gui_cpp rqt_gui_py rqt_msg rqt_py_common rttest rviz_assimp_vendor rviz_common rviz_default_plugins rviz_ogre_vendor rviz_rendering rviz_rendering_tests rviz_visual_testing_framework sensor_msgs shape_msgs shared_queues_vendor spdlog_vendor sqlite3_vendor sros2 statistics_msgs std_msgs std_srvs stereo_msgs test_interface_files test_msgs tf2 tf2_bullet tf2_eigen tf2_geometry_msgs tf2_kdl tf2_msgs tf2_py tf2_ros tf2_sensor_msgs tf2_tools tinyxml tinyxml2 tinyxml2_vendor tinyxml_vendor tlsf tlsf_cpp tracetools tracetools_launch tracetools_read tracetools_trace trajectory_msgs turtlesim uncrustify uncrustify_vendor unique_identifier_msgs urdf urdfdom urdfdom_headers visualization_msgs yaml_cpp_vendor zstd_vendor

The `ci_linux_coverage` expose `CI_BUILD_ARGS` and `CI_TEST_ARGS` arguments
when launching a new build. The previous set of packages can be injected in
the following ways:

 * `CI_BUILD_ARGS`: use `--packages-up-to` and copy the whole set plus the
   package under testing
 * `CI_TEST_ARGS`: use `--packages-selected` and copy the whole set plus the
   package under testing

How to calculate the coverage rate from the buildfarm report
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The coverage reports in the buildfarm include all the packages that were used in the ROS workspace. To calculate the number of code coverage related to a given package:

 * Go to the `Coverage Report` page as detailed in the Developer Workflow above
 * Scroll down to the `Coverage Breakdown by Package` table
 * In the table, the first column is colled `Name`, scroll down until the `src.ros2.` pattern starts
 * Find the package under testing entries with the pattern of `src.ros2.<repository_name>.<package_name>.`
 * There will one row for every directory/subdirectory of the code that was tested (i.e: src/ src/extra_feature/ include/)
 * For all the directories (excluding test/) under the pattern `src.ros2.<repository_name>.<package_name>.<dirs>` grab the two absolute values in the column `Lines`. For each cell the first value is the lines tested and the second is the total lines of code.
 * Aggregate all rows for getting the total of the lines tested and the total of lines of code under test. Divide to get the coverage rate.

Learning ROS 2 concepts at a high level
---------------------------------------

All ROS 2 design documents are available at http://design.ros2.org/ and there is some generated documentation at http://docs.ros2.org/.
