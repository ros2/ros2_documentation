.. _upcoming-release:

.. move this directive when next release page is created

ROS 2 Foxy Fitzroy (codename 'foxy'; June 5th, 2020)
====================================================

.. contents:: Table of Contents
   :depth: 2
   :local:

*Foxy Fitzroy* is the sixth release of ROS 2.

Supported Platforms
-------------------

Foxy Fitzroy is primarily supported on the following platforms:

Tier 1 platforms:

* Ubuntu 20.04 (Focal): ``amd64`` and ``arm64``
* Mac macOS 10.14 (Mojave)
* Windows 10 (Visual Studio 2019)

Tier 2 platforms:

* Ubuntu 20.04 (Focal): ``arm32``

Tier 3 platforms:

* Debian Buster (10): ``amd64``, ``arm64`` and ``arm32``
* OpenEmbedded Thud (2.6) / webOS OSE: ``arm32`` and ``x86``

For more information about RMW implementations, compiler / interpreter versions, and system dependency versions see `REP 2000 <http://www.ros.org/reps/rep-2000.html>`__.


New features in this ROS 2 release
----------------------------------

During the development the `Foxy meta-ticket <https://github.com/ros2/ros2/issues/830>`__ on GitHub contains an up-to-date state of the ongoing high level tasks as well as references specific tickets with more details.

Changes since the Eloquent release
----------------------------------

ament_export_interfaces replaced by ament_export_targets
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The CMake function ``ament_export_interfaces`` from the package ``ament_cmake_export_interfaces`` has been deprecated in favor of the function ``ament_export_targets`` in the new package ``ament_cmake_export_targets``.
See the GitHub ticket `ament/ament_cmake#237 <https://github.com/ament/ament_cmake/issues/237>`_ for more context.

Default working directory for ament_add_test
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The default working directory for tests added with ``ament_add_test`` has been changed to ``CMAKE_CURRENT_BINARY_DIR`` to match the behavior of CMake ``add_test``.
Either update the tests to work with the new default or pass ``WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}`` to restore the previous value.

Default Console Logging Format
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The default console logging output format was changed to include the timestamp by default, see:

- `https://github.com/ros2/rcutils/pull/190 <https://github.com/ros2/rcutils/pull/190>`_
- `https://discourse.ros.org/t/ros2-logging-format/11549 <https://discourse.ros.org/t/ros2-logging-format/11549>`_

Default Console Logging Output Stream
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As of Foxy, all logging messages at all severity levels get logged to stderr by default.
This ensures that logging messages come out immediately, and brings the ROS 2 logging system into alignment with most other logging systems.
It is possible to change the stream to stdout at runtime via the RCUTILS_LOGGING_USE_STDOUT environment variable, but all logging messages will still go to the same stream.
See `https://github.com/ros2/rcutils/pull/196 <https://github.com/ros2/rcutils/pull/196>`_ for more details.

launch_ros
^^^^^^^^^^

Node name and namespace parameters changed
""""""""""""""""""""""""""""""""""""""""""

The ``Node`` action parameters related to naming have been changed:

- ``node_name`` has been renamed to ``name``
- ``node_namespace`` has been renamed to ``namespace``
- ``node_executable`` has been renamed to ``executable``
- ``exec_name`` has been added for naming the process associated with the node.
  Previously, users would have used the ``name`` keyword argument.

The old parameters have been deprecated.

These changes were made to make the launch frontend more idiomatic.
For example, instead of

.. code-block:: xml

   <node pkg="demo_nodes_cpp" exec="talker" node-name="foo" />

we can now write

.. code-block:: xml

   <node pkg="demo_nodes_cpp" exec="talker" name="foo" />

This change also applies to ``ComposableNodeContainer``, ``ComposableNode``, and ``LifecycleNode``.
For examples, see the `relevant changes to the demos. <https://github.com/ros2/demos/pull/431>`_

`Related pull request in launch_ros. <https://github.com/ros2/launch_ros/pull/122>`_

rclcpp
^^^^^^

Change in Advanced Subscription Callback Signature
""""""""""""""""""""""""""""""""""""""""""""""""""

With the pull request [https://github.com/ros2/rclcpp/pull/1047](https://github.com/ros2/rclcpp/pull/1047) the signature of callbacks which receive the message info with the message has changed.
Previously it used the ``rmw`` type ``rmw_message_info_t``, but now uses the ``rclcpp`` type ``rclcpp::MessageInfo``.
The required changes are straightforward, and can be seen demonstrated in these pull requests:

- [https://github.com/ros2/system_tests/pull/423/files](https://github.com/ros2/system_tests/pull/423/files)
- [https://github.com/ros2/rosbag2/pull/375/files](https://github.com/ros2/rosbag2/pull/375/files)
- [https://github.com/ros2/ros1_bridge/pull/253/files](https://github.com/ros2/ros1_bridge/pull/253/files)

rclpy
^^^^^

Support for multiple on parameter set callbacks
"""""""""""""""""""""""""""""""""""""""""""""""

Use the ``Node`` methods ``add_on_set_parameters_callback`` and ``remove_on_set_parameters_callback`` for adding and removing functions that are called when parameters are set.

The method ``set_parameters_calblack`` has been deprecated.

Related pull requests: https://github.com/ros2/rclpy/pull/457, https://github.com/ros2/rclpy/pull/504

rmw_connext_cpp
^^^^^^^^^^^^^^^

Connext 5.1 locator kinds compatibility mode
""""""""""""""""""""""""""""""""""""""""""""

Up to and including ``Eloquent``, ``rmw_connext_cpp`` was setting ``dds.transport.use_510_compatible_locator_kinds`` property to ``true``.
This property is not being forced anymore, and shared transport communication between ``Foxy`` and previous releases will stop working.
Logs similar to:

.. code-block:: bash

  PRESParticipant_checkTransportInfoMatching:Warning: discovered remote participant 'RTI Administration Console' using the 'shmem' transport with class ID 16777216.
  This class ID does not match the class ID 2 of the same transport in the local participant 'talker'.
  These two participants will not communicate over the 'shmem' transport.
  Check the value of the property 'dds.transport.use_510_compatible_locator_kinds' in the local participant.
  See https://community.rti.com/kb/what-causes-error-discovered-remote-participant for additional info.

will be observed when this incompatibility happens.

If compatibility is needed, it can be set up in an external qos profiles files containing:

.. code-block:: xml

   <participant_qos>
      <property>
         <value>
               <element>
                  <name>
                     dds.transport.use_510_compatible_locator_kinds
                  </name>
                  <value>1</value>
               </element>
         </value>
      </property>
   </participant_qos>

Remember to set the ``NDDS_QOS_PROFILES`` environment variable to the qos profiles file path.
For more information, see ``How to Change Transport Settings in 5.2.0 Applications for Compatibility with 5.1.0`` section of `Transport_Compatibility <https://community.rti.com/static/documentation/connext-dds/5.2.0/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_ReleaseNotes/Content/ReleaseNotes/Transport_Compatibility.htm>`_.

rviz
^^^^

Tools timestamp messages using ROS time
"""""""""""""""""""""""""""""""""""""""

'2D Pose Estimate', '2D Nav Goal', and 'Publish Point' tools now timestamp their messages using ROS time instead of system time, in order for the ``use_sim_time`` parameter to have an effect on them.

Related pull request: https://github.com/ros2/rviz/pull/519

Timeline before the release
---------------------------

A few milestones leading up to the release:

    Wed. April 22nd, 2020
        API and feature freeze for ``ros_core`` [1]_ packages.
        Note that this includes ``rmw``, which is a recursive dependency of ``ros_core``.
        Only bug fix releases should be made after this point.
        New packages can be released independently.

    Mon. April 29th, 2020 (beta)
        Updated releases of ``desktop`` [2]_ packages available.
        Testing of the new features.

    Wed. May 27th, 2020 (release candidate)
        Updated releases of ``desktop`` [2]_ packages available.

    Wed. June 3rd, 2020
        Freeze rosdistro.
        No PRs for Foxy on the `rosdistro` repo will be merged (reopens after the release announcement).

.. [1] The ``ros_core`` variant described in the `variants <https://github.com/ros2/variants>`_ repository.
.. [2] The ``desktop`` variant described in the `variants <https://github.com/ros2/variants>`_ repository.
