.. _latest_release:

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

Tier 3 platforms:

* Ubuntu 20.04 (Focal): ``arm32``
* Debian Buster (10): ``amd64``, ``arm64`` and ``arm32``
* OpenEmbedded Thud (2.6) / webOS OSE: ``arm32`` and ``x86``

For more information about RMW implementations, compiler / interpreter versions, and system dependency versions see `REP 2000 <https://www.ros.org/reps/rep-2000.html>`__.

Installation
------------

`Install Foxy Fitzroy <../foxy/Installation/Summary.html>`__

New features in this ROS 2 release
----------------------------------

During the development the `Foxy meta-ticket <https://github.com/ros2/ros2/issues/830>`__ on GitHub contains an up-to-date state of the ongoing high-level tasks as well as references specific tickets with more details.

Changes in Patch Release 2
--------------------------

Bug in static_transform_publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
During the development of Foxy, a bug was introduced into the tf2_ros static_transform_publisher program.
The implementation of the order of the Euler angles passed to static_transform_publisher disagrees with the documentation.
Foxy patch release 2 `fixes <https://github.com/ros2/geometry2/pull/296>`_ the order so that the implementation agrees with the documentation (yaw, pitch, roll).
For users who have started using the initial Foxy release or patch release 1, this means that any launch files that use static_transform_publisher will have to have the command-line order swapped according to the new order.
For users who are coming from ROS 2 Dashing, ROS 2 Eloquent, or ROS 1, no changes need to be made to port to Foxy patch release 2.

Changes since the Eloquent release
----------------------------------

Classic CMake vs. modern CMake
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In "classic" CMake a package provides CMake variables like ``<pkgname>_INCLUDE_DIRS`` and ``<pkgname>_LIBRARIES`` when being ``find_package()``-ed.
With ``ament_cmake`` that is achieved by calling ``ament_export_include_directories`` and ``ament_export_libraries``.
In combination with ``ament_export_dependencies``, ``ament_cmake`` ensures that all include directories and libraries of recursive dependencies are concatenated and included in these variables.

In "modern" CMake a package provides an interface target instead (commonly named ``<pkgname>::<pkgname>``) which in itself encapsulates all recursive dependencies.
In order to export a library target to use modern CMake ``ament_export_targets`` needs to be called with an export name which is also used when installing the libraries using ``install(TARGETS <libA> <libB> EXPORT <export_name> ...)``.
The exported interface targets are available through the CMake variable ``<pkgname>_TARGETS``.
For library targets to be exportable like this they must not rely on classic functions affecting global state like ``include_directories()`` but set the include directories on the target itself - for the build as well as install environment - using generator expressions, e.g. ``target_include_directories(<target> PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>" "$<INSTALL_INTERFACE:include>")``.

When ``ament_target_dependencies`` is used to add dependencies to a library target the function uses modern CMake targets when they are available.
Otherwise it falls back to using classic CMake variables.
As a consequence you should only export modern CMake targets if all dependencies are also providing modern CMake targets.
**Otherwise the exported interface target will contain the absolute paths to include directories / libraries in the generated CMake logic which makes the package non-relocatable.**

For examples how packages have been updated to modern CMake in Foxy see `ros2/ros2#904 <https://github.com/ros2/ros2/issues/904>`_.

ament_export_interfaces replaced by ament_export_targets
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The CMake function ``ament_export_interfaces`` from the package ``ament_cmake_export_interfaces`` has been deprecated in favor of the function ``ament_export_targets`` in the new package ``ament_cmake_export_targets``.
See the GitHub ticket `ament/ament_cmake#237 <https://github.com/ament/ament_cmake/issues/237>`_ for more context.

rosidl_generator_c|cpp namespace / API changes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The packages ``rosidl_generator_c`` and ``rosidl_generator_cpp`` have been refactored with many headers and sources moved into the new packages ``rosidl_runtime_c`` and ``rosidl_runtime_cpp``.
The intention is to remove run dependencies on the generator packages and therefore the code generation tools using Python.
While moving the headers the include paths / namespaces were updated accordingly so in many cases changing include directives from the generator package to the runtime package is sufficient.

The generated C / C++ code has also been refactored.
The files ending in ``__struct.h|hpp``, ``__functions.h``, ``__traits.hpp``, etc. have been moved into a subdirectory ``detail`` but most code only includes the header named after the interface without any of these suffixes.

Some types regarding string and sequence bounds have also been renamed to match the naming conventions but they aren't expected to be used in user code (above RMW implementation and type support packages)

For more information see `ros2/rosidl#446 (for C) <https://github.com/ros2/rosidl/issues/446>`_ and `ros2/rosidl#447 (for C++) <https://github.com/ros2/rosidl/issues/447>`_.

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

With the pull request `https://github.com/ros2/rclcpp/pull/1047 <https://github.com/ros2/rclcpp/pull/1047>`_ the signature of callbacks which receive the message info with the message has changed.
Previously it used the ``rmw`` type ``rmw_message_info_t``, but now uses the ``rclcpp`` type ``rclcpp::MessageInfo``.
The required changes are straightforward, and can be seen demonstrated in these pull requests:

- `https://github.com/ros2/system_tests/pull/423/files <https://github.com/ros2/system_tests/pull/423/files>`_
- `https://github.com/ros2/rosbag2/pull/375/files <https://github.com/ros2/rosbag2/pull/375/files>`_
- `https://github.com/ros2/ros1_bridge/pull/253/files <https://github.com/ros2/ros1_bridge/pull/253/files>`_

Change in Serialized Message Callback Signature
"""""""""""""""""""""""""""""""""""""""""""""""

The pull request `ros2/rclcpp#1081 <https://github.com/ros2/rclcpp/pull/1081>`_ introduces a new signature of the callbacks for retrieving ROS messages in serialized form.
The previously used C-Struct `rcl_serialized_message_t <https://github.com/ros2/rmw/blob/master/rmw/include/rmw/serialized_message.h>`_ is being superseded by a C++ data type `rclcpp::SerializedMessage <https://github.com/ros2/rclcpp/blob/master/rclcpp/include/rclcpp/serialized_message.hpp>`_.

The example nodes in ``demo_nodes_cpp``, namely ``talker_serialized_message`` as well as ``listener_serialized_message`` reflect these changes.

Breaking change in Node Interface getters' signature
""""""""""""""""""""""""""""""""""""""""""""""""""""

With pull request `ros2/rclcpp#1069 <https://github.com/ros2/rclcpp/pull/1069>`_, the signature of node interface getters has been modified to return shared ownership of node interfaces (i.e. an ``std::shared_ptr``) instead of a non-owning raw pointer.
Required changes in downstream packages that relied on the previous signature are simple and straightforward: use the ``std::shared_ptr::get()`` method.

Deprecate set_on_parameters_set_callback
""""""""""""""""""""""""""""""""""""""""

Instead, use the ``rclcpp::Node`` methods ``add_on_set_parameters_callback`` and ``remove_on_set_parameters_callback`` for adding and removing functions that are called when parameters are set.

Related pull request: https://github.com/ros2/rclcpp/pull/1123

Breaking change in Publisher getter signature
""""""""""""""""""""""""""""""""""""""""""""""

With pull request `ros2/rclcpp#1119 <https://github.com/ros2/rclcpp/pull/1119>`_, the signature of publisher handle getter has been modified to return shared ownership of the underlying rcl structure (i.e. an ``std::shared_ptr``) instead of a non-owning raw pointer.
This was necessary to fix a segfault in certain circumstances.
Required changes in downstream packages that relied on the previous signature are simple and straightforward: use the ``std::shared_ptr::get()`` method.

rclcpp_action
^^^^^^^^^^^^^

Deprecate ClientGoalHandle::async_result()
""""""""""""""""""""""""""""""""""""""""""

Using this API, it is possible to run into a race condition causing an exception to be thrown.
Instead, prefer to use ``Client::async_get_result()``, which is safer.

See `ros2/rclcpp#1120 <https://github.com/ros2/rclcpp/pull/1120>`_ and the connected issue for more info.

rclpy
^^^^^

Support for multiple on parameter set callbacks
"""""""""""""""""""""""""""""""""""""""""""""""

Use the ``Node`` methods ``add_on_set_parameters_callback`` and ``remove_on_set_parameters_callback`` for adding and removing functions that are called when parameters are set.

The method ``set_parameters_callback`` has been deprecated.

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

If compatibility is needed, it can be set up in an external QoS profiles files containing:

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

Remember to set the ``NDDS_QOS_PROFILES`` environment variable to the QoS profiles file path.
For more information, see ``How to Change Transport Settings in 5.2.0 Applications for Compatibility with 5.1.0`` section of `Transport_Compatibility <https://community.rti.com/static/documentation/connext-dds/5.2.0/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_ReleaseNotes/Content/ReleaseNotes/Transport_Compatibility.htm>`_.

rviz
^^^^

Tools timestamp messages using ROS time
"""""""""""""""""""""""""""""""""""""""

'2D Pose Estimate', '2D Nav Goal', and 'Publish Point' tools now timestamp their messages using ROS time instead of system time, in order for the ``use_sim_time`` parameter to have an effect on them.

Related pull request: https://github.com/ros2/rviz/pull/519

std_msgs
^^^^^^^^

Deprecation of messages
"""""""""""""""""""""""

Although discouraged for a long time we have officially deprecated the following messages in ``std_msgs``.
There are copies in `example_interfaces <https://index.ros.org/p/example_interfaces>`_

- ``std_msgs/msg/Bool``
- ``std_msgs/msg/Byte``
- ``std_msgs/msg/ByteMultiArray``
- ``std_msgs/msg/Char``
- ``std_msgs/msg/Float32``
- ``std_msgs/msg/Float32MultiArray``
- ``std_msgs/msg/Float64``
- ``std_msgs/msg/Float64MultiArray``
- ``std_msgs/msg/Int16``
- ``std_msgs/msg/Int16MultiArray``
- ``std_msgs/msg/Int32``
- ``std_msgs/msg/Int32MultiArray``
- ``std_msgs/msg/Int64``
- ``std_msgs/msg/Int64MultiArray``
- ``std_msgs/msg/Int8``
- ``std_msgs/msg/Int8MultiArray``
- ``std_msgs/msg/MultiArrayDimension``
- ``std_msgs/msg/MultiArrayLayout``
- ``std_msgs/msg/String``
- ``std_msgs/msg/UInt16``
- ``std_msgs/msg/UInt16MultiArray``
- ``std_msgs/msg/UInt32``
- ``std_msgs/msg/UInt32MultiArray``
- ``std_msgs/msg/UInt64``
- ``std_msgs/msg/UInt64MultiArray``
- ``std_msgs/msg/UInt8``
- ``std_msgs/msg/UInt8MultiArray``

Security features
^^^^^^^^^^^^^^^^^

Use of security enclaves
""""""""""""""""""""""""

As of Foxy, domain participants are no longer mapped directly to ROS nodes.
As a result, ROS 2 security features (which are specific to domain participants) are also no longer mapped directly to ROS nodes.
Instead, Foxy introduces the concept of a security "enclave", where an "enclave" is a process or group of processes that will share the same identity and access control rules.

This means that security artifacts are **not** retrieved based on the node name anymore but based on the Security enclave name.
A node enclave name can be set by using the ROS argument ``--enclave``, e.g. ``ros2 run demo_nodes_py talker --ros-args --enclave /my_enclave``

Related design document: https://github.com/ros2/design/pull/274

Note that permissions files are limited by the underlying transport packet size, so grouping many permissions under the same enclave will **not** work if the resulting permissions file exceed 64kB.
Related issue `[ros2/sros2#228] <https://github.com/ros2/sros2/issues/228>`_

Renaming of the environment variables
"""""""""""""""""""""""""""""""""""""

.. list-table:: Environment variables renaming
   :widths: 25 25
   :header-rows: 1

   * - Name in Eloquent
     - Name in Foxy
   * - ROS_SECURITY_ROOT_DIRECTORY
     - ROS_SECURITY_KEYSTORE
   * - ROS_SECURITY_NODE_DIRECTORY
     - ROS_SECURITY_ENCLAVE_OVERRIDE


Known Issues
------------

* `[ros2/ros2#922] <https://github.com/ros2/ros2/issues/922>`_ Services' performance is flaky for ``rclcpp`` nodes using eProsima Fast-RTPS or ADLINK CycloneDDS as RMW implementation.
  Specifically, service clients sometimes do not receive the response from servers.


Timeline before the release
---------------------------

A few milestones leading up to the release:

    .. note::

      The dates below reflect an extension by roughly two weeks due to the coronavirus pandemic.

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
