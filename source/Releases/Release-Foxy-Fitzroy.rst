.. _upcoming-release:

.. move this directive when next release page is created

ROS 2 Foxy Fitzroy (codename 'foxy'; May 23rd, 2020)
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
* Mac OS X 10.14 (Mojave)
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

rclpy
^^^^^

Support for multiple on parameter set callbacks
"""""""""""""""""""""""""""""""""""""""""""""""

Use the ``Node`` methods ``add_on_set_parameters_callback`` and ``remove_on_set_parameters_callback`` for adding and removing functions that are called when parameters are set.

The method ``set_parameters_calblack`` has been deprecated.

Related pull requests: https://github.com/ros2/rclpy/pull/457, https://github.com/ros2/rclpy/pull/504

rviz
^^^^

Tools timestamp messages using ROS time
"""""""""""""""""""""""""""""""""""""""

'2D Pose Estimate', '2D Nav Goal', and 'Publish Point' tools now timestamp their messages using ROS time instead of system time, in order for the `use_sim_time` parameter to have an effect on them.

Related pull request: https://github.com/ros2/rviz/pull/519

Timeline before the release
---------------------------

A few milestones leading up to the release:

    Wed. April 8th, 2020
        API and feature freeze for ``ros_core`` [1]_ packages.
        Note that this includes ``rmw``, which is a recursive dependency of ``ros_core``.
        Only bug fix releases should be made after this point.
        New packages can be released independently.

    Mon. April 13th, 2020 (beta)
        Updated releases of ``desktop`` [2]_ packages available.
        Testing of the new features.

    Wed. May 13th, 2020 (release candidate)
        Updated releases of ``desktop`` [2]_ packages available.

    Wed. May 20, 2020
        Freeze rosdistro.
        No PRs for Foxy on the `rosdistro` repo will be merged (reopens after the release announcement).

.. [1] The ``ros_core`` variant described in the `variants <https://github.com/ros2/variants>`_ repository.
.. [2] The ``desktop`` variant described in the `variants <https://github.com/ros2/variants>`_ repository.
