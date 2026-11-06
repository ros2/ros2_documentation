.. redirect-from::

    How-To-Guides/Building-ROS-2-with-Tracing-Instrumentation
    How-To-Guides/Building-ROS-2-with-Tracing

.. meta::
   :contentType: how-to
   :experience: intermediate
   :area: debugging, tools
   :distribution: {DISTRO}
   :product: {PRODUCT}

Building ROS 2 with tracing - how-to
====================================

.. short-description::
   Tracing instrumentation is included in ROS by default on Linux, but some systems need builds without tracepoint overhead or instrumentation.
   This article describes how to rebuild ROS components with tracing tracepoints excluded or tracing instrumentation disabled.
   After following these steps, you will be able to validate the tracing configuration for your workspace.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. contents:: Table of Contents
   :depth: 2
   :local:

Summary
-------

Linux ROS installations include tracing support and can be traced out-of-the-box.

Tracing can be removed at two levels:

* To remove tracepoints, rebuild ``tracetools`` with ``-DTRACETOOLS_TRACEPOINTS_EXCLUDED=ON``.

* For binary installations, clone ``ros2_tracing`` into the workspace before rebuilding ``tracetools``.

* To remove both tracepoints and instrumentation function calls, build ROS from source with ``-DTRACETOOLS_DISABLED=ON``.

Validate the result with ``ros2 run tracetools status``.

Tracing instrumentation is included in the ROS 2 source code, and Linux installations of ROS 2 include the LTTng tracer as a dependency.
Therefore, ROS 2 can be traced out-of-the-box on Linux.

However, ROS 2 can be built from source to remove the tracepoints or completely remove the instrumentation.
This guide shows how to do that.
For more information, see `the repository <https://github.com/ros2/ros2_tracing>`__.

.. note::

   This guide only applies to Linux systems.

Prerequisites
-------------

Set up your system to build ROS 2 from source.
See :doc:`the source installation page <../../Get-Started/Installation/Alternatives/Ubuntu-Development-Setup>` for more information.

Build configurations
--------------------

The ROS 2 tracing instrumentation is split into two components: function instrumentation and tracepoints.
First, a ROS 2 core package (e.g., ``rclcpp``) calls a function provided by the ``tracetools`` package.
Then, that function triggers a tracepoint, which records data if the tracepoint is enabled at runtime.

By default, if the tracer is not `configured to trace or if the tracepoints are not enabled <https://github.com/ros2/ros2_tracing#tracing>`__, they will have virtually no impact on the execution.
However, the tracepoints can still be removed through a CMake option.
Furthermore, the functions can be completely removed through a CMake option, which implies that tracepoints are also removed.

Building without tracepoints
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This step depends on whether you are :doc:`building ROS 2 from source <../../Get-Started/Installation/Alternatives/Ubuntu-Development-Setup>` or using ROS 2 binaries (:doc:`deb packages <../../Get-Started/Installation/Ubuntu-Install-Debs>` or :doc:`binary archive <../../Get-Started/Installation/Alternatives/Ubuntu-Install-Binary>`).
To remove the tracepoints, (re)build ``tracetools`` and set the ``TRACETOOLS_TRACEPOINTS_EXCLUDED`` CMake option to ``ON``:

.. tabs::

  .. group-tab:: Source installation

    .. code-block:: console

       $ cd ~/ros2_{DISTRO}
       $ colcon build --packages-select tracetools --cmake-clean-cache --cmake-args -DTRACETOOLS_TRACEPOINTS_EXCLUDED=ON

  .. group-tab:: Binary installation

    Clone the ``ros2_tracing`` repository into your workspace and build:

    .. code-block:: console

       $ cd ~/ws
       $ git clone https://github.com/ros2/ros2_tracing.git -b {DISTRO} src/ros2_tracing
       $ colcon build --packages-select tracetools --cmake-args -DTRACETOOLS_TRACEPOINTS_EXCLUDED=ON

Building without instrumentation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To completely remove both tracepoints and function calls, :doc:`build ROS 2 from source <../../Get-Started/Installation/Alternatives/Ubuntu-Development-Setup>` and set the ``TRACETOOLS_DISABLED`` CMake option to ``ON``:

.. code-block:: console

   $ cd ~/ros2_{DISTRO}
   $ colcon build --cmake-args -DTRACETOOLS_DISABLED=ON --no-warn-unused-cli

Validating
----------

Validate that tracing is disabled:

.. code-block:: console

   $ cd ~/ws
   $ source install/setup.bash
   $ ros2 run tracetools status

It should print out:

.. tabs::

  .. group-tab:: Without tracepoints

    .. code-block:: bash

       Tracing disabled

  .. group-tab:: Without instrumentation

    .. code-block:: bash

       Tracing disabled through configuration

If something else is printed, then something went wrong.
