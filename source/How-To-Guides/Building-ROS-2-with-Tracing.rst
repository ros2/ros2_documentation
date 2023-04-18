.. redirect-from::

    How-To-Guides/Building-ROS-2-with-Tracing-Instrumentation

Building ROS 2 with tracing
===========================

.. contents:: Table of Contents
   :depth: 2
   :local:

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
See :doc:`the source installation page <../Installation/Alternatives/Ubuntu-Development-Setup>` for more information.

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

This step depends on whether you are :doc:`building ROS 2 from source <../Installation/Alternatives/Ubuntu-Development-Setup>` or using ROS 2 binaries (:doc:`Debian packages <../Installation/Ubuntu-Install-Debians>` or :doc:`"fat" archive <../Installation/Alternatives/Ubuntu-Install-Binary>`).
To remove the tracepoints, (re)build ``tracetools`` and set the ``TRACETOOLS_TRACEPOINTS_EXCLUDED`` CMake option to ``ON``:

.. tabs::

  .. group-tab:: Source installation

    .. code-block:: bash

       cd ~/ros2_{DISTRO}
       colcon build --packages-select tracetools --cmake-clean-cache --cmake-args -DTRACETOOLS_TRACEPOINTS_EXCLUDED=ON

  .. group-tab:: Binary installation

    Clone the ``ros2_tracing`` repository into your workspace and build:

    .. code-block:: bash

       cd ~/ws
       git clone https://github.com/ros2/ros2_tracing.git -b {DISTRO} src/ros2_tracing
       colcon build --packages-select tracetools --cmake-args -DTRACETOOLS_TRACEPOINTS_EXCLUDED=ON

Building without instrumentation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To completely remove both tracepoints and function calls, :doc:`build ROS 2 from source <../Installation/Alternatives/Ubuntu-Development-Setup>` and set the ``TRACETOOLS_DISABLED`` CMake option to ``ON``:

.. code-block:: bash

   cd ~/ros2_{DISTRO}
   colcon build --cmake-args -DTRACETOOLS_DISABLED=ON --no-warn-unused-cli

Validating
----------

Validate that tracing is disabled:

.. code-block:: bash

   cd ~/ws
   source install/setup.bash
   ros2 run tracetools status

It should print out:

.. tabs::

  .. group-tab:: Without tracepoints

    .. code-block:: bash

       Tracing disabled

  .. group-tab:: Without instrumentation

    .. code-block:: bash

       Tracing disabled through configuration

If something else is printed, then something went wrong.
