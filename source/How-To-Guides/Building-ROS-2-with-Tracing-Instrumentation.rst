Building ROS 2 with tracing instrumentation
===========================================

.. contents:: Table of Contents
   :depth: 2
   :local:

This guide shows you how to build ROS 2 with the tracing instrumentation provided by ``ros2_tracing``.
For more information, see `the repository <https://github.com/ros2/ros2_tracing>`__.

Instrumentation is included in the ROS 2 source code.
However, if using the binaries or when building from source, the instrumentation does not actually trigger tracepoints by default.
To get the tracepoints, the LTTng tracer needs to be installed, and then part of ROS 2 needs to be (re)built from source.

.. note::

   This guide only applies to Linux systems and assumes that Ubuntu is used.

Prerequisites
-------------

Set up your system to build ROS 2 from source.
See :doc:`the source installation page <../Installation/Alternatives/Ubuntu-Development-Setup>` for more information.

Installing the tracer
---------------------

Install the `LTTng tracer <https://lttng.org/docs>`__ and related tools and dependencies.

.. code-block:: bash

   sudo apt-get update
   sudo apt-get install -y lttng-tools liblttng-ust-dev python3-lttng python3-babeltrace babeltrace

This only installs the LTTng userspace tracer, and not the LTTng kernel tracer, since it is not needed to trace ROS 2 applications.

Building
--------

This step depends on whether you are building ROS 2 from source or using ROS 2 binaries.

With source installation
^^^^^^^^^^^^^^^^^^^^^^^^

If you have already :doc:`built ROS 2 from source <../Installation/Alternatives/Ubuntu-Development-Setup>` before installing LTTng, you will need to re-build at least up to the ``tracetools`` package:

.. code-block:: bash

   cd ~/ws
   colcon build --packages-up-to tracetools --cmake-force-configure

With binary installation
^^^^^^^^^^^^^^^^^^^^^^^^

If you rely on the ROS 2 binaries (:doc:`Debian packages <../Installation/Ubuntu-Install-Debians>` or :doc:`"fat" archive <../Installation/Alternatives/Ubuntu-Install-Binary>`), you will need to clone the ``ros2_tracing`` repository into your workspace and build at least up to the ``tracetools`` package:

.. code-block:: bash

   cd ~/ws/src
   git clone https://github.com/ros2/ros2_tracing.git
   cd ../
   colcon build --packages-up-to tracetools

Validating
----------

Source and validate that tracing is enabled:

.. code-block:: bash

   cd ~/ws
   source install/setup.bash
   ros2 run tracetools status

It should print out:

.. code-block:: bash

   Tracing enabled

If something else is printed, then something went wrong.

Disabling tracing
-----------------

If the LTTng userspace tracer is installed and found when building ``tracetools``, tracing will be automatically enabled.
Alternatively, to build and completely remove both the tracepoints and the tracing instrumentation from ROS 2, set the ``TRACETOOLS_DISABLED`` CMake option to ``ON``:

.. code-block:: bash

   colcon build --cmake-args -DTRACETOOLS_DISABLED=ON --no-warn-unused-cli
