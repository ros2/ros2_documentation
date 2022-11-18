.. _upcoming-release:

.. _iron-release:

.. move this directive when next release page is created

ROS 2 Iron Irwini (codename 'iron'; May, 2023)
==============================================

.. contents:: Table of Contents
   :depth: 2
   :local:

*Iron Irwini* is the ninth release of ROS 2.
What follows is highlights of the important changes and features in Iron Irwini since the last release.

Supported Platforms
-------------------

Iron Irwini is primarily supported on the following platforms:

Tier 1 platforms:

TBD

Tier 2 platforms:

TBD

Tier 3 platforms:

TBD

For more information about RMW implementations, compiler / interpreter versions, and system dependency versions see `REP 2000 <https://www.ros.org/reps/rep-2000.html>`__.

Installation
------------

To come.

New features in this ROS 2 release
----------------------------------

ros2topic
^^^^^^^^^

``now`` as keyword for ``builtin_interfaces.msg.Time`` and ``auto`` for ``std_msgs.msg.Header``
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
``ros2 topic pub`` now allows to set a ``builtin_interfaces.msg.Time`` message to the current time via the ``now`` keyword.
Similarly, a ``std_msg.msg.Header`` message will be automatically generated when passed the keyword ``auto``.
This behavior matches that of ROS 1's ``rostopic`` (http://wiki.ros.org/ROS/YAMLCommandLine#Headers.2Ftimestamps)

Related PR: `ros2/ros2cli#749 <https://github.com/ros2/ros2cli/pull/749>`_

Changes since the Humble release
----------------------------------

``launch_ros``
^^^^^^^^^^^^^^

Renamed classes which used ``Ros`` in the name to use ``ROS`` in line with PEP8
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Classes that were changed:

* ``launch.actions.RosTimer`` -> ``launch.actions.ROSTimer``
* ``launch.actions.PushRosNamespace`` -> ``launch.actions.PushROSNamespace``

The old class names are still there, but will be deprecated.

Change to the default console logging file flushing behavior
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This specifically applies to the default ``spdlog`` based logging backend in ROS 2, implemented in ``rcl_logging_spdlog``.
Log file flushing was changed to flush every time an "error" log message is used, e.g. each `RCLCPP_ERROR()` call, and also periodically every five seconds.

Previously, ``spdlog`` was used without configuring anything other than creating the sink for logging to a file.

We tested the change and did not find that the CPU overhead was significant, even on machines with slow disks (e.g. sd cards).
However, if this change is causing you problems, you can get the old behavior by setting the ``RCL_LOGGING_SPDLOG_EXPERIMENTAL_OLD_FLUSHING_BEHAVIOR=1`` environment variable.

Later we would like to have support for a full configuration file, giving you more flexibility in how the logging is done, but that is work that is only planned right now.
Therefore, this environment variable should be considered experimental and subject to removal without deprecation in the future, when we add config file support for the ``rcl_logging_spdlog`` logging backend.

See this issue for more details: https://github.com/ros2/rcl_logging/pull/95

Known Issues
------------

To come.

Release Timeline
----------------

    November, 2022 - Platform decisions
        REP 2000 is updated with the target platforms and major dependency versions.

    By January, 2023 - Rolling platform shift
        Build farm is updated with the new platform versions and dependency versions for Iron Irwini (if necessary).

    Mon. April 10, 2023 - Alpha + RMW freeze
        Preliminary testing and stabilization of ROS Base [1]_ packages, and API and feature freeze for RMW provider packages.

    Mon. April 17, 2023 - Freeze
        API and feature freeze for ROS Base [1]_ packages in Rolling Ridley.
        Only bug fix releases should be made after this point.
        New packages can be released independently.

    Mon. April 24, 2023 - Branch
        Branch from Rolling Ridley.
        ``rosdistro`` is reopened for Rolling PRs for ROS Base [1]_ packages.
        Iron development shifts from ``ros-rolling-*`` packages to ``ros-iron-*`` packages.

    Mon. May 1, 2023 - Beta
        Updated releases of ROS Desktop [2]_ packages available.
        Call for general testing.

    Mon. May 15, 2023 - Release Candidate
        Release Candidate packages are built.
        Updated releases of ROS Desktop [2]_ packages available.

    Thu. May 18, 2023 - Distro Freeze
        Freeze rosdistro.
        No PRs for Iron on the ``rosdistro`` repo will be merged (reopens after the release announcement).

    Tue. May 23, 2023 - General Availability
        Release announcement.
        ``rosdistro`` is reopened for Iron PRs.

.. [1] The ``ros_base`` variant is described in `REP 2001 (ros-base) <https://www.ros.org/reps/rep-2001.html#ros-base>`_.
.. [2] The ``desktop`` variant is described in `REP 2001 (desktop-variants) <https://www.ros.org/reps/rep-2001.html#desktop-variants>`_.

Development progress
--------------------

For progress on the development and release of Iron Irwini, see `the tracking GitHub issue <https://github.com/ros2/ros2/issues/1298>`__.

For the broad process followed by Iron Irwini, see the :doc:`process description page <Release-Process>`.
