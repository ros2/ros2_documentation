.. _upcoming-release:

.. _kilted-release:

Kilted Kaiju (codename 'kilted'; May, 2025)
===========================================

.. contents:: Table of Contents
   :depth: 2
   :local:

*Kilted Kaiju* is the eleventh release of ROS 2.
What follows is highlights of the important changes and features in Kilted Kaiju since the last release.

Supported Platforms
-------------------

Kilted Kaiju is primarily supported on the following platforms:

Tier 1 platforms:

* Ubuntu 24.04 (Noble): ``amd64`` and ``arm64``
* Windows 10 (Visual Studio 2019): ``amd64``

Tier 2 platforms:

* RHEL 9: ``amd64``

Tier 3 platforms:

* macOS: ``amd64``
* Debian Bookworm: ``amd64``

For more information about RMW implementations, compiler / interpreter versions, and system dependency versions see `REP 2000 <https://www.ros.org/reps/rep-2000.html>`__.

Installation
------------

TODO

New features in this ROS 2 release
----------------------------------

Development progress
--------------------

For progress on the development of Kiltled Kaiju, see `this project board <https://github.com/orgs/ros2/projects/63>`__.

For the broad process followed by Kilted Kaiju, see the :doc:`process description page <Release-Process>`.

Release Timeline
----------------

    December, 2024 - Platform decisions
        REP 2000 is updated with the target platforms and major dependency versions.

    Mon. April 7, 2025 - Alpha + RMW freeze
        Preliminary testing and stabilization of ROS Base [1]_ packages, and API and feature freeze for RMW provider packages.

    Mon. April 14, 2024 - Freeze
        API and feature freeze for ROS Base [1]_ packages in Rolling Ridley.
        Only bug fix releases should be made after this point.
        New packages can be released independently.

    Mon. April 21, 2024 - Branch
        Branch from Rolling Ridley.
        ``rosdistro`` is reopened for Rolling PRs for ROS Base [1]_ packages.
        Kilted development shifts from ``ros-rolling-*`` packages to ``ros-kilted-*`` packages.

    Mon. April 28, 2024 - Beta
        Updated releases of ROS Desktop [2]_ packages available.
        Call for general testing.

    Thu, May 1, 2024 - Kick off of Tutorial Party
        Tutorials hosted at https://github.com/osrf/ros2_test_cases are open for community testing.

    Mon. May 12, 2024 - Release Candidate
        Release Candidate packages are built.
        Updated releases of ROS Desktop [2]_ packages available.

    Mon. May 19, 2024 - Distro Freeze
        Freeze all Kilted branches on all `ROS 2 desktop packages <https://www.ros.org/reps/rep-2001.html#kilted-kaiju-may-2025-november-2026>`__ and ``rosdistro``.
        No pull requests for any ``kilted`` branch or targeting ``kilted/distribution.yaml`` in ``rosdistro`` repo will be merged.

    Fri. May 23, 2024 - General Availability
        Release announcement.
        `ROS 2 desktop packages <https://www.ros.org/reps/rep-2001.html#kilted-kaiju-may-2025-november-2026>`__ source freeze is lifted and ``rosdistro`` is reopened for Kilted pull requests.

.. [1] The ``ros_base`` variant is described in `REP 2001 (ros-base) <https://www.ros.org/reps/rep-2001.html#ros-base>`_.
.. [2] The ``desktop`` variant is described in `REP 2001 (desktop-variants) <https://www.ros.org/reps/rep-2001.html#desktop-variants>`_.
