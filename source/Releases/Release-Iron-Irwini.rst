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

To come.

Changes since the Humble release
----------------------------------

To come.

Known Issues
------------

To come.

Release Timeline
----------------

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
