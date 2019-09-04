
ROS 2 Eloquent Elusor (codename 'eloquent'; November 22nd, 2019)
================================================================

.. contents:: Table of Contents
   :depth: 2
   :local:

*Eloquent Elusor* is the fifth release of ROS 2.

Supported Platforms
-------------------

Eloquent Elusor is primarily supported on the following platforms:

Tier 1 platforms:

* Ubuntu 18.04 (Bionic): ``amd64`` and ``arm64``
* Mac OS X 10.12 (Sierra)
* Windows 10 (Visual Studio 2019)

Tier 2 platforms:

* Ubuntu 18.04 (Bionic): ``arm32``

Tier 3 platforms:

* Debian Stretch (9): ``amd64``, ``arm64`` and ``arm32``
* OpenEmbedded Thud (2.6) / webOS OSE: ``arm32`` and ``x86``

For more information about RMW implementations, compiler / interpreter versions, and system dependency versions see `REP 2000 <http://www.ros.org/reps/rep-2000.html>`__.


New features in this ROS 2 release
----------------------------------

During the development the `Eloquent meta ticket <https://github.com/ros2/ros2/issues/734>`__ on GitHub contains an up-to-date state of the ongoing high level tasks as well as references specific tickets with more details.



Timeline before the release
---------------------------

A few milestones leading up to the release:

    Mon. Sep 30th (alpha)
        First releases of core packages available.
        Testing can happen from now on (some features might not have landed yet).

    Fri. Oct 18th
        API and feature freeze for core packages
        Only bug fix releases should be made after this point.
        New packages can be released independently.

    Thu. Oct 24th (beta)
        Updated releases of core packages available.
        Additional testing of the latest features.

    Wed. Nov 13th (release candidate)
        Updated releases of core packages available.

    Tue. Nov 19th
        Freeze rosdistro.
        No PRs for Eloquent on the `rosdistro` repo will be merged (reopens after the release announcement).

