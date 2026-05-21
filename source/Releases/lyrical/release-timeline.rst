Lyrical Luth Release Timeline
=============================

For progress on the development of Lyrical Luth, see `this project board <https://github.com/orgs/ros2/projects/70>`__.
For the broad process followed by Lyrical Luth, see the :doc:`process description page <../Release-Process>`.

**As soon as possible** - Migrate ROS Rolling to ROS Lyrical's target platforms
    * RHEL 10 + Ubuntu 26.04: Migrate as soon as core packages successfully build on both platforms.
    * Windows 11: Migrate as soon as we have a green build

**Mon. April 13, 2026** - Alpha + RMW freeze (*Delayed; was originally April 6th*)
    * Preliminary testing of ROS Base packages
    * API and feature freeze for RMW provider packages.

**Mon. April 20, 2026** - Freeze (*Delayed; was originally April 13th*)
    * API and feature freeze for ROS Base packages in Rolling Ridley.
    * Only bug fix releases should be made after this point.
    * New packages can be released.

**Mon. April 21, 2026** - Branch (*Delayed; was originally April 20th*)
    * Branch from Rolling Ridley
    * ``rosdistro`` is reopened for Rolling PRs for ROS Base packages.
    * Lyrical development shifts from ``ros-rolling-*`` packages to ``ros-lyrical-*`` packages.

**Mon. April 27, 2026** - Beta
    * Updated releases of ROS Desktop packages available.
    * Call for general testing.

**Thu, April 30, 2026** - Kick off Tutorial Party
    * Open up tutorials for community testing.

**Mon. May 11, 2026** - Release Candidate
    * Build release candidate packages up to ROS Desktop

**Mon. May 18, 2026** - Distro Freeze
    * Freeze all Lyrical branches on all ROS desktop packages
    * No pull requests for any Lyrical branch or targeting ``lyrical/distribution.yaml`` in ``rosdistro`` repo will be merged.

**Friday May 22nd, 2026** - General Availability
    * Release announcement.
    * ROS desktop packages source freeze is lifted and ``rosdistro`` is reopened for Lyrical pull requests.

**May, 2031** - End-of-life
    * ROS Lyrical will stop receiving updates - including security updates
