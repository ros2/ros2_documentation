.. _PlatformEOLPolicy:

Platform EOL Policy
===================

.. contents:: Table of Contents
   :depth: 1
   :local:

Every :doc:`ROS distribution <Releases>` has **target platforms**, also known as operating systems, such as Windows 11 or Ubuntu 24.04.
This page explains what happens when a target platform reaches its own end-of-life (EOL) before the ROS distribution reaches EOL.

What to expect
--------------

When a target platform reaches EOL:

* All ROS distributions immediately drop support for that platform.
* Existing ROS packages will remain available and functional, but they will no longer be updated.

When a target platform reaches EOL, it stops receiving critical bug and security fixes.
To protect ourselves from potentially unpatched security vulnerabilities, we proactively drop all EOL platform job runners from the ROS build farm . 

If you're using a target platform that has reached EOL, you should expect to stop receiving updated ROS packages.
However, ROS Bosses may choose to perform one last update on a target platform after it reaches EOL.

For ROS Bosses
--------------

Before a target platform reaches EOL:

* Make sure the ROS distribution documentation includes an EOL date for any target platform that reaches EOL before the ROS distribution.
* Post an announcement about the target platform reaching EOL at least 2 syncs beforehand so that package maintainers have time to update their packages.
* Open a `pull request disabling buildfarm jobs for that platform <https://github.com/ros2/ros_buildfarm_config>`_ and seek review from the `Infrastructure PMC <https://osralliance.org/wp-content/uploads/2024/03/infrastructure_project_charter.pdf>`_.
* Make one last release to that target platform.

After a target platform reaches EOL:

* Update ROS distribution to state the target platform will not receive updated ROS packages.
* Announce that the ROS distribution has dropped support for that target platform on Discourse.
* Consider making one last release to that target platform if:
    * You did not already do so prior to EOL, and
    * The updates seem unlikely to have regressions, and
    * The ROS Buildfarm still has runners for that target platform.
* Merge your pull request to disable buildfarm jobs.
