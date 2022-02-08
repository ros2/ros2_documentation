.. redirect-from::

    Releasing-a-ROS-2-package-with-bloom
    Guides/Releasing-a-ROS-2-package-with-bloom
    Tutorials/Releasing-a-ROS-2-package-with-bloom

Releasing a ROS 2 package with bloom
====================================

.. contents:: Table of Contents
   :depth: 2
   :local:

Introduction
------------

This page describes how to prepare a repository for release on the public ROS 2 buildfarm. After you've created a package, this is the next step towards getting your package in to the publicly-available Debian packages (i.e., you will be able to install the package via ``apt``). This page includes the ROS 2-specific instructions to execute before following the `Bloom release tutorial on the ROS Wiki <https://wiki.ros.org/bloom/Tutorials/FirstTimeRelease>`__.

Required Tools
--------------

* ``bloom`` >= 0.10.7
* ``catkin_pkg`` >= 0.4.23

Ensure that you have the latest version of bloom and catkin_pkg
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

See above version requirements.


*
  Make sure you have the ros repositories in your sources (see instructions :ref:`here <linux-install-debians-setup-sources>`).

*
  Install the latest version of bloom and catkin_pkg:

  .. code-block:: bash

     sudo apt install python3-catkin-pkg python3-bloom

If you're using a version of bloom older than 0.6.8 you'll need to use the v3 index url for releasing.

.. code-block:: bash

   export ROSDISTRO_INDEX_URL='https://raw.githubusercontent.com/ros/rosdistro/master/index.yaml'

Minor differences from ROS 1 Bloom
----------------------------------

If you've bloomed packages before in ROS 1, the ROS 2 process should be familiar to you.
The major difference is that release repositories for ROS 2 packages live in a dedicated github organization:
`ROS 2 release repositories <https://github.com/ros2-gbp/>`.
In order to take advantage of new automation supporting the `Rolling distribution <../Releases/Release-Rolling-Ridley>` Open Robotics recommends that release repositories for ROS 2 are hosted in the dedicated `ros2-gbp GitHub organization <https://github.com/ros2-gbp/>`.

Release repositories hosted elsewhere are still supported for stable distributions if you are not planning to release the repository into Rolling.
Since stable distributions created from Rolling will start with release repositories in the ros2-gbp organization it is recommend that you use the ros2-gbp release repositories for all ROS 2 distributions to avoid fragmenting the release information.

A ros2-gbp release repository may become a hard requirement for Rolling in the future and maintaining a single release repository for all ROS 2 distributions simplifies the maintenance of releases for both the Rolling distribution maintainers and package maintainers.

In the meantime, a new workflow
`based on Terraform <https://discourse.ros.org/t/upcoming-changes-to-ros2-gbp-organization/21878/4>`
is in the making. Stay tuned!

Procedure
---------

Same as in ROS 1: `Following this tutorial <https://wiki.ros.org/bloom/Tutorials/FirstTimeRelease>`__

If porting a ROS 1 package to ROS 2, it's recommended to create a new ``-release`` repository.

Build Status
------------

* Individual build details on the build farm `Jenkins <http://build.ros2.org/>`__ frontend.
* The `ROS 2 Package Status Pages <http://repo.ros2.org/status_page/>`__ (e.g. `Galactic-Default <http://repo.ros2.org/status_page/ros_galactic_default.html>`__).
