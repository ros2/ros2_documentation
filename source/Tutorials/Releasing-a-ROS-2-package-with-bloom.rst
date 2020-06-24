.. redirect-from::

    Releasing-a-ROS-2-package-with-bloom

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

* ``bloom`` >= 0.9.7
* ``catkin_pkg`` >= 0.4.10

Ensure that you have the latest version of bloom and catkin_pkg
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

See above version requirements.


*
  Make sure you have the ros repositories in your sources (see instructions `here <linux-install-debians-setup-sources>`).

*
  Install the latest version of bloom and catkin_pkg:

  .. code-block:: bash

     sudo apt install python-catkin-pkg python-bloom

If you're using a version of bloom older than 0.6.8 you'll need to use the v3 index url for releasing.

.. code-block:: bash

   export ROSDISTRO_INDEX_URL='https://raw.githubusercontent.com/ros/rosdistro/master/index.yaml'

No differences from ROS 1 Bloom
-------------------------------

If you've bloomed packages before in ROS 1, the ROS 2 process is exactly the same.

Procedure
---------

Same as in ROS 1: `Following this tutorial <https://wiki.ros.org/bloom/Tutorials/FirstTimeRelease>`__

If porting a ROS 1 package to ROS 2, it's recommended to create a new ``-release`` repository.

Build Status
------------

* Individual build details on the build farm `Jenkins <http://build.ros2.org/>`__ frontend.
* The `ROS 2 Package Status Pages <http://repo.ros2.org/status_page/>`__ (e.g. `Bouncy-Default <http://repo.ros2.org/status_page/ros_bouncy_default.html>`__).
