
Introduction
------------

This page describes how to prepare a repository for release on the public ROS 2 buildfarm. After you've created a package, this is the next step towards getting your package in to the publicly-available Debian packages (i.e., you will be able to install the package via ``apt``\ ). This page includes the ROS 2-specific instructions to execute before following the `Bloom release tutorial on the ROS Wiki <http://wiki.ros.org/bloom/Tutorials/FirstTimeRelease>`__.

Required Tools
^^^^^^^^^^^^^^

For ROS 2 Bouncy:


* ``bloom`` >= 0.6.6
* ``catkin_pkg`` >= 0.4.5

for ROS 2 Crystal:


* ``bloom`` >= 0.6.9
* ``catkin_pkg`` >= 0.4.5


Ensure that you have the latest version of bloom and catkin_pkg
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

See above version requirements.


*
  Make sure you have the ros repositories in your sources (see `Instructions <linux-install-debians-setup-sources>`\ )

*
  Install the latest version of bloom and catkin_pkg:

  .. code-block:: bash

     sudo apt install python-catkin-pkg python-bloom

Differences from ROS 1 Bloom
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you've bloomed packages before in ROS 1, ROS 2's requirements will look familiar.

ROS 2 uses a forked rosdistro index located at https://github.com/ros2/rosdistro.
You can configure bloom to use it by setting the ``ROSDISTRO_INDEX_URL`` environment variable.

.. code-block:: bash

   export ROSDISTRO_INDEX_URL='https://raw.githubusercontent.com/ros2/rosdistro/ros2/index-v4.yaml'

If you're using a version of bloom older than 0.6.8 you'll need to use the v3 index url for releasing into Bouncy.

.. code-block:: bash

   export ROSDISTRO_INDEX_URL='https://raw.githubusercontent.com/ros2/rosdistro/ros2/index.yaml'


Procedure
---------

Same as in ROS 1: `Following this tutorial <http://wiki.ros.org/bloom/Tutorials/FirstTimeRelease>`__

If porting a ROS 1 package to ROS 2, it's recommended to create a new ``-release`` repository.

Build Status
------------

* Individual build details on the build farm `Jenkins <http://build.ros2.org/>`__ frontend.
* The `ROS2 Package Status Pages <http://repo.ros2.org/status_page/>`__ (e.g. `Bouncy-Default <http://repo.ros2.org/status_page/ros_bouncy_default.html>`__).

