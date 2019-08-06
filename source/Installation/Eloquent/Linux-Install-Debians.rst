Installing ROS2 via Debian Packages
===================================

.. contents:: Table of Contents
   :depth: 2
   :local:

Debian packages for ROS 2 Dashing Diademata are available for Ubuntu Bionic.

Resources
---------

* Status Page:

  * ROS 2 Dashing (Ubuntu Bionic): `amd64 <http://repo.ros2.org/status_page/ros_dashing_default.html>`__\ , `arm64 <http://repo.ros2.org/status_page/ros_dashing_ubv8.html>`__
* `Jenkins Instance <http://build.ros2.org/>`__
* `Repositories <http://repo.ros2.org>`__


Setup Locale
------------
Make sure you have a locale which supports ``UTF-8``.
If you are in a minimal environment, such as a docker container, the locale may be something minimal like POSIX.
We test with the following settings.
It should be fine if you're using a different UTF-8 supported locale.

.. code-block:: bash

   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8

Setup Sources
-------------

.. include:: ../_Apt-Repositories.rst

.. _Dashing_linux-install-debians-install-ros-2-packages:

Install ROS 2 packages
----------------------

Update your apt repository caches after setting up the repositories.

.. code-block:: bash

   sudo apt update

Desktop Install (Recommended): ROS, RViz, demos, tutorials.

.. code-block:: bash

   sudo apt install ros-dashing-desktop

ROS-Base Install (Bare Bones): Communication libraries, message packages, command line tools.
No GUI tools.

.. code-block:: bash

   sudo apt install ros-dashing-ros-base

See specific sections below for how to also install the :ref:`ros1_bridge <Dashing_linux-ros1-add-pkgs>`, :ref:`TurtleBot packages <Dashing_linux-ros1-add-pkgs>`, or :ref:`alternative RMW packages <Dashing_linux-install-additional-rmw-implementations>`.

Environment setup
-----------------

(optional) Install argcomplete
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ROS 2 command line tools use argcomplete to autocompletion.
So if you want autocompletion, installing argcomplete is necessary.

.. code-block:: bash

   sudo apt install python3-argcomplete


Sourcing the setup script
^^^^^^^^^^^^^^^^^^^^^^^^^

Set up your environment by sourcing the following file.

.. code-block:: bash

   source /opt/ros/dashing/setup.bash

You may want to add this to your ``.bashrc``.

.. code-block:: bash

   echo "source /opt/ros/dashing/setup.bash" >> ~/.bashrc

.. _Dashing_linux-install-additional-rmw-implementations:

Install additional RMW implementations
--------------------------------------

By default the RMW implementation ``FastRTPS`` is used.
If using Ardent OpenSplice is also installed.

To install support for OpenSplice or RTI Connext on Bouncy:

.. code-block:: bash

   sudo apt update
   sudo apt install ros-dashing-rmw-opensplice-cpp # for OpenSplice
   sudo apt install ros-dashing-rmw-connext-cpp # for RTI Connext (requires license agreement)

By setting the environment variable ``RMW_IMPLEMENTATION=rmw_opensplice_cpp`` you can switch to use OpenSplice instead.
For ROS 2 releases Bouncy and newer, ``RMW_IMPLEMENTATION=rmw_connext_cpp`` can also be selected to use RTI Connext.

If you want to install the Connext DDS-Security plugins please refer to `this page <../Install-Connext-Security-Plugins>`.

.. _Dashing_linux-ros1-add-pkgs:

`University, purchase or evaluation <../Install-Connext-University-Eval>` options are also available for RTI Connext.

Install additional packages using ROS 1 packages
------------------------------------------------

The ``ros1_bridge`` as well as the TurtleBot demos are using ROS 1 packages.
To be able to install them please start by adding the ROS 1 sources as documented `here <http://wiki.ros.org/Installation/Ubuntu?distro=melodic>`__.

If you're using Docker for isolation you can start with the image ``ros:melodic`` or ``osrf/ros:melodic-desktop`` (or Kinetic if using Ardent).
This will also avoid the need to setup the ROS sources as they will already be integrated.

Now you can install the remaining packages:

.. code-block:: bash

   sudo apt update
   sudo apt install ros-dashing-ros1-bridge

The turtlebot2 packages are not currently available in Dashing.

Build your own packages
-----------------------

If you would like to build your own packages, refer to the tutorial `"Using Colcon to build packages" </Tutorials/Colcon-Tutorial>`.
