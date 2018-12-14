
Installing ROS2 via Debian Packages
===================================

Debian packages for ROS 2 Bouncy (the latest release) are available for Ubuntu Bionic; packages for ROS 2 Ardent are available for Ubuntu Xenial.

Resources:


* `Jenkins Instance <http://build.ros2.org/>`__
* `Repositories <http://repo.ros2.org>`__
* Status Pages:

  * ROS 2 Crystal (Ubuntu Bionic): `amd64 <http://repo.ros2.org/status_page/ros_crystal_default.html>`__\ , `arm64 <http://repo.ros2.org/status_page/ros_crystal_ubv8.html>`__
  * ROS 2 Bouncy (Ubuntu Bionic): `amd64 <http://repo.ros2.org/status_page/ros_bouncy_default.html>`__\ , `arm64 <http://repo.ros2.org/status_page/ros_bouncy_ubv8.html>`__
  * ROS 2 Ardent (Ubuntu Xenial): `amd64 <http://repo.ros2.org/status_page/ros_ardent_default.html>`__\ , `arm64 <http://repo.ros2.org/status_page/ros_ardent_uxv8.html>`__

.. _linux-install-debians-setup-sources:

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

To install the Debian packages you will need to add our Debian repository to your apt sources.
First you will need to authorize our gpg key with apt like this:

.. code-block:: bash

   sudo apt update && sudo apt install curl gnupg2 lsb-release
   curl http://repo.ros2.org/repos.key | sudo apt-key add -

And then add the repository to your sources list:

.. code-block:: bash

   sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

Install ROS 2 packages
----------------------

First set an environment variable for the ROS 2 release you want to install so it can be used in other commands.

.. code-block:: bash

   export ROS_DISTRO=crystal  # or bouncy or ardent
   sudo apt update

Desktop Install (Recommended): ROS, RViz, demos, tutorials.

.. code-block:: bash

   sudo apt install ros-$ROS_DISTRO-desktop

ROS-Base Install (Bare Bones): Communication libraries, message packages, command line tools. No GUI tools.

.. code-block:: bash

   sudo apt install ros-$ROS_DISTRO-ros-base

See specific sections below for how to also install the ros1_bridge, TurtleBot packages, or alternative RMW packages.

Environment setup
-----------------

(optional) Install argcomplete
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ROS 2 command line tools use argcomplete to autocompletion. So if you want autocompletion, installing argcomplete is necessary.

Ubuntu 18.04
~~~~~~~~~~~~

.. code-block:: bash

   sudo apt install python3-argcomplete

Ubuntu 16.04 (argcomplete >= 0.8.5)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To install ``argcomplete`` on Ubuntu 16.04 (Xenial), you'll need to use pip, because the version available through ``apt`` will not work due to a bug in that version of ``argcomplete``\ :

.. code-block:: bash

   sudo apt install python3-pip
   sudo pip3 install argcomplete

Sourcing the setup script
^^^^^^^^^^^^^^^^^^^^^^^^^

Set up your environment by sourcing the following file (you may want to add this to your ``.bashrc``\ ).

.. code-block:: bash

   source /opt/ros/$ROS_DISTRO/setup.bash

Installing additional RMW implementations
-----------------------------------------

By default the RMW implementation ``FastRTPS`` is used.
If using Ardent OpenSplice is also installed.

To install support for OpenSplice or RTI Connext on Bouncy:

.. code-block:: bash

   sudo apt update
   sudo apt install ros-$ROS_DISTRO-rmw-opensplice-cpp # for OpenSplice
   sudo apt install ros-$ROS_DISTRO-rmw-connext-cpp # for RTI Connext (requires license agreement)

By setting the environment variable ``RMW_IMPLEMENTATION=rmw_opensplice_cpp`` you can switch to use OpenSplice instead.
For ROS 2 releases Bouncy and newer, ``RMW_IMPLEMENTATION=rmw_connext_cpp`` can also be selected to use RTI Connext.

If you want to install the Connext DDS-Security plugins please refer to `this page <Install-Connext-Security-Plugins>`

Additional packages using ROS 1 packages
----------------------------------------

The ``ros1_bridge`` as well as the TurtleBot demos are using ROS 1 packages.
To be able to install them please start by adding the ROS 1 sources as documented `here <http://wiki.ros.org/Installation/Ubuntu?distro=melodic>`__.

If you're using Docker for isolation you can start with the image ``ros:melodic`` or ``osrf/ros:melodic-desktop`` (or Kinetic if using Ardent).
This will also avoid the need to setup the ROS sources as they will already be integrated.

Now you can install the remaining packages:

.. code-block:: bash

   sudo apt update
   sudo apt install ros-$ROS_DISTRO-ros1-bridge ros-$ROS_DISTRO-turtlebot2-*
