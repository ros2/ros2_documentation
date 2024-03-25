Using ``ros1_bridge`` with upstream ROS on Ubuntu 22.04
=======================================================

.. contents:: Table of Contents
   :depth: 1
   :local:

The release of ROS 2 Humble (and Rolling) on Ubuntu 22.04 Jammy Jellyfish marks the first ROS 2 release on a platform with no official ROS 1 release.
While ROS 1 Noetic will continue to be supported through the duration of its `long term support window <https://www.ros.org/reps/rep-0003.html#noetic-ninjemys-may-2020-may-2025>`__, it will only target Ubuntu 20.04.
Alternatively, there are `upstream variants of ROS 1 packages <https://packages.ubuntu.com/jammy/ros-desktop>`__ in Debian and Ubuntu that are not maintained as an official distribution by the ROS maintainers.

This guide outlines the current mechanism for bridging ROS 2 releases with these upstream packages on Ubuntu 22.04 Jammy Jellyfish.
This provides a migration path for users who still depend on ROS 1, but desire moving to newer ROS 2 and Ubuntu releases.

ROS 2 via Debian packages
-------------------------

Installing :doc:`ROS 2 from Debian packages <../Installation/Ubuntu-Install-Debians>` currently does not work for ROS 2 on Ubuntu Jammy.
The version of ``catkin-pkg-modules`` available in the Ubuntu repository conflicts with that in the ROS 2 package repository.

If the ROS 2 apt repository is in the available apt repositories (``/etc/apt/sources.list.d``), no ROS 1 packages will be installable.
The error will be:

.. code-block:: bash

  $ apt install ros-core-dev
  Reading package lists... Done
  Building dependency tree... Done
  Reading state information... Done
  Some packages could not be installed. This may mean that you have
  requested an impossible situation or if you are using the unstable
  distribution that some required packages have not yet been created
  or been moved out of Incoming.
  The following information may help to resolve the situation:

  The following packages have unmet dependencies:
   ros-core-dev : Depends: catkin but it is not installable
  E: Unable to correct problems, you have held broken packages.

To correct this, remove packages.ros.org from your ``sources.list``.
If you were following the ROS 2 installation guide, simply remove ``/etc/apt/sources.list.d/ros2.list``

For now, to support ``ros1_bridge``, follow the instructions below for building ROS 2 from source.

ROS 2 from source
-----------------

Installing :doc:`ROS 2 from Source <../Installation/Alternatives/Ubuntu-Development-Setup>` is the only configuration that works for ROS 2 on Ubuntu Jammy.

Below is a summary of the necessary instructions from the source build instructions.
The substantial deviation is that we skip using the ROS 2 apt repositories because of conflicting packages.

Install development tools and ROS tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Since we aren't using the ROS 2 apt repositories, ``colcon`` must be installed via ``pip``.

.. code-block:: bash

   sudo apt update && sudo apt install -y \
     build-essential \
     cmake \
     git \
     python3-flake8 \
     python3-flake8-blind-except \
     python3-flake8-builtins \
     python3-flake8-class-newline \
     python3-flake8-comprehensions \
     python3-flake8-deprecated \
     python3-flake8-docstrings \
     python3-flake8-import-order \
     python3-flake8-quotes \
     python3-pip \
     python3-pytest \
     python3-pytest-cov \
     python3-pytest-repeat \
     python3-pytest-rerunfailures \
     python3-rosdep \
     python3-setuptools \
     wget

   # Install colcon from PyPI, rather than apt packages
   python3 -m pip install -U colcon-common-extensions vcstool

From here, continue with the :doc:`source install guide <../Installation/Alternatives/Ubuntu-Development-Setup>` to build ROS 2.

Install ROS 1 from Ubuntu packages
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   sudo apt update && sudo apt install -y ros-core-dev


Build ``ros1_bridge``
^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    # Create a workspace for the ros1_bridge
    mkdir -p ~/ros1_bridge/src
    cd ~/ros1_bridge/src
    git clone https://github.com/ros2/ros1_bridge
    cd ~/ros1_bridge

    # Source the ROS 2 workspace
    . ~/ros2_humble/install/local_setup.bash

    # Build
    colcon build

After building all of ``ros1_bridge``, the remainder of the `ros1_bridge examples <https://github.com/ros2/ros1_bridge#example-1-run-the-bridge-and-the-example-talker-and-listener>`__ should work with your new installation

