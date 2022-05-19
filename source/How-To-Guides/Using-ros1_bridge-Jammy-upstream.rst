Using ``ros1_bridge`` with upstream ROS on Ubuntu 22.04
=======================================================

.. contents:: Table of Contents
   :depth: 1
   :local:

The release of ROS 2 Humble on Ubuntu 22.04 Jammy Jellyfish marks the first ROS 2 release on a platform with no official ROS 1 release.
While ROS 1 Noetic will continue to be supported through the duration of its `long term support window <https://www.ros.org/reps/rep-0003.html#noetic-ninjemys-may-2020-may-2025>`__, it will only target Ubuntu 20.04.
Alternatively, there are :doc:`upstream variants of ROS 1 packages <metapackages>` in Debian and Ubuntu that are not maintained as an official distribution by the ROS maintainers.

This guide outlines the current mechanism for bridging ROS 2 Humble with these upstream packages on Ubuntu 22.04 Jammy Jellyfish.
This provides a migration path for users who still depend on ROS 1, but desire moving to Humble/Jammy.

ROS 2 via Debian packages
-------------------------

Installing :doc:`ROS 2 from Debian packages <../Installation/Ubuntu-Install-Debians>` currently does not work for ROS 2 Humble on Ubuntu Jammy.
The version of ``catkin-pkg-modules`` available in the Ubuntu repository conflicts with that in the ROS 2 package repository.

For now, to support ``ros1_bridge``, follow the instructions below for building ROS 2 from source.

ROS 2 from source
-----------------

Installing :doc:`ROS 2 from Source <../Installation/Alternatives/Ubuntu-Development-Setup` is the only configuration that works for ROS 2 Humble on Ubuntu Jammy.

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

.. _linux-dev-get-ros2-code:


From here, continue with the source install guide to build ROS 2 Humble.

Install ROS 1 from Ubuntu packages
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   sudo apt update && sudo apt install -y ros-core-dev

.. _linux-dev-install-ros1:


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

.. _linux-dev-build-ros1:

After building all of ``ros1_bridge``, the remainder of the :doc:`ros1_bridge examples <https://github.com/ros2/ros1_bridge#example-1-run-the-bridge-and-the-example-talker-and-listener>` should work with your new installation

