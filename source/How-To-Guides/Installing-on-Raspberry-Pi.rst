ROS 2 on Raspberry Pi
=====================

ROS 2 is supported on both 32 bit (arm32) and 64 bit (arm64) ARM processors.
However, you can see `here <https://www.ros.org/reps/rep-2000.html>`__ that arm64 receives Tier 1 support, while arm32 is Tier 3.
Tier 1 support means distribution specific packages and binary archives are available, while Tier 3 requires the user to compile ROS 2 from source.

The fastest and simplest way to use ROS 2 is to use a Tier 1 supported configuration.

This would mean either installing 64 bit Ubuntu on to the Raspberry Pi, or using the 64 bit version of Raspberry Pi OS and running ROS 2 in Docker.

Ubuntu Linux on Raspberry Pi with binary ROS 2 install
------------------------------------------------------

Ubuntu for Raspberry Pi is available `here <https://ubuntu.com/download/raspberry-pi>`__.

Make sure to confirm that you have selected the correct version as described in `REP-2000 <https://www.ros.org/reps/rep-2000.html>`__.

You can now install ROS 2 using the normal binary installation instructions for Ubuntu Linux.

Raspberry Pi OS with ROS 2 in docker
------------------------------------

Raspberry Pi OS 64 bit version is `available here <https://www.raspberrypi.com/software/operating-systems/>`__.

Raspberry Pi OS is based on Debian which receives Tier 3 support, but it can run Ubuntu docker containers for Tier 1 support.

After flashing the OS, `install Docker <https://docs.docker.com/engine/install/debian/#install-using-the-convenience-script>`__.

The official ROS 2 Docker images can be found `here <https://hub.docker.com/_/ros/tags>`__.

You may choose from ros-core, ros-base, or perception.
See `here <https://www.ros.org/reps/rep-2001.html>`__ for more information on these variants.

Fetch and run an image:

.. code-block:: console

    $ docker pull ros:{DISTRO}-ros-core
    $ docker run -it --rm ros:{DISTRO}-ros-core

You can also build images yourself:

Clone the `docker_images git repo <https://github.com/osrf/docker_images>`__ onto the Raspberry Pi, change in to the directory linked above, then to the directory with your preferred variant.

Inside of the directory, build the container with:

.. code-block:: console

    $ docker build -t ros_docker .

On a supported system it will only take a minute or two to build the docker containers, as the source code is already built in to binaries.
