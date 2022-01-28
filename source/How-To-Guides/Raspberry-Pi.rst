ROS2 on Raspberry Pi
==============

ROS2 is supported on ARM processors both with 32 bit (arm32) and 64 bit (arm64) OS. However the two options have different Support Tiers.

You can see `here<https://www.ros.org/reps/rep-2000.html#galactic-geochelone-may-2021-november-2022>` that arm64 receives Tier 1 support, while arm32 is Tier 3:

Tier 1 support means distribution specific packages and binary archives are available, while Tier 3 requires the user to compile ROS2 from source.

The fastest and simplest way to use ROS2 is to use a Tier 1 supported configuration.

This would mean either installing 64 bit Ubuntu 20.04 on to the Raspberry Pi, or using the 64 bit version of Raspberry Pi OS and running ROS2 in Docker.

Ubuntu Linux on Raspberry Pi with binary ROS2 install
-----------------------------------------------------

Ubuntu for Raspberry Pi is available `here<https://ubuntu.com/download/raspberry-pi>`.

Make sure to confirm that you have selected the correct version as described in `REP-2000<https://www.ros.org/reps/rep-2000.html#galactic-geochelone-may-2021-november-2022>`. For ROS2 Galactic this would be Ubuntu Focal 20.04 64 bit.

You can now install ROS2 using the normal binary installation instructions for Ubuntu Linux.

Raspberry Pi OS with ROS2 in docker
-----------------------------------

Raspberry Pi OS 64 bit version is not listed on the Raspberry Pi downloads page, but the images are `available here<https://downloads.raspberrypi.org/raspios_arm64/images/>`.

Raspberry Pi OS is based on Debian which receives Tier 3 support, but it can run Ubuntu Focal docker containers for Tier 1 support.

After flashing the OS, `install Docker<https://docs.docker.com/engine/install/debian/#install-using-the-convenience-script>`.

The official OSRF ROS2 Docker container definitions can be found `here<https://github.com/osrf/docker_images/blob/master/ros/galactic/ubuntu/focal/>`.

You may choose from ros-core, ros-base, or ros-desktop. See `here<https://www.ros.org/reps/rep-2001.html#galactic-geochelone-may-2021-november-2022>` for more information on these variants.

Clone the `docker_images git repo<https://github.com/osrf/docker_images>` onto the Raspberry Pi, change in to the directory linked above, then to the directory with your preferred variant.

Inside of the directory, build the container with:

.. code-block:: bash

    docker build -t ros_docker .

On a supported system it will only take a minute or two to build the docker containers, as the source code is already built in to binaries.

Pre-built Docker container
--------------------------

A pre-built container for the desktop variant is available as well, which only requires a docker pull command.

See `this page<../How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers>` for more information.
