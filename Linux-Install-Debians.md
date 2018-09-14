# Installing ROS2 via Debian Packages

Debian packages for ROS 2 Bouncy (the latest release) are available for Ubuntu Bionic; packages for ROS 2 Ardent are available for Ubuntu Xenial.

Resources:
 - [Jenkins Instance](http://build.ros2.org/)
 - [Repositories](http://repo.ros2.org)
 - Status Pages:
    - ROS 2 Bouncy (Ubuntu Bionic): [amd64](http://repo.ros2.org/status_page/ros_bouncy_default.html), [arm64](http://repo.ros2.org/status_page/ros_bouncy_ubv8.html)
    - ROS 2 Ardent (Ubuntu Xenial): [amd64](http://repo.ros2.org/status_page/ros_ardent_default.html), [arm64](http://repo.ros2.org/status_page/ros_ardent_uxv8.html)

## Setup Sources

To install the Debian packages you will need to add our Debian repository to your apt sources.
First you will need to authorize our gpg key with apt like this:

```
sudo apt update && sudo apt install curl
curl http://repo.ros2.org/repos.key | sudo apt-key add -
```

And then add the repository to your sources list:

```
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
```

## Install ROS 2 packages

First set an environment variable for the ROS 2 release you want to install so it can be used in other commands.

```
export ROS_DISTRO=bouncy  # or ardent
sudo apt update
```

Desktop Install (Recommended): ROS, RViz, demos, tutorials.
```
sudo apt install ros-$ROS_DISTRO-desktop
```

ROS-Base Install (Bare Bones): Communication libraries, message packages, command line tools. No GUI tools.
```
sudo apt install ros-$ROS_DISTRO-ros-base
```

See specific sections below for how to also install the ros1_bridge, TurtleBot packages, or alternative RMW packages.

## Environment setup

### (optional) Install argcomplete

ROS 2 commadline tools use argcomplete to autocompletion. So if you want autocompletion, installing argcomplete is necessary.

#### Ubuntu 18.04

```
sudo apt install python3-argcomplete
```

#### Ubuntu 16.04 (argcomplete >= 0.8.5)

To install `argcomplete` on Ubuntu 16.04 (Xenial), you'll need to use pip, because the version available through `apt` will not work due to a bug in that version of `argcomplete`:

```
sudo apt install python3-pip
sudo pip3 install argcomplete
```

### Sourcing the setup script

Set up your environment by sourcing the following file (you may want to add this to your `.bashrc`).
```
source /opt/ros/$ROS_DISTRO/setup.bash
```

## Installing additional RMW implementations

By default the RMW implementation `FastRTPS` is used.
If using Ardent OpenSplice is also installed.

To install support for OpenSplice or RTI Connext on Bouncy:
```
sudo apt update
sudo apt install ros-$ROS_DISTRO-rmw-opensplice-cpp # for OpenSplice
sudo apt install ros-$ROS_DISTRO-rmw-connext-cpp # for RTI Connext (requires license agreement)
```

By setting the environment variable `RMW_IMPLEMENTATION=rmw_opensplice_cpp` you can switch to use OpenSplice instead.
For ROS 2 releases Bouncy and newer, `RMW_IMPLEMENTATION=rmw_connext_cpp` can also be selected to use RTI Connext.

If you want to install the Connext DDS-Security plugins please refer to [this page](https://github.com/ros2/ros2/wiki/Install-Connext-Security-Plugins)

## Additional packages using ROS 1 packages

The `ros1_bridge` as well as the TurtleBot demos are using ROS 1 packages.
To be able to install them please start by adding the ROS 1 sources as documented [here](http://wiki.ros.org/Installation/Ubuntu?distro=melodic).

If you're using Docker for isolation you can start with the image `ros:melodic` or `osrf/ros:melodic-desktop` (or Kinetic if using Ardent).
This will also avoid the need to setup the ROS sources as they will already be integrated.

Now you can install the remaining packages:

```
sudo apt update
sudo apt install ros-$ROS_DISTRO-ros1-bridge ros-$ROS_DISTRO-turtlebot2-*
```
