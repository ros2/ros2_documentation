## Introduction

This page describes how to prepare a repository for release on the public ROS 2 buildfarm. After you've created a package, this is the next step towards getting your package in to the publicly-available Debian packages (i.e., you will be able to install the package via `apt`). This page includes the ROS 2-specific instructions to execute before following the [Bloom release tutorial on the ROS Wiki](http://wiki.ros.org/bloom/Tutorials/FirstTimeRelease).

### Required Tools

For ROS 2 Bouncy:
* `bloom` >= 0.6.6
* `catkin_pkg` >= 0.4.5

#### Ensure that you have the latest version of bloom and catkin_pkg

See above version requirements.

* Make sure you have the ros repositories in your sources (see [Instructions](Linux-Install-Debians.md#setup-sources))

* Install the latest version of bloom and catkin_pkg:
```
sudo apt install python-catkin-pkg python-bloom
```

### Differences from ROS 1 Bloom

If you've bloomed packages before in ROS 1, ROS 2's requirements will look familiar.

ROS 2 uses a forked rosdistro index located at https://github.com/ros2/rosdistro.
You can configure bloom to use it by setting the `ROSDISTRO_INDEX_URL` environment variable.

```
export ROSDISTRO_INDEX_URL='https://raw.githubusercontent.com/ros2/rosdistro/ros2/index.yaml'
```

## Procedure

Same as in ROS 1: [Following this tutorial]( http://wiki.ros.org/bloom/Tutorials/FirstTimeRelease.md)
