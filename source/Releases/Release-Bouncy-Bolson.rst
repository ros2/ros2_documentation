.. redirect-from::

  Release-Bouncy-Bolson

Bouncy Bolson (``bouncy``)
==========================

.. contents:: Table of Contents
   :depth: 2
   :local:

*Bouncy Bolson* is the second release of ROS 2.

Supported Platforms
-------------------

This version of ROS 2 is supported on four platforms (see `REP 2000 <https://reps.openrobotics.org/rep-2000/#bouncy-bolson-june-2018-june-2019>`__ for full details):


* Ubuntu 18.04 (Bionic)

  * Debian packages for amd64 as well as arm64

* Ubuntu 16.04 (Xenial)

  * no Debian packages but building from source is supported

* Mac macOS 10.12 (Sierra)
* Windows 10 with Visual Studio 2017

Binary packages as well as instructions for how to compile from source are provided (see `install instructions <../../Installation>` as well as `documentation <https://docs.ros2.org/bouncy/>`__).

Targeted platforms:

<table border="1">
  <colgroup>
    <col width="10%">
    <col width="16%">
    <col width="21%">
    <col width="17%">
    <col width="19%">
    <col width="17%">
  </colgroup>
  <thead valign="bottom">
    <tr><th class="head">&nbsp;</th>
      <th class="head" colspan="4">Required Support</th>
      <th class="head">Recommended Support</th>
    </tr>
    <tr><th class="head">Architecture</th>
      <th class="head">Ubuntu Bionic (18.04)</th>
      <th class="head">MacOS Sierra (10.12)</th>
      <th class="head">Windows 10 (VS2017)</th>
      <th class="head">Ubuntu Xenial (16.04) [s]</th>
      <th class="head">Debian Stretch (9) [s]</th>
    </tr>
  </thead>
  <tbody valign="top">
    <tr><td>amd64</td>
      <td>X</td>
      <td>X</td>
      <td>X</td>
      <td>X [s]</td>
      <td>X [s]</td>
    </tr>
    <tr><td>arm64</td>
      <td>X</td>
      <td>&nbsp;</td>
      <td>&nbsp;</td>
      <td>X [s]</td>
      <td>X [s]</td>
    </tr>
  </tbody>
</table>

\" \[s\] \" Compilation from source, the ROS buildfarm will not produce
any binary packages for these platforms.

Minimum language requirements:

- C11[^3]
- C++14
- Python 3.5

Dependency Requirements:

<table border="1">
  <colgroup>
    <col width="9%">
    <col width="17%">
    <col width="16%">
    <col width="17%">
    <col width="20%">
    <col width="22%">
  </colgroup>
  <thead valign="bottom">
    <tr><th class="head">&nbsp;</th>
      <th class="head" colspan="4">Required Support</th>
      <th class="head">Recommended Support</th>
    </tr>
    <tr><th class="head">Package</th>
      <th class="head">Ubuntu  Bionic</th>
      <th class="head">MacOS**</th>
      <th class="head">Windows 10**</th>
      <th class="head">Ubuntu Xenial [s]</th>
      <th class="head">Debian Stretch [s]</th>
    </tr>
  </thead>
  <tbody valign="top">
    <tr><td>CMake</td>
      <td>3.10.2</td>
      <td>3.11.0</td>
      <td>3.10.2</td>
      <td>3.5.1</td>
      <td>3.7.2</td>
    </tr>
    <tr><td>EmPY</td>
      <td>3.3.2</td>
      <td>3.6.5</td>
      <td>3.3.2</td>
      <td>3.3.2</td>
      <td>3.3.2</td>
    </tr>
    <tr><td>Ogre</td>
      <td>1.10*</td>
      <td>1.10*</td>
      <td>1.10*</td>
      <td>1.10*</td>
      <td>1.10*</td>
    </tr>
    <tr><td>OpenCV</td>
      <td>3.2.0</td>
      <td>3.4.1</td>
      <td>3.4.1*</td>
      <td>2.4.9</td>
      <td>3.2*</td>
    </tr>
    <tr><td>Poco</td>
      <td>1.8.0</td>
      <td>1.9.0</td>
      <td>1.8.0*</td>
      <td>1.8.0*</td>
      <td>1.8.0*</td>
    </tr>
    <tr><td>Python</td>
      <td>3.6.5</td>
      <td>3.6.5</td>
      <td>3.6.5</td>
      <td>3.5.1</td>
      <td>3.5.3</td>
    </tr>
    <tr><td>Qt</td>
      <td>5.9.5</td>
      <td>5.10.0</td>
      <td>5.10.0</td>
      <td>5.5.1</td>
      <td>5.7.1</td>
    </tr>
    <tr><td colspan="6"><strong>Linux only (used for turtlebot demo)</strong></td>
    </tr>
    <tr><td>PCL</td>
      <td>1.8.1</td>
      <td>N/A</td>
      <td>N/A</td>
      <td>1.7.2</td>
      <td>1.8.0</td>
    </tr>
  </tbody>
</table>

\" \* \" means that this is not the upstream version (available on the
official Operating System repositories) but a package distributed by
OSRF or the community (package built and distributed on custom
repositories).

\" \*\* \" Rolling distributions will see multiple version changes of
these dependencies during their lifetime.

\" \[s\] \" Compilation from source, the ROS buildfarm will not produce
any binary packages for these platforms.

This document only captures the version at the first release of a ROS
distribution and will not be updated as the dependencies move forward.
These versions are thus a low watermark.

Package manager use for dependencies:

- Ubuntu Bionic: apt
- MacOS: Homebrew, pip
- Windows: Chocolatey, pip
- Ubuntu Xenial, Debian Stretch: apt

Build System Support:

- ament_cmake
- cmake
- setuptools

Middleware Implementation Support:

- eProsima Fast-RTPS
- RTI Connext
- ADLINK OpenSplice

Features
--------

New features in this ROS 2 release
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


* `New launch system <../Tutorials/Intermediate/Launch/Launch-system>` featuring a much more capable and flexible Python API.
* Parameters can be passed as `command line arguments <../How-To-Guides/Node-arguments>` to C++ executables.
* Static remapping via `command line arguments <../How-To-Guides/Node-arguments>`.
* Various improvements to the Python client library.
* Support for publishing and subscribing to serialized data.
  This is the foundation for the upcoming work towards a native rosbag implementation.
* More `command line tools <../../Concepts/Basic/About-Command-Line-Tools>`\ , e.g. for working with parameters and lifecycle states.
* Binary packages / fat archives support three RMW implementations by default (without the need to build from source):

  * eProsima's Fast RTPS (default)
  * RTI's Connext
  * ADLINK's OpenSplice

For an overview of all features available, including those from earlier releases, please see the `Features <../../The-ROS2-Project/Features>` page.

Changes since the Ardent release
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Changes since the `Ardent Apalone <Release-Ardent-Apalone>` release:


* The Python package ``launch`` has been redesigned.
  The previous Python API has been moved into a submodule ``launch.legacy``.
  You can update existing launch files to continue to use the legacy API if a transition to the new Python API is not desired.
* The ROS topic names containing namespaces are mapped to DDS topics including their namespaces.
  DDS partitions are not being used anymore for this.
* The recommended build tool is now ``colcon`` instead of ``ament_tools``.
  This switch has no `implications <https://design.ros2.org/articles/build_tool.html#implications>`__ for the code in each ROS 2 package.
  The install instructions have been updated and the `read-the-docs page <https://colcon.readthedocs.io/en/main/migration/ament_tools.html>`__ describes how to map an existing ``ament_tools`` call to ``colcon``.
* The argument order of `this rclcpp::Node::create_subscription() signature <https://docs.ros2.org/bouncy/api/rclcpp/classrclcpp_1_1_node.html#a283fb006c46470cf43a4ae5ef4a16ccd>`__ has been modified.

Known Issues
------------


* New-style launch files `may hang on shutdown <https://github.com/ros2/launch/issues/89>`__ for some combinations of platform and RMW implementation.
* Static remapping of namespaces `not working correctly <https://github.com/ros2/rcl/issues/262>`__ when addressed to a particular node.
* `Opensplice error messages may be printed <https://github.com/ros2/rmw_opensplice/issues/237>`__ when using ``ros2 param`` and ``ros2 lifecycle`` command-line tools.
