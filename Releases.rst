
ROS 2 Releases
==============

A summary of releases of ROS 2 software is listed below.
For more details about each release, see the corresponding release overview.

.. list-table::
   :header-rows: 1

   * - Release Overview
     - Date
   * - `Crystal Clemmys <Release-Crystal-Clemmys>`
     - 14 December 2018
   * - `Bouncy Bolson <Release-Bouncy-Bolson>`
     - 2 July 2018
   * - `Ardent Apalone <Release-Ardent-Apalone>`
     - 8 December 2017
   * - `beta3 <Beta3-Overview>`
     - 13 September 2017
   * - `beta2 <Beta2-Overview>`
     - 5 July 2017
   * - `beta1 <Beta1-Overview>`
     - 19 December 2016
   * - `alpha1-8 <Alpha-Overview>`
     - 31 August 2015 - 4 October 2016

Release practices
-----------------

Core packages
~~~~~~~~~~~~~

New ROS 2 distributions are currently released every 6 months.
During the release process, Open Robotics packages what is in the `ros2.repos
file <https://github.com/ros2/ros2/blob/master/ros2.repos>`__ into installables for the supported platforms.
These installables take the form of "fat archives" on MacOS, Windows and Linux, and additionally Debian packages are created for Linux.
Check the release page for a distribution for details of the supported platform versions.

After the first release of a distribution, patch releases may be made that include bug fixes and platform updates (particularly on platforms with rolling dependencies like Windows and MacOS) for the distribution.
New features are not typically released into an existing distribution, but are saved for the next distribution.

The `roadmap <Roadmap>` details the upcoming features that are targeted for the next distribution.

While we do aim to keep the API as stable as possible, 100% API compatibility is not guaranteed between distributions.

Internal notes on how a release is made: `Release-Howto <Release-Howto>`

External packages
~~~~~~~~~~~~~~~~~

Packages outside of the `ros2.repos
file <https://github.com/ros2/ros2/blob/master/ros2.repos>`__ may be released by community members into a ROS 2 distribution.
This is currently supported for Linux Debian packages, following `these instructions <Releasing-a-ROS-2-package-with-bloom>`.
These packages can be released at any time during the lifecycle of a ROS 2 distribution.

