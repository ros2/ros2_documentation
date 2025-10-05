RTI Connext DDS
=================

RTI Connext DDS is trusted in over 2000 of the world's most demanding system designs, distributing critical real-time data with the highest levels of performance, reliability, and security.
It is available on a free academic or non-commercial license, or a commercial license with support, for dozens of production-grade platforms.
More information can be found at the `RTI website <https://www.rti.com/products>`__.


Prerequisites
-------------

Install RTI Connext DDS
^^^^^^^^^^^^^^^^^^^^^^^
  To build and use ``rmw_connextdds`` requires a version of Connext DDS compatible with the distribution of ROS 2 in use.
  Connext DDS is included when installing ``rmw_connextdds`` using apt, or can be installed manually for building from source.
  The following table details which Connext DDS versions are installed using ``apt``, and which versions are required for building from source:

  ==================  ===================  ====================
  ROS 2 Distribution  Installed using apt  To Build from Source
  ==================  ===================  ====================
  rolling             n/a                  ``7.3.0``
  kilted              ``7.3.0``            ``7.3.0``
  jazzy               ``6.0.1``            ``6.0.1``
  iron                ``6.0.1``            ``6.0.1``
  humble              ``6.0.1``            ``6.0.1``
  ==================  ===================  ====================

RTI Connext Pro is available through a variety of channels:

**ROS 2 apt repositories**
  ROS 2 users can install a non-commercial-use version of the RTI Connext DDS libraries from the ROS apt repository using the following command:

  .. tabs::

     .. group-tab:: v7.3.0

        .. code-block:: console

           $ sudo apt update && sudo apt install -q -y rti-connext-dds-7.3.0-ros

     .. group-tab:: v6.0.1

        .. code-block:: console

           $ sudo apt update && sudo apt install -q -y rti-connext-dds-6.0.1

  Note that this includes the RTI Connext libraries only, and does not include the full RTI Connext Pro suite of tools and services.
  Note also that the Connext libraries are automatically installed when installing ``rmw_connextdds`` using apt.

**Other Installation Options**
RTI Connext DDS is a proprietary DDS implementation with a number of advanced features and commercial support options.
RTI provides both a `non-commercial / research license <https://www.rti.com/free-trial/university-program>`__ for students and researchers and a `time-limited free trial license <https://content.rti.com/l/983311/2025-06-26/q5tyw3>`__ for commercial users.
Detailed instructions for building and tuning the RMW and ROS 2 applications for a variety of platforms, and enabling DDS Security and safety-cert options are available on the `RTI ROS Community <https://community.rti.com/ros>`__ pages.


Install rmw_connextdds binary packages
--------------------------------------

To install the binary packages for ``rmw_connextdds`` and the Connext libraries from the ROS 2 apt repositories, use the following command:

.. code-block:: console

   $ sudo apt update && sudo apt install -q -y ros-{DISTRO}-rmw-connextdds


Building rmw_connextdds from source code
----------------------------------------

Building from source code can ensure the RMW is matched to your system and installed correctly.
The following instructions assume a Linux x86_64 build host and target; the `RTI ROS Community <https://community.rti.com/ros>`__
pages have instructions for building for other platforms and targets, including Arm, Windows, and macOS.

Clone the repository for ``rmw_connextdds`` into your ROS 2 workspace and select the branch that matches the ROS 2 distribution in use:

.. code-block:: console

   $ mkdir -p ros2_ws/src
   $ cd ros2_ws
   $ git clone -b {DISTRO} https://github.com/ros2/rmw_connextdds src/rmw_connextdds

Then, install necessary packages for RTI Connext DDS.

.. code-block:: console

   $ cd ..
   $ rosdep install --from src -i

Set up the environment to help colcon discover where RTI Connext is installed.
This can be done by manually setting the environment variable ``NDDSHOME`` to the location of the RTI Connext installation, or by using a script that comes with the RTI Connext installation.
For example, for version 7.3.0, you can run the following code to execute the helper script:

.. code-block:: console

   $ source /opt/rti.com/rti_connext_dds-7.3.0/resource/scripts/rtisetenv_x64Linux4gcc7.3.0.bash

If the previous command failed, and you can't find the location of the RTI Connext installation, run this to find all Connext installations (and their corresponding helper scripts) in your system:

.. code-block:: console

   $ find /opt -name rtisetenv*.bash

.. note::

   Replace ``.bash`` with your shell if you're not using bash.
   Possible values are: ``rtisetenv*.bash``, ``rtisetenv*.sh``, ``rtisetenv*.zsh``, ``rtisetenv*.tcsh``.

Make sure you have the ROS 2 environment set up:

.. code-block:: console

   $ source /opt/ros/{DISTRO}/setup.bash

Build the RMW using colcon:

.. code-block:: console

   $ colcon build --symlink-install

After the build completes successfully, be sure to source the setup file for the workspace:

.. code-block:: console

   $ source install/setup.bash


Use the resulting rmw_connextdds
--------------------------------

Set the environment variable ``RMW_IMPLEMENTATION`` to tell ROS 2 which RMW to use:

.. code-block:: console

   $ export RMW_IMPLEMENTATION=rmw_connextdds

See also: :doc:`Working with multiple RMW implementations <../../../How-To-Guides/Working-with-multiple-RMW-implementations>`

Run the talker and listener
---------------------------

Now run ``talker`` and ``listener`` to test RTI Connext DDS

.. code-block:: console

   $ source /opt/ros/{DISTRO}/setup.bash
   $ ros2 run demo_nodes_cpp talker

.. code-block:: console

   $ source /opt/ros/{DISTRO}/setup.bash
   $ ros2 run demo_nodes_cpp listener
