RTI Connext DDS
=================

RTI Connext DDS is trusted in over 2000 of the world's most demanding system designs, distributing critical real-time data with the highest levels of performance, reliability, and security.
It is available on a free academic or non-commercial license, or a commercial license with support, for dozens of production-grade platforms.
More information can be found at the `RTI website <https://www.rti.com/products>`__.


Prerequisites
-------------

**Install RTI Connext DDS**  
  To build and use `rmw_connextdds` requires a version of Connext DDS compatible with the distribution of ROS 2 in use, per the following table:

  ==================  ===================
  ROS 2 Distribution  RTI Connext Version
  ==================  ===================
  rolling             7.3.0 or 6.x.x
  jazzy               7.3.0 or 6.x.x
  iron                7.3.0 or 6.x.x
  humble              6.x.x
  ==================  ===================

RTI Connext Pro is available through a variety of channels:

**ROS 2 apt repositories**
  ROS 2 users can install a non-commercial-use version of the RTI Connext DDS libraries for x86_64 Linux from the ROS apt repository using the following command:

  .. code-block:: bash

     sudo apt update && sudo apt install -q -y rti-connext-dds-[7.3.0 or 6.0.1]

  Note that this includes the RTI Connext libraries only, and does not include the full RTI Connext Pro suite of tools and services.

**Free Licenses**
  RTI Connext DDS is available for research/non-commercial, university/education, or time-limited evaluation from the `RTI website <https://www.rti.com/free-trial>`__.
  This includes a full-featured version of Connext DDS with diagnostic tools and layered services, access to self-guided training and the RTI AI Chatbot.
  Libraries are available for Linux(x86_64, armv7, armv8), Windows(x86_64), and macOS(x86_64, armv8).

**Purchase**
  RTI Connext DDS is also available as a subscription with training and architectural guidance, and support for dozens of platforms (OS/RTOS, CPU, toolchain), safety certification and security.
  Please `contact RTI <https://www.rti.com/company/contact>`__ for more information.


Detailed instructions for building and tuning the RMW and ROS 2 applications for a variety of platforms, and enabling DDS Security and safety-cert options are available on the `RTI ROS Community <https://community.rti.com/ros>`__ pages.


Building rmw_connextdds from source code
----------------------------------------

Building from source code can ensure the RMW is matched to your system and installed correctly.
The following instructions assume a Linux x86_64 build host and target; the `RTI ROS Community <https://community.rti.com/ros>`__
pages have instructions for building for other platforms and targets, including Arm, Windows, and macOS.

Clone the repository for rmw_connextdds into your ROS 2 workspace and select the branch that matches the ROS 2 distribution in use:

.. code-block:: bash

   cd ros2_ws/src
   git clone https://github.com/ros2/rmw_connextdds -b {ROS_BRANCH_IN_USE}

Set up the environment to help colcon discover where RTI Connext is installed.
This can be done by manually setting the environment variable ``NDDSHOME`` to the location of the RTI Connext installation,
or by using a script that comes with the RTI Connext installation:

.. code-block:: bash

   source {RTI_CONNEXT_INSTALL_LOCATION}/resource/scripts/rtisetenv_x64Linux4gcc7.3.0.bash

Build the RMW using colcon:

.. code-block:: bash

   colcon build --symlink-install

After the build completes successfully, be sure to source the setup file for the workspace:

.. code-block:: bash

   source install/setup.bash


Use the resulting rmw_connextdds
--------------------------------

Set the environment variable `RMW_IMPLEMENTATION` to tell ROS 2 which RMW to use:

.. code-block:: bash

   export RMW_IMPLEMENTATION=rmw_connextdds

See also: :doc:`Working with multiple RMW implementations <../../How-To-Guides/Working-with-multiple-RMW-implementations>`

Run the talker and listener
---------------------------

Now run ``talker`` and ``listener`` to test RTI Connext DDS

.. code-block:: bash

   ros2 run demo_nodes_cpp talker

.. code-block:: bash

   ros2 run demo_nodes_cpp listener

