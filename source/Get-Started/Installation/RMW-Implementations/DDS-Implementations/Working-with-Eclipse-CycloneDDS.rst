.. meta::
   :contentType: how-to
   :experience: intermediate, expert
   :area: installation
   :distribution: {DISTRO}
   :product: {PRODUCT}

.. redirect-from::

    Working-with-Eclipse-CycloneDDS
    Installation/RMW-Implementations/DDS-Implementations/Working-with-Eclipse-CycloneDDS

Working with Eclipse Cyclone DDS - how-to
=========================================

.. short-description::
   Eclipse Cyclone DDS is an open-source Data Distribution Service (DDS) implementation for ROS systems that need a performant and robust middleware option.
   This article describes how to install ``rmw_cyclonedds``, select it as your active RMW implementation, and verify that it is working.
   After you follow these steps, you can run ROS nodes using Eclipse Cyclone DDS.

Eclipse Cyclone DDS is a very performant and robust open-source DDS implementation.
Cyclone DDS is developed completely in the open as an Eclipse IoT project.
See also: https://projects.eclipse.org/projects/iot.cyclonedds

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. contents:: Contents
   :depth: 2
   :local:

Prerequisites
-------------

Have :doc:`rosdep installed <../../../../ROS-Framework/client-libraries/Working-with-Client-Libraries/Rosdep>`.

Install packages
----------------

The easiest way is to install from ROS 2 apt repository.

.. code-block:: console

   $ sudo apt install ros-{DISTRO}-rmw-cyclonedds-cpp

Build from source code
----------------------

Building from source code is also another way to install.

First, clone Cyclone DDS and rmw_cyclonedds in the ROS 2 workspace source directory.
To determine the correct branches to checkout, you need to find what versions are specified in your `ROS distribution's ros2.repos file <https://raw.githubusercontent.com/ros2/ros2/refs/heads/{DISTRO}/ros2.repos>`_.

Alternatively, you can run the following code to fetch the correct branch/tag needed for Cyclone DDS:

.. code-block:: console

   $ CYCLONEDDS_BRANCH=$(curl -s https://raw.githubusercontent.com/ros2/ros2/refs/heads/{DISTRO}/ros2.repos | grep -A 3 "eclipse-cyclonedds/cyclonedds:" | grep "version:" | awk '{print $2}')

And now, clone and checkout the code:

.. code-block:: console

   $ cd ros2_ws/src
   $ git clone https://github.com/ros2/rmw_cyclonedds ros2/rmw_cyclonedds -b {DISTRO}
   $ git clone https://github.com/eclipse-cyclonedds/cyclonedds eclipse-cyclonedds/cyclonedds -b ${CYCLONEDDS_BRANCH}

Then, install necessary packages for Cyclone DDS.

.. code-block:: console

   $ cd ..
   $ rosdep install --from src -i

Finally, run colcon build.

.. code-block:: console

   $ colcon build --symlink-install

Switch to rmw_cyclonedds
------------------------

Switch from other rmw to rmw_cyclonedds by specifying the environment variable.

.. code-block:: console

   $ export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

See also: :doc:`Working with multiple RMW implementations <../Working-with-multiple-RMW-implementations>`

Run the talker and listener
---------------------------

Now run ``talker`` and ``listener`` to test Cyclone DDS.

.. code-block:: console

   $ source /opt/ros/{DISTRO}/setup.bash
   $ ros2 run demo_nodes_cpp talker

.. code-block:: console

   $ source /opt/ros/{DISTRO}/setup.bash
   $ ros2 run demo_nodes_cpp listener
