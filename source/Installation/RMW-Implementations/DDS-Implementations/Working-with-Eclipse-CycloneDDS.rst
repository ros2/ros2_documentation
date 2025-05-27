.. redirect-from::

    Working-with-Eclipse-CycloneDDS

Eclipse Cyclone DDS
===================

Eclipse Cyclone DDS is a very performant and robust open-source DDS implementation.
Cyclone DDS is developed completely in the open as an Eclipse IoT project.
See also: https://projects.eclipse.org/projects/iot.cyclonedds


Prerequisites
-------------

Have :doc:`rosdep installed <../../../Tutorials/Intermediate/Rosdep>`.

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

See also: :doc:`Working with multiple RMW implementations <../../../How-To-Guides/Working-with-multiple-RMW-implementations>`

Run the talker and listener
---------------------------

Now run ``talker`` and ``listener`` to test Cyclone DDS.

.. code-block:: console

   $ source /opt/ros/{DISTRO}/setup.bash
   $ ros2 run demo_nodes_cpp talker

.. code-block:: console

   $ source /opt/ros/{DISTRO}/setup.bash
   $ ros2 run demo_nodes_cpp listener
