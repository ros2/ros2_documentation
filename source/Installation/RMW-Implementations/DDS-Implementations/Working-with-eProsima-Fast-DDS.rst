eProsima Fast DDS
=================

eProsima Fast DDS is a complete open-source DDS implementation for real time embedded architectures and operating systems.
See also: https://www.eprosima.com/index.php/products-all/eprosima-fast-dds


Prerequisites
-------------

Have :doc:`rosdep installed <../../../Tutorials/Intermediate/Rosdep>`.

Install packages
----------------

The easiest way is to install from ROS 2 apt repository.

.. code-block:: console

   $ sudo apt install ros-{DISTRO}-rmw-fastrtps-cpp

Build from source code
----------------------

Building from source code is also another way to install.

First, clone Fast DDS and rmw_fastrtps in the ROS 2 workspace source directory.

.. code-block:: console

   $ cd ros2_ws/src
   $ git clone https://github.com/ros2/rmw_fastrtps ros2/rmw_fastrtps -b {REPOS_FILE_BRANCH}
   $ git clone https://github.com/eProsima/Fast-DDS eProsima/fastrtps

Then, install necessary packages for Fast DDS.

.. code-block:: console

   $ cd ..
   $ rosdep install --from src -i

Finally, run colcon build.

.. code-block:: console

   $ colcon build --symlink-install

Switch to rmw_fastrtps
----------------------

The eProsima Fast DDS RMW can be selected by specifying the environment variable:

.. code-block:: console

   $ export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

See also: :doc:`Working with multiple RMW implementations <../../../How-To-Guides/Working-with-multiple-RMW-implementations>`

Run the talker and listener
---------------------------

Now run ``talker`` and ``listener`` to test Fast DDS.

.. code-block:: console

   $ source /opt/ros/{DISTRO}/setup.bash
   $ ros2 run demo_nodes_cpp talker

.. code-block:: console

   $ source /opt/ros/{DISTRO}/setup.bash
   $ ros2 run demo_nodes_cpp listener
