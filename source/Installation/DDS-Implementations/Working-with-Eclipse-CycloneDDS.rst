.. redirect-from::

    Working-with-Eclipse-CycloneDDS

Working with Eclipse Cyclone DDS
================================

Eclipse Cyclone DDS is a very performant and robust open-source DDS implementation.
Cyclone DDS is developed completely in the open as an Eclipse IoT project.
See also: https://projects.eclipse.org/projects/iot.cyclonedds


Prerequisites
-------------

Have `rosdep installed  <https://wiki.ros.org/rosdep#Installing_rosdep>`__

Install packages
----------------

The easiest way is to install from ROS 2 apt repository.

.. code-block:: bash

   sudo apt install ros-rolling-rmw-cyclonedds-cpp

Build from source code
----------------------

Building from source code is also another way to install.

First, clone Cyclone DDS and rmw_cyclonedds in the ROS 2 workspace source directory.

.. code-block:: bash

   cd ros2_ws/src
   git clone https://github.com/ros2/rmw_cyclonedds ros2/rmw_cyclonedds
   git clone https://github.com/eclipse-cyclonedds/cyclonedds eclipse-cyclonedds/cyclonedds

Then, install necessary packages for Cyclone DDS.

.. code-block:: bash

   cd ..
   rosdep install --from src -i

Finally, run colcon build.

.. code-block:: bash

   colcon build --symlink-install

Switch to rmw_cyclonedds
------------------------

Switch from other rmw to rmw_cyclonedds by specifying the environment variable.

.. code-block:: bash

   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

See also: https://index.ros.org/doc/ros2/Tutorials/Working-with-multiple-RMW-implementations/

Run the talker and listener
---------------------------

Now run ``talker`` and ``listener`` to test Cyclone DDS.

.. code-block:: bash

   ros2 run demo_nodes_cpp talker

.. code-block:: bash

   ros2 run demo_nodes_cpp listener
