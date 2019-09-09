.. redirect-from::

    Working-with-Eclipse-CycloneDDS

Working with Eclipse CycloneDDS [community-contributed]
=========================================================

Eclipse Cyclone DDS is a very performant and robust open-source DDS implementation.
Cyclone DDS is developed completely in the open as an Eclipse IoT project.
See also: https://projects.eclipse.org/projects/iot.cycloned

Build Eclipse Cyclone DDS and rmw_cyclonedds  
--------------------------------------------

First, we should clone Cyclone DDS and rmw_cyclonedds in the ROS2 workspace source directory.

.. code-block:: bash

   cd ros2_ws/src
   git clone https://github.com/atolab/rmw_cyclonedds atolab/rmw_cyclonedds
   git clone https://github.com/eclipse-cyclonedds/cyclonedds eclipse-cyclonedds/cyclonedds

Then, we can run colcon build.

.. code-block:: bash

   cd ..
   colcon build --symlink-install

Switch to rmw_cyclonedds
------------------------

Switching from other rmw to rmw_cyclonedds is very easy.
You can seamlessly switch to rmw_cyclonedds by specifying the environment:

.. code-block:: bash

   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

See also: https://index.ros.org/doc/ros2/Tutorials/Working-with-multiple-RMW-implementations/

Run the talker and listener
---------------------------

Now you can test the ``talker`` and ``listener``.

.. code-block:: bash

   ros2 run demo_nodes_cpp talker

.. code-block:: bash

   ros2 run demo_nodes_cpp listener
