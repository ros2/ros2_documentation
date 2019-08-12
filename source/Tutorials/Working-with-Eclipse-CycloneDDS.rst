.. redirect-from::

    Working-with-Eclipse-CycloneDDS

Working with Eclipse CycloneDDS [community-contributed]
=========================================================

Eclipse Cyclone DDS is a very performant and robust open-source DDS implementation. Cyclone DDS is developed completely in the open as an Eclipse IoT project. (see also: https://projects.eclipse.org/projects/iot.cyclonedds)

Before using Cyclone DDS and it's rmw package, you need a CycloneDDS installed in your system. CycloneDDS follows cmake building system, so you will need to install cmake before compile it from source.

Compile and install Eclipse CycloneDDS from source
---------------------------------------------------

First, we need to clone the source code from github.

.. code-block:: bash

   git clone https://github.com/eclipse-cyclonedds/cyclonedds.git -b <branch or tag>

Then, use cmake to build the project

.. code-block:: bash

   cmake -Bbuild -Hsrc -DCMAKE_BUILD_TYPE=Release
   cmake --build build
   cmake --build build --target install

After installation, CycloneDDS will be installed under your system.

Import rmw_cyclonedds into your ros2 working space
--------------------------------------------------

To use CycloneDDS for your ros2 communication, you will need to to install rmw_cyclonedds on your ros2 working space. rmw_cyclonedds is a ros2 package, you can use colcon to build it, and import it from the install/local_setup.bash script.

.. code-block:: bash

   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/atolab/rmw_cyclonedds.git
   cd ~/ros2_ws
   colcon build
   source install/local_setup.bash


Switch to rmw_cyclonedds
------------------------

Switching from other rmw to rmw_cyclonedds is very easy. You can seamless switch to rmw_cyclonedds by specifying the environment:

.. code-block:: bash

   export RMW_IMPLEMENTATION=rmw_cyclonedds

Run the talker and listener
---------------------------

Now you can test the talker and listenr.

.. code-block:: bash

   ros2 run demo_nodes_cpp talker

.. code-block:: bash

   ros2 run demo_nodes_cpp listener
