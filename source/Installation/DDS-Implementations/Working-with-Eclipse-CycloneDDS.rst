.. redirect-from::

    Working-with-Eclipse-CycloneDDS

Eclipse Cyclone DDS
===================

Eclipse Cyclone DDS es una implementación de DDS de código abierto robusta y de gran rendimiento.
Cyclone DDS se desarrolla completamente abierto como un proyecto Eclipse IoT.
Consulta también: https://projects.eclipse.org/projects/iot.cyclonedds


requisitos previos
------------------

Tener `rosdep instalado <https://wiki.ros.org/rosdep#Installing_rosdep>`__

Instalar paquetes
-----------------

La forma más fácil es instalar desde el repositorio apt de ROS 2.

.. code-block:: bash

   sudo apt install ros-{DISTRO}-rmw-cyclonedds-cpp

Compilar a partir del código fuente
-----------------------------------

Compilar a partir del código fuente también es otra forma de instalar.

Primero, clona Cyclone DDS y rmw_cyclonedds en el directorio src del espacio de trabajo de ROS 2.

.. code-block:: bash

   cd ros2_ws/src
   git clone https://github.com/ros2/rmw_cyclonedds ros2/rmw_cyclonedds -b {REPOS_FILE_BRANCH}
   git clone https://github.com/eclipse-cyclonedds/cyclonedds eclipse-cyclonedds/cyclonedds

Luego, instala los paquetes necesarios para Cyclone DDS.

.. code-block:: bash

   cd ..
   rosdep install --from src -i

Finalmente, ejecute colcon build.

.. code-block:: bash

   colcon build --symlink-install

Cambiar a rmw_cyclonedds
------------------------

Cambie de otro rmw a rmw_cyclonedds especificando la variable de entorno.

.. code-block:: bash

   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

Consulta también: :doc:`Trabajar con varias implementaciones de RMW <../../How-To-Guides/Working-with-multiple-RMW-implementations>`

Ejecuta el talker y el listener
-------------------------------

Ahora ejecuta ``talker`` y ``listener`` para probar Cyclone DDS.

.. code-block:: bash

   ros2 run demo_nodes_cpp talker

.. code-block:: bash

   ros2 run demo_nodes_cpp listener
