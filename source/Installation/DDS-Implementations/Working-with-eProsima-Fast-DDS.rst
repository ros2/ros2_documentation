eProsima Fast DDS
=================

eProsima Fast DDS es una implementación completa de DDS de código abierto para sistemas operativos y arquitecturas embebidas en tiempo real.
Ver también: https://www.eprosima.com/index.php/products-all/eprosima-fast-dds

Prerequisitos
-------------

Tener `rosdep instalado <https://wiki.ros.org/rosdep#Installing_rosdep>`__

Instalar paquetes
-----------------

La forma más fácil es instalar desde el repositorio apt de ROS 2.

.. code-block:: bash

   sudo apt install ros-{DISTRO}-rmw-fastrtps-cpp

Compilar a partir del código fuente
-----------------------------------

Compilar a partir del código fuente también es otra forma de instalar.

Primero, clona Fast DDS y rmw_fastrtps en el directorio src del espacio de trabajo de ROS 2.

.. code-block:: bash

   cd ros2_ws/src
   git clone https://github.com/ros2/rmw_fastrtps ros2/rmw_fastrtps
   git clone https://github.com/eProsima/Fast-DDS eProsima/fastrtps

Luego, instala los paquetes necesarios para Fast DDS.

.. code-block:: bash

   cd ..
   rosdep install --from src -i

Finalmente, ejecuta colcon build.

.. code-block:: bash

   colcon build --symlink-install

Cambiar a rmw_fastrtps
----------------------

eProsima Fast DDS RMW se puede seleccionar especificando la variable de entorno:

.. code-block:: bash

   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

Consulta también: :doc:`Trabajar con varias implementaciones de RMW <../../How-To-Guides/Working-with-multiple-RMW-implementations>`

Ejecuta el talker y el listener
-------------------------------

Ahora ejecuta ``talker`` y ``listener`` para probar Fast DDS.

.. code-block:: bash

   ros2 run demo_nodes_cpp talker

.. code-block:: bash

   ros2 run demo_nodes_cpp listener

