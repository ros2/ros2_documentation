.. redirect-from::

    Rosbag-with-ROS1-Bridge
    Tutorials/Rosbag-with-ROS1-Bridge

Grabación y reproducción de datos con ``rosbag`` utilizando ROS 1 bridge
========================================================================

Este tutorial es una continuación de la demostración de *Comunicación puente entre ROS 1 y ROS 2* que se puede encontrar `aquí <https://github.com/ros2/ros1_bridge/blob/{REPOS_FILE_BRANCH}/README.md>`__ , y en lo siguiente se supone que ya ha completado ese tutorial.

El ros1_bridge puede instalarse desde `paquetes binarios <../../Installation>` o `construido desde la fuente <https://github.com/ros2/ros1_bridge/blob/{REPOS_FILE_BRANCH}/README.md#building- el-puente-desde-la-fuente>`__; ambos funcionan para estos ejemplos.

Lo que sigue es una serie de ejemplos adicionales, como los que vienen al final de la demo antes mencionada *Puente de comunicación entre ROS 1 y ROS 2*.

Grabación de datos de topics con rosbag y ROS 1 Bridge
------------------------------------------------------

En este ejemplo, usaremos el programa de demostración ``cam2image`` que viene con ROS 2 y un script de Python para emular los datos del sensor de un robot similar a un robot tortuga simple para que podamos conectarlo a ROS 1 y usar rosbag para grabar eso.

Primero ejecutaremos un ROS 1 ``roscore`` en una nueva shell:

.. code-block:: bash

   # Shell A:
   . /opt/ros/kinetic/setup.bash
   # O, en OSX, algo como:
   # . ~/ros_catkin_ws/install_isolated/setup.bash
   roscore

Luego ejecutaremos ROS 1 <=> ROS 2 ``dynamic_bridge`` con la opción ``--bridge-all-topics`` (para que podamos hacer ``rostopic list`` y verlos) en otra shell :

.. code-block:: bash

   # Shell B:
   . /opt/ros/kinetic/setup.bash
   # O, en OSX, algo como:
   # . ~/ros_catkin_ws/install_isolated/setup.bash
   . /opt/ros/ardent/setup.bash
   # O, si compila ROS 2 desde fuentes:
   # . <workspace-with-bridge>/install/setup.bash
   export ROS_MASTER_URI=http://localhost:11311
   ros2 run ros1_bridge dynamic_bridge --bridge-all-topics

Recuerda reemplazar ``<workspace-with-bridge>`` con la ruta a donde extrajo el binario ROS 2 o donde creó ROS 2 desde fuentes.

----

Ahora podemos iniciar los programas ROS 2 que emularán nuestro robot tipo tortuga.
Primero ejecutaremos el programa ``cam2image`` con la opción ``-b`` para que no requiera una cámara para funcionar:

.. code-block:: bash

   # Shell C:
   . /opt/ros/ardent/setup.bash
   # O, si compila ROS 2 desde fuentes:
   # . <workspace-with-bridge>/install/setup.bash
   ros2 run image_tools cam2image -- -b

TODO: usar nombres de topics con espacios de nombres

Luego, ejecutaremos un script de Python simple para emular los topics ``odom`` e ``imu_data`` desde una base de Kobuki.
Usaría el nombre de tema ``~sensors/imu_data`` más preciso para los datos de imu, pero todavía no tenemos soporte de espacio de nombres en ROS 2 (¡ya viene!).
Coloca este script en un archivo llamado ``emulate_kobuki_node.py``:

.. code-block:: python

   #!/usr/bin/env python3

   import sys
   import time

   import rclpy

   from nav_msgs.msg import Odometry
   from sensor_msgs.msg import Imu

   def main():
       rclpy.init(args=sys.argv)

       node = rclpy.create_node('emulate_kobuki_node')

       imu_publisher = node.create_publisher(Imu, 'imu_data')
       odom_publisher = node.create_publisher(Odometry, 'odom')

       imu_msg = Imu()
       odom_msg = Odometry()
       counter = 0
       while True:
           counter += 1
           now = time.time()
           if (counter % 50) == 0:
               odom_msg.header.stamp.sec = int(now)
               odom_msg.header.stamp.nanosec = int(now * 1e9) % 1000000000
               odom_publisher.publish(odom_msg)
           if (counter % 100) == 0:
               imu_msg.header.stamp.sec = int(now)
               imu_msg.header.stamp.nanosec = int(now * 1e9) % 1000000000
               imu_publisher.publish(imu_msg)
               counter = 0
           time.sleep(0.001)


   if __name__ == '__main__':
       sys.exit(main())

Puedes ejecutar este script de python en una nueva shell de ROS 2:

.. code-block:: bash

   # Shell D:
   . /opt/ros/ardent/setup.bash
   # O, si compila ROS 2 desde fuentes:
   # . <workspace-with-bridge>/install/setup.bash
   python3 emulate_kobuki_node.py

----

Ahora que todas las fuentes de datos y el puente dinámico se están ejecutando, podemos ver los topics disponibles en un nuevo shell de ROS 1:

.. code-block:: bash

   # Shell E:
   . /opt/ros/kinetic/setup.bash
   # O, en OSX, algo como:
   # . ~/ros_catkin_ws/install_isolated/setup.bash
   rostopic list

Deberías ver algo como esto:

::

   % rostopic list
   /image
   /imu_data
   /odom
   /rosout
   /rosout_agg

Ahora podemos registrar estos datos con ``rosbag record`` en el mismo shell:

.. code-block:: bash

   # Shell E:
   rosbag record /image /imu_data /odom

Después de unos segundos, puedes ``Ctrl-c`` el comando ``rosbag`` y hacer ``ls -lh`` para ver qué tan grande es el archivo, es posible que veas algo como esto:

.. code-block:: bash

   % ls -lh
   total 0
   -rw-rw-r-- 1 william william  12M Feb 23 16:59 2017-02-23-16-59-47.bag

Aunque el nombre del archivo será diferente para su bolso (ya que se deriva de la fecha y la hora).

Reproducción de datos de topics con rosbag y ROS 1 Bridge
---------------------------------------------------------

Ahora que tenemos un archivo bag, puede usar cualquiera de las herramientas de ROS 1 para introspeccionar el archivo bag, como ``rosbag info <archivo bag>``, ``rostopic list -b <archivo bag>``, o `` rqt_bag <archivo de bolsa>``.
Sin embargo, también podemos reproducir datos de bolsa en ROS 2 usando ``rosbag play`` y ROS 1 <=> ROS 2 ``dynamic_bridge``.

Primero cierra todos los shells que abrió para el tutorial anterior, deteniendo cualquier programa en ejecución.

Luego, en una nueva shell, inicia ``roscore``:

.. code-block:: bash

   # Shell P:
   . /opt/ros/kinetic/setup.bash
   # O, en OSX, algo como:
   # . ~/ros_catkin_ws/install_isolated/setup.bash
   roscore

Luego ejecuta ``dynamic_bridge`` en otro shell:

.. code-block:: bash

   # Shell Q:
   . /opt/ros/kinetic/setup.bash
   # O, en OSX, algo como:
   # . ~/ros_catkin_ws/install_isolated/setup.bash
   . /opt/ros/ardent/setup.bash
   # O, si compila ROS 2 desde fuentes:
   # . <workspace-with-bridge>/install/setup.bash
   export ROS_MASTER_URI=http://localhost:11311
   ros2 run ros1_bridge dynamic_bridge --bridge-all-topics

Luego reproduzca los datos de la bolsa con ``rosbag play`` en otro shell nuevo, usando la opción ``--loop`` para que no tengamos que reiniciarlo continuamente para bolsas cortas:

.. code-block:: bash

   # Shell R:
   . /opt/ros/kinetic/setup.bash
   # O, en OSX, algo como:
   # . ~/ros_catkin_ws/install_isolated/setup.bash
   rosbag play --loop path/to/bag_file

Asegúrate de reemplazar ``path/to/bag_file`` con la ruta al archivo de bolsa que desea reproducir.

----

Ahora que los datos se están reproduciendo y el puente se está ejecutando, podemos ver los datos que se encuentran en ROS 2.

.. code-block:: bash

   # Shell S:
   . /opt/ros/ardent/setup.bash
   # O, si compila ROS 2 desde fuentes:
   # . <workspace-with-bridge>/install/setup.bash
   ros2 topic list
   ros2 topic echo /odom

Deberías ver algo como:

::

   % ros2 topic list
   /clock
   /image
   /imu_data
   /odom
   /parameter_events

También puedes ver la imagen que se está reproduciendo desde la bolsa utilizando la herramienta ``showimage``:

.. code-block:: bash

   ros2 run image_tools showimage
