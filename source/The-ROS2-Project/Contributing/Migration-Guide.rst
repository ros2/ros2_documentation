.. redirect-from::

    Migration-Guide
    Contributing/Migration-Guide

Guía de migración desde ROS 1
=============================

.. contents:: Tabla de Contenido
   :depth: 2
   :local:

Hay dos tipos diferentes de migraciones de paquetes:

* Migrar el código fuente de un paquete existente de ROS 1 a ROS 2 con la intención de que una parte significativa del código fuente permanezca igual o al menos similar.
   Un ejemplo de esto podría ser `pluginlib <https://github.com/ros/pluginlib>`_ donde el código fuente se mantiene en diferentes ramas dentro del mismo repositorio y, por lo general, los parches se pueden transferir entre esas ramas cuando sea necesario.
* Implementar la misma o similar funcionalidad de un paquete ROS 1 para ROS 2 pero asumiendo que el código fuente será significativamente diferente.
   Un ejemplo de esto podría ser `roscpp <https://github.com/ros/ros_comm/tree/melodic-devel/clients/roscpp>`_ en ROS 1 y `rclcpp <https://github.com/ros2/ rclcpp/tree/rolling/rclcpp>`_ en ROS 2, que son repositorios separados y no comparten ningún código.

Este artículo se centra en el caso anterior y describe los pasos de alto nivel para migrar un paquete ROS 1 a ROS 2.
No pretende ser una instrucción de migración paso a paso y no se considera la "solución" *final*.
Las versiones futuras tendrán como objetivo hacer que la migración sea más fluida y con menos esfuerzo hasta el punto de mantener un solo paquete desde la misma rama para ROS 1 y ROS 2.

requisitos previos
------------------

Antes de poder migrar un paquete de ROS 1 a ROS 2, todas sus dependencias deben estar disponibles en ROS 2.

Pasos de migración
------------------

.. contents::
   :depth: 1
   :local:

Manifiestos de paquete
^^^^^^^^^^^^^^^^^^^^^^

ROS 2 no admite el formato 1 de la especificación del paquete, sino solo las versiones de formato más nuevas (2 y superiores).
Por lo tanto, el archivo ``package.xml`` debe actualizarse al menos al formato 2 si usa el formato 1.
Dado que ROS 1 admite todos los formatos, es seguro realizar esa conversión en el paquete ROS 1.

Algunos paquetes pueden tener nombres diferentes en ROS 2, por lo que es posible que las dependencias deban actualizarse en consecuencia.

Metapaquetes
^^^^^^^^^^^^

ROS 2 no tiene un tipo de paquete especial para metapaquetes.
Los metapaquetes aún pueden existir como paquetes regulares que solo contienen dependencias de tiempo de ejecución.
Al migrar metapaquetes desde ROS 1, simplemente elimina la etiqueta ``<metapackage />`` en el archivo de manifiesto de tu paquete.

Definiciones de mensajes, servicios y acciones
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Los archivos de mensajes deben terminar en ``.msg`` y deben ubicarse en la subcarpeta ``msg``.
Los archivos de servicio deben terminar en ``.srv`` y deben estar ubicados en la subcarpeta ``srv``.
Los archivos de acciones deben terminar en ``.action`` y deben ubicarse en la subcarpeta ``action``.

Es posible que estos archivos deban actualizarse para cumplir con la definición de la interfaz `ROS <https://design.ros2.org/articles/interface_definition.html>`__.
Se han eliminado algunos tipos primitivos y los tipos ``duration`` y ``time`` que eran tipos incorporados en ROS 1 se han reemplazado con definiciones de mensajes normales y deben usarse desde `builtin_interfaces <https://github.com /ros2/rcl_interfaces/tree/{REPOS_FILE_BRANCH}/builtin_interfaces>`__ paquete.
Además, algunas convenciones de nomenclatura son más estrictas que en ROS 1.

En tu ``package..xml``:


* Agregar ``<buildtool_depend>rosidl_default_generators</buildtool_depend>``.
* Agregar ``<exec_depend>rosidl_default_runtime</exec_depend>``.
* Para cada paquete de mensajes dependiente, agregue ``<depend>message_package</depend>``.

En tu ``CMakeLists.txt``:

* Comienza habilitando C++14

.. code-block:: cmake

   set(CMAKE_CXX_STANDARD 14)


* Añadir ``find_package(rosidl_default_generators REQUIRED)``
* Para cada paquete de mensajes dependiente, agregue ``find_package(message_package REQUIRED)`` y Remplaza la llamada de función CMake a ``generate_messages`` con ``rosidl_generate_interfaces``.

Esto reemplazará la lista ``add_message_files`` y ``add_service_files`` de todos los archivos de mensajes y servicios, que se pueden eliminar.

sistema de construcción
^^^^^^^^^^^^^^^^^^^^^^^

El sistema de construcción en ROS 2 se llama `ament <https://design.ros2.org/articles/ament.html>`__
y la herramienta de compilación es :doc:`colcon <../../Tutorials/Beginner-Client-Libraries/Colcon-Tutorial>`.
Ament se basa en CMake: ``ament_cmake`` proporciona funciones de CMake para facilitar la escritura de archivos ``CMakeLists.txt``.

Herramienta de compilación
~~~~~~~~~~~~~~~~~~~~~~~~~~

En lugar de usar ``catkin_make``, ``catkin_make_isolated`` o ``catkin build``, ROS 2 usa la herramienta de línea de comandos `colcon <https://design.ros2.org/articles/build_tool.html>`__ para construir e instalar un conjunto de paquetes.

Paquete Python puro
~~~~~~~~~~~~~~~~~~~

Si el paquete ROS 1 usa CMake solo para invocar el archivo ``setup.py`` y no contiene nada además del código Python (por ejemplo, no tiene mensajes, servicios, etc.), debe convertirse en un paquete Python puro en ROS 2 :


*
   Actualice o agregue el tipo de compilación en el archivo ``package.xml``:

   .. code-block:: xml

     <export>
       <build_type>ament_python</build_type>
     </export>

*
   Elimina el archivo ``CMakeLists.txt``

*
   Actualiza los ``setup.py`` para que sea un script de configuración estándar de Python

ROS 2 solo es compatible con Python 3.
Si bien cada paquete puede optar por admitir también Python 2, debe invocar ejecutables con Python 3 si utiliza cualquier API proporcionada por otros paquetes de ROS 2.

Actualiza el *CMakeLists.txt* para usar *ament_cmake*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Aplique los siguientes cambios para usar ``ament_cmake`` en lugar de ``catkin``:


*
   Configure el tipo de compilación en la sección de exportación del archivo ``package.xml``:

  .. code-block:: xml

     <export>
       <build_type>ament_cmake</build_type>
     </export>

*
   Remplaza la invocación ``find_package`` con ``catkin`` y ``COMPONENTS`` con:

  .. code-block:: cmake

     find_package(ament_cmake REQUIRED)
     find_package(component1 REQUIRED)
     # ...
     find_package(componentN REQUIRED)

*
   Mueve y actualiza la invocación ``catkin_package`` con:


   *
     En su lugar, invoca ``ament_package`` pero **después** de que se hayan registrado todos los objetivos.

   *
     El único argumento válido para `ament_package <https://github.com/ament/ament_cmake/blob/{REPOS_FILE_BRANCH}/ament_cmake_core/cmake/core/ament_package.cmake>`__ es ``CONFIG_EXTRAS``.
     Todos los demás argumentos están cubiertos por funciones separadas que deben invocarse *antes* de ``ament_package``:

     * En lugar de pasar ``CATKIN_DEPENDS ...`` llamar ``ament_export_dependencies(...)`` antes.
     * En lugar de pasar ``INCLUDE_DIRS ...`` llame a ``ament_export_include_directories(...)`` antes.
     * En lugar de pasar ``BIBLIOTECAS ...`` llamar ``ament_export_libraries(...)`` antes.

   *
     **POR REALIZAR documento ament_export_targets (``ament_export_interfaces`` en Eloquent y anteriores)?**

*
   Remplaza la invocación de ``add_message_files``, ``add_service_files`` y ``generate_messages`` con `rosidl_generate_interfaces <https://github.com/ros2/rosidl/blob/{REPOS_FILE_BRANCH}/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake>`__.


   *
     El primer argumento es el ``target_name``.
     Si está creando una sola biblioteca, es ``${PROJECT_NAME}``

   *
     Seguido de la lista de nombres de archivos de mensajes, relativos a la raíz del paquete.


     * Si va a utilizar la lista de nombres de archivos varias veces, se recomienda redactar una lista de archivos de mensajes y pasar la lista a la función para mayor claridad.

   *
     El último argumento de palabra clave de múltiples valores para ``generar_mensajes`` es ``DEPENDENCIAS``, que requiere la lista de paquetes de mensajes dependientes.

     .. code-block:: cmake

       rosidl_generate_interfaces(${PROJECT_NAME}
         ${msg_files}
         DEPENDENCIES std_msgs
       )

*
   Elimine cualquier aparición del *espacio de desarrollo*.
   Las variables de CMake relacionadas como ``CATKIN_DEVEL_PREFIX`` ya no existen.


  * Los argumentos ``CATKIN_DEPENDS`` y ``DEPENDS`` se pasan a la nueva función `ament_export_dependencies <https://github.com/ament/ament_cmake/blob/{REPOS_FILE_BRANCH}/ament_cmake_export_dependencies/cmake/ament_export_dependencies.cmake>`__.
  * ``CATKIN_GLOBAL_BIN_DESTINATION``: ``bin``
  * ``CATKIN_GLOBAL_INCLUDE_DESTINATION``: ``include``
  * ``CATKIN_GLOBAL_LIB_DESTINATION``: ``lib``
  * ``CATKIN_GLOBAL_LIBEXEC_DESTINATION``: ``lib``
  * ``CATKIN_GLOBAL_SHARE_DESTINATION``: ``share``
  * ``CATKIN_PACKAGE_BIN_DESTINATION``: ``lib/${PROJECT_NAME}``
  * ``CATKIN_PACKAGE_INCLUDE_DESTINATION``: ``include/${PROJECT_NAME}``
  * ``CATKIN_PACKAGE_LIB_DESTINATION``: ``lib``
  * ``CATKIN_PACKAGE_SHARE_DESTINATION``: ``share/${PROJECT_NAME}``

Pruebas unitarias
~~~~~~~~~~~~~~~~~

Si estás utilizando gtest:

Remplaza ``CATKIN_ENABLE_TESTING`` con ``BUILD_TESTING``.
Remplaza ``catkin_add_gtest`` con ``ament_add_gtest``.

.. code-block:: diff

   -   if (CATKIN_ENABLE_TESTING)
   -     find_package(GTest REQUIRED)  # or rostest
   -     include_directories(${GTEST_INCLUDE_DIRS})
   -     catkin_add_gtest(${PROJECT_NAME}-some-test src/test/some_test.cpp)
   -     target_link_libraries(${PROJECT_NAME}-some-test
   -       ${PROJECT_NAME}_some_dependency
   -       ${catkin_LIBRARIES}
   -       ${GTEST_LIBRARIES})
   -   endif()
   +   if (BUILD_TESTING)
   +     find_package(ament_cmake_gtest REQUIRED)
   +     ament_add_gtest(${PROJECT_NAME}-some-test src/test/test_something.cpp)
   +     ament_target_dependencies(${PROJECT_NAME)-some-test
   +       "rclcpp"
   +       "std_msgs")
   +     target_link_libraries(${PROJECT_NAME}-some-test
   +       ${PROJECT_NAME}_some_dependency)
   +   endif()

Agregue ``<test_depend>ament_cmake_gtest</test_depend>`` a tu ``package.xml``.

.. code-block:: diff

   -   <test_depend>rostest</test_depend>
   +   <test_depend>ament_cmake_gtest</test_depend>

Linters
~~~~~~~

En ROS 2 estamos trabajando para mantener un código limpio usando linters.
Los estilos para diferentes idiomas se definen en nuestra `Guía para desarrolladores <<Developer-Guide>`.

Si está comenzando un proyecto desde cero, se recomienda seguir la guía de estilo y activar las pruebas automáticas de unidades de linter agregando estas líneas justo debajo de ``if(BUILD_TESTING)`` (hasta la versión alfa 5, esto era ``AMENT_ENABLE_TESTING``).

.. code-block:: cmake

   find_package(ament_lint_auto REQUIRED)
   ament_lint_auto_find_test_dependencies()

También deberás agregar las siguientes dependencias a tu ``package.xml``:

.. code-block:: xml

   <test_depend>ament_lint_auto</test_depend>
   <test_depend>ament_lint_common</test_depend>

Continuar usando ``catkin`` en CMake
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ROS 2 usa ament como sistema de compilación, pero por compatibilidad con versiones anteriores, ROS 2 tiene un paquete llamado ``catkin`` que proporciona casi la misma API que catkin en ROS 1.
Para utilizar esta API de compatibilidad con versiones anteriores, ``CMakeLists.txt`` solo debe actualizarse para llamar a la función ``catkin_ament_package()`` *después* de todos los objetivos.

**NOTA: Esto aún no se ha implementado y es solo una idea por el momento.
Debido a la cantidad de cambios relacionados con las dependencias, aún no se ha decidido si esta API de compatibilidad es lo suficientemente útil como para justificar el esfuerzo.**

Actualizar código fuente
^^^^^^^^^^^^^^^^^^^^^^^^

Mensajes, servicios y acciones
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

El espacio de nombres de los mensajes, servicios y acciones de ROS 2 utiliza un subespacio de nombres (``msg``, ``srv`` o ``action``, respectivamente) después del nombre del paquete.
Por lo tanto, una inclusión se parece a: ``#include <my_interfaces/msg/my_message.hpp>``.
El tipo de C++ entonces se llama: ``my_interfaces::msg::MyMessage``.

Los tipos de punteros compartidos se proporcionan como typedefs dentro de las estructuras del mensaje: ``my_interfaces::msg::MyMessage::SharedPtr`` así como ``my_interfaces::msg::MyMessage::ConstSharedPtr``.

Para obtener más detalles, Consulta el artículo sobre `interfaces C++ generadas <https://design.ros2.org/articles/generated_interfaces_cpp.html>`__.

La migración requiere incluir cambios :


* insertar la subcarpeta ``msg`` entre el nombre del paquete y el tipo de datos del mensaje
* cambiar el nombre de archivo incluido de CamelCase a la separación de guión bajo
* cambiar de ``*.h`` a ``*.hpp``

.. code-block:: cpp

   // ROS 1 style is in comments, ROS 2 follows, uncommented.
   // # include <geometry_msgs/PointStamped.h>
   #include <geometry_msgs/msg/point_stamped.hpp>

   // geometry_msgs::PointStamped point_stamped;
   geometry_msgs::msg::PointStamped point_stamped;

La migración requiere código para insertar el espacio de nombres ``msg`` en todas las instancias.

Uso de objetos de servicio
~~~~~~~~~~~~~~~~~~~~~~~~~~

Las devoluciones de llamada de servicio en ROS 2 no tienen valores de retorno booleanos.
En lugar de devolver falso en caso de fallas, se recomienda lanzar excepciones.

.. code-block:: cpp

   // ROS 1 style is in comments, ROS 2 follows, uncommented.
   // #include "nav_msgs/GetMap.h"
   #include "nav_msgs/srv/get_map.hpp"

   // bool service_callback(
   //   nav_msgs::GetMap::Request & request,
   //   nav_msgs::GetMap::Response & response)
   void service_callback(
     const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
     std::shared_ptr<nav_msgs::srv::GetMap::Response> response)
   {
     // ...
     // return true;  // or false for failure
   }

Usos de ros::Time
~~~~~~~~~~~~~~~~~

Para usos de ``ros::Time``:

* Remplaza todas las instancias de ``ros::Time`` con ``rclcpp::Time``

* Si tus mensajes o código utilizan std_msgs::Time:

   * Convierte todas las instancias de std_msgs::Time a builtin_interfaces::msg::Time

   * Convierte todo ``#include "std_msgs/time.h`` a ``#include "builtin_interfaces/msg/time.hpp"``

   * Convierte todas las instancias usando el campo std_msgs::Time ``nsec`` al campo builtin_interfaces::msg::Time ``nanosec``

Usos de ros::Rate
~~~~~~~~~~~~~~~~~

Hay un objeto de tipo equivalente ``rclcpp::Rate`` que es básicamente un reemplazo directo para ``ros::Rate``.

Biblioteca de cliente ROS
~~~~~~~~~~~~~~~~~~~~~~~~~

.. toctree::
   :titlesonly:

   Migration-Guide-Python


**NOTA: Otros por escribir**

Boost
~~~~~

Gran parte de la funcionalidad proporcionada anteriormente por Boost se ha integrado en la biblioteca estándar de C++.
Como tal, nos gustaría aprovechar las nuevas funciones principales y evitar la dependencia de Boost cuando sea posible.

Punteros compartidos
""""""""""""""""""""

Para cambiar los punteros compartidos de boost a C++ estándar, Remplaza las instancias de:


* ``#incluye <boost/shared_ptr.hpp>`` con ``#include <memory>```
* ``boost::shared_ptr`` con ``std::shared_ptr``

También puede haber variantes como ``weak_ptr`` que también desee convertir.

También se recomienda como práctica usar ``using`` en lugar de ``typedef``.
``using`` tiene la capacidad de funcionar mejor en lógica con plantilla.
Para obtener detalles `ver aquí <https://stackoverflow.com/questions/10747810/what-is-the-difference-between-typedef-and-using-in-c11>`__

Hilos/Mutexes
""""""""""""""""""

Otra parte común de boost que se usa en las bases de código de ROS son los mutexes en ``boost::thread``.


* Reemplaza ``boost::mutex::scoped_lock`` con ``std::unique_lock<std::mutex>``
* Remplaza ``boost::mutex`` con ``std::mutex``
* Remplaza ``#include <boost/thread/mutex.hpp>`` con ``#include <mutex>``

Mapa desordenado
""""""""""""""""

Reemplazar:


* ``#include <boost/unordered_map.hpp>`` con ``#include <unordered_map>``
* ``boost::unordered_map`` con ``std::unordered_map``

función
""""""""

Reemplazar:


* ``#include <boost/function.hpp>`` con ``#include <funcional>``
* ``boost::function`` con ``std::function``

Parámetros
----------

En ROS 1, los parámetros están asociados con un servidor central que permitía recuperar parámetros en tiempo de ejecución mediante el uso de las API de la red.
En ROS 2, los parámetros están asociados por nodo y son configurables en tiempo de ejecución con los servicios de ROS.

* Consulta el documento de diseño de parámetros `ROS 2 <https://design.ros2.org/articles/ros_parameters.html>`_ para obtener más detalles sobre el modelo del sistema.

* Consulta :doc:`Uso de la CLI de ROS 2 <../../Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters>` para comprender mejor cómo funcionan las herramientas de la CLI y sus diferencias con el utillaje ROS 1.

* Consulta :doc:`../../How-To-Guides/Parameters-YAML-files-migration-guide` para ver cómo se analizan los archivos de parámetros YAML en ROS 2 y sus diferencias con la implementación de ROS.

Iniciar archivos
----------------

Mientras que los archivos de lanzamiento en ROS 1 siempre se especifican usando archivos `.xml <https://wiki.ros.org/roslaunch/XML>`__, ROS 2 admite scripts de Python para permitir una mayor flexibilidad (Consulta `paquete de lanzamiento <https://github.com/ros2/launch/tree/{REPOS_FILE_BRANCH}/launch>`__), así como archivos XML y YAML.
Consulta el `tutorial <../../How-To-Guides/Launch-files-migration-guide>` sobre la migración de archivos de inicio de ROS 1 a ROS 2.

Ejemplo: convertir un paquete ROS 1 existente para usar ROS 2
-------------------------------------------------------------

Digamos que tenemos un paquete ROS 1 simple llamado ``talker`` que usa ``roscpp``
en un nodo, llamado ``talker``.
Este paquete está en un espacio de trabajo catkin, ubicado en ``~/ros1_talker``.

El código ROS 1
^^^^^^^^^^^^^^^

Aquí está el diseño del directorio de nuestro espacio de trabajo catkin:

.. code-block:: bash

   $ cd ~/ros1_talker
   $ find .
   .
   ./src
   ./src/talker
   ./src/talker/package.xml
   ./src/talker/CMakeLists.txt
   ./src/talker/talker.cpp

Aquí está el contenido de esos tres archivos:

`src/talker/package.xml``:

.. code-block:: xml

   <package>
     <name>talker</name>
     <version>0.0.0</version>
     <description>talker</description>
     <maintainer email="gerkey@osrfoundation.org">Brian Gerkey</maintainer>
     <license>Apache 2.0</license>
     <buildtool_depend>catkin</buildtool_depend>
     <build_depend>roscpp</build_depend>
     <build_depend>std_msgs</build_depend>
     <run_depend>roscpp</run_depend>
     <run_depend>std_msgs</run_depend>
   </package>

``src/talker/CMakeLists.txt``:

.. code-block:: cmake

   cmake_minimum_required(VERSION 2.8.3)
   project(talker)
   find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
   catkin_package()
   include_directories(${catkin_INCLUDE_DIRS})
   add_executable(talker talker.cpp)
   target_link_libraries(talker ${catkin_LIBRARIES})
   install(TARGETS talker
     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

``src/talker/talker.cpp``:

.. code-block:: cpp

   #include <sstream>
   #include "ros/ros.h"
   #include "std_msgs/String.h"
   int main(int argc, char **argv)
   {
     ros::init(argc, argv, "talker");
     ros::NodeHandle n;
     ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
     ros::Rate loop_rate(10);
     int count = 0;
     std_msgs::String msg;
     while (ros::ok())
     {
       std::stringstream ss;
       ss << "hello world " << count++;
       msg.data = ss.str();
       ROS_INFO("%s", msg.data.c_str());
       chatter_pub.publish(msg);
       ros::spinOnce();
       loop_rate.sleep();
     }
     return 0;
   }

Construyendo el código ROS 1
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Obtenemos un archivo de configuración del entorno (en este caso para Jade usando bash), luego
construimos nuestro paquete usando ``catkin_make install``:

.. code-block:: bash

   . /opt/ros/jade/setup.bash
   cd ~/ros1_talker
   catkin_make install

Ejecutando el nodo ROS 1
~~~~~~~~~~~~~~~~~~~~~~~~

Si aún no hay uno ejecutándose, comenzamos un ``roscore``, primero buscando el
archivo de instalación de nuestro árbol de instalación ``catkin`` (el archivo de instalación del sistema en
``/opt/ros/jade/setup.bash`` también funcionaría aquí):

.. code-block:: bash

   . ~/ros1_talker/install/setup.bash
   roscore

En otro shell, ejecutamos el nodo desde el espacio de instalación ``catkin`` usando
``rosrun``, de nuevo obteniendo primero el archivo de instalación (en este caso debe ser el que
desde nuestro espacio de trabajo):

.. code-block:: bash

   . ~/ros1_talker/install/setup.bash
   rosrun talker talker

Migración a ROS 2
^^^^^^^^^^^^^^^^^

Comencemos por crear un nuevo espacio de trabajo en el que trabajar:

.. code-block:: bash

   mkdir ~/ros2_talker
   cd ~/ros2_talker

Copiaremos el árbol fuente de nuestro paquete ROS 1 en ese espacio de trabajo, donde podemos modificarlo:

.. code-block:: bash

   mkdir src
   cp -a ~/ros1_talker/src/talker src

Ahora modificaremos el código C++ en el nodo.
La biblioteca ROS 2 C++, llamada ``rclcpp``, proporciona una API diferente a esa
proporcionada por ``roscpp``.
Los conceptos son muy similares entre las dos bibliotecas, lo que hace que los cambios
sean razonablemente fáciles de hacer.

Encabezados incluidos
~~~~~~~~~~~~~~~~~~~~~

En lugar de ``ros/ros.h``, que nos dio acceso a la API de la biblioteca ``roscpp``,
necesitamos incluir ``rclcpp/rclcpp.hpp``, que nos da acceso a ``rclcpp``
API de la biblioteca:

.. code-block:: cpp

   //#include "ros/ros.h"
   #include "rclcpp/rclcpp.hpp"

Para obtener la definición de mensaje ``std_msgs/String``, en lugar de
``std_msgs/String.h``, necesitamos incluir ``std_msgs/msg/string.hpp``:

.. code-block:: cpp

   //#include "std_msgs/String.h"
   #include "std_msgs/msg/string.hpp"

Cambiar las llamadas a la biblioteca de C++
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

En lugar de pasar el nombre del nodo a la llamada de inicialización de la biblioteca, hacemos
la inicialización, luego pasamos el nombre del nodo a la creación del objeto del nodo
(Podemos usar la palabra clave ``auto`` porque ahora se requiere un compilador C++14):

.. code-block:: cpp

   //  ros::init(argc, argv, "talker");
   //  ros::NodeHandle n;
       rclcpp::init(argc, argv);
       auto node = rclcpp::Node::make_shared("talker");

La creación de los objetos publisher y rate es bastante similar, con algunos
cambios en los nombres de espacio de nombres y métodos.

.. code-block:: cpp

   //  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
   //  ros::Rate loop_rate(10);
     auto chatter_pub = node->create_publisher<std_msgs::msg::String>("chatter",
       1000);
     rclcpp::Rate loop_rate(10);

Para controlar aún más cómo se maneja la entrega de mensajes, se podría pasar
un perfil de calidad de servicio(``QoS``) .
El perfil predeterminado es ``rmw_qos_profile_default``.
Para obtener más detalles, Consulta el
`documento de diseño <https://design.ros2.org/articles/qos.html>`__
y `descripción general del concepto <../../Concepts/About-Quality-of-Service-Settings>`.

La creación del mensaje saliente es diferente en el espacio de nombres:

.. code-block:: cpp

   //  std_msgs::String msg;
     std_msgs::msg::String msg;

En lugar de ``ros::ok()``, llamamos ``rclcpp::ok()``:

.. code-block:: cpp

   //  while (ros::ok())
     while (rclcpp::ok())

Dentro del ciclo de publicación, accedemos al campo ``datos`` como antes:

.. code-block:: cpp

       msg.data = ss.str();

Para imprimir un mensaje de consola, en lugar de usar ``ROS_INFO()``, usamos
``RCLCPP_INFO()`` y sus varios primos.
La diferencia clave es que ``RCLCPP_INFO()`` toma un objeto Logger como el primero
argumento.

.. code-block:: cpp

   //    ROS_INFO("%s", msg.data.c_str());
       RCLCPP_INFO(node->get_logger(), "%s\n", msg.data.c_str());

Publicar el mensaje es igual que antes:

.. code-block:: cpp

       chatter_pub->publish(msg);

Spinning (es decir, dejar que el sistema de comunicaciones procese cualquier
mensajes entrantes/salientes) es diferente en que la llamada ahora toma el nodo como
un argumento:

.. code-block:: cpp

   //    ros::spinOnce();
       rclcpp::spin_some(node);

Dormir usando el objeto de tarifa no cambia.

Poniéndolo todo junto, el nuevo ``talker.cpp`` se ve así:

.. code-block:: cpp

   #include <sstream>
   // #include "ros/ros.h"
   #include "rclcpp/rclcpp.hpp"
   // #include "std_msgs/String.h"
   #include "std_msgs/msg/string.hpp"
   int main(int argc, char **argv)
   {
   //  ros::init(argc, argv, "talker");
   //  ros::NodeHandle n;
     rclcpp::init(argc, argv);
     auto node = rclcpp::Node::make_shared("talker");
   //  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
   //  ros::Rate loop_rate(10);
     auto chatter_pub = node->create_publisher<std_msgs::msg::String>("chatter", 1000);
     rclcpp::Rate loop_rate(10);
     int count = 0;
   //  std_msgs::String msg;
     std_msgs::msg::String msg;
   //  while (ros::ok())
     while (rclcpp::ok())
     {
       std::stringstream ss;
       ss << "hello world " << count++;
       msg.data = ss.str();
   //    ROS_INFO("%s", msg.data.c_str());
       RCLCPP_INFO(node->get_logger(), "%s\n", msg.data.c_str());
       chatter_pub->publish(msg);
   //    ros::spinOnce();
       rclcpp::spin_some(node);
       loop_rate.sleep();
     }
     return 0;
   }

Cambiando el ``package.xml``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ROS 2 no admite el formato 1 de la especificación del paquete, sino solo las versiones de formato más nuevas (2 y superiores).
Empezamos especificando la versión del formato en la etiqueta ``package``:

.. code-block:: xml

   <!-- <package> -->
   <package format="2">

ROS 2 usa una versión más nueva de ``catkin``, llamada ``ament_cmake``, que especificamos en la
etiqueta ``buildtool_depend``:

.. code-block:: xml

   <!--  <buildtool_depend>catkin</buildtool_depend> -->
     <buildtool_depend>ament_cmake</buildtool_depend>

En nuestras dependencias de compilación, en lugar de ``roscpp`` usamos ``rclcpp``, que proporciona
la API de C++ que usamos.

.. code-block:: xml

   <!--  <build_depend>roscpp</build_depend> -->
     <build_depend>rclcpp</build_depend>

Hacemos la misma adición en las dependencias de ejecución y también actualizamos desde el
etiqueta ``run_depend`` a la etiqueta ``exec_depend`` (parte de la actualización a la versión 2 de
el formato del paquete):

.. code-block:: xml

   <!--  <run_depend>roscpp</run_depend> -->
     <exec_depend>rclcpp</exec_depend>
   <!--  <run_depend>std_msgs</run_depend> -->
     <exec_depend>std_msgs</exec_depend>

En ROS 1, usamos ``<depend>`` para simplificar la especificación de dependencias para ambos
tiempo de compilación y tiempo de ejecución.
Podemos hacer lo mismo en ROS 2:

.. code-block:: xml

     <depend>rclcpp</depend>
     <depend>std_msgs</depend>

También necesitamos decirle a la herramienta de compilación qué *tipo* de paquete somos, para que sepa cómo
construirnos.
Debido a que estamos usando ``ament`` y CMake, agregamos las siguientes líneas para declarar nuestro
tipo de compilación para ser ``ament_cmake``:

.. code-block:: xml

     <export>
       <build_type>ament_cmake</build_type>
     </export>

Poniéndolo todo junto, nuestro ``package.xml`` ahora se ve así:

.. code-block:: xml

   <!-- <package> -->
   <package format="2">
     <name>talker</name>
     <version>0.0.0</version>
     <description>talker</description>
     <maintainer email="gerkey@osrfoundation.org">Brian Gerkey</maintainer>
     <license>Apache License 2.0</license>
   <!--  <buildtool_depend>catkin</buildtool_depend> -->
     <buildtool_depend>ament_cmake</buildtool_depend>
   <!--  <build_depend>roscpp</build_depend> -->
   <!--  <run_depend>roscpp</run_depend> -->
   <!--  <run_depend>std_msgs</run_depend> -->
     <depend>rclcpp</depend>
     <depend>std_msgs</depend>
     <export>
       <build_type>ament_cmake</build_type>
     </export>
   </package>

**POR HACER: muestra una versión más simple de este archivo simplemente usando la etiqueta ``<depend>``, que es
habilitado por la versión 2 del formato del paquete (también soportado en ``catkin`` así que,
estrictamente hablando, ortogonal a ROS 2).**

Cambiar el código CMake
~~~~~~~~~~~~~~~~~~~~~~~

ROS 2 se basa en una versión superior de CMake:

.. code-block:: bash

   #cmake_minimum_required(VERSION 2.8.3)
   cmake_minimum_required(VERSION 3.5)

ROS 2 se basa en el estándar C++14.
Según el compilador que esté usando, es posible que la compatibilidad con C++ 14 no esté habilitada
por defecto.
Usando ``gcc`` 5.3 (que es lo que se usa en Ubuntu Xenial), necesitamos habilitarlo
explícitamente, lo que hacemos agregando esta línea cerca de la parte superior del archivo:

.. code-block:: cmake

   set(CMAKE_CXX_STANDARD 14)

La forma preferida de trabajar en todas las plataformas es esta:

.. code-block:: cmake

   if(NOT CMAKE_CXX_STANDARD)
     set(CMAKE_CXX_STANDARD 14)
   endif()
   if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
     add_compile_options(-Wall -Wextra -Wpedantic)
   endif()

Usando ``catkin``, especificamos los paquetes contra los que queremos construir pasándolos
como argumentos ``COMPONENTS`` al encontrar inicialmente ``catkin``.
Con ``ament_cmake``, encontramos cada paquete individualmente, comenzando con ``ament_cmake``:

.. code-block:: cmake

   #find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
   find_package(ament_cmake REQUIRED)
   find_package(rclcpp REQUIRED)
   find_package(std_msgs REQUIRED)

Las dependencias del sistema se pueden encontrar como antes:

.. code-block:: cmake

   find_package(Boost REQUIRED COMPONENTS system filesystem thread)

Llamamos a ``catkin_package()`` para autogenerar cosas como la configuración de CMake
archivos para otros paquetes que usan nuestro paquete.
Mientras que esa llamada ocurre *antes* de especificar objetivos para construir, ahora llamamos a la
``ament_package()`` análogo *después* de los objetivos:

.. code-block:: cmake

   # catkin_package()
   # At the bottom of the file:
   ament_package()

Los únicos directorios que deben incluirse manualmente son los directorios locales
y dependencias que no son paquetes de ament:

.. code-block:: cmake

   #include_directories(${catkin_INCLUDE_DIRS})
   include_directories(include ${Boost_INCLUDE_DIRS})

Una mejor alternativa es especificar incluir directorios para cada objetivo
individualmente, en lugar de incluir todos los directorios para todos los objetivos:

.. code-block:: cmake

   target_include_directories(target PUBLIC include ${Boost_INCLUDE_DIRS})

Similar a cómo encontramos cada paquete dependiente por separado, necesitamos vincular
cada uno al objetivo de compilación.
Para enlazar con paquetes dependientes que son paquetes complementarios, en lugar de usar
``target_link_libraries()``, ``ament_target_dependencies()`` es una forma más
concisa y más completa de manejar las banderas de compilación.
Maneja automáticamente tanto los directorios de inclusión definidos en
``_INCLUDE_DIRS`` y bibliotecas de enlace definidas en ``_LIBRARIES``.

.. code-block:: cmake

   #target_link_libraries(talker ${catkin_LIBRARIES})
   ament_target_dependencies(talker
     rclcpp
     std_msgs)

Para venlazar con paquetes que no son paquetes de ament, como dependencias del sistema
como ``Boost``, o una biblioteca construida en el mismo ``CMakeLists.txt``, use
``target_link_libraries()```:

.. code-block:: cmake

   target_link_libraries(target ${Boost_LIBRARIES})

Para la instalación, ``catkin`` define variables como ``CATKIN_PACKAGE_BIN_DESTINATION``.
Con ``ament_cmake``, solo damos una ruta relativa a la raíz de la instalación, como ``bin``
para ejecutables:

.. code-block:: cmake

   #install(TARGETS talker
   #  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
   install(TARGETS talker
     DESTINATION lib/${PROJECT_NAME})

Opcionalmente, podemos instalar y exportar los directorios incluidos para paquetes posteriores:

.. code-block:: cmake

   install(DIRECTORY include/
     DESTINATION include)
   ament_export_include_directories(include)

Opcionalmente, podemos exportar dependencias para paquetes posteriores:

.. code-block:: cmake

   ament_export_dependencies(std_msgs)

Poniéndolo todo junto, el nuevo ``CMakeLists.txt`` se ve así:

.. code-block:: cmake

   #cmake_minimum_required(VERSION 2.8.3)
   cmake_minimum_required(VERSION 3.5)
   project(talker)
   if(NOT CMAKE_CXX_STANDARD)
     set(CMAKE_CXX_STANDARD 14)
   endif()
   if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
     add_compile_options(-Wall -Wextra -Wpedantic)
   endif()
   #find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
   find_package(ament_cmake REQUIRED)
   find_package(rclcpp REQUIRED)
   find_package(std_msgs REQUIRED)
   #catkin_package()
   #include_directories(${catkin_INCLUDE_DIRS})
   include_directories(include)
   add_executable(talker talker.cpp)
   #target_link_libraries(talker ${catkin_LIBRARIES})
   ament_target_dependencies(talker
     rclcpp
     std_msgs)
   #install(TARGETS talker
   #  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
   install(TARGETS talker
     DESTINATION lib/${PROJECT_NAME})
   install(DIRECTORY include/
     DESTINATION include)
   ament_export_include_directories(include)
   ament_export_dependencies(std_msgs)
   ament_package()

**POR HACER: Muestra cómo se vería esto con ``ament_auto``.**

Construyendo el código ROS 2
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Obtenemos un archivo de configuración del entorno (en este caso, el generado al seguir
el tutorial de instalación de ROS 2, que se construye en ``~/ros2_ws``, luego construimos nuestro
paquete usando ``colcon build``:

.. code-block:: bash

   . ~/ros2_ws/install/setup.bash
   cd ~/ros2_talker
   colcon build

Ejecutando el nodo ROS 2
~~~~~~~~~~~~~~~~~~~~~~~~

Debido a que instalamos el ejecutable ``talker`` en ``bin``, después de obtener el
archivo de instalación, desde nuestro árbol de instalación, podemos invocarlo por nombre directamente
(además, todavía no hay un equivalente de ROS 2 para ``rosrun``):

.. code-block:: bash

   . ~/ros2_ws/install/setup.bash
   talker

Actualizar scripts
^^^^^^^^^^^^^^^^^^

Argumentos de la CLI de ROS
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Desde `ROS Eloquent <../../Releases/Release-Eloquent-Elusor>`, los argumentos de ROS deben incluirse en el ámbito ``--ros-args`` y un final ``--`` (el guión doble final puede ser elidido si no le siguen argumentos).

La reasignación de nombres es similar a ROS 1, tomando la forma ``from:=to``, excepto que debe estar precedida por un indicador ``--remap`` (o ``-r``).
Por ejemplo:

.. code-block:: bash

   ros2 run some_package some_ros_executable --ros-args -r foo:=bar

Usamos una sintaxis similar para los parámetros, usando el indicador ``--param`` (o ``-p``):


.. code-block:: bash

   ros2 run some_package some_ros_executable --ros-args -p my_param:=value

Ten en cuenta que esto es diferente al uso de un guión bajo inicial en ROS 1.

Para cambiar el nombre de un nodo, use ``__node`` (el equivalente de ROS 1 es ``__name``):

.. code-block:: bash

   ros2 run some_package some_ros_executable --ros-args -r __node:=new_node_name

Ten en cuenta el uso de la bandera ``-r``.
Se necesita el mismo indicador de reasignación para cambiar el espacio de nombres ``__ns``:

.. code-block:: bash

   ros2 run some_package some_ros_executable --ros-args -r __ns:=/new/namespace

No hay equivalente en ROS 2 para las siguientes teclas ROS 1:

- ``__log`` (pero ``--log-config-file`` se puede usar para proporcionar un archivo de configuración del registrador)
- ``__ip``
- ``__hostname``
- ``__master``

Para obtener más información, Consulta el `documento de diseño <https://design.ros2.org/articles/ros_command_line_arguments.html>`_.

Referencia rápida
"""""""""""""""""

+----------------+-------------+----------------+
| Característica | ROS 1       | RO2            |
+================+=============+================+
| remapeo        | foo:=bar    | -r foo:=bar    |
+----------------+-------------+----------------+
| parámetros     | _foo:=bar   | -p foo:=bar    |
+----------------+-------------+----------------+
| nombre de nodo | __name:=foo | -r __node:=foo |
+----------------+-------------+----------------+
| namespace      | __ns:=foo   | -r __ns:=foo   |
+----------------+-------------+----------------+


Más ejemplos y herramientas
^^^^^^^^^^^^^^^^^^^^^^^^^^^

- El Migrador de archivos de lanzamiento que convierte un archivo launch XML de ROS 1 en un archivo launch de Python de ROS 2: https://github.com/aws-robotics/ros2-launch-file-migrator
- Amazon ha expuesto sus herramientas para portar robots ROS 1 a ROS 2
    https://github.com/awslabs/ros2-migration-tools/tree/master/porting\_tools


Licencia
--------

En ROS 2, nuestra licencia recomendada es `Apache 2.0 License <https://www.apache.org/licenses/LICENSE-2.0>`__.
En ROS 1, nuestra licencia recomendada era la `Licencia BSD de 3 cláusulas <https://opensource.org/licenses/BSD-3-Clause>`__.

Para cualquier proyecto nuevo recomendamos utilizar la Licencia Apache 2.0, ya sea ROS 1 o ROS 2.

Sin embargo, al migrar código de ROS 1 a ROS 2 no podemos simplemente cambiar la licencia.
La licencia existente debe conservarse para cualquier contribución preexistente.

Con ese fin, si se migra un paquete, recomendamos mantener la licencia existente y continuar contribuyendo a ese paquete bajo la licencia OSI existente, que esperamos sea la licencia BSD para los elementos principales.

Esto mantendrá las cosas claras y fáciles de entender.

Cambiar la licencia
^^^^^^^^^^^^^^^^^^^

Es posible cambiar la licencia, sin embargo, deberá ponerse en contacto con todos los colaboradores y obtener permiso.
Para la mayoría de los paquetes, es probable que esto sea un esfuerzo significativo y no valga la pena considerarlo.
Si el paquete tiene un pequeño conjunto de contribuyentes, entonces esto puede ser factible.
