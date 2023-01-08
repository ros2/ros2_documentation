.. redirect-from::

    Tutorials/Parameters/Understanding-ROS2-Parameters

.. _ROS2Params:

Comprender los Parámetros
=========================

**Objetivo:** Aprenda a obtener, configurar, guardar y recargar parámetros en ROS 2.

**Nivel del Tutorial:** Principiant

**Tiempo:** 5 minutos

.. contents:: Contenido
   :depth: 2
   :local:

Historial
---------

Un parámetro es un valor de configuración de un nodo.
Puedes pensar en los parámetros como configuraciones de nodo.
Un nodo puede almacenar parámetros como enteros, flotantes, booleanos, cadenas y listas.
En ROS 2, cada nodo mantiene sus propios parámetros.
Para obtener más información sobre los parámetros, puedes consultar :doc:`el documento de conceptos<../../../Concepts/About-ROS-2-Parameters>`.

Requisitos previos
------------------

Este tutorial utiliza el paquete :doc:`turtlesim <../Introducing-Turtlesim/Introducing-Turtlesim>`.

Como siempre, no olvide ejecutar `source` con el archivo de setup :doc:`en cada nueva terminal que abra<../Configuring-ROS2-Environment>`.

Tareas
------

1 Configuración
^^^^^^^^^^^^^^^

Inicia los dos nodos de turtlesim, ``/turtlesim`` y ``/teleop_turtle``.

Abra una nueva terminal y ejecute:

.. code-block:: console

    ros2 run turtlesim turtlesim_node

Abre otra terminal y ejecuta:

.. code-block:: console

    ros2 run turtlesim turtle_teleop_key


2 ros2 param list
^^^^^^^^^^^^^^^^^

Para ver los parámetros que pertenecen a sus nodos, abre una nueva terminal e ingresa el comando:

.. code-block:: console

    ros2 param list

Verás los espacios de nombres de los nodos, ``/teleop_turtle`` y ``/turtlesim``, seguidos de los parámetros de cada nodo:

.. code-block:: console

  /teleop_turtle:
    qos_overrides./parameter_events.publisher.depth
    qos_overrides./parameter_events.publisher.durability
    qos_overrides./parameter_events.publisher.history
    qos_overrides./parameter_events.publisher.reliability
    scale_angular
    scale_linear
    use_sim_time
  /turtlesim:
    background_b
    background_g
    background_r
    qos_overrides./parameter_events.publisher.depth
    qos_overrides./parameter_events.publisher.durability
    qos_overrides./parameter_events.publisher.history
    qos_overrides./parameter_events.publisher.reliability
    use_sim_time

Cada nodo tiene el parámetro ``use_sim_time``; no es exclusivo de turtlesim.

Según sus nombres, parece que los parámetros de ``/turtlesim`` determinan el color de fondo de la ventana de turtlesim usando valores de color RGB.

Para determinar el tipo de un parámetro, puede usar ``ros2 param get``.


3 ros2 param get
^^^^^^^^^^^^^^^^

Para mostrar el tipo y el valor actual de un parámetro, use el comando:

.. code-block:: console

    ros2 param get <node_name> <parameter_name>

Por ejemplo, para ver el valor actual del parámetro ``background_g`` de ``/turtlesim``, ejecuta:

.. code-block:: console

    ros2 param get /turtlesim background_g

Lo que devolverá el valor:

.. code-block:: console

    Integer value is: 86

Now you know ``background_g`` holds an integer value.

Ahora sabes que ``background_g`` tiene un valor entero.

Si ejecutas el mismo comando en ``background_r`` y ``background_b``, obtendrás los valores 69 y 255, respectivamente.

4 ros2 param set
^^^^^^^^^^^^^^^^

Para cambiar el valor de un parámetro en tiempo de ejecución, usa el comando:

.. code-block:: console

    ros2 param set <node_name> <parameter_name> <value>

Cambiemos el color de fondo de ``/turtlesim``:

.. code-block:: console

    ros2 param set /turtlesim background_r 150

La terminal debería devolver el mensaje:

.. code-block:: console

  Set parameter successful

Y el fondo de la ventana de turtlesim debería cambiar de color:

.. image:: images/set.png

Establecer parámetros con el comando ``set`` solo los cambiará en su sesión actual, no de forma permanente.
Sin embargo, puedes guardar tu configuración y volver a cargarla la próxima vez que inicie un nodo.

5 ros2 param dump
^^^^^^^^^^^^^^^^^

Puedes ver todos los parámetros y sus valores actuales de un nodo usando el comando:

.. code-block:: console

  ros2 param dump <node_name>

El comando se imprime en la salida estándar (stdout) de forma predeterminada, pero también puede redirigir los valores de los parámetros a un archivo para guardarlos más adelante.
Para guardar la configuración actual de los parámetros de ``/turtlesim`` en el archivo ``turtlesim.yaml``, ingresa el comando:

.. code-block:: console

  ros2 param dump /turtlesim > turtlesim.yaml

Encontrarás un nuevo archivo en el directorio de trabajo en el que se está ejecutando tu terminal.
Si abres este archivo, verá el siguiente contenido:

.. code-block:: YAML

  /turtlesim:
    ros__parameters:
      background_b: 255
      background_g: 86
      background_r: 150
      qos_overrides:
        /parameter_events:
          publisher:
            depth: 1000
            durability: volatile
            history: keep_last
            reliability: reliable
      use_sim_time: false

Guardar los parámetros resulta útil si deseas volver a cargar el nodo con los mismos parámetros en el futuro.

6 ros2 param load
^^^^^^^^^^^^^^^^^

Puedes cargar parámetros desde un archivo a un nodo actualmente en ejecución usando el comando:

.. code-block:: console

  ros2 param load <node_name> <parameter_file>

Para cargar el archivo ``turtlesim.yaml`` generado con ``ros2 param dump`` en los parámetros del nodo ``/turtlesim``, ingresa el comando:

.. code-block:: console

  ros2 param load /turtlesim turtlesim.yaml

La terminal devolverá el mensaje:

.. code-block:: console

  Set parameter background_b successful
  Set parameter background_g successful
  Set parameter background_r successful
  Set parameter qos_overrides./parameter_events.publisher.depth failed: parameter 'qos_overrides./parameter_events.publisher.depth' cannot be set because it is read-only
  Set parameter qos_overrides./parameter_events.publisher.durability failed: parameter 'qos_overrides./parameter_events.publisher.durability' cannot be set because it is read-only
  Set parameter qos_overrides./parameter_events.publisher.history failed: parameter 'qos_overrides./parameter_events.publisher.history' cannot be set because it is read-only
  Set parameter qos_overrides./parameter_events.publisher.reliability failed: parameter 'qos_overrides./parameter_events.publisher.reliability' cannot be set because it is read-only
  Set parameter use_sim_time successful

.. note::

  Los parámetros de solo lectura solo se pueden modificar al inicio y no después, por eso hay algunas advertencias para los parámetros 'qos_overrides'.

7 Cargar archivo de parámetros al iniciar el nodo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Para iniciar el mismo nodo usando los valores de parámetros guardados, ejecuta:

.. code-block:: console

  ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>

Stop your running turtlesim node so you can try reloading it with your saved parameters, using:
Este es el mismo comando que utilizas para iniciar turtlesim, con las banderas añadidas ``--ros-args`` y ``--params-file``, seguidas del archivo que desea cargar.

Intentá detener el nodo turtlesim en ejecución, para volver a cargarlo con sus parámetros guardados usando:

.. code-block:: console

  ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml

La ventana de turtlesim debería aparecer como de costumbre, pero con el fondo morado que configuraste anteriormente.

.. note::

  En este caso, los parámetros se modifican al inicio, por lo que los parámetros de solo lectura especificados también tendrán efecto.

Resumen
-------

Los nodos tienen parámetros para definir sus valores de configuración predeterminados.
Puedes obtener y establecer valores de parámetros desde la línea de comandos.
También puedes guardar la configuración de los parámetros en un archivo para volver a cargarlos en una sesión futura.

Pasos siguientes
----------------

Volviendo a los métodos de comunicación de ROS 2, en el próximo tutorial aprenderás sobre :doc:`acciones <../Understanding-ROS2-Actions/Understanding-ROS2-Actions>`.
