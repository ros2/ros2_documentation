.. redirect-from::

    Tutorials/Understanding-ROS2-Nodes

.. _ROS2Nodes:

Comprender los Nodos
====================

**Objetivo:** Aprender sobre la función de los nodos en ROS 2 y las herramientas para interactuar con ellos.

**Nivel del Tutorial:** Principiante

**Tiempo:** 10 minutos

.. contents:: Contenido
   :depth: 2
   :local:

Historial
---------

1 El grafo ROS 2
^^^^^^^^^^^^^^^^^

En los próximos tutoriales, aprenderás sobre una serie de conceptos básicos de ROS 2 que componen lo que se conoce como el 'grafo de ROS (2)'.

El grafo ROS es una red de elementos de ROS 2, que procesan datos al mismo tiempo.
Abarca todos los ejecutables y las conexiones entre ellos, como si tuviera que mapearlos y visualizarlos.

2 Nodos in ROS 2
^^^^^^^^^^^^^^^^

Cada nodo en ROS debe ser responsable de un solo propósito, en forma modular (por ejemplo, un nodo para controlar los motores de las ruedas, un nodo para controlar un telémetro láser, etc.).
Cada nodo puede enviar y recibir datos a otros nodos a través de topics, servicios, acciones o parámetros.

.. image:: images/Nodes-TopicandService.gif

Un sistema robótico completo se compone de muchos nodos que trabajan en conjunto.
En ROS 2, un único ejecutable (programa C++, programa Python, etc.) puede contener uno o más nodos.

Requisitos previos
------------------

El :doc:`tutorial previo<../Introducing-Turtlesim/Introducing-Turtlesim>` mustra como instalar el paquete ``turtlesim`` utilizado a continuación.

Como siempre, no olvides hacer un source ROS 2 en :doc:`cada terminal nueva<../Configuring-ROS2-Environment>`.

Tareas
------

1 ros2 run
^^^^^^^^^^

El comando ``ros2 run`` lanza un ejecutable desde un paquete.

.. code-block:: console

    ros2 run <package_name> <executable_name>

Para ejecutar Turtlesim, abre una nueva terminal e ingresa el siguiente comando:

.. code-block:: console

    ros2 run turtlesim turtlesim_node

Se abrirá la ventana de turtlesim, como se vió en el :doc:`tutorial previo<../Introducing-Turtlesim/Introducing-Turtlesim>`.

Aquí, el nombre del paquete es ``turtlesim`` y el nombre del ejecutable es ``turtlesim_node``.

Sin embargo, todavía no sabemos el nombre del nodo.
Puedes encontrar nombres de nodos usando el comando ``ros2 node list``.

2 ros2 node list
^^^^^^^^^^^^^^^^

``ros2 node list`` te mostrará los nombres de todos los nodos que están actualmente en ejecución.
Esto es especialmente útil cuando desea interactuar con un nodo o cuando tiene un sistema que ejecuta muchos nodos y necesitas realizar un seguimiento de ellos.

Abre una nueva terminal mientras turtlesim aún se está ejecutando en la otra, e ingrese el siguiente comando:

.. code-block:: console

    ros2 node list

El terminal devolverá el nombre del nodo:

.. code-block:: console

  /turtlesim

Abre otra terminal nueva e inicia el nodo teleop con el comando:

.. code-block:: console

    ros2 run turtlesim turtle_teleop_key

Aquí estamos buscando de nuevo en el paquete turtlesim, esta vez el ejecutable llamado ``turtle_teleop_key``.

Regresa a la terminal donde se ejecutó ``ros2 node list`` y vuelve a ejecutarlo.
Ahora verás los nombres de dos nodos activos:

.. code-block:: console

  /turtlesim
  /teleop_turtle

2.1 Reasignación
~~~~~~~~~~~~~~~~

La reasignación te permite cambiar propiedades predeterminadas de los nodos, como su nombre, nombre del topic, nombres de servicios, etc., a valores personalizados.
En el último tutorial, utilizaste la reasignación en ``turtle_teleop_key`` para cambiar la tortuga que se controla.

Ahora, vamos a reasignar el nombre de nuestro nodo ``/turtlesim``.
En una nueva terminal, ejecuta el siguiente comando:

.. code-block:: console

  ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle

Ya que estás llamando a ``ros2 run`` para que se ejecute en turtlesim nuevamente, se abrirá otra ventana de turtlesim.
Sin embargo, ahora, si regresas a la terminal donde ejecutó la lista de nodos ros2 y lo vuelves a ejecutar, verás tres nombres de nodos:

.. code-block:: console

    /my_turtle
    /turtlesim
    /teleop_turtle

3 ros2 node info
^^^^^^^^^^^^^^^^

Ahora que conoces los nombres de tus nodos, puedes acceder a más información sobre ellos con:

.. code-block:: console

    ros2 node info <node_name>

Para obtener información del nodo ``my_turtle``, ejecuta el siguiente comando:

.. code-block:: console

    ros2 node info /my_turtle

``ros2 node info`` devuelve una lista de suscriptores, publicadores, servicios y acciones (las conexiones del grafo de ROS) que interactúan con ese nodo.
La salida debería verse así:

.. code-block:: console

  /my_turtle
    Subscribers:
      /parameter_events: rcl_interfaces/msg/ParameterEvent
      /turtle1/cmd_vel: geometry_msgs/msg/Twist
    Publishers:
      /parameter_events: rcl_interfaces/msg/ParameterEvent
      /rosout: rcl_interfaces/msg/Log
      /turtle1/color_sensor: turtlesim/msg/Color
      /turtle1/pose: turtlesim/msg/Pose
    Service Servers:
      /clear: std_srvs/srv/Empty
      /kill: turtlesim/srv/Kill
      /my_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
      /my_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
      /my_turtle/get_parameters: rcl_interfaces/srv/GetParameters
      /my_turtle/list_parameters: rcl_interfaces/srv/ListParameters
      /my_turtle/set_parameters: rcl_interfaces/srv/SetParameters
      /my_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
      /reset: std_srvs/srv/Empty
      /spawn: turtlesim/srv/Spawn
      /turtle1/set_pen: turtlesim/srv/SetPen
      /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
      /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
    Service Clients:

    Action Servers:
      /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
    Action Clients:

Ahora intenta ejecutar el mismo comando en el nodo ``/teleop_turtle`` y vea cómo las conexiones difieren de ``my_turtle``.

Aprenderás más sobre los conceptos de conexión de gráficos de ROS, incluidos los tipos de mensajes, en los próximos tutoriales.

Resumen
-------

Un nodo es un elemento fundamental de ROS 2, que es modular y tiene un único propósito en un sistema de robótica.

En este tutorial, utilizaste nodos creados a partir del paquete ``turtlesim`` ejecutando ``turtlesim_node`` y ``turtle_teleop_key``.

Aprendiste a usar el comando ``ros2 node list`` para descubrir nombres de nodos activos y el comando ``ros2 node info`` para obtener información de un nodo en particular.
Estas herramientas son vitales para comprender el flujo de datos en un sistema robótico complejo del mundo real.

Pasos siguientes
----------------

Ahora que comprendes los nodos en ROS 2, puedes continuar con el tutorial :doc:`de topics <../Understanding-ROS2-Topics/Understanding-ROS2-Topics>`.
Los topics son uno de los tipos de comunicación que conecta los nodos.

Contenido Relacionado
---------------------

La página :doc:`../../../Concepts` agrega más detalles al concepto de nodos.
