.. redirect-from::

  Tutorials/Launch-Files/Creating-Launch-Files
  Tutorials/Launch/Creating-Launch-Files

Crear un fichero de launch
==========================

**Objetivo:** Crear un fichero de launch para ejecutar un sistema complejo de ROS2.

**Nivel del tutorial:** Intermedio

**Tiempo:** 10 minutos

.. contents:: Contenidos
   :depth: 2
   :local:

Prerequisitos
-------------

Este tutorial usa los paquetes :doc:`rqt_graph y turtlesim <../../Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim>`.

También deberás utilizar un editor de texto de su preferencia.

Como siempre, no te olvides de sincronizar las fuentes de ROS2 en :doc:`cada terminal que abra <../../Beginner-CLI-Tools/Configuring-ROS2-Environment>`.

Antecedentes
------------

El sistema de launch en ROS2 es responsable de ayudar al usuario en la descripción de la configuración de su sistema y de ejecutarlo como fue descrito.
La configuración del sistema incluye que programas se deben de ejecutar, donde ejecutarlos, que argumentos se les deben de asignar, así como convenciones específicas de ROS que hacen más sencillo la reusabilidad de los componentes a través del sistema proporcionándoles una configuración diferente.
También es responsable de monitorizar el estado de los procesos ejecutados, y reportar y/o reaccionar a los cambios en los estados de dichos procesos.

Los ficheros de launch escritos en Python, XML, o YAML pueden empezar o parar diferentes nodos, así como también activar y actuar a varios eventos.
Vea :doc:`../../../How-To-Guides/Launch-file-different-formats` para una descripción de los diferentes formatos.
El paquete proveedor de este framework es ``launch_ros``, el cual usa de fondo el framework no específico de ROS ``launch``.

El `documento de diseño <https://design.ros2.org/articles/roslaunch.html>`__ detalla el objetivo detrás del diseño del sistema de launch de ROS2 (actualmente no todas las funcionalidades están disponibles).

Tareas
------

1 Configuración
^^^^^^^^^^^^^^^

Cree una nueva carpeta donde guardar sus ficheros de launch:

.. code-block:: console

  mkdir launch

2 Escribir un fichero de launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Vamos a componer un fichero de launch de ROS2 usando el paquete de ``turtlesim`` y sus ejecutables.
Como fue mencionado anteriormente, este puede ser es Python, XML, o YAML.

.. tabs::

  .. group-tab:: Python

    Copia y pega el código completo en el fichero ``launch/turtlesim_mimic_launch.py``:

    .. code-block:: python

      from launch import LaunchDescription
      from launch_ros.actions import Node

      def generate_launch_description():
          return LaunchDescription([
              Node(
                  package='turtlesim',
                  namespace='turtlesim1',
                  executable='turtlesim_node',
                  name='sim'
              ),
              Node(
                  package='turtlesim',
                  namespace='turtlesim2',
                  executable='turtlesim_node',
                  name='sim'
              ),
              Node(
                  package='turtlesim',
                  executable='mimic',
                  name='mimic',
                  remappings=[
                      ('/input/pose', '/turtlesim1/turtle1/pose'),
                      ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
                  ]
              )
          ])

  .. group-tab:: XML

    Copia y pega el código completo en el fichero ``launch/turtlesim_mimic_launch.xml``:

    .. code-block:: xml

      <launch>
        <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim1"/>
        <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim2"/>
        <node pkg="turtlesim" exec="mimic" name="mimic">
          <remap from="/input/pose" to="/turtlesim1/turtle1/pose"/>
          <remap from="/output/cmd_vel" to="/turtlesim2/turtle1/cmd_vel"/>
        </node>
      </launch>

  .. group-tab:: YAML

    Copia y pega el código completo en el fichero ``launch/turtlesim_mimic_launch.yaml``:

    .. code-block:: yaml

      launch:

      - node:
          pkg: "turtlesim"
          exec: "turtlesim_node"
          name: "sim"
          namespace: "turtlesim1"

      - node:
          pkg: "turtlesim"
          exec: "turtlesim_node"
          name: "sim"
          namespace: "turtlesim2"

      - node:
          pkg: "turtlesim"
          exec: "mimic"
          name: "mimic"
          remap:
          -
              from: "/input/pose"
              to: "/turtlesim1/turtle1/pose"
          -
              from: "/output/cmd_vel"
              to: "/turtlesim2/turtle1/cmd_vel"


2.1 Examine el fichero de launch
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Todos los ficheros de launch anteriores están ejecutando tres nodos, todos del paquete de ``turtlesim``.
El objetivo del sistema es lanzar dos ventanas de turtlesim, y que una tortuga imite los movimientos de la otra.

Al lanzar los dos nodos de turtlesim, la única diferencia entre ellos son sus valores de namespace.
Namespaces únicos permite al sistema empezar dos nodos sin tener conflictos de nombres del nodo o conflictos de nombre de los topics.
Ambas tortugas en este sistema reciben comandos a través del mismo topic y publican su pose a través del mismo topic.

Con namespaces únicos, se pueden distinguir los mensajes destinados a diferentes tortugas.

El nodo final también es del paquete de ``turtlesim``, pero es un ejecutable diferente ``mimic``.
Este nodo ha añadido detalles de configuración en forma de reasignaciones
El topic ``/input/pose`` de ``mimic`` es reasignado a ``/turtlesim1/turtle1/pose`` y su topic ``/output/cmd_vel`` a ``/turtlesim2/turtle1/cmd_vel``
Esto significa que ``mimic`` se subscribirá al topic de pose de ``/turtlesim1/sim`` y lo republicará al topic de comando de velocidad de ``/turtlesim2/sim``.
En otras palabras, ``turtlesim2`` imitará los movimientos de ``turtlesim1``.

.. tabs::

  .. group-tab:: Python

    Estas declaraciones de importación extraerán algunos modulos ``launch`` de Python.

    .. code-block:: python

      from launch import LaunchDescription
      from launch_ros.actions import Node

    A continuación, comienza la descripción del launch propiamente dicha:

    .. code-block:: python

      def generate_launch_description():
         return LaunchDescription([

         ])

    Las primeras dos acciones en la descripción de launch, lanza las dos ventanas de turtlesim:

    .. code-block:: python

      Node(
          package='turtlesim',
          namespace='turtlesim1',
          executable='turtlesim_node',
          name='sim'
      ),
      Node(
          package='turtlesim',
          namespace='turtlesim2',
          executable='turtlesim_node',
          name='sim'
      ),

    La acción final lanza el nodo de imitación con las reasignaciones:

    .. code-block:: python

      Node(
          package='turtlesim',
          executable='mimic',
          name='mimic',
          remappings=[
            ('/input/pose', '/turtlesim1/turtle1/pose'),
            ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
          ]
      )

  .. group-tab:: XML

    Las primeras dos aciones lanzan las dos ventanas de turtlesim:

    .. code-block:: xml

      <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim1"/>
      <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim2"/>

    La acción final lanza el nodo de imitación con las reasignaciones:

    .. code-block:: xml

      <node pkg="turtlesim" exec="mimic" name="mimic">
        <remap from="/input/pose" to="/turtlesim1/turtle1/pose"/>
        <remap from="/output/cmd_vel" to="/turtlesim2/turtle1/cmd_vel"/>
      </node>

  .. group-tab:: YAML

    Las primeras dos aciones lanzan las dos ventanas de turtlesim:

    .. code-block:: yaml

      - node:
          pkg: "turtlesim"
          exec: "turtlesim_node"
          name: "sim"
          namespace: "turtlesim1"

      - node:
          pkg: "turtlesim"
          exec: "turtlesim_node"
          name: "sim"
          namespace: "turtlesim2"

    La acción final lanza el nodo de imitación con las reasignaciones:

    .. code-block:: yaml

      - node:
          pkg: "turtlesim"
          exec: "mimic"
          name: "mimic"
          remap:
          -
              from: "/input/pose"
              to: "/turtlesim1/turtle1/pose"
          -
              from: "/output/cmd_vel"
              to: "/turtlesim2/turtle1/cmd_vel"


3 ROS2 launch
^^^^^^^^^^^^^

Para ejecturar el fichero de launch creado arriba, entre a la carpeta creada anteriormente y ejecuta el siguiente comando:

.. tabs::

  .. group-tab:: Python

    .. code-block:: console

      cd launch
      ros2 launch turtlesim_mimic_launch.py

  .. group-tab:: XML

    .. code-block:: console

      cd launch
      ros2 launch turtlesim_mimic_launch.xml

  .. group-tab:: YAML

    .. code-block:: console

      cd launch
      ros2 launch turtlesim_mimic_launch.yaml

.. note::

  Es posible lanzar un fichero de launch directamente (como lo hacemos arriba), o proporcionado por un paquete.
  Cuando es proporcionado por un paquete, la sintaxis es:

  .. code-block:: console

      ros2 launch <package_name> <launch_file_name>

  Aprendiste a crear paquetes en :doc:`../../Beginner-Client-Libraries/Creating-Your-First-ROS2-Package`.

.. note::

  Para paquetes con ficheros de launch, es buena idea añadir una dependencia ``exec_depend`` del paquete ``ros2launch`` en el ``package.xml`` de tu paquete.

  .. code-block:: xml

    <exec_depend>ros2launch</exec_depend>

  Esto ayuda a asegurar que el comando ``ros2  launch`` esta disponible despues de compilar tu paquete.
  Esto tambien asugura que todos los :doc:`formatos de ficheros de launch <../../../How-To-Guides/Launch-file-different-formats>` son reconocidos.

Dos ventanas de turtlesim se abrirán, y verás lo siguientes mensajes de ``[INFO]`` diciendo cuales nodos fueron inicializados por tu fichero de launch.

.. code-block:: console

  [INFO] [launch]: Default logging verbosity is set to INFO
  [INFO] [turtlesim_node-1]: process started with pid [11714]
  [INFO] [turtlesim_node-2]: process started with pid [11715]
  [INFO] [mimic-3]: process started with pid [11716]

Para ver el sistema en acción, abra una nueva terminal y ejecuta el comando ``ros2 topic pub`` en el topic ``/turtlesim1/turtle1/cmd_vel`` para ver la primera tortuga moverse:

.. code-block:: console

  ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"

Verás que ambas tortugas seguirán el mismo camino.

.. image:: images/mimic.png

4 Introspección del sistema con rqt_graph
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Mientras el sistema sigue corriendo, abre una nueva terminal y ejecuta ``rqt_graph`` para tener una mejor idea de las relaciones entre los nodos en tu fichero de launch.

Ejecuta el comando:

.. code-block:: console

  rqt_graph

.. image:: images/mimic_graph.png


Un nodo escondido (el comando ``ros2 topic pub`` que ejecutaste) esta publicando datos al topic ``/turtlesim1/turtle1/cmd_vel`` en la izquierda, al cual el nodo ``/turtlesim1/sim`` esta subscrito.
El resto del grafo muestra lo que se describió anteriormente: ``mimic`` esta subscrito al topic de la pose de ``/turtlesim1/sim``, y publica al topic de comando de velocidad de ``/turtlesim2/sim``.

Resumen
-------

Los ficheros de launch simplifican la ejecución de sistemas complejos con muchos nodos y especifican los detalles de configuración.
Tu puedes crear ficheros de launch usando Python, XML, o YAML, y ejecturarlos usando el comando ``ros2 launch``.
