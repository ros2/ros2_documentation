l. redirect-from::

    Logging-and-logger-configuration
    Tutorials/Logging-and-logger-configuration

Registro de mensajes
====================

.. contents:: Tabla de contenidos
   :depth: 2
   :local:

Consulta `la página de registro <../../Concepts/About-Logging>`para obtener detalles sobre la funcionalidad disponible.

Uso de sentencias de registro en el código
------------------------------------------

Regstro básico
^^^^^^^^^^^^^^

El siguiente código generará un mensaje de registro desde un nodo ROS 2 con la gravedad ``DEBUG``:

.. tabs::

    .. group-tab:: C++

        .. code-block:: C++

            // printf style
            RCLCPP_DEBUG(node->get_logger(), "My log message %d", 4);

            // C++ stream style
            RCLCPP_DEBUG_STREAM(node->get_logger(), "My log message " << 4);

    .. group-tab:: Python

        .. code-block:: python

            node.get_logger().debug('My log message %d' % (4))

Ten en cuenta que, en ambos casos, no se agrega una nueva línea final, ya que la infraestructura de registro agregará una automáticamente.

Registro de mensajes solo la primera vez
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

El siguiente código generará un mensaje de registro desde un nodo ROS 2 con la gravedad ``INFO``, pero solo la primera vez que se activa:

.. tabs::

    .. group-tab:: C++

        .. code-block:: C++

            // printf style
            RCLCPP_INFO_ONCE(node->get_logger(), "My log message %d", 4);

            // C++ stream style
            RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "My log message " << 4);

    .. group-tab:: Python

        .. code-block:: python

            num = 4
            node.get_logger().info(f'My log message {num}', once=True)

Registro de todos los mensajes excepto el primero
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

El siguiente código generará un mensaje de registro desde un nodo ROS 2 con la gravedad ``ADVERTIR``, pero no la primera vez que se activa:

.. tabs::

    .. group-tab:: C++

        .. code-block:: C++

            // printf style
            RCLCPP_WARN_SKIPFIRST(node->get_logger(), "My log message %d", 4);

            // C++ stream style
            RCLCPP_WARN_STREAM_SKIPFIRST(node->get_logger(), "My log message " << 4);

    .. group-tab:: Python

        .. code-block:: python

            num = 4
            node.get_logger().warning('My log message {0}'.format(num), skip_first=True)

Registro de menasjes acelerado
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

El siguiente código generará un mensaje de registro desde un nodo ROS 2 con la gravedad ``ERROR``, pero no más de una vez por segundo.

El parámetro de intervalo que especifica milisegundos entre mensajes debe tener un tipo de datos entero para que pueda convertirse en ``rcutils_duration_value_t`` (un ``int64_t``):

.. tabs::

    .. group-tab:: C++

        .. code-block:: C++

            // printf style
            RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "My log message %d", 4);

            // C++ stream style
            RCLCPP_ERROR_STREAM_THROTTLE(node->get_logger(), *node->get_lock(), 1000, "My log message " << 4);

            // For now, use the nanoseconds() method to use an existing rclcpp::Duration value, see https://github.com/ros2/rclcpp/issues/1929
            RCLCPP_ERROR_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), msg_interval.nanoseconds()/1000000, "My log message " << 4);

    .. group-tab:: Python

        .. code-block:: python

            num = 4
            node.get_logger().error(f'My log message {num}', throttle_duration_sec=1)

Registro acelerado de todos los mensajes menos el primero
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

El siguiente código generará un mensaje de registro desde un nodo ROS 2 con la gravedad ``DEBUG``, no más de una vez por segundo, omitiendo la primera vez que se llama:

.. tabs::

    .. group-tab:: C++

        .. code-block:: C++

            // printf style
            RCLCPP_DEBUG_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "My log message %d", 4);

            RCLCPP_DEBUG_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "My log message " << 4);

    .. group-tab:: Python

        .. code-block:: python

            num = 4
            node.get_logger().debug(f'My log message {num}', skip_first=True, throttle_duration_sec=1.0)

Demo registro de mensajes
-------------------------

En esta `demo <https://github.com/ros2/demos/tree/{REPOS_FILE_BRANCH}/logging_demo>`_, se muestran diferentes tipos de llamadas de registro y el nivel de gravedad de diferentes registradores se configura local y externamente.

Como iniciar la demo:

.. code-block:: bash

   ros2 run logging_demo logging_demo_main

Con el tiempo, verás el resultado de varias llamadas de registro con diferentes propiedades.
Para empezar, solo verás el resultado de las llamadas de registro con gravedad ``INFO`` y superior (``WARN``, ``ERROR``, ``FATAL``).
Ten en cuenta que el primer mensaje solo se registrará una vez, aunque se alcanza la línea en cada iteración, ya que esa es una propiedad de la llamada de registro utilizada para ese mensaje.

Configuración del directorio de registro de mensajes
----------------------------------------------------

El directorio de registro de mensajes se puede configurar a través de dos variables de entorno: ``ROS_LOG_DIR`` y ``ROS_HOME``.
La logica es la siguiente:

* Usa ``$ROS_LOG_DIR`` si ``ROS_LOG_DIR`` está establecido y no está vacío.
* De lo contrario, usa ``$ROS_HOME/log``, usando ``~/.ros`` para ``ROS_HOME`` si no está configurado o si está vacío.

Por ejemplo, para establecer el directorio de registro en ``~/my_logs``:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      export ROS_LOG_DIR=~/my_logs
      ros2 run logging_demo logging_demo_main

  .. group-tab:: macOS

    .. code-block:: bash

      export ROS_LOG_DIR=~/my_logs
      ros2 run logging_demo logging_demo_main

  .. group-tab:: Windows

    .. code-block:: bash

      set "ROS_LOG_DIR=~/my_logs"
      ros2 run logging_demo logging_demo_main

Luego encontrarás los registros en ``~/my_logs/``.

Alternativamente, puedes configurar ``ROS_HOME`` y el directorio de registro será relativo a él (``$ROS_HOME/log``).
``ROS_HOME`` está destinado a ser utilizado por cualquier cosa que necesite un directorio base.
Ten en cuenta que ``ROS_LOG_DIR`` tiene que estar sin configurar o vacío.
Por ejemplo, con ``ROS_HOME`` establecido en ``~/my_ros_home``:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      export ROS_HOME=~/my_ros_home
      ros2 run logging_demo logging_demo_main

  .. group-tab:: macOS

    .. code-block:: bash

      export ROS_HOME=~/my_ros_home
      ros2 run logging_demo logging_demo_main

  .. group-tab:: Windows

    .. code-block:: bash

      set "ROS_HOME=~/my_ros_home"
      ros2 run logging_demo logging_demo_main

Luego encontrarás los registros en ``~/my_ros_home/log/``.

Configuración de nivel de registros: programáticamente
------------------------------------------------------

Después de 10 iteraciones, el nivel de registro se establecerá en ``DEBUG``, lo que hará que se registren mensajes adicionales.

Algunos de estos mensajes de depuración hacen que se evalúen funciones/expresiones adicionales, que anteriormente se omitieron porque las llamadas de registro ``DEBUG`` no estaban habilitadas.
Consulta `el código fuente <https://github.com/ros2/demos/blob/{REPOS_FILE_BRANCH}/logging_demo/src/logger_usage_component.cpp>`__ de la demo para obtener una explicación más detallada de las llamadas utilizadas y consulte el registro de rclcpp documentación para obtener una lista completa de las llamadas de registro admitidas.

Configuración de nivel del registro: externamente
-------------------------------------------------

En el futuro, habrá un enfoque generalizado para la configuración externa de registros en tiempo de ejecución (similar a cómo `rqt_logger_level <https://wiki.ros.org/rqt_logger_level>`__ en ROS 1 permite la configuración de registros a través de llamadas de procedimiento remotas).
**Este concepto aún no se admite oficialmente en ROS 2.**
Mientras tanto, esta demo proporciona un servicio de **ejemplo** al que se puede llamar externamente para solicitar la configuración de los niveles de registro para los nombres conocidos de los registros en el proceso.

La demo iniciada anteriormente ya está ejecutando este servicio de ejemplo.
Para volver a establecer el nivel del registrador de la demostración en ``INFO``\ , llama al servicio con:

.. code-block:: bash

   ros2 service call /config_logger logging_demo/srv/ConfigLogger "{logger_name: 'logger_usage_demo', level: INFO}"

Esta llamada de servicio funcionará en cualquier registro que se esté ejecutando en el proceso, siempre que sepa su nombre.
Esto incluye los registradores en el núcleo de ROS 2, como ``rcl`` (el paquete de biblioteca de cliente común).
Para habilitar el registro de depuración para ``rcl``, llama a:

.. code-block:: bash

   ros2 service call /config_logger logging_demo/srv/ConfigLogger "{logger_name: 'rcl', level: DEBUG}"

Deberías ver como la salida de depuración de ``rcl`` comienza a mostrarse.

Usando el componente de configuración del registro
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

El servidor que responde a las solicitudes de configuración del registro se ha desarrollado como un componente para que pueda agregarse a un sistema basado en composición existente.
Por ejemplo, si estás utilizando `un contenedor para ejecutar sus nodos <../Intermediate/Composition>`, para poder configurar los registros solo necesitas solicitar que cargue adicionalmente el componente ``logging_demo::LoggerConfig`` en el contenedor.

Como ejemplo, si deseas depurar la demostración de ``composition::Talker``, puedes iniciar el hablante normalmente con:

Terminal 1:

.. code-block:: bash

   ros2 run rclcpp_components component_container

Terminal 2:

.. code-block:: bash

   ros2 component load /ComponentManager composition composition::Talker

Y luego, cuando desees habilitar el registro de depuración, carga el componente ``LoggerConfig`` con:

Terminal 2

.. code-block:: bash

   ros2 component load /ComponentManager logging_demo logging_demo::LoggerConfig

Por último, configura todos los registros no establecidos con la gravedad de depuración dirigiéndose al registro con nombre vacío.
Ten en cuenta que los registros que se configuraron específicamente para usar una gravedad particular no se verán afectados por esta llamada.

Terminal 2:

.. code-block:: bash

   ros2 service call /config_logger logging_demo/srv/ConfigLogger "{logger_name: '', level: DEBUG}"

Deberías ver la salida de depuración de cualquier registrador no configurado previamente en el proceso que comienza a aparecer, incluso desde el núcleo de ROS 2.

Configuración de nivel de registro: línea de comando
----------------------------------------------------

A partir del lanzamiento de Bouncy ROS 2, el nivel de severidad para los registros que no han tenido su severidad configurada explícitamente se puede configurar desde la línea de comandos.
Reinicia la demo incluyendo el siguiente argumento de línea de comando:


.. code-block:: bash

   ros2 run logging_demo logging_demo_main --ros-args --log-level debug

Esto configura la gravedad predeterminada para cualquier registro no configurado en el nivel de gravedad de depuración.
Deberías ver la salida de depuración de los registradores de la demostración y del núcleo de ROS 2.

El nivel de gravedad de los registradores individuales se puede configurar desde la línea de comandos.
Reinicia la demo incluyendo los siguientes argumentos de línea de comando:

.. code-block:: bash

   ros2 run logging_demo logging_demo_main --ros-args --log-level logger_usage_demo:=debug


Formato de salida de la consola
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Si deseas un formato más o menos detallado, puedes usar la variable de entorno RCUTILS_CONSOLE_OUTPUT_FORMAT.
Por ejemplo, para obtener adicionalmente la marca de tiempo y la ubicación de las llamadas de registro, deten la demo y reiníciala con la variable de entorno configurada:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"

  .. group-tab:: macOS

    .. code-block:: bash

      export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"

  .. group-tab:: Windows

    .. code-block:: bash

       # set "RCUTILS_CONSOLE_OUTPUT_FORMAT=[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
       ros2 run logging_demo logging_demo_main

Deberías ver la marca de tiempo en segundos y el nombre de la función, el nombre del archivo y el número de línea impresos adicionalmente con cada mensaje.
*La opción ``time`` solo se admite a partir de la versión ROS 2 Bouncy.*

Colorización de la salida de la consola
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

De forma predeterminada, la salida se colorea cuando se dirige a un terminal.
Si deseas forzar su activación o desactivación, puede utilizar la variable de entorno ``RCUTILS_COLORIZED_OUTPUT``.
Por ejemplo:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      export RCUTILS_COLORIZED_OUTPUT=0  # 1 for forcing it

  .. group-tab:: macOS

    .. code-block:: bash

      export RCUTILS_COLORIZED_OUTPUT=0  # 1 for forcing it

  .. group-tab:: Windows

    .. code-block:: bash

       # set "RCUTILS_COLORIZED_OUTPUT=0"
       ros2 run logging_demo logging_demo_main

Deberías ver que los registros de depuración, advertencia, error y fatales no están coloreados ahora.

.. note::

   En Linux y MacOS, forzar la salida coloreada significa que si redirige la salida a un archivo, aparecerán los códigos de color de escape ansi.
   En Windows, el método de colorización se basa en las API de la consola.
   Si es forzado, recibirás una nueva advertencia que indica que la coloración falló.
   El comportamiento predeterminado ya verifica si la salida es una consola o no, por lo que no se recomienda forzar la coloración.

Transmisión predeterminada para la salida de la consola
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

En Foxy y versiones posteriores, la salida de todos los niveles de depuración va a stderr de forma predeterminada. Es posible forzar que toda la salida vaya a la salida estándar configurando la variable de entorno ``RCUTILS_LOGGING_USE_STDOUT`` en ``1``.
Por ejemplo:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      export RCUTILS_LOGGING_USE_STDOUT=1

  .. group-tab:: macOS

    .. code-block:: bash

      export RCUTILS_LOGGING_USE_STDOUT=1

  .. group-tab:: Windows

    .. code-block:: bash

      set "RCUTILS_LOGGING_USE_STDOUT=1"


Salida de consola con búfer de línea
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

De forma predeterminada, todos los resultados de registro no están almacenados en búfer.
Puedes forzar que se almacene en búfer configurando la variable de entorno ``RCUTILS_LOGGING_BUFFERED_STREAM`` en 1.
Por ejemplo:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      export RCUTILS_LOGGING_BUFFERED_STREAM=1

  .. group-tab:: macOS

    .. code-block:: bash

      export RCUTILS_LOGGING_BUFFERED_STREAM=1

  .. group-tab:: Windows

    .. code-block:: bash

      set "RCUTILS_LOGGING_BUFFERED_STREAM=1"

Entonces usa:

.. code-block:: bash

    ros2 run logging_demo logging_demo_main
