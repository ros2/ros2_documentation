.. redirect-from::

    Tutorials/Launch-Files/Using-Event-Handlers
    Tutorials/Launch/Using-Event-Handlers

Utilizar controladores de eventos
=================================

**Objetivo:** Aprender acerca de los controladores de eventos en ficheros de launch de ROS 2

**Nivel del tutorial:** Intermedio

**Tiempo:** 15 minutos

.. contents:: Tabla de Contenido
   :depth: 2
   :local:

Antecedentes
------------

Launch en ROS 2 es un sistema que ejecuta y maneja procesos definidos por un usuario.
Es responsable de monitorizar el estado de los procesos launched, así como de reportar y reaccionar a los cambios en el estado de esos procesos.
Estos cambios son llamados eventos y pueden ser controlados al registrar un controlador de eventos con el sistema de launch.
Controladores de eventos pueden ser registrados para eventos específicos y puede ser útiles para monitorizar el estado de procesos.
Adicionalmente, ellos puede ser usados para definir un conjunto complejo de reglas, las cuales pueden ser usadas para modificar dinámicamente el fichero de launch.

Este tutorial muestra ejemplos de uso de controladores de eventos en ficheros de launch de ROS 2.

Prerequisitos
-------------

Este tutorial usa el paquete de :doc:`turtlesim <../../Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim>`.
Este tutorial también asume que tu has :doc:`creado un nuevo paquete <../../Beginner-Client-Libraries/Creating-Your-First-ROS2-Package>` de tipo de compilación ``ament_python`` llamado ``launch_tutorial``.

Este tutorial extiende el código mostrado en el tutorial :doc:`Utilizar substituciones en ficheros de launch <./Using-Substitutions>`.

Utilizar controladores de eventos
---------------------------------

1 Ejemplo de fichero de launch con controladores de evento
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Crea un nuevo fichero llamado ``example_event_handlers_launch.py`` en la carpeta ``launch`` del paquete ``launch_tutorial``.

.. code-block:: python

    from launch_ros.actions import Node

    from launch import LaunchDescription
    from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                                LogInfo, RegisterEventHandler, TimerAction)
    from launch.conditions import IfCondition
    from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                    OnProcessIO, OnProcessStart, OnShutdown)
    from launch.events import Shutdown
    from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                    LaunchConfiguration, LocalSubstitution,
                                    PythonExpression)


    def generate_launch_description():
        turtlesim_ns = LaunchConfiguration('turtlesim_ns')
        use_provided_red = LaunchConfiguration('use_provided_red')
        new_background_r = LaunchConfiguration('new_background_r')

        turtlesim_ns_launch_arg = DeclareLaunchArgument(
            'turtlesim_ns',
            default_value='turtlesim1'
        )
        use_provided_red_launch_arg = DeclareLaunchArgument(
            'use_provided_red',
            default_value='False'
        )
        new_background_r_launch_arg = DeclareLaunchArgument(
            'new_background_r',
            default_value='200'
        )

        turtlesim_node = Node(
            package='turtlesim',
            namespace=turtlesim_ns,
            executable='turtlesim_node',
            name='sim'
        )
        spawn_turtle = ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                ' service call ',
                turtlesim_ns,
                '/spawn ',
                'turtlesim/srv/Spawn ',
                '"{x: 2, y: 2, theta: 0.2}"'
            ]],
            shell=True
        )
        change_background_r = ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                ' param set ',
                turtlesim_ns,
                '/sim background_r ',
                '120'
            ]],
            shell=True
        )
        change_background_r_conditioned = ExecuteProcess(
            condition=IfCondition(
                PythonExpression([
                    new_background_r,
                    ' == 200',
                    ' and ',
                    use_provided_red
                ])
            ),
            cmd=[[
                FindExecutable(name='ros2'),
                ' param set ',
                turtlesim_ns,
                '/sim background_r ',
                new_background_r
            ]],
            shell=True
        )

        return LaunchDescription([
            turtlesim_ns_launch_arg,
            use_provided_red_launch_arg,
            new_background_r_launch_arg,
            turtlesim_node,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=turtlesim_node,
                    on_start=[
                        LogInfo(msg='Turtlesim started, spawning turtle'),
                        spawn_turtle
                    ]
                )
            ),
            RegisterEventHandler(
                OnProcessIO(
                    target_action=spawn_turtle,
                    on_stdout=lambda event: LogInfo(
                        msg='Spawn request says "{}"'.format(
                            event.text.decode().strip())
                    )
                )
            ),
            RegisterEventHandler(
                OnExecutionComplete(
                    target_action=spawn_turtle,
                    on_completion=[
                        LogInfo(msg='Spawn finished'),
                        change_background_r,
                        TimerAction(
                            period=2.0,
                            actions=[change_background_r_conditioned],
                        )
                    ]
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=turtlesim_node,
                    on_exit=[
                        LogInfo(msg=(EnvironmentVariable(name='USER'),
                                ' closed the turtlesim window')),
                        EmitEvent(event=Shutdown(
                            reason='Window closed'))
                    ]
                )
            ),
            RegisterEventHandler(
                OnShutdown(
                    on_shutdown=[LogInfo(
                        msg=['Launch was asked to shutdown: ',
                            LocalSubstitution('event.reason')]
                    )]
                )
            ),
        ])

Las acciones ``RegisterEventHandler`` para los eventos ``OnProcessStart``, ``OnProcessIO``, ``OnExecutionComplete``, ``OnProcessExit`` y ``OnShutdown`` fueron definidas en la descripción del launch.

El controlador de evento ``OnProcessStart`` es usado para registrar una función callback que es ejecutada cuando el nodo turtlesim empieza.
Registra un mensaje en la consola y ejecuta la acción ``spawn_turtle`` cuando el nodo turtlesim empieza.

.. code-block:: python

    RegisterEventHandler(
        OnProcessStart(
            target_action=turtlesim_node,
            on_start=[
                LogInfo(msg='Turtlesim started, spawning turtle'),
                spawn_turtle
            ]
        )
    ),

El controlador de evento ``OnProcessIO`` es usado para registrar una función callback que es ejecutada cuando la acción ``spawn_turtle`` escribe su salida estándar.
Registra el resultado de la solicitud de despliegue.

.. code-block:: python

    RegisterEventHandler(
        OnProcessIO(
            target_action=spawn_turtle,
            on_stdout=lambda event: LogInfo(
                msg='Spawn request says "{}"'.format(
                    event.text.decode().strip())
            )
        )
    ),

El controlador de evento ``OnExecutionComplete`` es usado para registrar una función callback que se ejecuta cuando la acción ``spawn_turtle`` es completada.
Registra un mensaje en la consola y ejecuta las acciones ``change_background_r`` y ``change_background_r_conditioned`` cuando la acción de despliegue es completada.

.. code-block:: python

    RegisterEventHandler(
        OnExecutionComplete(
            target_action=spawn_turtle,
            on_completion=[
                LogInfo(msg='Spawn finished'),
                change_background_r,
                TimerAction(
                    period=2.0,
                    actions=[change_background_r_conditioned],
                )
            ]
        )
    ),

El controlador de evento ``OnProcessExit`` es usado para registrar una función callback que se ejecuta cuando el nodo turtlesim termina.
Registra un mensaje en la consola y ejecuta la acción ``EmitEvent`` para emitir un evento ``Shutdown`` cuando el nodo turtlesim termina.
Esto significa que el proceso launch terminará cuando la ventana de turtlesim se cierre.

.. code-block:: python

    RegisterEventHandler(
        OnProcessExit(
            target_action=turtlesim_node,
            on_exit=[
                LogInfo(msg=(EnvironmentVariable(name='USER'),
                        ' closed the turtlesim window')),
                EmitEvent(event=Shutdown(
                    reason='Window closed'))
            ]
        )
    ),

Finalmente, el controlador de evento ``OnShutdown`` es usado para registrar una función callback que se ejecuta cuando se solicita la terminación del fichero de launch.
Registra un mensaje en la consola sobre el porque se solicitó la terminación del fichero de launch.
Registra el mensaje con una razón para la terminación como el cierre de la ventana de turtlesim o la señal ``ctrl-c`` hecha por el usuario.

.. code-block:: python

    RegisterEventHandler(
        OnShutdown(
            on_shutdown=[LogInfo(
                msg=['Launch was asked to shutdown: ',
                    LocalSubstitution('event.reason')]
            )]
        )
    ),

Compila el paquete
------------------

Ve a la raíz del workspace, y compila el paquete.

.. code-block:: console

  colcon build

También recuerda ejecutar source al workspace después de compilar.

Ejemplo de launching
--------------------

Ahora puedes hacer launch al fichero ``example_event_handlers_launch.py`` usando el comando ``ros2 launch``.

.. code-block:: console

    ros2 launch launch_tutorial example_event_handlers_launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200

Esto hará lo siguiente:

#. Empieza un nodo turtlesim con el fondo azul
#. Despliega la segunda tortuga
#. Change the color to purple
#. Cambia el color a morado.
#. Cambia el color a rosa después de dos segundos si el argumento ``background_r`` es ``200`` y el argumento ``use_provided_red`` es ``True``
#. Termina el fichero de launch cuando la ventana de turtlesim se cierra

Adicionalmente, registrará mensajes en la consola cuando:

#. El nodo turtlesim empieza
#. La acción de despliegue se ejecuta
#. La acción ``change_background_r`` se ejecuta
#. La acción ``change_background_r_conditioned`` se ejecuta
#. El nodo turtlesim termina
#. Se solicita el terminado al proceso de launch

Documentación
-------------

`La documentación de launch <https://github.com/ros2/launch/blob/{REPOS_FILE_BRANCH}/launch/doc/source/architecture.rst>`_ provee información detallada acerca de los controladores de eventos disponibles.

Resumen
-------

En este tutorial, aprendiste acerca de la utilización de controladores de eventos en los ficheros de launch.
Aprendiste acerca de su sintaxis y ejemplos de uso para definir un conjunto complejo de reglas que modifican dinámicamente ficheros de launch.
