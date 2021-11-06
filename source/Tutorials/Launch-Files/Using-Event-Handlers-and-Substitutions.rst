Using event handlers and substitutions in launch files
======================================================

**Goal:** Learn about event handlers and substitutions in ROS 2 launch files

**Tutorial level:** Advanced

**Time:** 20 minutes

.. contents:: Table of Contents
   :depth: 1
   :local:

Background
----------

Launch in ROS 2 is a system that executes and manages user-defined processes.
It is responsible for monitoring the state of processes it launched, as well as reporting and reacting to changes in the state of those processes.
These changes are called events and can be handled by registering an event handler with the launch system.
Event handlers can be registered for specific events and can be useful for monitoring the state of processes.
Additionally, they can be used to define a complex set of rules which can be used to dynamically modify the launch file.

In addition to event handlers, launch files can also contain substitutions.
Substitutions are variables that can be evaluated during launch and can be used to acquire specific information like get a launch configuration, get an environment variable, or evaluate arbitrary Python expressions.

This tutorial shows usage examples of event handlers and substitutions in ROS 2 launch files.

Prerequisites
-------------

This tutorial uses the :doc:`turtlesim <../Turtlesim/Introducing-Turtlesim>` package.
This tutorial also assumes you have :doc:`created a new package <../Creating-Your-First-ROS2-Package>` of build type ``ament_python`` called ``launch_tutorial``.

Using event handlers and substitutions
--------------------------------------

1 Parent launch file
^^^^^^^^^^^^^^^^^^^^

Firstly, we will create a launch file that will call another launch file.
To do this, create an ``example_main.launch.py`` file in the ``/launch`` folder of our ``launch_tutorial`` package.

.. code-block:: python

    from launch_ros.substitutions import FindPackageShare

    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import PathJoinSubstitution, TextSubstitution


    def generate_launch_description():
        colors = {
            'background_r': '200'
        }

        return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('launch_tutorial'),
                        'launch',
                        'example.launch.py'
                    ])
                ]),
                launch_arguments={
                    'turtlesim_ns': 'turtlesim2',
                    'use_provided_red': 'True',
                    'new_background_r': TextSubstitution(text=str(colors['background_r']))
                }.items()
            )
        ])

In the ``example_main.launch.py`` file, the ``FindPackageShare`` substitution is used to find the path to the ``launch_tutorial`` package.
The ``PathJoinSubstitution`` substitution is then used to join the path to that package path with the ``example.launch.py`` file name.

.. code-block:: python

    PathJoinSubstitution([
        FindPackageShare('launch_tutorial'),
        'launch',
        'example.launch.py'
    ])

The ``launch_arguments`` dictionary with ``turtlesim_ns`` and ``use_provided_red`` arguments is passed to the ``IncludeLaunchDescription`` action.
The ``TextSubstitution`` substitution is used to define the ``new_background_r`` argument with the value of the ``background_r`` key in the ``colors`` dictionary.

.. code-block:: python

    launch_arguments={
        'turtlesim_ns': 'turtlesim2',
        'use_provided_red': 'True',
        'new_background_r': TextSubstitution(text=str(colors['background_r']))
    }.items()

2 Child launch file
^^^^^^^^^^^^^^^^^^^

Now create an ``example.launch.py`` file in the same folder.

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
                                        PythonExpression, TextSubstitution)


    def generate_launch_description():
        turtlesim_ns = LaunchConfiguration('turtlesim_ns')
        use_provided_red = LaunchConfiguration('use_provided_red')
        new_background_r = LaunchConfiguration('new_background_r')

        turtlesim_ns_launch_arg = DeclareLaunchArgument(
            'turtlesim_ns',
            default_value=TextSubstitution(text='turtlesim1')
        )
        use_provided_red_launch_arg = DeclareLaunchArgument(
            'use_provided_red',
            default_value=TextSubstitution(text='False')
        )
        new_background_r_launch_arg = DeclareLaunchArgument(
            'new_background_r',
            default_value=TextSubstitution(text='200')
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
                    ' == ',
                    TextSubstitution(text=str(200)),
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

2.1 Usage of substitutions
~~~~~~~~~~~~~~~~~~~~~~~~~~

In the ``example.launch.py`` file, ``turtlesim_ns``, ``use_provided_red``, and ``new_background_r`` launch configurations and arguments are defined.
These ``LaunchConfiguration`` substitutions allow us to acquire the value of the launch argument in any part of the launch description.
``DeclareLaunchArgument`` is used to define the launch argument that can be passed from the above launch file or from the console.

.. code-block:: python

    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value=TextSubstitution(text='turtlesim1')
    )
    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value=TextSubstitution(text='False')
    )
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value=TextSubstitution(text='200')
    )

The ``turtlesim_node`` node with the ``namespace`` set to ``turtlesim_ns`` ``LaunchConfiguration`` substitution is defined.

.. code-block:: python

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )

Afterwards, the ``ExecuteProcess`` action called ``spawn_turtle`` is defined with the corresponding ``cmd`` argument and it makes a call to the spawn service of the turtlesim node.
The ``cmd`` argument is defined using the ``FindExecutable`` substitution to find the path to the ``ros2`` executable.
Additionally, the ``LaunchConfiguration`` substitution is used to get the value of the ``turtlesim_ns`` launch argument to construct a command string.

.. code-block:: python

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

The same approach is used for the ``change_background_r`` and ``change_background_r_conditioned`` actions that change the turtlesim background's red color parameter.
The difference is that the ``change_background_r_conditioned`` action is only executed if the provided ``new_background_r`` argument equals ``200`` and the ``use_provided_red`` launch argument is set to ``True``.

.. code-block:: python

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
                ' == ',
                TextSubstitution(text=str(200)),
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


2.2 Usage of event handlers
~~~~~~~~~~~~~~~~~~~~~~~~~~~

``RegisterEventHandler`` actions for the ``OnProcessStart``, ``OnProcessIO``, ``OnExecutionComplete``, ``OnProcessExit``, and ``OnShutdown`` events were defined in the launch description.

The ``OnProcessStart`` event handler is used to register a callback function that is executed when the turtlesim node starts.
It logs a message to the console and executes the ``spawn_turtle`` action when the turtlesim node starts.

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

The ``OnProcessIO`` event handler is used to register a callback function that is executed when the ``spawn_turtle`` action writes to its standard output.
It logs the result of the spawn request.

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

The ``OnExecutionComplete`` event handler is used to register a callback function that is executed when the ``spawn_turtle`` action completes.
It logs a message to the console and executes the ``change_background_r`` and ``change_background_r_conditioned`` actions when the spawn action completes.

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

The ``OnProcessExit`` event handler is used to register a callback function that is executed when the turtlesim node exits.
It logs a message to the console and executes the ``EmitEvent`` action to emit a ``Shutdown`` event when the turtlesim node exits.
It means that the launch process will shutdown when the turtlesim window is closed.

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

Finally, the ``OnShutdown`` event handler is used to register a callback function that is executed when the launch file is asked to shutdown.
It logs a message to the console why the launch file is asked to shutdown.
It logs the message with a reason for shutdown like the closure of turtlesim window or ``ctrl-c`` signal made by the user.

.. code-block:: python

    RegisterEventHandler(
        OnShutdown(
            on_shutdown=[LogInfo(
                msg=['Launch was asked to shutdown: ',
                    LocalSubstitution('event.reason')]
            )]
        )
    ),

Launching example
-----------------

Now you can launch the ``example.launch.py`` file using the ``ros2 launch`` command.

.. code-block:: console

    ros2 launch launch_tutorial example_main.launch.py

This will do the following:

#. Start a turtlesim node with a blue background
#. Spawn the second turtle
#. Change the color to purple
#. Change the color to pink after two seconds if the provided ``background_r`` argument is ``200`` and ``use_provided_red`` argument is ``True``
#. Shutdown the launch file when the turtlesim window is closed

Additionally, it will log messages to the console when:

#. The turtlesim node starts
#. The spawn action is executed
#. The ``change_background_r`` action is executed
#. The ``change_background_r_conditioned`` action is executed
#. The turtlesim node exits
#. The launch process is asked to shutdown.

Modifying launch arguments
--------------------------

If you want to change the provided launch arguments, you can either update them in ``launch_arguments`` dictionary in the ``example_main.launch.py`` or launch the ``example.launch.py`` with preferred arguments.
To see arguments that may be given to the launch file, run the following command:

.. code-block:: console

    ros2 launch launch_tutorial example.launch.py --show-args

This will show the arguments that may be given to the launch file and their default values.

.. code-block:: console

    Arguments (pass arguments as '<name>:=<value>'):

        'turtlesim_ns':
            no description given
            (default: 'turtlesim1')

        'use_provided_red':
            no description given
            (default: 'False')

        'new_background_r':
            no description given
            (default: '200')

Now you can pass the desired arguments to the launch file as follows:

.. code-block:: console

    ros2 launch launch_tutorial example.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200

Documentation
-------------

`The launch documentation <https://github.com/ros2/launch/blob/master/launch/doc/source/architecture.rst>`_ provides detailed information about the event handlers and substitutions.
