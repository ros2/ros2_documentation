.. redirect-from::

  Guides/Launch-file-different-formats

Using Python, XML, and YAML for ROS 2 Launch Files
==================================================

.. contents:: Table of Contents
   :depth: 1
   :local:

ROS 2 launch files can be written in Python, XML, and YAML.
This guide shows how to use these different formats to accomplish the same task.
It also discusses when to use each format.

Primer on running launch files
------------------------------

Launching
^^^^^^^^^

Any of the example launch files can be run with ``ros2 launch``.
To try them locally, you can either create a new package and use

.. code-block:: console

  ros2 launch <package_name> <launch_file_name>

or run the file directly by specifying the path to the launch file

.. code-block:: console

  ros2 launch <path_to_launch_file>

Setting arguments
^^^^^^^^^^^^^^^^^

To set the arguments that are passed to the launch file, you should use ``key:=value`` syntax.
For example, you can set the value of argument ``arg`` in the following way:

.. code-block:: console

  ros2 launch <package_name> <launch_file_name> arg:=value

or

.. code-block:: console

  ros2 launch <path_to_launch_file> arg:=value

Example launch files
--------------------

Taking arguments
^^^^^^^^^^^^^^^^

Each launch file performs the following actions:

* Declares arguments with and without defaults.
* Logs a message that uses those arguments.

.. tabs::

   .. group-tab:: Python

      .. code-block:: python

        # example_taking_arguments_launch.py

        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument
        from launch.actions import LogInfo
        from launch.substitutions import LaunchConfiguration

        def generate_launch_description():
            return LaunchDescription([
                DeclareLaunchArgument('who'),
                DeclareLaunchArgument('where', default_value='home'),

                LogInfo(msg=[LaunchConfiguration('who'), ' is at ', LaunchConfiguration('where')])
            ])

   .. group-tab:: XML

      .. code-block:: xml

        <!-- example_taking_arguments_launch.xml -->

        <launch>
            <arg name="who"/>
            <arg name="where" default="home"/>

            <log message="$(var who) is at $(var where)"/>
        </launch>

   .. group-tab:: YAML

      .. code-block:: yaml

        # example_taking_arguments_launch.yaml

        launch:
        - arg:
            name: "who"
        - arg:
            name: "where"
            default: "home"
        - log:
            message: "$(var who) is at $(var where)"

On execution, availability of the ``who`` argument is enforced, while for the ``where`` argument its default applies if not provided.
Then, a message is logged after both arguments are substituted against the current context (holding their values).

This can be reproduced with a local copy of any of these launch files e.g.:

.. code-block:: console

    $ ros2 launch example_taking_arguments_launch.xml

    [INFO] [launch]: All log files can be found below /...
    [INFO] [launch]: Default logging verbosity is set to INFO
    [ERROR] [launch]: Caught exception in launch (see debug for traceback): Included launch description missing required argument 'who' (description: 'no description given'), given: []

.. code-block:: shell

    $ ros2 launch example_taking_arguments_launch.xml who:=someone

    [INFO] [launch]: All log files can be found below /...
    [INFO] [launch]: Default logging verbosity is set to INFO
    [INFO] [launch.user]: someone is at home

.. code-block:: shell

    $ ros2 launch example_taking_arguments_launch.xml who:=someone where:="the movies"

    [INFO] [launch]: All log files can be found below ...
    [INFO] [launch]: Default logging verbosity is set to INFO
    [INFO] [launch.user]: someone is at the movies

Requiring processes
^^^^^^^^^^^^^^^^^^^

Each launch file performs the following actions:

* Declares an argument with defaults.
* Sleeps for some time, then shuts down launch.

.. tabs::

   .. group-tab:: Python

      .. code-block:: python

        # example_requiring_processes_launch.py

        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument
        from launch.actions import ExecuteProcess
        from launch.actions import LogInfo
        from launch.actions import Shutdown
        from launch.substitutions import LaunchConfiguration

        def generate_launch_description():
            return LaunchDescription([
                DeclareLaunchArgument('sleep_duration', default_value='10', description='Time to sleep, in seconds'),
                ExecuteProcess(
                    cmd=['sleep', LaunchConfiguration('sleep_duration')],
                    on_exit=[
                        LogInfo(msg='Done sleeping!'),
                        Shutdown(reason='main process terminated')
                    ]
                ),
                LogInfo(msg=['Sleeping for ', LaunchConfiguration('sleep_duration'), ' seconds']),
            ])

   .. group-tab:: XML

      .. code-block:: xml

        <!-- Currently unsupported -->

   .. group-tab:: YAML

      .. code-block:: yaml

        # Currently unsupported

On execution, availability of the ``sleep_duration`` argument is enforced, applying its default value if necessary.
Then, a ``sleep`` command is started to wait for as long as ``sleep_duration`` specifies.
Additionally, when this command exits, a shutdown is initiated.
Until then (or user interrupt), ``launch`` remain idle.

This can be reproduced with a local copy of the Python launch file (XML and YAML launch files cannot describe this example yet) e.g.:

.. code-block:: console

    $ ros2 launch example_requiring_processes_launch.py

    [INFO] [launch]: All log files can be found below /...
    [INFO] [launch]: Default logging verbosity is set to INFO
    [INFO] [launch.user]: Sleeping for 10 seconds
    [INFO] [sleep-1]: process started with pid [NNNNNN]
    [INFO] [sleep-1]: process has finished cleanly [pid NNNNNN]
    [INFO] [launch.user]: Done sleeping!
    [INFO] [launch]: process[sleep-1] was required: shutting down launched system

Replicating hierarchies
^^^^^^^^^^^^^^^^^^^^^^^

Each launch file performs the following actions:

* Declares an argument without defaults.
* Generates multiple namespaced groups of nodes based on that argument.

.. tabs::

   .. group-tab:: Python

      .. code-block:: python

        # example_replicating_hierarchies_launch.py

        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument
        from launch.actions import GroupAction
        from launch.actions import OpaqueFunction
        from launch.substitutions import LaunchConfiguration
        from launch_ros.actions import Node
        from launch_ros.actions import PushRosNamespace

        def generate_turtles_description(context, turtles):
            return [
                GroupAction(
                    actions=[
                        PushRosNamespace(turtle_name),
                        Node(
                            package='turtlesim',
                            executable='turtlesim_node',
                            output='screen')
                    ]
                )
                for turtle_name in turtles.perform(context).split()
            ]

        def generate_launch_description():
            return LaunchDescription([
                DeclareLaunchArgument('turtles', description='A space-separated list of turtle names'),
                OpaqueFunction(
                    function=generate_turtles_description,
                    args=[LaunchConfiguration('turtles')])
            ])

   .. group-tab:: XML

      .. code-block:: xml

        <!-- Currently unsupported -->

   .. group-tab:: YAML

      .. code-block:: yaml

        # Currently unsupported

On execution, availability of the ``turtles`` argument is first enforced.
Then, the ``generate_turtles_description`` Python function is invoked with both the current context and the ``turtles`` configuration variable as arguments.
This function has no side effect -- it is used as a escape hatch to dynamically extend the ``launch`` description.
To do so, it evaluates the ``turtles`` configuration variable in context and, assuming it is a spaced-separated list of turtle names, it produces a group per turtle.
Each group sets the corresponding turtle name as namespace for all ROS 2 nodes within and starts a ``turtlesim`` node.

This can be reproduced with a local copy of the Python launch file (XML and YAML launch files cannot describe this example yet) e.g.:

.. code-block:: console

    $ ros2 launch example_replicating_hierarchies_launch.py turtles:="alice bob"

    [INFO] [launch]: All log files can be found below /...
    [INFO] [launch]: Default logging verbosity is set to INFO
    [INFO] [turtlesim_node-1]: process started with pid [NNNNNN]
    [INFO] [turtlesim_node-2]: process started with pid [NNNNNN]
    [turtlesim_node-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [alice.turtlesim]: Starting turtlesim with node name /alice/turtlesim
    [turtlesim_node-2] [INFO] [TTTTTTTTTT.TTTTTTTTT] [bob.turtlesim]: Starting turtlesim with node name /bob/turtlesim
    [turtlesim_node-2] [INFO] [TTTTTTTTTT.TTTTTTTTT] [bob.turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
    [turtlesim_node-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [alice.turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]

A pair of ``turtlesim`` windows will pop up: one for ``alice``, one for ``bob``.

Cleaning after
^^^^^^^^^^^^^^

Each launch file performs the following actions:

* Declares an argument without defaults.
* Registers an action to execute on shutdown.
* Forces shutdown after a time specified by the argument.

.. tabs::

   .. group-tab:: Python

      .. code-block:: python

        # example_cleaning_after_launch.py

        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument
        from launch.actions import ExecuteProcess
        from launch.actions import RegisterEventHandler
        from launch.actions import Shutdown
        from launch.actions import TimerAction
        from launch.event_handlers import OnShutdown
        from launch.substitutions import LaunchConfiguration

        def generate_launch_description():
            return LaunchDescription([
                DeclareLaunchArgument('timeout', description='Timeout before shutdown, in seconds'),
                RegisterEventHandler(OnShutdown(on_shutdown=[
                    ExecuteProcess(cmd=['rm', '-f', '/tmp/resource']),
                ])),
                ExecuteProcess(cmd=['touch', '/tmp/resource']),
                TimerAction(period=LaunchConfiguration('timeout'), actions=[
                    Shutdown(reason='launch timed out!')
                ])
            ])

   .. group-tab:: XML

      .. code-block:: xml

        <!-- Currently unsupported -->

   .. group-tab:: YAML

      .. code-block:: yaml

        # Currently unsupported

On execution, availability of the ``timeout`` argument is enforced first.
Then, an ``rm -f`` command to drop a temporary ``/tmp/resource`` file is registered for execution upon shutdown.
That temporary file is created by a ``touch`` command initiated right after.
A ``launch`` shutdown is finally scheduled to occur after ``timeout`` seconds.

This can be reproduced with a local copy of the Python launch file (XML and YAML launch files cannot describe this example yet) e.g.:

.. code-block:: console

    $ ros2 launch example_cleaning_after_launch.py timeout:=5

    [INFO] [launch]: All log files can be found below /...
    [INFO] [launch]: Default logging verbosity is set to INFO
    [INFO] [touch-1]: process started with pid [NNNNNN]
    [INFO] [touch-1]: process has finished cleanly [pid NNNNNN]
    # ...and after 5 seconds
    [INFO] [rm-2]: process started with pid [NNNNNN]
    [INFO] [rm-2]: process has finished cleanly [pid NNNNNN]

Configuring nodes
^^^^^^^^^^^^^^^^^

Each talker launch file performs the following actions:

* Declares an argument without defaults.
* Starts a talker node, passing command line arguments to it.

Each example launch file performs the following actions:

* Declares an argument with defaults.
* Forces a name remapping on all nodes.
* Includes the talker launch file with a given argument.
* Starts a listener node.

.. tabs::

   .. group-tab:: Python

      .. code-block:: python

        # talker_launch.py

        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument
        from launch.actions import Shutdown
        from launch.substitutions import LaunchConfiguration
        from launch_ros.actions import Node

        def generate_launch_description():
            return LaunchDescription([
                DeclareLaunchArgument('cycles', description='number of times to talk'),
                Node(
                    package='demo_nodes_py', executable='talker_qos', output='screen',
                    arguments=['--reliable', '-n', LaunchConfiguration('cycles')]),
            ])

    .. code-block:: python

        # example_configuring_nodes_launch.py

        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument
        from launch.actions import IncludeLaunchDescription
        from launch.substitutions import LaunchConfiguration
        from launch_ros.actions import Node
        from launch_ros.actions import SetRemap

        def generate_launch_description():
            return LaunchDescription([
                DeclareLaunchArgument('shared_topic' default_value='chat'),
                SetRemap('chatter', dst=LaunchConfiguration('shared_topic')),
                IncludeLaunchDescription('talker_launch.py', launch_arguments=[('cycles', '10')]),
                Node(package='demo_nodes_py', executable='listener', output='screen')
            ])

   .. group-tab:: XML

      .. code-block:: xml

        <!-- talker_launch.xml -->

        <launch>
            <arg name="cycles" description="number of times to talk"/>
            <node pkg="demo_nodes_py" exec="talker_qos" args="--reliable -n $(var cycles)" output="screen"/>
        </launch>

      .. code-block:: xml

        <!-- example_configuring_nodes_launch.py -->

        <launch>
            <arg name="shared_topic" default="chat"/>
            <set_remap from="chatter" to="$(var shared_topic)"/>
            <include file="talker_launch.xml">
                <arg name="cycles" value="10"/>
            </include>
            <node pkg="demo_nodes_py" exec="listener" output="screen"/>
        </launch>

   .. group-tab:: YAML

      .. code-block:: yaml

        # talker_launch.yaml

        launch:
        - arg:
            name: "cycles"
            description: "number of times to talk"
        - node:
            pkg: "demo_nodes_py"
            exec: "talker_qos"
            args: "--reliable -n $(var cycles)"
            output: "screen"

      .. code-block:: yaml

        # example_configuring_nodes_launch.yaml

        launch:
        - arg:
            name: "shared_topic"
            default: "chat"
        - set_remap:
            from: "chatter"
            to: "$(var shared_topic)"
        - include:
            file: "talker_launch.yaml"
            arg:
            - name: "cycles"
              value: "10"
        - node:
            pkg: "demo_nodes_py"
            exec: "listener"
            output: "screen"

On execution, availability of the ``shared_topic`` argument is enforced, applying defaults if necessary.
Then, a remapping rule for the ``chatter`` topic is applied globally, to all nodes in the launch file.
The talker launch file is included, providing a value for the ``cycles`` argument.
This ``cycles`` argument is passed as the number of talking cycles for the talker node on start.
Finally, a listener node is started.

This can be reproduced with a local copy of any of the launch file sets e.g.:

.. code-block:: console

    $ ros2 launch example_configuring_nodes_launch.yaml

    [INFO] [launch]: All log files can be found below /...
    [INFO] [launch]: Default logging verbosity is set to INFO
    [INFO] [talker_qos-1]: process started with pid [NNNNNN]
    [INFO] [listener-2]: process started with pid [NNNNNN]
    [talker_qos-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [talker_qos]: Reliable talker
    [talker_qos-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [talker_qos]: Publishing: "Hello World: 0"
    [listener-2] [INFO] [TTTTTTTTTT.TTTTTTTTT] [listener]: I heard: [Hello World: 0]
    [talker_qos-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [talker_qos]: Publishing: "Hello World: 1"
    [listener-2] [INFO] [TTTTTTTTTT.TTTTTTTTT] [listener]: I heard: [Hello World: 1]
    [talker_qos-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [talker_qos]: Publishing: "Hello World: 2"
    [listener-2] [INFO] [TTTTTTTTTT.TTTTTTTTT] [listener]: I heard: [Hello World: 2]
    [talker_qos-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [talker_qos]: Publishing: "Hello World: 3"
    [listener-2] [INFO] [TTTTTTTTTT.TTTTTTTTT] [listener]: I heard: [Hello World: 3]
    [talker_qos-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [talker_qos]: Publishing: "Hello World: 4"
    [listener-2] [INFO] [TTTTTTTTTT.TTTTTTTTT] [listener]: I heard: [Hello World: 4]
    [talker_qos-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [talker_qos]: Publishing: "Hello World: 5"
    [listener-2] [INFO] [TTTTTTTTTT.TTTTTTTTT] [listener]: I heard: [Hello World: 5]
    [talker_qos-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [talker_qos]: Publishing: "Hello World: 6"
    [listener-2] [INFO] [TTTTTTTTTT.TTTTTTTTT] [listener]: I heard: [Hello World: 6]
    [talker_qos-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [talker_qos]: Publishing: "Hello World: 7"
    [listener-2] [INFO] [TTTTTTTTTT.TTTTTTTTT] [listener]: I heard: [Hello World: 7]
    [talker_qos-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [talker_qos]: Publishing: "Hello World: 8"
    [listener-2] [INFO] [TTTTTTTTTT.TTTTTTTTT] [listener]: I heard: [Hello World: 8]
    [talker_qos-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [talker_qos]: Publishing: "Hello World: 9"
    [listener-2] [INFO] [TTTTTTTTTT.TTTTTTTTT] [listener]: I heard: [Hello World: 9]
    [INFO] [talker_qos-1]: process has finished cleanly [pid NNNNNN]

Switching modes
^^^^^^^^^^^^^^^

Each launch file performs the following actions:

* Declares a boolean argument, false by default.
* Starts talker and a listener composable nodes if the argument is true.
* Starts talker and a listener nodes if the argument is false.

.. tabs::

   .. group-tab:: Python

      .. code-block:: python

        # example_switching modes_launch.py

        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument
        from launch.actions import GroupAction
        from launch.actions import IncludeLaunchDescription
        from launch.actions import LogInfo
        from launch.conditions import IfCondition
        from launch.conditions import UnlessCondition
        from launch.substitutions import LaunchConfiguration
        from launch_ros.actions import Node
        from launch_ros.actions import ComposableNodeContainer
        from launch_ros.descriptions import ComposableNode
        from launch_ros.substitutions import FindPackagePrefix

        def generate_launch_description():
            return LaunchDescription([
                DeclareLaunchArgument('use_composition', default_value='false'),
                GroupAction(
                    condition=IfCondition(LaunchConfiguration('use_composition')),
                    actions=[
                        LogInfo(msg='Running talker and listener composable nodes in the same process'),
                        LogInfo(msg=["Components taken from the 'composition' package (in ", FindPackagePrefix('composition'), ")"]),
                        ComposableNodeContainer(
                            package='rclcpp_components',
                            executable='component_container',
                            name='container',
                            namespace='/',
                            output='screen',
                            composable_node_descriptions=[
                                ComposableNode(package='composition', plugin='composition::Talker', name='talker'),
                                ComposableNode(package='composition', plugin='composition::Listener', name='listener')
                            ]
                        ),
                    ]
                ),
                GroupAction(
                    condition=UnlessCondition(LaunchConfiguration('use_composition')),
                    actions=[
                        LogInfo(msg='Running talker and listener nodes in separate processes'),
                        LogInfo(msg=["Nodes taken from the 'demo_nodes_cpp' package (in ", FindPackagePrefix('demo_nodes_cpp'), ")"]),
                        Node(package='demo_nodes_cpp', executable='talker', name='talker', output='screen'),
                        Node(package='demo_nodes_cpp', executable='listener', name='listener', output='screen')
                    ]
                )
            ])

   .. group-tab:: XML

      .. code-block:: xml

        <!-- example_switching modes_launch.xml -->

        <launch>
            <arg name="use_composition" default="false"/>
            <group if="$(var use_composition)">
                <log message="Running talker and listener composable nodes in the same process"/>
                <log message="Components taken from the 'composition' package in $(find-pkg-prefix composition)"/>
                <node_container pkg="rclcpp_components" exec="component_container" name="container" namespace="/" output="screen">
                    <composable_node pkg="composition" plugin="composition::Talker" name="talker"/>
                    <composable_node pkg="composition" plugin="composition::Listener" name="listener"/>
                </node_container>
            </group>
            <group unless="$(var use_composition)">
                <log message="Running talker and listener nodes in separate processes"/>
                <log message="Nodes taken from the 'demo_nodes_cpp' package in $(find-pkg-prefix demo_nodes_cpp)"/>
                <node pkg="demo_nodes_cpp" exec="talker" name="talker" output="screen"/>
                <node pkg="demo_nodes_cpp" exec="listener" name="listener" output="screen"/>
            </group>
        </launch>

   .. group-tab:: YAML

      .. code-block:: yaml

        # example_switching_modes_launch.yaml

        launch:
        - arg:
            name: "use_composition"
            default: "false"
        - group:
            if: "$(var use_composition)"
            children:
              - log:
                  message: "Running talker and listener composable nodes in the same process"
              - log:
                  message: "Components taken from the 'composition' package in $(find-pkg-prefix composition)"
              - node_container:
                  pkg: "rclcpp_components"
                  exec: "component_container"
                  name: "container"
                  namespace: "/"
                  output: "screen"
                  composable_node:
                    - pkg: "composition"
                      plugin: "composition::Talker"
                      name: "talker"
                    - pkg: "composition"
                      plugin: "composition::Listener"
                      name: "listener"
        - group:
            unless: "$(var use_composition)"
            children:
              - log:
                  message: "Running talker and listener nodes in separate processes"
              - log:
                  message: "Nodes taken from the 'demo_nodes_cpp' package in $(find-pkg-prefix demo_nodes_cpp)"
              - node:
                  pkg: "demo_nodes_py"
                  exec: "talker"
                  name: "talker"
                  output: "screen"
              - node:
                  pkg: "demo_nodes_py"
                  exec: "listener"
                  name: "listener"
                  output: "screen"

On execution, availability of the ``use_composition`` argument is enforced, applying defaults if necessary.
If ``use_composition`` is true, talker and listener composable nodes are started in a component container.
If ``use_composition`` is false, talker and listener nodes are started in their own processes.
In both cases, informational logs are produced.

This can be reproduced with a local copy of any of the launch file sets e.g.:

.. code-block:: console

    $ ros2 launch example_switching_modes_launch.xml

    [INFO] [launch]: All log files can be found below /...
    [INFO] [launch]: Default logging verbosity is set to INFO
    [INFO] [launch.user]: Running talker and listener nodes in separate processes
    [INFO] [launch.user]: Nodes taken from the 'demo_nodes_cpp' package in /opt/ros/...
    [INFO] [talker-1]: process started with pid [NNNNNN]
    [INFO] [listener-2]: process started with pid [NNNNNN]
    [talker-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [talker]: Publishing: 'Hello World: 1'
    [listener-2] [INFO] [TTTTTTTTTT.TTTTTTTTT] [listener]: I heard: [Hello World: 1]
    # ... and on and on

.. code-block:: console

    $ ros2 launch example_switching_modes_launch.xml use_composition:=true

    [INFO] [launch]: All log files can be found below /...
    [INFO] [launch]: Default logging verbosity is set to INFO
    [INFO] [launch.user]: Running talker and listener composable nodes in the same process
    [INFO] [launch.user]: Components taken from the 'composition' package in /opt/ros/...
    [INFO] [component_container-1]: process started with pid [NNNNNN]
    [component_container-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [container]: Load Library: /opt/ros/humble/lib/libtalker_component.so
    [component_container-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [container]: Found class: rclcpp_components::NodeFactoryTemplate<composition::Talker>
    [component_container-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<composition::Talker>
    [INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/talker' in container '/container'
    [component_container-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [container]: Load Library: /opt/ros/humble/lib/liblistener_component.so
    [component_container-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [container]: Found class: rclcpp_components::NodeFactoryTemplate<composition::Listener>
    [component_container-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<composition::Listener>
    [INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/listener' in container '/container'
    [component_container-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [talker]: Publishing: 'Hello World: 1'
    [component_container-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [listener]: I heard: [Hello World: 1]
    # ... and on and on

Managing lifecycles
^^^^^^^^^^^^^^^^^^^

Each launch file performs the following actions:

* Registers a custom event handler to manage lifecycles synchronously.
* Starts talker and listener managed nodes.

Note this is a significantly more complex example, effectively extending ``launch`` to achieve its goal.
For further reference on managed nodes, see :doc:`../Tutorials/Demos/Managed-Nodes`.

.. tabs::

   .. group-tab:: Python

      .. code-block:: python

        # example_managing_lifecycles_launch.py

        from launch import LaunchDescription
        from launch.actions import RegisterEventHandler
        from launch.actions import EmitEvent
        from launch.event_handler import BaseEventHandler
        from launch.events import Shutdown
        from launch.events import matches_action
        from launch.events.process import ProcessStarted
        from launch_ros.actions import LifecycleNode
        from launch_ros.events.lifecycle import ChangeState
        from launch_ros.events.lifecycle import StateTransition

        import lifecycle_msgs.msg

        class LifecycleManager(BaseEventHandler):

            BRINGUP_SEQUENCE = {
                lifecycle_msgs.msg.State.PRIMARY_STATE_UNCONFIGURED: lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                lifecycle_msgs.msg.State.PRIMARY_STATE_INACTIVE: lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
            }

            def __init__(self, nodes):
                # Track the state of all managed nodes in a dictionary.
                self.managees = {node: lifecycle_msgs.msg.State.PRIMARY_STATE_UNKNOWN for node in nodes}
                # Configure event matcher to handle both process start and state transition events from all managed nodes.
                matcher = (lambda event: (
                    isinstance(event, (ProcessStarted, StateTransition)) and event.action in self.managees))
                super().__init__(matcher=matcher)

            def handle(self, event, context):
                if isinstance(event, ProcessStarted):  # node just started, unconfigured
                    self.managees[event.action] = lifecycle_msgs.msg.State.PRIMARY_STATE_UNCONFIGURED
                if isinstance(event, StateTransition):  # node changed state, track it
                    self.managees[event.action] = event.msg.goal_state.id
                states = list(set(self.managees.values()))
                common_state = states[0] if len(states) == 1 else None
                if common_state in self.BRINGUP_SEQUENCE:
                    # all managed nodes have reached the same state and the bringup sequence is not complete
                    matcher = lambda action: action in self.managees
                    # trigger state transitions for all managed nodes to next state in the bringup sequence
                    return [
                        EmitEvent(event=ChangeState(
                            lifecycle_node_matcher=matcher,
                            transition_id=self.BRINGUP_SEQUENCE[common_state]))]
                return None  # do nothing

        def generate_launch_description():
            first_talker_node = LifecycleNode(
                package='lifecycle', executable='lifecycle_talker',
                name='first_talker', namespace='', output='screen')
            second_talker_node = LifecycleNode(
                package='lifecycle', executable='lifecycle_talker',
                name='second_talker', namespace='', output='screen')
            listener_node = LifecycleNode(
                package='lifecycle', executable='lifecycle_listener',
                name='listener', namespace='', output='screen',
                remappings=[('/lc_talker/transition_event',
                             '/first_talker/transition_event')])
            manager = LifecycleManager([first_talker_node, second_talker_node])
            return LaunchDescription([
                RegisterEventHandler(manager),
                first_talker_node, second_talker_node, listener_node
            ])

   .. group-tab:: XML

      .. code-block:: xml

        <!-- Currently unsupported -->

   .. group-tab:: YAML

      .. code-block:: yaml

        # Currently unsupported

During configuration, a ``LifecycleManager`` event handler is instantiated, taking references to both talker nodes.
On execution, the ``LifecycleManager`` event handler is registered and all nodes are started.
The ``LifecycleManager`` event handler then takes care of managing both talker nodes from the unconfigured state
through the configured state and to the active state.

This can be reproduced with a local copy of the Python launch file (``launch`` cannot be extended like this in XML and YAML launch files) e.g.:

.. code-block:: console

    $ ros2 launch example_managing_lifecycles_launch.py

    [INFO] [launch]: All log files can be found below /...
    [INFO] [launch]: Default logging verbosity is set to INFO
    [INFO] [lifecycle_talker-1]: process started with pid [NNNNNN]
    [INFO] [lifecycle_talker-2]: process started with pid [NNNNNN]
    [INFO] [lifecycle_listener-3]: process started with pid [NNNNNN]
    [lifecycle_listener-3] [INFO] [TTTTTTTTTT.TTTTTTTTT] [listener]: notify callback: Transition from state unconfigured to configuring
    [lifecycle_listener-3] [INFO] [TTTTTTTTTT.TTTTTTTTT] [listener]: notify callback: Transition from state configuring to inactive
    [lifecycle_talker-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [first_talker]: on_configure() is called.
    [lifecycle_talker-2] [INFO] [TTTTTTTTTT.TTTTTTTTT] [second_talker]: on_configure() is called.
    [lifecycle_talker-2] [INFO] [TTTTTTTTTT.TTTTTTTTT] [second_talker]: on_activate() is called.
    [lifecycle_talker-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [first_talker]: on_activate() is called.
    [lifecycle_listener-3] [INFO] [TTTTTTTTTT.TTTTTTTTT] [listener]: notify callback: Transition from state inactive to activating
    [lifecycle_talker-2] [INFO] [TTTTTTTTTT.TTTTTTTTT] [second_talker]: Lifecycle publisher is active. Publishing: [Lifecycle HelloWorld #1]
    [lifecycle_listener-3] [INFO] [TTTTTTTTTT.TTTTTTTTT] [listener]: data_callback: Lifecycle HelloWorld #1
    [lifecycle_talker-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [first_talker]: Lifecycle publisher is active. Publishing: [Lifecycle HelloWorld #1]
    [lifecycle_listener-3] [INFO] [TTTTTTTTTT.TTTTTTTTT] [listener]: notify callback: Transition from state activating to active
    [lifecycle_listener-3] [INFO] [TTTTTTTTTT.TTTTTTTTT] [listener]: data_callback: Lifecycle HelloWorld #1
    # ... and on and on until user interruption

Launching many nodes
^^^^^^^^^^^^^^^^^^^^

Each launch file performs the following actions:

* Setup command line arguments with defaults
* Include another launch file
* Include another launch file in another namespace
* Start a node and setting its namespace
* Start a node, setting its namespace, and setting parameters in that node (using the args)
* Create a node to remap messages from one topic to another

.. tabs::

   .. group-tab:: Python

      .. code-block:: python

        # example_many_nodes_launch.py

        import os

        from ament_index_python import get_package_share_directory

        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument
        from launch.actions import GroupAction
        from launch.actions import IncludeLaunchDescription
        from launch.launch_description_sources import PythonLaunchDescriptionSource
        from launch.substitutions import LaunchConfiguration
        from launch.substitutions import TextSubstitution
        from launch_ros.actions import Node
        from launch_ros.actions import PushROSNamespace
        from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
        from launch_yaml.launch_description_sources import YAMLLaunchDescriptionSource


        def generate_launch_description():

            # args that can be set from the command line or a default will be used
            background_r_launch_arg = DeclareLaunchArgument(
                "background_r", default_value=TextSubstitution(text="0")
            )
            background_g_launch_arg = DeclareLaunchArgument(
                "background_g", default_value=TextSubstitution(text="255")
            )
            background_b_launch_arg = DeclareLaunchArgument(
                "background_b", default_value=TextSubstitution(text="0")
            )
            chatter_py_ns_launch_arg = DeclareLaunchArgument(
                "chatter_py_ns", default_value=TextSubstitution(text="chatter/py/ns")
            )
            chatter_xml_ns_launch_arg = DeclareLaunchArgument(
                "chatter_xml_ns", default_value=TextSubstitution(text="chatter/xml/ns")
            )
            chatter_yaml_ns_launch_arg = DeclareLaunchArgument(
                "chatter_yaml_ns", default_value=TextSubstitution(text="chatter/yaml/ns")
            )

            # include another launch file
            launch_include = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('demo_nodes_cpp'),
                        'launch/topics/talker_listener_launch.py'))
            )
            # include a Python launch file in the chatter_py_ns namespace
            launch_py_include_with_namespace = GroupAction(
                actions=[
                    # push_ros_namespace first to set namespace of included nodes for following actions
                    PushROSNamespace('chatter_py_ns'),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                get_package_share_directory('demo_nodes_cpp'),
                                'launch/topics/talker_listener_launch.py'))
                    ),
                ]
            )

            # include a xml launch file in the chatter_xml_ns namespace
            launch_xml_include_with_namespace = GroupAction(
                actions=[
                    # push_ros_namespace first to set namespace of included nodes for following actions
                    PushROSNamespace('chatter_xml_ns'),
                    IncludeLaunchDescription(
                        XMLLaunchDescriptionSource(
                            os.path.join(
                                get_package_share_directory('demo_nodes_cpp'),
                                'launch/topics/talker_listener_launch.xml'))
                    ),
                ]
            )

            # include a yaml launch file in the chatter_yaml_ns namespace
            launch_yaml_include_with_namespace = GroupAction(
                actions=[
                    # push_ros_namespace first to set namespace of included nodes for following actions
                    PushROSNamespace('chatter_yaml_ns'),
                    IncludeLaunchDescription(
                        YAMLLaunchDescriptionSource(
                            os.path.join(
                                get_package_share_directory('demo_nodes_cpp'),
                                'launch/topics/talker_listener_launch.yaml'))
                    ),
                ]
            )

            # start a turtlesim_node in the turtlesim1 namespace
            turtlesim_node = Node(
                package='turtlesim',
                namespace='turtlesim1',
                executable='turtlesim_node',
                name='sim'
            )

            # start another turtlesim_node in the turtlesim2 namespace
            # and use args to set parameters
            turtlesim_node_with_parameters = Node(
                package='turtlesim',
                namespace='turtlesim2',
                executable='turtlesim_node',
                name='sim',
                parameters=[{
                    "background_r": LaunchConfiguration('background_r'),
                    "background_g": LaunchConfiguration('background_g'),
                    "background_b": LaunchConfiguration('background_b'),
                }]
            )

            # perform remap so both turtles listen to the same command topic
            forward_turtlesim_commands_to_second_turtlesim_node = Node(
                package='turtlesim',
                executable='mimic',
                name='mimic',
                remappings=[
                    ('/input/pose', '/turtlesim1/turtle1/pose'),
                    ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
                ]
            )

            return LaunchDescription([
                background_r_launch_arg,
                background_g_launch_arg,
                background_b_launch_arg,
                chatter_py_ns_launch_arg,
                chatter_xml_ns_launch_arg,
                chatter_yaml_ns_launch_arg,
                launch_include,
                launch_py_include_with_namespace,
                launch_xml_include_with_namespace,
                launch_yaml_include_with_namespace,
                turtlesim_node,
                turtlesim_node_with_parameters,
                forward_turtlesim_commands_to_second_turtlesim_node,
            ])


   .. group-tab:: XML

      .. code-block:: xml

        <!-- example_many_nodes_launch.xml -->

        <launch>

            <!-- args that can be set from the command line or a default will be used -->
            <arg name="background_r" default="0" />
            <arg name="background_g" default="255" />
            <arg name="background_b" default="0" />
            <arg name="chatter_py_ns" default="chatter/py/ns" />
            <arg name="chatter_xml_ns" default="chatter/xml/ns" />
            <arg name="chatter_yaml_ns" default="chatter/yaml/ns" />

            <!-- include another launch file -->
            <include file="$(find-pkg-share demo_nodes_cpp)/launch/topics/talker_listener_launch.py" />
            <!-- include a Python launch file in the chatter_py_ns namespace-->
            <group>
                <!-- push_ros_namespace to set namespace of included nodes -->
                <push_ros_namespace namespace="$(var chatter_py_ns)" />
                <include file="$(find-pkg-share demo_nodes_cpp)/launch/topics/talker_listener_launch.py" />
            </group>
            <!-- include a xml launch file in the chatter_xml_ns namespace-->
            <group>
                <!-- push_ros_namespace to set namespace of included nodes -->
                <push_ros_namespace namespace="$(var chatter_xml_ns)" />
                <include file="$(find-pkg-share demo_nodes_cpp)/launch/topics/talker_listener_launch.xml" />
            </group>
            <!-- include a yaml launch file in the chatter_yaml_ns namespace-->
            <group>
                <!-- push_ros_namespace to set namespace of included nodes -->
                <push_ros_namespace namespace="$(var chatter_yaml_ns)" />
                <include file="$(find-pkg-share demo_nodes_cpp)/launch/topics/talker_listener_launch.yaml" />
            </group>

            <!-- start a turtlesim_node in the turtlesim1 namespace -->
            <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim1" />
            <!-- start another turtlesim_node in the turtlesim2 namespace
                and use args to set parameters -->
            <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim2">
                <param name="background_r" value="$(var background_r)" />
                <param name="background_g" value="$(var background_g)" />
                <param name="background_b" value="$(var background_b)" />
            </node>
            <!-- perform remap so both turtles listen to the same command topic -->
            <node pkg="turtlesim" exec="mimic" name="mimic">
                <remap from="/input/pose" to="/turtlesim1/turtle1/pose" />
                <remap from="/output/cmd_vel" to="/turtlesim2/turtle1/cmd_vel" />
            </node>
        </launch>

   .. group-tab:: YAML

      .. code-block:: yaml

        # example_many_nodes_launch.yaml

        launch:

        # args that can be set from the command line or a default will be used
        - arg:
            name: "background_r"
            default: "0"
        - arg:
            name: "background_g"
            default: "255"
        - arg:
            name: "background_b"
            default: "0"
        - arg:
            name: "chatter_py_ns"
            default: "chatter/py/ns"
        - arg:
            name: "chatter_xml_ns"
            default: "chatter/xml/ns"
        - arg:
            name: "chatter_yaml_ns"
            default: "chatter/yaml/ns"


        # include another launch file
        - include:
            file: "$(find-pkg-share demo_nodes_cpp)/launch/topics/talker_listener_launch.py"

        # include a Python launch file in the chatter_py_ns namespace
        - group:
            - push_ros_namespace:
                namespace: "$(var chatter_py_ns)"
            - include:
                file: "$(find-pkg-share demo_nodes_cpp)/launch/topics/talker_listener_launch.py"

        # include a xml launch file in the chatter_xml_ns namespace
        - group:
            - push_ros_namespace:
                namespace: "$(var chatter_xml_ns)"
            - include:
                file: "$(find-pkg-share demo_nodes_cpp)/launch/topics/talker_listener_launch.xml"

        # include a yaml launch file in the chatter_yaml_ns namespace
        - group:
            - push_ros_namespace:
                namespace: "$(var chatter_yaml_ns)"
            - include:
                file: "$(find-pkg-share demo_nodes_cpp)/launch/topics/talker_listener_launch.yaml"

        # start a turtlesim_node in the turtlesim1 namespace
        - node:
            pkg: "turtlesim"
            exec: "turtlesim_node"
            name: "sim"
            namespace: "turtlesim1"

        # start another turtlesim_node in the turtlesim2 namespace and use args to set parameters
        - node:
            pkg: "turtlesim"
            exec: "turtlesim_node"
            name: "sim"
            namespace: "turtlesim2"
            param:
            -
              name: "background_r"
              value: "$(var background_r)"
            -
              name: "background_g"
              value: "$(var background_g)"
            -
              name: "background_b"
              value: "$(var background_b)"

        # perform remap so both turtles listen to the same command topic
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

On execution, availability of background color and chatter namespace arguments is enforced, applying defaults if necessary.
Then, talker-listener pairs, as described in a demo launch file, are launched with different namespaces.
Finally, multiple ``turtlesim`` nodes are started, using default parameters, setting custom background colors, and remapping topics.

This can be reproduced with a local copy of any of the launch files e.g.:

.. code-block:: console

    $ ros2 launch example_many_nodes_launch.py

    [INFO] [launch]: All log files can be found below /home/hidmic/.ros/log/2023-09-19-20-16-48-522323-mhidalgo-spot-197115
    [INFO] [launch]: Default logging verbosity is set to INFO
    [INFO] [talker-1]: process started with pid [NNNNNN]
    [INFO] [listener-2]: process started with pid [NNNNNN]
    [INFO] [talker-3]: process started with pid [NNNNNN]
    [INFO] [listener-4]: process started with pid [NNNNNN]
    [INFO] [talker-5]: process started with pid [NNNNNN]
    [INFO] [listener-6]: process started with pid [NNNNNN]
    [INFO] [talker-7]: process started with pid [NNNNNN]
    [INFO] [listener-8]: process started with pid [NNNNNN]
    [INFO] [turtlesim_node-9]: process started with pid [NNNNNN]
    [INFO] [turtlesim_node-10]: process started with pid [NNNNNN]
    [INFO] [mimic-11]: process started with pid [NNNNNN]
    [turtlesim_node-10] [INFO] [TTTTTTTTTT.TTTTTTTTT] [turtlesim2.sim]: Starting turtlesim with node name /turtlesim2/sim
    [turtlesim_node-9] [INFO] [TTTTTTTTTT.TTTTTTTTT] [turtlesim1.sim]: Starting turtlesim with node name /turtlesim1/sim
    [turtlesim_node-10] [INFO] [TTTTTTTTTT.TTTTTTTTT] [turtlesim2.sim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
    [turtlesim_node-9] [INFO] [TTTTTTTTTT.TTTTTTTTT] [turtlesim1.sim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
    [talker-1] [INFO] [TTTTTTTTTT.TTTTTTTTT] [talker]: Publishing: 'Hello World: 1'
    [listener-2] [INFO] [TTTTTTTTTT.TTTTTTTTT] [listener]: I heard: [Hello World: 1]
    [talker-3] [INFO] [TTTTTTTTTT.TTTTTTTTT] [chatter.py.ns.talker]: Publishing: 'Hello World: 1'
    [listener-4] [INFO] [TTTTTTTTTT.TTTTTTTTT] [chatter.py.ns.listener]: I heard: [Hello World: 1]
    [talker-5] [INFO] [TTTTTTTTTT.TTTTTTTTT] [chatter.xml.ns.talker]: Publishing: 'Hello World: 1'
    [listener-6] [INFO] [TTTTTTTTTT.TTTTTTTTT] [chatter.xml.ns.listener]: I heard: [Hello World: 1]
    [talker-7] [INFO] [TTTTTTTTTT.TTTTTTTTT] [chatter.yaml.ns.talker]: Publishing: 'Hello World: 1'
    [listener-8] [INFO] [TTTTTTTTTT.TTTTTTTTT] [chatter.yaml.ns.listener]: I heard: [Hello World: 1]

To test that the remapping is working, you can control the turtles by running the following command in another terminal:

.. code-block:: console

    ros2 run turtlesim turtle_teleop_key --ros-args --remap __ns:=/turtlesim1

Python, XML, or YAML: Which should I use?
-----------------------------------------

.. note::

  Launch files in ROS 1 were written in XML, so XML may be the most familiar to people coming from ROS 1.
  To see what's changed, you can visit :doc:`Migrating-from-ROS1/Migrating-Launch-Files`.

For most applications the choice of which ROS 2 launch format comes down to developer preference.
However, if your launch file requires flexibility that you cannot achieve with XML or YAML, you can use Python to write your launch file.
Using Python for ROS 2 launch is more flexible because:

* Python is a scripting language, and thus you can leverage the language and its libraries in your launch files.
* `ros2/launch <https://github.com/ros2/launch>`_ (general launch features) and `ros2/launch_ros <https://github.com/ros2/launch_ros>`_ (ROS 2 specific launch features) are written in Python and thus you have lower level access to launch features that may not be yet exposed via XML and YAML.

That being said, a launch file written in Python may be more complex and verbose than one in XML or YAML.
