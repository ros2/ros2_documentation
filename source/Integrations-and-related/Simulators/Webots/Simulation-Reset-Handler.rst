Setting up a Reset Handler
==========================

**Goal:** Extend a robot simulation with a reset handler to restart nodes when the reset button of Webots is pressed.

**Tutorial level:** Advanced

**Time:** 10 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

In this tutorial, you will learn how to implement a reset handler in a robot simulation using Webots.
The Webots reset button reverts the world to the initial state and restarts controllers.
It is convenient as it quickly resets the simulation, but in the context of ROS 2, robot controllers are not started again making the simulation stop.
The reset handler allows you to restart specific nodes or perform additional actions when the reset button in Webots is pressed.
This can be useful for scenarios where you need to reset the state of your simulation or restart specific components without completely restarting the complete ROS system.

Prerequisites
-------------

Before proceeding with this tutorial, make sure you have completed the following:

- Understanding of ROS 2 nodes and topics covered in the beginner :doc:`../../../../Tutorials`.
- Knowledge of Webots and ROS 2 and its interface package.
- Familiarity with :doc:`./Setting-Up-Simulation-Webots-Basic`.


Reset Handler for Simple Cases (Controllers Only)
-------------------------------------------------

In the launch file of your package, add the ``respawn`` parameter.

.. code-block:: python

  def generate_launch_description():
      robot_driver = WebotsController(
          robot_name='my_robot',
          parameters=[
              {'robot_description': robot_description_path}
          ],

          # Every time one resets the simulation the controller is automatically respawned
          respawn=True
      )

      # Starts Webots
      webots = WebotsLauncher(world=PathJoinSubstitution([package_dir, 'worlds', world]))

      return LaunchDescription([
          webots,
          robot_driver
      ])

On reset, Webots kills all driver nodes.
Therefore, to start them again after reset, you should set the ``respawn`` property of the driver node to ``True``.
It will ensure driver nodes are up and running after the reset.

Reset Handler for Multiple Nodes (No Shutdown Required)
-------------------------------------------------------

If you have some other nodes that have to be started along with the driver node (e.g. ``ros2_control`` nodes), then you can use the ``OnProcessExit`` event handler:

.. code-block:: python

  def get_ros2_control_spawners(*args):
      # Declare here all nodes that must be restarted at simulation reset
      ros_control_node = Node(
          package='controller_manager',
          executable='spawner',
          arguments=['diffdrive_controller']
      )
      return [
          ros_control_node
      ]

  def generate_launch_description():
      robot_driver = WebotsController(
          robot_name='my_robot',
          parameters=[
              {'robot_description': robot_description_path}
          ],

          # Every time one resets the simulation the controller is automatically respawned
          respawn=True
      )

      # Starts Webots
      webots = WebotsLauncher(world=PathJoinSubstitution([package_dir, 'worlds', world]))

      # Declare the reset handler that respawns nodes when robot_driver exits
      reset_handler = launch.actions.RegisterEventHandler(
          event_handler=launch.event_handlers.OnProcessExit(
              target_action=robot_driver,
              on_exit=get_ros2_control_spawners,
          )
      )

      return LaunchDescription([
          webots,
          robot_driver,
          reset_handler
      ] + get_ros2_control_spawners())

It is not possible to use the ``respawn`` property on the ``ros2_control`` node, as the spawner exits during launch time and not when the simulation is reset.
Instead we should declare a list of nodes in a function (e.g. ``get_ros2_control_spawners``).
The nodes of this list are started along other nodes when executing the launch file.
With the ``reset_handler``, the function is also declared as action to start when the ``robot_driver`` node exits, which corresponds to the moment when the simulation is reset in the Webots interface.
The ``robot_driver`` node still has the ``respawn`` property set to ``True``, so that it gets restarted along with ``ros2_control`` nodes.

Reset Handler Requiring Node Shutdown
-------------------------------------

With the current ROS 2 launch API, there is no way to make the reset work in launch files where nodes need to be shutdown before the restart (e.g. ``Nav2`` or ``RViz``).
The reason is that currently, ROS 2 doesn't allow to shutdown specific nodes from a launch file.
There is a solution, but it requires to manually restart nodes after pushing the reset button.

Webots needs to be started in a specific launch file without other nodes.

.. code-block:: python

  def generate_launch_description():
      # Starts Webots
      webots = WebotsLauncher(world=PathJoinSubstitution([package_dir, 'worlds', world]))

      return LaunchDescription([
          webots
      ])


A second launch file must be started from another process.
This launch file contains all other nodes, including robot controllers/plugins, Navigation2 nodes, RViz, state publishers, etc.

.. code-block:: python

  def generate_launch_description():
      robot_driver = WebotsController(
          robot_name='my_robot',
          parameters=[
              {'robot_description': robot_description_path}
          ]
      )

      ros_control_node = Node(
          package='controller_manager',
          executable='spawner',
          arguments=['diffdrive_controller']
      )

      nav2_node = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(os.path.join(
              get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
          launch_arguments=[
              ('map', nav2_map),
              ('params_file', nav2_params),
          ],
      )

      rviz = Node(
          package='rviz2',
          executable='rviz2',
          output='screen'
      )

      # Declare the handler that shuts all nodes down when robot_driver exits
      shutdown_handler = launch.actions.RegisterEventHandler(
          event_handler=launch.event_handlers.OnProcessExit(
              target_action=robot_driver,
              on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
          )
      )

      return LaunchDescription([
          robot_driver,
          ros_control_node,
          nav2_node,
          rviz,
          shutdown_handler
      ])

The second launch file contains a handler that triggers a shutdown event when the driver node exits (which is the case when the simulation is reset).
This second launch file must be manually restarted from the command line after pressing the reset button.

Summary
-------

In this tutorial, you learned how to implement a reset handler in a robot simulation using Webots.
The reset handler allows you to restart specific nodes or perform additional actions when the reset button in Webots is pressed.
You explored different approaches based on the complexity of your simulation and the requirements of your nodes.
