.. _UsingROS2LaunchForLargeProjects:

Using ROS2 Launch for large projects
====================================

**Goal:** Learn best practices of managing large projects using ROS 2 launch files

**Tutorial level:** Intermediate

**Time:** 20 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

This tutorial describes some tips for writing launch files for large projects.
The focus is on how to structure launch files so they may be reused as much as possible in different situations.
We will use the ``turtlesim`` and ``turtle_tf2_py`` packages for a case study.
We will create a new package called a ``launch_tutorial`` with 6 different launch files and cover different ROS 2 launch tools and tips.

Introduction
------------

Large applications on a robot typically involve several interconnected nodes, each of which has many parameters.
Simulation of multiple turtles in the turtle simulator can serve as a good example.
The turtle simulation consists of multiple turtle nodes, the world configuration, and TF broadcaster and listener nodes.
Collectively, there is a number of ROS parameters that affect the behavior and appearance of these nodes.

ROS 2 launch file allows us to define all this in one place.
By the end of a tutorial, launching the ``launch_turtlesim.launch.py`` in the ``launch_tutorial`` package will bring up everything required for the turtles to follow each other.
In this tutorial, we'll go over this launch file and all related features used.

Top-level organization
----------------------

On a top-level, we would like launch files to be as reusable as possible.
In this case, moving between identical robots can be done without changing the launch files at all.
Even a change such as moving from the robot to a simulator can be done with only a few changes.
We'll go over how the launch file is structured to make this possible.

Firstly, we will create a top-level launch file that will call separate launch files.
To do this, let's create a ``launch_turtlesim.launch.py`` file in the ``/launch`` folder of our ``launch_tutorial`` package.

.. code-block:: Python

   import os

   from ament_index_python.packages import get_package_share_directory

   from launch import LaunchDescription
   from launch.actions import IncludeLaunchDescription
   from launch.launch_description_sources import PythonLaunchDescriptionSource


   def generate_launch_description():
      turtlesim_world_1 = IncludeLaunchDescription(
         PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('launch_tutorial'), 'launch'),
            '/turtlesim_world_1.launch.py'])
         )
      turtlesim_world_2 = IncludeLaunchDescription(
         PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('launch_tutorial'), 'launch'),
            '/turtlesim_world_2.launch.py'])
         )
      broadcaster_listener_nodes = IncludeLaunchDescription(
         PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('launch_tutorial'), 'launch'),
            '/broadcaster_listener.launch.py']),
         launch_arguments={'target_frame': 'carrot1'}.items(),
         )
      mimic_node = IncludeLaunchDescription(
         PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('launch_tutorial'), 'launch'),
            '/mimic.launch.py'])
         )
      fixed_frame_node = IncludeLaunchDescription(
         PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('launch_tutorial'), 'launch'),
            '/fixed_broadcaster.launch.py'])
         )
      rviz_node = IncludeLaunchDescription(
         PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('launch_tutorial'), 'launch'),
            '/turtlesim_rviz.launch.py'])
         )

      return LaunchDescription([
         turtlesim_world_1,
         turtlesim_world_2,
         broadcaster_listener_nodes,
         mimic_node,
         fixed_frame_node,
         rviz_node
      ])

This launch file includes a set of other launch files.
Each of these included launch files contains nodes and parameters, and possibly nested includes, pertaining to one part of the system.
To be exact, we launch two turtlesim simulation worlds, TF broadcaster and listener nodes, mimic and fixed frame broadcaster node.
It can also be noted that some launch files just start one node, which is not the best way to do that, but it is done here to cover as many launch concepts as possible.

.. note:: Design Tip: Top-level launch files should be short, and consist of include's to other files corresponding to subcomponents of the application, and commonly changed parameters.

Writing launch files in the following manner makes it easy to swap out one piece of the system, as we'll see later.

However, there are cases when some nodes or launch files have to be launched separately due to performance and usage reasons.
There is therefore no universal answer on whether or not to split things into multiple launch files.

.. note:: Design tip: Be aware of the tradeoffs when deciding how many top-level launch files your application requires.


Parameters
----------

1 Setting parameters in the launch file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Now let's create a new file called ``turtlesim_world_1.launch.py``.

.. code-block:: Python

   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument
   from launch.substitutions import LaunchConfiguration, TextSubstitution

   from launch_ros.actions import Node


   def generate_launch_description():
      background_r_launch_arg = DeclareLaunchArgument(
         'background_r', default_value=TextSubstitution(text='0')
      )
      background_g_launch_arg = DeclareLaunchArgument(
         'background_g', default_value=TextSubstitution(text='84')
      )
      background_b_launch_arg = DeclareLaunchArgument(
         'background_b', default_value=TextSubstitution(text='122')
      )

      return LaunchDescription([
         background_r_launch_arg,
         background_g_launch_arg,
         background_b_launch_arg,
         Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            parameters=[{
               'background_r': LaunchConfiguration('background_r'),
               'background_g': LaunchConfiguration('background_g'),
               'background_b': LaunchConfiguration('background_b'),
            }]
         ),
      ])

This launch file starts the ``turtlesim_node`` node that starts the turtlesim simulation.
In addition to that, we declared parameters that are passed to our nodes.
This makes it easy to declare constants used for a particular robot setup.

2 Loading parameters from YAML file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Now create a ``turtlesim_world_2.launch.py`` file.

.. code-block:: Python

   import os

   from ament_index_python.packages import get_package_share_directory

   from launch import LaunchDescription
   from launch_ros.actions import Node


   def generate_launch_description():
      config = os.path.join(
         get_package_share_directory('launch_tutorial'),
         'config',
         'turtlesim.yaml'
         )

      return LaunchDescription([
         Node(
            package='turtlesim',
            executable='turtlesim_node',
            namespace='turtlesim2',
            name = 'sim',
            parameters=[config]
         )
      ])

This launch file will launch the same node, but with different parameter values.
In addition, you could notice that parameters here loaded directly from the YAML file.
Let's now create a ``turtlesim.yaml`` in the ``/config`` folder of our package.

.. code-block:: YAML

   /turtlesim2/sim:
      ros__parameters:
         background_b: 255
         background_g: 86
         background_r: 150

If we now start the ``turtlesim_world_2.launch.py`` launch file, we will start the ``turtlesim_node`` with preconfigured background colors.

To learn more about using parameters and using YAML files, take a look at the :ref:`Understanding ROS 2 parameters <ROS2Params>` tutorial.

In the latter launch file, you could notice that we have defined the ``namespace='turtlesim2'``.
Unique namespaces allow the system to start two simulators without node name nor topic name conflicts.

Reusing nodes
-------------

Create a ``broadcaster_listener.launch.py`` file.

.. code-block:: Python

   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument
   from launch.substitutions import LaunchConfiguration

   from launch_ros.actions import Node


   def generate_launch_description():
      return LaunchDescription([
         DeclareLaunchArgument(
            'target_frame', default_value='turtle1',
            description='Target frame name.'
         ),
         Node(
            package='turtle_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[
               {'turtlename': 'turtle1'}
            ]
         ),
         Node(
            package='turtle_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='broadcaster2',
            parameters=[
               {'turtlename': 'turtle2'}
            ]
         ),
         Node(
            package='turtle_tf2_py',
            executable='turtle_tf2_listener',
            name='listener',
            parameters=[
               {'target_frame': LaunchConfiguration('target_frame')}
            ]
         ),
      ])

In this example, we reuse the same ``turtle_tf2_broadcaster`` node using different names and parameters during launch.
This allows us to duplicate the same node without conflicts.

In addition to that, we have declared the ``target_frame`` launch argument with a default value of ``turtle1``.
It means that this launch file can receive a parameter that it can pass forward to its nodes.
In case the argument is not provided, it will pass the default value.

Parameter overrides
-------------------

Recall that we called the ``broadcaster_listener.launch.py`` file in our top-level launch file.
In addition to that, we have provided it and passed ``target_frame`` launch argument.

.. code-block:: Python

   broadcaster_listener_nodes = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/broadcaster_listener.launch.py']),
      launch_arguments={'target_frame': 'carrot1'}.items(),
      )

This technique allowed us to change the default goal target frame to ``carrot1``.
If you would like ``turtle2`` to follow ``turtle1`` instead of the ``carrot1``, just remove the line that defines ``launch_arguments``.

Remapping
---------

Now create a ``mimic.launch.py`` file.

.. code-block:: Python

   from launch import LaunchDescription
   from launch_ros.actions import Node


   def generate_launch_description():
      return LaunchDescription([
         Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
               ('/input/pose', '/turtle2/pose'),
               ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
         )
      ])

This launch file will start the ``mimic`` node, which is designed to receive the target pose on the topic ``/input/pose``.
In our case, we want to make the target pose that is published on the ``/turtle2/pose`` topic, so we remap it.
Finally, we remap the ``/output/cmd_vel`` topic to ``/turtlesim2/turtle1/cmd_vel``.
This way ``turtle1`` in our ``turtlesim2`` simulation world will follow ``turtle2`` in our initial turtlesim world.

.. note:: Design tip: Use topic remapping when a given type of information is published on different topics in different situations.

Config files
------------

Let's now create a file called ``turtlesim_rviz.launch.py``.

.. code-block:: Python

   import os

   from ament_index_python.packages import get_package_share_directory

   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
      rviz_config = os.path.join(
         get_package_share_directory('turtle_tf2_py'),
         'rviz',
         'turtle_rviz.rviz'
         )

      return LaunchDescription([
         Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
         )
      ])

This launch file will start the RViz with the configuration defined in the ``turtle_tf2_py`` package.
This RViz configuration will set the world frame, enable TF visualization and a top-down view.

Environment Variables
---------------------

Let's now create the last launch file called ``fixed_broadcaster.launch.py`` in our package.

.. code-block:: Python

   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument
   from launch.substitutions import EnvironmentVariable, LaunchConfiguration
   from launch_ros.actions import Node


   def generate_launch_description():
      return LaunchDescription([
         DeclareLaunchArgument(
               'node_prefix',
               default_value=[EnvironmentVariable('USER'), '_'],
               description='prefix for node name'
         ),
         Node(
               package='turtle_tf2_py',
               executable='fixed_frame_tf2_broadcaster',
               name=[LaunchConfiguration('node_prefix'), 'fixed_broadcaster'],
         ),
      ])

Simply shows the way environment variables can be called inside the launch files.
Environment variables can be used to define namespaces for distinguishing nodes on different computers or robots.

Summary
-------

In this tutorial, you learned about various tips and practices of managing large projects using ROS 2 launch files.
