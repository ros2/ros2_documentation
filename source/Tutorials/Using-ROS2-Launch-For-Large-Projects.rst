.. _UsingROS2LaunchForLargeProjects:

Using ROS2 Launch for large projects
====================================

**Goal:**

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

Large applications on a robot typically involve several interconnected nodes, each of which have many parameters.
Simulation of multiple turtles in the turtle simulator can serve as a good example.
The turtle simulation consists of multiple turtle nodes, the world configuration, and TF broadcaster and listener nodes.
Collectively, there is a number of ROS parameters that affect the behavior and appearance of these nodes.

ROS 2 launch file allows us to define all this in one place.
By the end of a tutorial, launching the ``launch_turtlesim.launch.py`` in the ``launch_tutorial`` package will bring up everything required for the turtles to follow each other.
In this tutorial, we'll go over this launch file and all related features used.

Top-level organization
----------------------

On a top level, we would like launch files to be as reusable as possible.
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
         launch_arguments={'target_frame': 'turtle1'}.items(),
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

      return LaunchDescription([
         turtlesim_world_1,
         turtlesim_world_2,
         broadcaster_listener_nodes,
         mimic_node,
         fixed_frame_node
      ])

This launch file includes a set of other launch files.
Each of these included launch files contains nodes and parameters, and possibly nested includes, pertaining to one part of the system.
To be exact, we launch two turtlesim simulation worlds, TF broadcaster and listener nodes, mimic and fixed frame broadcaster node.
It can also be noted that some launch files just start one node, which is not the best way to do that, but it is done here to cover as much launch concepts as possible.

.. note:: Design Tip: Top-level launch files should be short, and consist of include's to other files corresponding to subcomponents of the application, and commonly changed parameters.

Writing launch files in a following manner makes it easy to swap out one piece of the system, as we'll see later.

However, there are cases when some nodes or launch files have to launched separately due to performance and usage reasons.
There is therefore no universal answer on whether or not to split things into multiple launch files.

.. note:: Design tip: Be aware of the tradeoffs when deciding how many top-level launch files your application requires.


Parameters
----------

1 Setting parameters in launch file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
In addition to that we declared parameters that are passed to our nodes.
This makes it easy to declare constants used for particular robot setup.

2 Loading parameters from yaml file
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
In addition, you could notice that pararmeters here loaded directly from the yaml file.
Let's now create a ``turtlesim.yaml`` in the ``/config`` folder of our package.

.. code-block:: YAML

   /turtlesim2/sim:
      ros__parameters:
         background_b: 255
         background_g: 86
         background_r: 150

If we now start the ``turtlesim_world_2.launch.py`` launch file, we will start the ``turtlesim_node`` with preconfigured background colors.

To learn more about using parameters and using yaml files, take a look at the :ref:`Understanding ROS 2 parameters <ROS2Params>` tutorial.

In the latter launch file you could notice that we have defined the ``namespace='turtlesim2'``. 
Unique namespaces allow the system to start two simulators without node name nor topic name conflicts.

Reusing launch files
--------------------

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


In this example we reuse the same ``turtle_tf2_broadcaster`` node using different names and parameters during launch.
This allows us to duplicate the same node without conflicts. 


In addition to that we have declared the launch arguments with a default value.
It means that this launch file can receive a paramter that it can pass forward to its nodes.
This feature is used to change the target frame to carrot1 in our initial call in the main launch file.

Recall that in our top-level launch file called this launch file and passed ``target_frame`` launch argument.

.. code-block:: Python

   broadcaster_listener_nodes = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('launch_tutorial'), 'launch'),
            '/broadcaster_listener.launch.py']),
      launch_arguments={'target_frame': 'turtle1'}.items(),
      )

Remapping
---------

``mimic.launch.py``

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

Mimic node is designed to receive pose on the topic ``/input/pose``.
In the case of the turtlesim, pose is published on the ``/turtlesim1/turtle1/pose`` topic, so we remap it.

.. note:: Design tip: Use topic remapping when a given type of information is published on different topics in different situations.


Parameter overrides
-------------------
private overriding- Ros2 launch arguments
like --debug, --noninteractive, etc

Config files
------------
importing files like rviz configs (edited)


Environment Variables
---------------------

how to define and use environment variables https://answers.ros.org/question/318416/add-an-environment-variable-in-executeprocess/

``fixed_broadcaster.launch.py``

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
