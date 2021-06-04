Using Python, XML, and YAML for ROS 2 Launch Files
==================================================

.. contents:: Table of Contents
   :depth: 1
   :local:

This guide shows an example of how to write a ROS 2 launch file in Python, XML, and YAML.

Background
----------

Below is the same launch file implemented in Python, XML, and YAML.

.. tabs::

   .. group-tab:: Python

      .. code-block:: python

        import launch
        import launch_ros
        from ament_index_python import get_package_share_directory
        from launch.launch_description_sources import PythonLaunchDescriptionSource
        from launch_ros.actions import Node

        import os


        def generate_launch_description():

            background_r_launch_arg = launch.actions.DeclareLaunchArgument(
                "background_r", default_value=launch.substitutions.TextSubstitution(text="0")
            )
            background_g_launch_arg = launch.actions.DeclareLaunchArgument(
                "background_g", default_value=launch.substitutions.TextSubstitution(text="255")
            )
            background_b_launch_arg = launch.actions.DeclareLaunchArgument(
                "background_b", default_value=launch.substitutions.TextSubstitution(text="0")
            )
            chatter_ns_launch_arg = launch.actions.DeclareLaunchArgument(
                "chatter_ns", default_value=launch.substitutions.TextSubstitution(text="my/chatter/ns")
            )

            group_with_include = launch.actions.GroupAction(
                actions=[
                    launch_ros.actions.PushRosNamespace(
                        launch.substitutions.LaunchConfiguration('chatter_ns'),

                    ),
                    launch.actions.IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(get_package_share_directory('demo_nodes_cpp'),
                                        'launch/topics/talker_listener.launch.py'))
                    ),
                ]
            )

            return launch.LaunchDescription([
                background_r_launch_arg,
                background_g_launch_arg,
                background_b_launch_arg,
                chatter_ns_launch_arg,
                group_with_include,
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
                    name='sim',
                    parameters=[{
                        "background_r": launch.substitutions.LaunchConfiguration('background_r'),
                        "background_g": launch.substitutions.LaunchConfiguration('background_g'),
                        "background_b": launch.substitutions.LaunchConfiguration('background_b'),
                    }]
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


        if __name__ == '__main__':
            ls = launch.LaunchService()
            ls.include_launch_description(generate_launch_description())
            ls.run()

   .. group-tab:: XML

      .. code-block:: xml

        <launch>

          <!-- args that can be set from the command line or a default will be used-->
          <arg name="background_r" default="0"/>
          <arg name="background_g" default="125"/>
          <arg name="background_b" default="0"/>
          <arg name="chatter_ns" default="my/chatter/ns"/>

          <!-- include another launch file -->
          <include file="$(find-pkg-share demo_nodes_cpp)/launch/topics/talker_listener.launch.py"/>
          <!-- include another launch file in the chatter_ns namespace-->
          <group>
            <!-- push-ros-namespace to set namespace of included nodes -->
            <push-ros-namespace namespace="$(var chatter_ns)"/>
            <include file="$(find-pkg-share demo_nodes_cpp)/launch/topics/talker_listener.launch.py"/>
          </group>

          <!-- start a turtlesim_node in the turtlesim1 namespace -->
          <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim1"/>
          <!-- start another turtlesim_node in the turtlesim2 namespace
              and use args to set parameters -->
          <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim2">
            <param name="background_r" value="$(var background_r)"/>
            <param name="background_g" value="$(var background_g)"/>
            <param name="background_b" value="$(var background_b)"/>
          </node>
          <!-- perform remap so both turtles listen to the same command topic -->
          <node pkg="turtlesim" exec="mimic" name="mimic">
            <remap from="/input/pose" to="/turtlesim1/turtle1/pose"/>
            <remap from="/output/cmd_vel" to="/turtlesim2/turtle1/cmd_vel"/>
          </node>
        </launch>

   .. group-tab:: YAML

      .. code-block:: yaml

        launch:

        - arg:
            name: "background_r"
            default: "0"
        - arg:
            name: "background_g"
            default: "125"
        - arg:
            name: "background_b"
            default: "0"
        - arg:
            name: "chatter_ns"
            default: "my/chatter/ns"


        # Comment
        - group:
            - push-ros-namespace:
                namespace: "$(var chatter_ns)"
            - include:
                file: "$(find-pkg-share demo_nodes_cpp)/launch/topics/talker_listener.launch.py"

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


Set the args with

.. code-block:: console

  ros2 run <path to launch file> background_r:=255

You can control the turtles by running the following command in another terminal:

.. code-block:: console

  ros2 run turtlesim turtle_teleop_key --ros-args --remap __ns:=/turtlesim1

Python, XML, or YAML: Which should I use?
-----------------------------------------

Since it is a scripting language, using Python for your launch files will be the most flexible.
XML and YAML are equivalent, in that the both expose the same features to a parser;
although, XML is probably most familiar for those coming from ROS 1.
