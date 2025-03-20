.. redirect-from::

  Guides/Launch-file-different-formats

Using XML, YAML, and Python for ROS 2 Launch Files
==================================================

.. contents:: Table of Contents
   :depth: 1
   :local:

ROS 2 launch files can be written in XML, YAML, and Python.
This guide shows how to use these different formats to accomplish the same task, as well as has some discussion on when to use each format.

Launch file examples
--------------------

Below is a launch file implemented in XML, YAML, and Python.
Each launch file performs the following actions:

* Setup command line arguments with defaults
* Include another launch file
* Include another launch file in another namespace
* Start a node and setting its namespace
* Start a node, setting its namespace, and setting parameters in that node (using the args)
* Create a node to remap messages from one topic to another

.. tabs::

   .. group-tab:: XML

      .. code-block:: xml

        <?xml version="1.0" encoding="UTF-8"?>
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
            - name: "background_r"
              value: "$(var background_r)"
            - name: "background_g"
              value: "$(var background_g)"
            - name: "background_b"
              value: "$(var background_b)"

        # perform remap so both turtles listen to the same command topic
        - node:
            pkg: "turtlesim"
            exec: "mimic"
            name: "mimic"
            remap:
            - from: "/input/pose"
              to: "/turtlesim1/turtle1/pose"
            - from: "/output/cmd_vel"
              to: "/turtlesim2/turtle1/cmd_vel"

   .. group-tab:: Python

      .. code-block:: python

        #!/usr/bin/env python3
        import os

        from ament_index_python import get_package_share_directory

        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument
        from launch.actions import GroupAction
        from launch.actions import IncludeLaunchDescription
        from launch.launch_description_sources import AnyLaunchDescriptionSource
        from launch.substitutions import LaunchConfiguration
        from launch.substitutions import TextSubstitution
        from launch_ros.actions import Node
        from launch_ros.actions import PushROSNamespace

        def generate_launch_description():
            return LaunchDescription([
                # args that can be set from the command line or a default will be used
                DeclareLaunchArgument(
                    "background_r", default_value=TextSubstitution(text="0")
                ),
                DeclareLaunchArgument(
                    "background_g", default_value=TextSubstitution(text="255")
                ),
                DeclareLaunchArgument(
                    "background_b", default_value=TextSubstitution(text="0")
                ),
                DeclareLaunchArgument(
                    "chatter_py_ns", default_value=TextSubstitution(text="chatter/py/ns")
                ),
                DeclareLaunchArgument(
                    "chatter_xml_ns", default_value=TextSubstitution(text="chatter/xml/ns")
                ),
                DeclareLaunchArgument(
                    "chatter_yaml_ns", default_value=TextSubstitution(text="chatter/yaml/ns")
                ),
                # include another launch file
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('demo_nodes_cpp'),
                            'launch/topics/talker_listener_launch.py'))
                ),
                # include a Python launch file in the chatter_py_ns namespace
                GroupAction(
                    actions=[
                        # push_ros_namespace first to set namespace of included nodes for following actions
                        PushROSNamespace('chatter_py_ns'),
                        IncludeLaunchDescription(
                            AnyLaunchDescriptionSource(
                                os.path.join(
                                    get_package_share_directory('demo_nodes_cpp'),
                                    'launch/topics/talker_listener_launch.py'))
                        ),
                    ]
                ),
                # include a xml launch file in the chatter_xml_ns namespace
                GroupAction(
                    actions=[
                        # push_ros_namespace first to set namespace of included nodes for following actions
                        PushROSNamespace('chatter_xml_ns'),
                        IncludeLaunchDescription(
                            AnyLaunchDescriptionSource(
                                os.path.join(
                                    get_package_share_directory('demo_nodes_cpp'),
                                    'launch/topics/talker_listener_launch.xml'))
                        ),
                    ]
                ),
                # include a yaml launch file in the chatter_yaml_ns namespace
                GroupAction(
                    actions=[
                        # push_ros_namespace first to set namespace of included nodes for following actions
                        PushROSNamespace('chatter_yaml_ns'),
                        IncludeLaunchDescription(
                            AnyLaunchDescriptionSource(
                                os.path.join(
                                    get_package_share_directory('demo_nodes_cpp'),
                                    'launch/topics/talker_listener_launch.yaml'))
                        ),
                    ]
                ),
                # start a turtlesim_node in the turtlesim1 namespace
                Node(
                    package='turtlesim',
                    namespace='turtlesim1',
                    executable='turtlesim_node',
                    name='sim'
                ),
                # start another turtlesim_node in the turtlesim2 namespace
                # and use args to set parameters
                Node(
                    package='turtlesim',
                    namespace='turtlesim2',
                    executable='turtlesim_node',
                    name='sim',
                    parameters=[{
                        "background_r": LaunchConfiguration('background_r'),
                        "background_g": LaunchConfiguration('background_g'),
                        "background_b": LaunchConfiguration('background_b'),
                    }]
                ),
                # perform remap so both turtles listen to the same command topic
                Node(
                    package='turtlesim',
                    executable='mimic',
                    name='mimic',
                    remappings=[
                        ('/input/pose', '/turtlesim1/turtle1/pose'),
                        ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
                    ]
                ),
            ])

Using the Launch files from the command line
--------------------------------------------

Launching
^^^^^^^^^

Any of the launch files above can be run with ``ros2 launch``.
To try them locally, you can either create a new package and use

.. code-block:: console

  ros2 launch <package_name> <launch_file_name>

or run the file directly by specifying the path to the launch file

.. code-block:: console

  ros2 launch <path_to_launch_file>

Setting arguments
^^^^^^^^^^^^^^^^^

To set the arguments that are passed to the launch file, you should use ``key:=value`` syntax.
For example, you can set the value of ``background_r`` in the following way:

.. code-block:: console

  ros2 launch <package_name> <launch_file_name> background_r:=255

or

.. code-block:: console

  ros2 launch <path_to_launch_file> background_r:=255

Controlling the turtles
^^^^^^^^^^^^^^^^^^^^^^^

To test that the remapping is working, you can control the turtles by running the following command in another terminal:

.. code-block:: console

  ros2 run turtlesim turtle_teleop_key --ros-args --remap __ns:=/turtlesim1


.. _launch-file-different-formats-which:

XML, YAML, or Python: Which should I use?
-----------------------------------------

.. note::

  Launch files in ROS 1 were written in XML, so XML may be the most familiar to people coming from ROS 1.
  To see what's changed, you can visit :doc:`Migrating-from-ROS1/Migrating-Launch-Files`.

To achieve a more declarative self-documenting style, you should prefer XML or YAML launch files, between them it comes down to developer preference.

However, if your launch file requires advanced functionality that you cannot achieve with XML or YAML, you can turn to the Python launch API.
Using Python for ROS 2 launch allows far more flexibility because it is a full scripting language and has access to the underlying implementation -- not all of which is exposed to XML/YAML -- but using it comes with the drawback of being more verbose and harder to reason about the resulting launch description.
