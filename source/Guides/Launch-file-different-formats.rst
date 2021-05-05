Using Python, XML, and YAML for ROS 2 Launch Files
==================================================

.. contents:: Table of Contents
   :depth: 1
   :local:

This guide shows an example of how to write a ROS 2 launch file in Python, XML, and YAML.

Background
----------

Below is the same launch file implemented in Python, XML, and YAML.

.. note::

  TODO
  * Include an include
  * Use args

.. tabs::

   .. group-tab:: Python

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
                    name='sim',
                    parameters=[{
                            "background_r": 255,
                            "background_g": 0,
                            "background_b": 0,
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

   .. group-tab:: XML

      .. code-block:: xml

        <launch>
          <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim1"/>
          <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim2">
            <param name="background_r" value="255"/>
            <param name="background_g" value="0"/>
            <param name="background_b" value="0"/>
          </node>
          <node pkg="turtlesim" exec="mimic" name="mimic">
            <remap from="/input/pose" to="/turtlesim1/turtle1/pose"/>
            <remap from="/output/cmd_vel" to="/turtlesim2/turtle1/cmd_vel"/>
          </node>
        </launch>

   .. group-tab:: YAML

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
            param:
            -
              name: "background_r"
              value: 255
            -
              name: "background_g"
              value: 0
            -
              name: "background_b"
              value: 0
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

You can control the turtles by running the following command in another terminal:

.. code-block:: console

  ros2 run turtlesim turtle_teleop_key --ros-args --remap __ns:=/turtlesim1

Python, XML, or YAML: Which should I use?
-------------------

Since it is a scripting language, using Python for your launch files will be the most flexible.
XML and YAML are equivalent, in that the both expose the same features to a parser;
although, XML is probably most familiar for those coming from ROS 1.
