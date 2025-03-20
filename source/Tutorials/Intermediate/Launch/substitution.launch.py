#!/usr/bin/env python3
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    arg_turtlesim_ns = 'turtlesim_ns'
    arg_use_provided_red = 'use_provided_red'
    arg_new_background_r = 'new_background_r'

    return LaunchDescription([
        DeclareLaunchArgument(
            arg_turtlesim_ns,
            default_value='turtlesim1',
        ),
        DeclareLaunchArgument(
            arg_use_provided_red,
            default_value='False',
        ),
        DeclareLaunchArgument(
            arg_new_background_r,
            default_value='200',
        ),
        Node(
            package='turtlesim',
            namespace=LaunchConfiguration(arg_turtlesim_ns),
            executable='turtlesim_node',
            name='sim',
        ),
        ExecuteProcess(
            cmd=[[
                'ros2 service call ',
                LaunchConfiguration(arg_turtlesim_ns),
                '/spawn ',
                'turtlesim_msgs/srv/Spawn ',
                '"{x: 2, y: 2, theta: 0.2}"',
            ]],
            shell=True,
        ),
        ExecuteProcess(
            cmd=[[
                'ros2 param set ',
                LaunchConfiguration(arg_turtlesim_ns),
                '/sim background_r ',
                '120',
            ]],
            shell=True,
        ),
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    condition=IfCondition(
                        PythonExpression([
                            LaunchConfiguration(arg_new_background_r),
                            ' == 200',
                            ' and ',
                            LaunchConfiguration(arg_use_provided_red),
                        ])
                    ),
                    cmd=[[
                        'ros2 param set ',
                        LaunchConfiguration(arg_turtlesim_ns),
                        '/sim background_r ',
                        LaunchConfiguration(arg_new_background_r),
                    ]],
                    shell=True,
                ),
            ],
        ),
    ])
