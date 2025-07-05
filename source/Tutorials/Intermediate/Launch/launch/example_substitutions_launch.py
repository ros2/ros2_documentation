from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    return LaunchDescription([
        DeclareLaunchArgument(
            'turtlesim_ns',
            default_value='turtlesim1'
        ),
        DeclareLaunchArgument(
            'use_provided_red',
            default_value='False'
        ),
        DeclareLaunchArgument(
            'new_background_r',
            default_value='200'
        ),
        Node(
            package='turtlesim',
            namespace=turtlesim_ns,
            executable='turtlesim_node',
            name='sim'
        ),
        ExecuteProcess(
            cmd=[[
                'ros2 service call ',
                turtlesim_ns,
                '/spawn ',
                'turtlesim/srv/Spawn ',
                '"{x: 2, y: 2, theta: 0.2}"'
            ]],
            shell=True
        ),
        ExecuteProcess(
            cmd=[[
                'ros2 param set ',
                turtlesim_ns,
                '/sim background_r ',
                '120'
            ]],
            shell=True
        ),
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    condition=IfCondition(
                        PythonExpression([
                            new_background_r,
                            ' == 200',
                            ' and ',
                            use_provided_red
                        ])
                    ),
                    cmd=[[
                        'ros2 param set ',
                        turtlesim_ns,
                        '/sim background_r ',
                        new_background_r
                    ]],
                    shell=True
                ),
            ],
        )
    ])
