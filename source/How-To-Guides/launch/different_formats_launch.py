from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_dir = PathJoinSubstitution([FindPackageShare('demo_nodes_cpp'), 'launch', 'topics'])
    return LaunchDescription([
        # args that can be set from the command line or a default will be used
        DeclareLaunchArgument('background_r', default_value='0'),
        DeclareLaunchArgument('background_g', default_value='255'),
        DeclareLaunchArgument('background_b', default_value='0'),
        DeclareLaunchArgument('chatter_ns', default_value='my/chatter/ns'),

        # include another launch file
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'talker_listener.launch.py'])
        ),

        # include a Python launch file in the chatter_py_ns namespace
        GroupAction(
            actions=[
                # push_ros_namespace first to set namespace of included nodes for following actions
                PushROSNamespace(LaunchConfiguration('chatter_ns')),
                IncludeLaunchDescription(
                    PathJoinSubstitution([launch_dir, 'talker_listener.launch.py'])),
            ]
        ),

        # include a xml launch file in the chatter_xml_ns namespace
        GroupAction(
            actions=[
                # push_ros_namespace first to set namespace of included nodes for following actions
                PushROSNamespace('chatter_xml_ns'),
                IncludeLaunchDescription(
                    PathJoinSubstitution([launch_dir, 'talker_listener.launch.xml'])),
            ]
        ),

        # include a yaml launch file in the chatter_yaml_ns namespace
        GroupAction(
            actions=[
                # push_ros_namespace first to set namespace of included nodes for following actions
                PushROSNamespace('chatter_yaml_ns'),
                IncludeLaunchDescription(
                    PathJoinSubstitution([launch_dir, 'talker_listener.launch.yaml'])),
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
                'background_r': LaunchConfiguration('background_r'),
                'background_g': LaunchConfiguration('background_g'),
                'background_b': LaunchConfiguration('background_b'),
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
