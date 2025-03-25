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
        'background_r', default_value=TextSubstitution(text='0')
    )
    background_g_launch_arg = DeclareLaunchArgument(
        'background_g', default_value=TextSubstitution(text='255')
    )
    background_b_launch_arg = DeclareLaunchArgument(
        'background_b', default_value=TextSubstitution(text='0')
    )
    chatter_py_ns_launch_arg = DeclareLaunchArgument(
        'chatter_py_ns', default_value=TextSubstitution(text='chatter/py/ns')
    )
    chatter_xml_ns_launch_arg = DeclareLaunchArgument(
        'chatter_xml_ns', default_value=TextSubstitution(text='chatter/xml/ns')
    )
    chatter_yaml_ns_launch_arg = DeclareLaunchArgument(
        'chatter_yaml_ns', default_value=TextSubstitution(text='chatter/yaml/ns')
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
            'background_r': LaunchConfiguration('background_r'),
            'background_g': LaunchConfiguration('background_g'),
            'background_b': LaunchConfiguration('background_b'),
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
