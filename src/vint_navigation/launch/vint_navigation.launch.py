#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Get package share directory
    pkg_share = FindPackageShare('vint_navigation')
    
    # Declare launch arguments
    topomap_dir_arg = DeclareLaunchArgument(
        'topomap_dir',
        default_value='',
        description='Absolute path to topomap images directory'
    )
    
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='vint',
        description='Model name (vint or gnm)'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Input camera image topic'
    )
    
    waypoint_topic_arg = DeclareLaunchArgument(
        'waypoint_topic',
        default_value='/vint/waypoint',
        description='Output waypoint topic'
    )
    
    # Navigator node
    vint_node = Node(
        package='vint_navigation',
        executable='vint_navigator_node.py',
        name='vint_navigator',
        namespace='uav1',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'vint_params.yaml']),
            {
                'topomap_dir': LaunchConfiguration('topomap_dir'),
                'model': LaunchConfiguration('model'),
                'image_topic': LaunchConfiguration('image_topic'),
                'waypoint_topic': LaunchConfiguration('waypoint_topic'),
            }
        ],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        topomap_dir_arg,
        model_arg,
        image_topic_arg,
        waypoint_topic_arg,
        vint_node,
    ])
