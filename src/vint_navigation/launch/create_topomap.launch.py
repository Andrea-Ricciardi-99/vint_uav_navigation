#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Declare launch arguments
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Camera image topic to record from'
    )
    
    topomap_dir_arg = DeclareLaunchArgument(
        'topomap_dir',
        default_value='',
        description='Base directory for topomaps (default: ~/topomaps)'
    )
    
    topomap_name_arg = DeclareLaunchArgument(
        'topomap_name',
        default_value='my_topomap',
        description='Name of the topomap to create'
    )
    
    dt_arg = DeclareLaunchArgument(
        'dt',
        default_value='1.0',
        description='Time interval between saved images (seconds)'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Automatically start recording on launch'
    )
    
    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='5.0',
        description='Timeout for no images before auto-stopping (seconds)'
    )
    
    # Topomap creator node
    topomap_creator_node = Node(
        package='vint_navigation',
        executable='create_topomap_node.py',
        name='topomap_creator',
        output='screen',
        namespace='uav1',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'topomap_dir': LaunchConfiguration('topomap_dir'),
            'topomap_name': LaunchConfiguration('topomap_name'),
            'dt': LaunchConfiguration('dt'),
            'auto_start': LaunchConfiguration('auto_start'),
            'timeout': LaunchConfiguration('timeout'),
        }],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        image_topic_arg,
        topomap_dir_arg,
        topomap_name_arg,
        dt_arg,
        auto_start_arg,
        timeout_arg,
        topomap_creator_node,
    ])
