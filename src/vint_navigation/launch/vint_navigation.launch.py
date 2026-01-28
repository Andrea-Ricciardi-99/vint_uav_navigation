#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import conditions


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

    predicted_path_topic_arg = DeclareLaunchArgument(
        'predicted_path_topic',
        default_value='/vint/viz/predicted_path',
        description='Predicted path visualization topic'
    )

    markers_topic_arg = DeclareLaunchArgument(
        'markers_topic',
        default_value='/vint/viz/markers',
        description='Markers visualization topic'
    )

    annotated_image_topic_arg = DeclareLaunchArgument(
        'annotated_image_topic',
        default_value='/vint/viz/annotated_image',
        description='Annotated image visualization topic'
    )

    enable_viz_arg = DeclareLaunchArgument(
        'enable_viz',
        default_value='true',
        description='Enable visualization node'
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
    
    # Visualizer node
    visualizer_node = Node(
        package='vint_navigation',
        executable='vint_visualizer_node.py',
        name='vint_visualizer',
        namespace='uav1',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'waypoint_topic': LaunchConfiguration('waypoint_topic'),
            'closest_node_topic': '/vint/closest_node',
            'frame_id': 'uav1/fcu',  # Adjust to your MRS frame
            'predicted_path_topic': LaunchConfiguration('predicted_path_topic'),
            'markers_topic': LaunchConfiguration('markers_topic'),
            'annotated_image_topic': LaunchConfiguration('annotated_image_topic'),
        }],
        emulate_tty=True,
        condition=conditions.IfCondition(LaunchConfiguration('enable_viz'))
    )
    
    return LaunchDescription([
        topomap_dir_arg,
        model_arg,
        image_topic_arg,
        waypoint_topic_arg,
        predicted_path_topic_arg,
        markers_topic_arg,
        annotated_image_topic_arg,
        enable_viz_arg,
        vint_node,
        visualizer_node,
    ])
