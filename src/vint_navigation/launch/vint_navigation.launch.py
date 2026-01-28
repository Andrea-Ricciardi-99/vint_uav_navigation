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
    
    enable_control_arg = DeclareLaunchArgument(
        'enable_control',
        default_value='false',
        description='Enable MRS control (sends goto commands)'
    )
    
    # Frame arguments
    world_frame_arg = DeclareLaunchArgument(
        'world_frame',
        default_value='world',
        description='World frame name'
    )
    
    fcu_frame_arg = DeclareLaunchArgument(
        'fcu_frame',
        default_value='',
        description='FCU frame name (empty = auto {uav_name}/fcu)'
    )
    
    # Camera and waypoint parameters
    camera_rotation_arg = DeclareLaunchArgument(
        'camera_rotation_deg',
        default_value='90.0',
        description='Camera rotation from fcu frame (degrees, clockwise positive)'
    )
    
    scale_factor_arg = DeclareLaunchArgument(
        'scale_factor',
        default_value='1.0',
        description='Scale factor for ViNT waypoint distances'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='4.0',
        description='Rate (Hz) for sending goto commands to MRS'
    )
    
    use_heading_arg = DeclareLaunchArgument(
        'use_heading',
        default_value='true',
        description='Use ViNT heading predictions'
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
    
    # Visualizer node (optional)
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
            'frame_id': 'uav1/fcu',
            'predicted_path_topic': LaunchConfiguration('predicted_path_topic'),
            'markers_topic': LaunchConfiguration('markers_topic'),
            'annotated_image_topic': LaunchConfiguration('annotated_image_topic'),
        }],
        emulate_tty=True,
        condition=conditions.IfCondition(LaunchConfiguration('enable_viz'))
    )
    
    # MRS waypoint converter (optional - for actual control)
    mrs_converter_node = Node(
        package='vint_navigation',
        executable='vint_to_mrs_waypoint_node.py',
        name='vint_to_mrs_waypoint',
        namespace='uav1',
        output='screen',
        parameters=[{
            'waypoint_topic': LaunchConfiguration('waypoint_topic'),
            'uav_name': 'uav1',
            'world_frame': LaunchConfiguration('world_frame'),
            'fcu_frame': LaunchConfiguration('fcu_frame'),
            'camera_rotation_deg': LaunchConfiguration('camera_rotation_deg'),
            'use_heading': LaunchConfiguration('use_heading'),
            'scale_factor': LaunchConfiguration('scale_factor'),
            'update_rate': LaunchConfiguration('update_rate'),
        }],
        emulate_tty=True,
        condition=conditions.IfCondition(LaunchConfiguration('enable_control'))
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
        enable_control_arg,
        world_frame_arg,
        fcu_frame_arg,
        camera_rotation_arg,
        scale_factor_arg,
        update_rate_arg,
        use_heading_arg,
        vint_node,
        visualizer_node,
        mrs_converter_node,
    ])
