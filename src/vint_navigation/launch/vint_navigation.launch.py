#!/usr/bin/env python3
"""
ViNT Navigation Launch File for MRS UAV System

Launches:
  - ViNT Navigator: Visual navigation model
  - Visualizer: RViz visualization (optional)
  - Velocity Reference Generator: Converts ViNT waypoints to MRS velocity commands (optional)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import conditions


def generate_launch_description():
    
    # Get package share directory
    pkg_share = FindPackageShare('vint_navigation')
    
    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================
    
    # UAV name must be declared FIRST (used in other arguments)
    uav_name_arg = DeclareLaunchArgument(
        'uav_name',
        default_value='uav1',
        description='UAV name for MRS topics'
    )
    
    # --- ViNT Navigator Arguments ---
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
        default_value='/uav1/bluefox_front/image_raw',
        description='Input camera image topic (use /uavX/... for specific UAV)'
    )
    
    # Use relative topic names - namespace is prepended automatically
    waypoint_topic_arg = DeclareLaunchArgument(
        'waypoint_topic',
        default_value='vint/waypoint',
        description='Output waypoint topic (relative to namespace)'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug logging in navigator node'
    )
    
    # --- Visualization Arguments ---
    predicted_path_topic_arg = DeclareLaunchArgument(
        'predicted_path_topic',
        default_value='vint/viz/predicted_path',
        description='Predicted path visualization topic (relative to namespace)'
    )
    
    markers_topic_arg = DeclareLaunchArgument(
        'markers_topic',
        default_value='vint/viz/markers',
        description='Markers visualization topic (relative to namespace)'
    )
    
    annotated_image_topic_arg = DeclareLaunchArgument(
        'annotated_image_topic',
        default_value='vint/viz/annotated_image',
        description='Annotated image visualization topic (relative to namespace)'
    )
    
    closest_node_topic_arg = DeclareLaunchArgument(
        'closest_node_topic',
        default_value='vint/closest_node',
        description='Closest node topic (relative to namespace)'
    )
    
    enable_viz_arg = DeclareLaunchArgument(
        'enable_viz',
        default_value='true',
        description='Enable visualization node'
    )
    
    # --- Control Arguments ---
    enable_control_arg = DeclareLaunchArgument(
        'enable_control',
        default_value='false',
        description='Enable MRS velocity control'
    )
    
    # --- Velocity Control Parameters ---
    max_horizontal_speed_arg = DeclareLaunchArgument(
        'max_horizontal_speed',
        default_value='2.5',
        description='Maximum horizontal velocity (m/s)'
    )
    
    max_vertical_speed_arg = DeclareLaunchArgument(
        'max_vertical_speed',
        default_value='1.0',
        description='Maximum vertical velocity (m/s)'
    )
    
    max_yaw_rate_arg = DeclareLaunchArgument(
        'max_yaw_rate',
        default_value='0.8',
        description='Maximum yaw rate (rad/s)'
    )
    
    dt_arg = DeclareLaunchArgument(
        'dt',
        default_value='0.1',
        description='Control timestep for velocity conversion (seconds)'
    )
    
    velocity_damping_arg = DeclareLaunchArgument(
        'velocity_damping',
        default_value='0.7',
        description='Velocity smoothing factor [0-1]'
    )
    
    altitude_hold_arg = DeclareLaunchArgument(
        'altitude_hold',
        default_value='2.0',
        description='Target altitude for navigation (meters)'
    )
    
    # --- ViNT Model Parameters (from training) ---
    vint_is_normalized_arg = DeclareLaunchArgument(
        'vint_is_normalized',
        default_value='true',
        description='Whether ViNT outputs are normalized (from training config)'
    )
    
    vint_max_v_arg = DeclareLaunchArgument(
        'vint_max_v',
        default_value='0.5',
        description='MAX_V from ViNT training robot.yaml'
    )
    
    vint_rate_arg = DeclareLaunchArgument(
        'vint_rate',
        default_value='10.0',
        description='frame_rate from ViNT training robot.yaml'
    )
    
    use_heading_arg = DeclareLaunchArgument(
        'use_heading',
        default_value='false',
        description='Use ViNT heading predictions (learn_angle from training)'
    )

    # --- Collision Avoidance Parameters ---
    use_collision_avoidance_arg = DeclareLaunchArgument(
        'use_collision_avoidance',
        default_value='true',
        description='Enable depth-based collision avoidance'
    )

    collision_threshold_arg = DeclareLaunchArgument(
        'collision_threshold',
        default_value='1.0',
        description='Minimum safe distance to obstacles (meters)'
    )

    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/uav1/rgbd_front/depth/image_raw',
        description='Depth camera topic for collision avoidance'
    )
    
    # ========================================================================
    # NODES
    # ========================================================================
    
    # --- ViNT Navigator Node ---
    vint_node = Node(
        package='vint_navigation',
        executable='vint_navigator_node.py',
        name='vint_navigator',
        namespace=LaunchConfiguration('uav_name'),
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'vint_params.yaml']),
            {
                'topomap_dir': LaunchConfiguration('topomap_dir'),
                'model': LaunchConfiguration('model'),
                'image_topic': LaunchConfiguration('image_topic'),
                'waypoint_topic': LaunchConfiguration('waypoint_topic'),
                'closest_node_topic': LaunchConfiguration('closest_node_topic'),
                'debug': LaunchConfiguration('debug'),
            }
        ],
        emulate_tty=True,
    )
    
    # --- Visualization Node (Optional) ---
    visualizer_node = Node(
        package='vint_navigation',
        executable='vint_visualizer_node.py',
        name='vint_visualizer',
        namespace=LaunchConfiguration('uav_name'),
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'waypoint_topic': LaunchConfiguration('waypoint_topic'),
            'closest_node_topic': LaunchConfiguration('closest_node_topic'),
            'frame_id': PythonExpression(["'", LaunchConfiguration('uav_name'), "/gps_garmin_origin'"]),
            'predicted_path_topic': LaunchConfiguration('predicted_path_topic'),
            'markers_topic': LaunchConfiguration('markers_topic'),
            'annotated_image_topic': LaunchConfiguration('annotated_image_topic'),
        }],
        emulate_tty=True,
        condition=conditions.IfCondition(LaunchConfiguration('enable_viz'))
    )
    
    # --- Velocity Reference Generator (Optional - for MRS control) ---
    velocity_control_node = Node(
        package='vint_navigation',
        executable='vint_velocity_reference_generator.py',
        name='vint_velocity_reference_generator',
        namespace=LaunchConfiguration('uav_name'),
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'velocity_reference_generator.yaml']),
            {
                # UAV configuration
                'uav_name': LaunchConfiguration('uav_name'),
                'waypoint_topic': LaunchConfiguration('waypoint_topic'),
                
                # Velocity limits
                'max_horizontal_speed': LaunchConfiguration('max_horizontal_speed'),
                'max_vertical_speed': LaunchConfiguration('max_vertical_speed'),
                'max_yaw_rate': LaunchConfiguration('max_yaw_rate'),
                
                # Control parameters
                'dt': LaunchConfiguration('dt'),
                'velocity_damping': LaunchConfiguration('velocity_damping'),
                
                # ViNT model configuration
                'vint_is_normalized': LaunchConfiguration('vint_is_normalized'),
                'vint_max_v': LaunchConfiguration('vint_max_v'),
                'vint_rate': LaunchConfiguration('vint_rate'),
                'use_heading': LaunchConfiguration('use_heading'),
                
                # Flight parameters
                'altitude_hold': LaunchConfiguration('altitude_hold'),

                # Collision avoidance parameters
                'use_collision_avoidance': LaunchConfiguration('use_collision_avoidance'),
                'collision_threshold': LaunchConfiguration('collision_threshold'),
                'depth_topic': LaunchConfiguration('depth_topic'),
            }
        ],
        emulate_tty=True,
        condition=conditions.IfCondition(LaunchConfiguration('enable_control'))
    )
    
    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================
    
    return LaunchDescription([
        # UAV name MUST be first
        uav_name_arg,
        
        # ViNT Navigator arguments
        topomap_dir_arg,
        model_arg,
        image_topic_arg,
        waypoint_topic_arg,
        debug_arg,
        
        # Visualization arguments
        predicted_path_topic_arg,
        markers_topic_arg,
        annotated_image_topic_arg,
        closest_node_topic_arg,
        enable_viz_arg,
        
        # Control arguments
        enable_control_arg,
        
        # Velocity control parameters
        max_horizontal_speed_arg,
        max_vertical_speed_arg,
        max_yaw_rate_arg,
        dt_arg,
        velocity_damping_arg,
        altitude_hold_arg,
        
        # ViNT model parameters
        vint_is_normalized_arg,
        vint_max_v_arg,
        vint_rate_arg,
        use_heading_arg,

        # Collision avoidance parameters
        use_collision_avoidance_arg,
        collision_threshold_arg,
        depth_topic_arg,
        
        # Nodes
        vint_node,
        visualizer_node,
        velocity_control_node,
    ])
