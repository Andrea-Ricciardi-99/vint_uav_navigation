#!/usr/bin/env python3
"""
ViNT Velocity Reference Generator for MRS UAV System

Converts ViNT spatial waypoints to velocity references for MRS controllers.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from mrs_msgs.msg import VelocityReferenceStamped
import numpy as np


def euler_from_quaternion(x, y, z, w):
    """Convert quaternion to Euler angles (roll, pitch, yaw)."""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch = np.arcsin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)
    
    return roll, pitch, yaw


def clip_angle(theta):
    """Normalize angle to [-π, π]."""
    theta %= 2 * np.pi
    if -np.pi < theta < np.pi:
        return theta
    return theta - 2 * np.pi


def normalize_angle(angle):
    """Normalize angle to [-π, π]."""
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle


class ViNTVelocityReferenceGenerator(Node):
    """
    Generates velocity references from ViNT waypoints for MRS controllers.
    
    Transforms ViNT output from camera frame to world frame velocity commands.
    """
    
    def __init__(self):
        super().__init__('vint_velocity_reference_generator')
        
        # Parameters
        self.declare_parameter('uav_name', 'uav1')
        self.declare_parameter('waypoint_topic', '/vint/waypoint')
        self.declare_parameter('max_horizontal_speed', 1.5)
        self.declare_parameter('max_vertical_speed', 0.5)
        self.declare_parameter('max_yaw_rate', 0.8)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('use_heading', True)
        self.declare_parameter('altitude_hold', 1.5)
        self.declare_parameter('vint_is_normalized', True)
        self.declare_parameter('vint_max_v', 0.5)
        self.declare_parameter('vint_rate', 5.0)
        self.declare_parameter('velocity_damping', 0.3)
        
        self.uav_name = self.get_parameter('uav_name').value
        self.max_h_speed = self.get_parameter('max_horizontal_speed').value
        self.max_v_speed = self.get_parameter('max_vertical_speed').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.dt = self.get_parameter('dt').value
        self.use_heading = self.get_parameter('use_heading').value
        self.altitude_hold = self.get_parameter('altitude_hold').value
        self.vint_normalized = self.get_parameter('vint_is_normalized').value
        self.vint_max_v = self.get_parameter('vint_max_v').value
        self.vint_rate = self.get_parameter('vint_rate').value
        self.velocity_damping = self.get_parameter('velocity_damping').value
        
        # Camera-to-FCU transformation: 90° rotation with reflection
        # Camera frame: x_cam (forward), y_cam (right)
        # FCU frame: x_fcu = -y_cam, y_fcu = -x_cam
        self.rotation_matrix = np.array([
            # [0,  -1],
            # [-1,  0]
                [1, 0],
                [0, 1]
        ])
        
        # State variables
        self.current_odom = None
        self.latest_waypoint = None
        self.last_velocity = np.array([0.0, 0.0])
        self.EPS = 1e-8
        
        # Subscribers
        self.waypoint_sub = self.create_subscription(
            Float32MultiArray,
            self.get_parameter('waypoint_topic').value,
            self.waypoint_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.uav_name}/estimation_manager/odom_main',
            self.odom_callback,
            10
        )
        
        # Publisher
        self.velocity_ref_pub = self.create_publisher(
            VelocityReferenceStamped,
            f'/{self.uav_name}/control_manager/velocity_reference',
            10
        )
        
        # Control loop timer
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("ViNT Velocity Reference Generator Started")
        self.get_logger().info(f"  UAV: {self.uav_name}")
        self.get_logger().info(f"  Max speeds: h={self.max_h_speed} m/s, v={self.max_v_speed} m/s")
        self.get_logger().info(f"  Max yaw rate: {np.rad2deg(self.max_yaw_rate):.1f}°/s")
        self.get_logger().info(f"  Camera rotation: 90° with reflection")
        self.get_logger().info(f"  ViNT normalized: {self.vint_normalized}")
        if self.vint_normalized:
            self.get_logger().info(f"    Scaling: MAX_V={self.vint_max_v}, RATE={self.vint_rate}")
        self.get_logger().info(f"  Altitude hold: {self.altitude_hold} m")
        self.get_logger().info(f"  Velocity damping: {self.velocity_damping}")
        self.get_logger().info("=" * 60)
    
    def waypoint_callback(self, msg):
        """Store latest ViNT waypoint."""
        self.latest_waypoint = np.array(msg.data)
    
    def odom_callback(self, msg):
        """Store current odometry."""
        self.current_odom = msg
    
    def compute_velocity_reference(self, waypoint_camera, current_yaw, current_altitude):
        """Transform ViNT waypoint to velocity reference."""
        
        if len(waypoint_camera) == 0:
            return np.array([0.0, 0.0, 0.0, 0.0])
        
        # Extract and denormalize
        dx_cam = float(waypoint_camera[0])
        dy_cam = float(waypoint_camera[1])
        
        if self.vint_normalized:
            scale_factor = self.vint_max_v / self.vint_rate
            dx_cam *= scale_factor
            dy_cam *= scale_factor
        
        # Camera → FCU transformation
        waypoint_cam = np.array([dx_cam, dy_cam])
        waypoint_fcu = self.rotation_matrix @ waypoint_cam
        dx_fcu = waypoint_fcu[0]
        dy_fcu = waypoint_fcu[1]

        # ========================================================================
        # KEY CHANGE: Use BOTH forward AND lateral velocity
        # Don't turn aggressively - UAVs can strafe!
        # ========================================================================
        
        # Forward and lateral velocities in FCU frame
        v_forward_fcu = dx_fcu / self.dt
        v_lateral_fcu = dy_fcu / self.dt

        # Velocity gain multiplier (tune as needed)
        VELOCITY_GAIN = 4.0  # Start with 2.0, adjust based on testing
        v_forward_fcu *= VELOCITY_GAIN
        v_lateral_fcu *= VELOCITY_GAIN
        
        # Clip to limits
        v_forward_fcu = np.clip(v_forward_fcu, -self.max_h_speed, self.max_h_speed)
        v_lateral_fcu = np.clip(v_lateral_fcu, -self.max_h_speed, self.max_h_speed)
        
        # ========================================================================
        # Heading control using ViNT's heading output
        # ========================================================================
        yaw_rate = 0.0

        if self.use_heading and len(waypoint_camera) >= 4:
            # Extract heading from ViNT output (components 2 and 3)
            sin_heading_cam = float(waypoint_camera[2])
            cos_heading_cam = float(waypoint_camera[3])
            
            # Normalize (for safety)
            norm = np.sqrt(sin_heading_cam**2 + cos_heading_cam**2)
            if norm > 1e-6:
                sin_heading_cam /= norm
                cos_heading_cam /= norm
            
            # Transform heading vector from camera to body frame
            heading_vec_cam = np.array([sin_heading_cam, cos_heading_cam])
            heading_vec_fcu = self.rotation_matrix @ heading_vec_cam
            
            # Convert to angle in body frame (relative to current orientation)
            target_heading_body = np.arctan2(heading_vec_fcu[1], heading_vec_fcu[0])
            
            # Compute heading rate
            HEADING_TIME_CONSTANT = 2.0  # Adjust this: larger = slower turning
            yaw_rate = target_heading_body / HEADING_TIME_CONSTANT
            
            # Clip to limits
            yaw_rate = np.clip(yaw_rate, -self.max_yaw_rate, self.max_yaw_rate)
            
            # Optional deadband to reduce jitter
            if abs(target_heading_body) < np.radians(5):
                yaw_rate *= 0.3

        # ========================================================================
        # Keep velocities in FCU (body) frame
        # MPC will handle world transformation based on frame_id
        # ========================================================================

        # Smoothing in body frame
        velocity_horizontal = np.array([v_forward_fcu, v_lateral_fcu])
        velocity_horizontal = (self.velocity_damping * self.last_velocity + 
                            (1 - self.velocity_damping) * velocity_horizontal)
        self.last_velocity = velocity_horizontal

        vx_fcu = velocity_horizontal[0]
        vy_fcu = velocity_horizontal[1]

        # Ensure total speed doesn't exceed limit
        h_speed = np.sqrt(vx_fcu**2 + vy_fcu**2)
        if h_speed > self.max_h_speed:
            scale = self.max_h_speed / h_speed
            vx_fcu *= scale
            vy_fcu *= scale

        
        # Altitude control
        altitude_error = self.altitude_hold - current_altitude
        vz_fcu = np.clip(altitude_error * 0.5, -self.max_v_speed, self.max_v_speed)

        return np.array([vx_fcu, vy_fcu, vz_fcu, yaw_rate])

    
    def control_loop(self):
        """Main control loop - computes and publishes velocity references."""
        
        if self.latest_waypoint is None:
            self.get_logger().warn("Waiting for ViNT waypoints...", throttle_duration_sec=3.0)
            self.publish_zero_velocity()
            return
        
        if self.current_odom is None:
            self.get_logger().warn("Waiting for odometry...", throttle_duration_sec=3.0)
            self.publish_zero_velocity()
            return
        
        # Extract current state
        current_pos = self.current_odom.pose.pose.position
        current_ori = self.current_odom.pose.pose.orientation
        _, _, current_yaw = euler_from_quaternion(
            current_ori.x, current_ori.y, current_ori.z, current_ori.w
        )
        current_altitude = current_pos.z
        
        # Compute velocity reference
        velocities = self.compute_velocity_reference(
            self.latest_waypoint,
            current_yaw,
            current_altitude
        )
        
        vx, vy, vz, yaw_rate = velocities
        
        # Publish to MRS
        self.publish_velocity_reference(vx, vy, vz, yaw_rate, current_yaw)

        # Log status
        if len(self.latest_waypoint) >= 2:
            heading_info = ""
            if self.use_heading and len(self.latest_waypoint) >= 4:
                sin_h = self.latest_waypoint[2]
                cos_h = self.latest_waypoint[3]
                heading_cam = np.rad2deg(np.arctan2(sin_h, cos_h))
                heading_info = f"h_cam={heading_cam:+.0f}° "
            
            self.get_logger().info(
                f"WP[{self.latest_waypoint[0]:+.3f},{self.latest_waypoint[1]:+.3f}] {heading_info}→ "
                f"Vel[{vx:+.2f},{vy:+.2f},{vz:+.2f}m/s, ω={np.rad2deg(yaw_rate):+.1f}°/s] "
                f"Yaw:{np.rad2deg(current_yaw):+.0f}° Alt:{current_altitude:.2f}m",
                throttle_duration_sec=0.5
            )


    
    def publish_velocity_reference(self, vx, vy, vz, yaw_rate, current_yaw):
        """Publish velocity reference to MRS control_manager."""
        
        vel_ref_msg = VelocityReferenceStamped()
        vel_ref_msg.header.stamp = self.get_clock().now().to_msg()
        vel_ref_msg.header.frame_id = f'{self.uav_name}/fcu'
        
        vel_ref_msg.reference.velocity.x = float(vx)
        vel_ref_msg.reference.velocity.y = float(vy)
        vel_ref_msg.reference.velocity.z = float(vz)
        
        vel_ref_msg.reference.heading = float(current_yaw)
        vel_ref_msg.reference.heading_rate = float(yaw_rate)
        vel_ref_msg.reference.use_heading = False
        vel_ref_msg.reference.use_heading_rate = True
        
        self.velocity_ref_pub.publish(vel_ref_msg)
    
    def publish_zero_velocity(self):
        """Safety fallback: publish zero velocity."""
        self.publish_velocity_reference(0.0, 0.0, 0.0, 0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = ViNTVelocityReferenceGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
