#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from mrs_msgs.srv import Vec4
import numpy as np
from tf2_ros import TransformException, Buffer, TransformListener
import tf2_geometry_msgs
from rclpy.duration import Duration


def euler_from_quaternion(x, y, z, w):
    """
    Convert quaternion to euler angles (roll, pitch, yaw)
    """
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


def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert euler angles to quaternion [x, y, z, w]
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return [x, y, z, w]


class ViNTtoMRSWaypointNode(Node):
    """
    Converts ViNT local waypoint deltas to MRS goto commands.
    
    Pipeline:
    1. Receive ViNT waypoint [dx, dy, dtheta, distance] in camera frame
    2. Apply rotation to convert camera frame to fcu frame
    3. Transform to world frame using TF
    4. Add to current position from odometry
    5. Send as goto command to MRS via service
    """
    
    def __init__(self):
        super().__init__('vint_to_mrs_waypoint')
        
        # Declare parameters
        self.declare_parameter('waypoint_topic', '/vint/waypoint')
        self.declare_parameter('uav_name', 'uav1')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('fcu_frame', '')
        self.declare_parameter('camera_rotation_deg', 0.0)
        self.declare_parameter('use_heading', True)
        self.declare_parameter('waypoint_height_offset', 0.0)
        self.declare_parameter('scale_factor', 1.0)
        self.declare_parameter('update_rate', 2.0)  # Hz - how often to send goto commands
        
        # Get parameters
        self.uav_name = self.get_parameter('uav_name').value
        self.world_frame = self.get_parameter('world_frame').value
        fcu_frame_param = self.get_parameter('fcu_frame').value
        self.fcu_frame = fcu_frame_param if fcu_frame_param else f'{self.uav_name}/fcu'
        self.camera_rotation_deg = self.get_parameter('camera_rotation_deg').value
        self.use_heading = self.get_parameter('use_heading').value
        self.height_offset = self.get_parameter('waypoint_height_offset').value
        self.scale_factor = self.get_parameter('scale_factor').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Set up rotation matrix from camera to FCU frame
        # Camera frame: x=forward (same as FCU), y=right (opposite of FCU left)
        # FCU frame: x=forward, y=left
        # Transformation: dx_fcu = dx_camera, dy_fcu = -dy_camera
        
        if abs(self.camera_rotation_deg) < 0.1:
            # No rotation, just y-axis flip (camera y points right, FCU y points left)
            self.rotation_matrix = np.array([
                [ 1,  0],
                [ 0, -1]
            ])
        elif abs(self.camera_rotation_deg - 90.0) < 0.1:
            # 90° CW rotation case (if needed later)
            self.rotation_matrix = np.array([
                [ 0, -1],
                [-1,  0]
            ])
        else:
            # General case: rotate by -camera_rotation to go from camera to FCU
            theta_rad = -np.deg2rad(self.camera_rotation_deg)
            cos_theta = np.cos(theta_rad)
            sin_theta = np.sin(theta_rad)
            self.rotation_matrix = np.array([
                [cos_theta, -sin_theta],
                [sin_theta,  cos_theta]
            ])

        
        # State
        self.current_odom = None
        self.latest_waypoint = None
        self.pending_goto = False
        
        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.waypoint_sub = self.create_subscription(
            Float32MultiArray,
            self.get_parameter('waypoint_topic').value,
            self.waypoint_callback,
            10
        )
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.uav_name}/estimation_manager/odom_main',
            self.odom_callback,
            10
        )
        
        # MRS goto service client
        self.goto_client = self.create_client(
            Vec4,
            f'/{self.uav_name}/control_manager/goto'
        )
        
        # Timer for sending goto commands
        self.timer = self.create_timer(1.0 / self.update_rate, self.send_goto_command)
        
        self.get_logger().info(f"ViNT to MRS Waypoint Converter initialized")
        self.get_logger().info(f"  UAV: {self.uav_name}")
        self.get_logger().info(f"  World frame: {self.world_frame}")
        self.get_logger().info(f"  FCU frame: {self.fcu_frame}")
        self.get_logger().info(f"  Camera rotation: {self.camera_rotation_deg}° CW from fcu")
        self.get_logger().info(f"  Rotation matrix:\n{self.rotation_matrix}")
        self.get_logger().info(f"  Use heading: {self.use_heading}")
        self.get_logger().info(f"  Scale factor: {self.scale_factor}")
        self.get_logger().info(f"  Update rate: {self.update_rate} Hz")
        self.get_logger().info(f"  Waiting for MRS goto service...")
    
    def waypoint_callback(self, msg):
        """Store latest ViNT waypoint"""
        self.latest_waypoint = np.array(msg.data)
    
    def odom_callback(self, msg):
        """Store current odometry"""
        self.current_odom = msg
    
    def send_goto_command(self):
        """Main loop: transform waypoint and send goto command"""
        
        # Check if we have necessary data
        if self.latest_waypoint is None:
            self.get_logger().warn(
                "No waypoint from ViNT yet...",
                throttle_duration_sec=2.0
            )
            return
        
        if self.current_odom is None:
            self.get_logger().warn(
                "No odometry data yet...",
                throttle_duration_sec=2.0
            )
            return
        
        if len(self.latest_waypoint) < 2:
            self.get_logger().error("Invalid waypoint format")
            return
        
        # Don't send new command if previous one is still pending
        if self.pending_goto:
            self.get_logger().debug("Previous goto still pending, skipping...")
            return
        
        # Extract ViNT waypoint components (in camera frame)
        # ViNT: x=forward in image, y=right in image
        dx_camera = float(self.latest_waypoint[0]) * self.scale_factor
        dy_camera = float(self.latest_waypoint[1]) * self.scale_factor
        dtheta_vint = float(self.latest_waypoint[2]) if len(self.latest_waypoint) > 2 else 0.0
        
        # Apply rotation from camera frame to fcu frame
        waypoint_camera = np.array([dx_camera, dy_camera])
        waypoint_fcu_2d = self.rotation_matrix @ waypoint_camera
        
        dx_fcu = waypoint_fcu_2d[0]
        dy_fcu = waypoint_fcu_2d[1]
        dz_fcu = 0.0  # ViNT doesn't predict altitude
        
        self.get_logger().debug(
            f"Camera frame: [{dx_camera:.3f}, {dy_camera:.3f}] → "
            f"FCU frame: [{dx_fcu:.3f}, {dy_fcu:.3f}]"
        )
        
        # Create Vector3Stamped in fcu frame
        waypoint_fcu = Vector3Stamped()
        waypoint_fcu.header.stamp = self.get_clock().now().to_msg()
        waypoint_fcu.header.frame_id = self.fcu_frame
        waypoint_fcu.vector.x = dx_fcu
        waypoint_fcu.vector.y = dy_fcu
        waypoint_fcu.vector.z = dz_fcu
        
        try:
            # Transform from fcu to world frame using TF
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.fcu_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            
            # Transform the waypoint vector
            waypoint_world = tf2_geometry_msgs.do_transform_vector3(
                waypoint_fcu,
                transform
            )
            
        except TransformException as ex:
            self.get_logger().error(
                f'Could not transform {self.fcu_frame} to {self.world_frame}: {ex}',
                throttle_duration_sec=1.0
            )
            return
        
        # Add transformed waypoint to current position
        current_pos = self.current_odom.pose.pose.position
        current_ori = self.current_odom.pose.pose.orientation
        
        target_x = current_pos.x + waypoint_world.vector.x
        target_y = current_pos.y + waypoint_world.vector.y
        target_z = current_pos.z + waypoint_world.vector.z + self.height_offset
        
        # Handle heading
        if self.use_heading and abs(dtheta_vint) > 0.01:
            # Convert current quaternion to yaw
            _, _, current_yaw = euler_from_quaternion(
                current_ori.x, current_ori.y, current_ori.z, current_ori.w
            )
            
            # ViNT's heading is also in camera frame
            # Need to account for camera rotation
            target_yaw = current_yaw + dtheta_vint - np.deg2rad(self.camera_rotation_deg)
        else:
            # Keep current heading
            _, _, target_yaw = euler_from_quaternion(
                current_ori.x, current_ori.y, current_ori.z, current_ori.w
            )
        
        # Send goto command to MRS
        self.send_goto(target_x, target_y, target_z, target_yaw)
        
        self.get_logger().info(
            f"ViNT[{dx_camera:.2f}, {dy_camera:.2f}] → "
            f"FCU[{dx_fcu:.2f}, {dy_fcu:.2f}] → "
            f"Goto[{target_x:.2f}, {target_y:.2f}, {target_z:.2f}, {np.rad2deg(target_yaw):.1f}°]",
            throttle_duration_sec=1.0
        )
    
    def send_goto(self, x, y, z, yaw):
        """Send goto command to MRS"""
        if not self.goto_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().error(
                "MRS goto service not available",
                throttle_duration_sec=2.0
            )
            return
        
        request = Vec4.Request()
        request.goal = [float(x), float(y), float(z), float(yaw)]
        
        self.pending_goto = True
        future = self.goto_client.call_async(request)
        future.add_done_callback(self.goto_response_callback)
    
    def goto_response_callback(self, future):
        """Handle goto service response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().debug("Goto command accepted by MRS")
            else:
                self.get_logger().warn(f"Goto command rejected: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Goto service call failed: {str(e)}")
        finally:
            self.pending_goto = False


def main(args=None):
    rclpy.init(args=args)
    node = ViNTtoMRSWaypointNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
