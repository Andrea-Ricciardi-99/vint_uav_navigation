#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class ViNTPathVisualizer(Node):
    """
    Visualizes ViNT waypoint predictions by accumulating them with proper rotation.
    Each waypoint's [dx, dy] is in the LOCAL frame, so we need to rotate by accumulated yaw.
    """

    def __init__(self):
        super().__init__('vint_path_visualizer')

        # Declare parameters
        self.declare_parameter('waypoint_topic', '/vint/waypoint')
        self.declare_parameter('path_topic', '/vint_predicted_path')
        self.declare_parameter('max_waypoints', 50)
        self.declare_parameter('clear_on_restart', True)

        # Get parameters
        waypoint_topic = self.get_parameter('waypoint_topic').value
        path_topic = self.get_parameter('path_topic').value
        self.max_waypoints = self.get_parameter('max_waypoints').value
        self.clear_on_restart = self.get_parameter('clear_on_restart').value

        # State - accumulated pose
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0  # Accumulated yaw (robot's heading)

        # Path storage
        self.path = Path()
        self.path.header.frame_id = 'world'

        # Add initial pose at origin
        self.add_pose_to_path(self.x, self.y, self.z, self.yaw)

        # Counter
        self.waypoint_count = 0

        # Subscribers
        self.waypoint_sub = self.create_subscription(
            Float32MultiArray,
            waypoint_topic,
            self.waypoint_callback,
            10
        )

        # Publisher
        self.path_pub = self.create_publisher(Path, path_topic, 10)

        # Timer to republish path for RViz
        self.timer = self.create_timer(0.5, self.republish_path)

        self.get_logger().info('ViNT Path Visualizer started')
        self.get_logger().info(f'  Waypoint topic: {waypoint_topic}')
        self.get_logger().info(f'  Path topic: {path_topic}')
        self.get_logger().info(f'  Max waypoints: {self.max_waypoints}')
        self.get_logger().info(f'  Starting from origin (0, 0) with yaw=0°')

    def add_pose_to_path(self, x, y, z, yaw):
        """Helper to create and add a pose to the path"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'world'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        # Convert yaw to quaternion
        qz = np.sin(yaw / 2.0)
        qw = np.cos(yaw / 2.0)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.path.poses.append(pose)
        self.path.header.stamp = pose.header.stamp

        # Limit path length
        if len(self.path.poses) > self.max_waypoints:
            self.path.poses.pop(0)

    def waypoint_callback(self, msg):
        """
        Accumulate ViNT waypoints with proper rotation handling.

        Key insight: [dx, dy] is in the robot's LOCAL frame (forward/lateral),
        so we must rotate by the current accumulated yaw before adding to global position.
        """
        if len(msg.data) < 4:
            self.get_logger().warn('Invalid waypoint (expected 4 values)')
            return

        # Extract waypoint in LOCAL frame
        dx_local = msg.data[0]  # Forward in robot's current frame
        dy_local = msg.data[1]  # Lateral in robot's current frame
        hx = msg.data[2]        # Heading x-component
        hy = msg.data[3]        # Heading y-component

        # STEP 1: Rotate [dx_local, dy_local] by current yaw to get global displacement
        cos_yaw = np.cos(self.yaw)
        sin_yaw = np.sin(self.yaw)

        dx_global = dx_local * cos_yaw - dy_local * sin_yaw
        dy_global = dx_local * sin_yaw + dy_local * cos_yaw

        # STEP 2: Add to accumulated position
        self.x += dx_global
        self.y += dy_global
        # z stays 0 for 2D navigation

        # STEP 3: Update heading
        # The heading vector [hx, hy] is also in LOCAL frame, rotate it too
        hx_global = hx * cos_yaw - hy * sin_yaw
        hy_global = hx * sin_yaw + hy * cos_yaw

        # Calculate new absolute yaw
        self.yaw = np.arctan2(hy_global, hx_global)

        # STEP 4: Add pose to path
        self.add_pose_to_path(self.x, self.y, self.z, self.yaw)

        # STEP 5: Publish immediately
        self.path_pub.publish(self.path)

        self.waypoint_count += 1
        self.get_logger().info(
            f'Waypoint {self.waypoint_count}: '
            f'Local[dx={dx_local:.3f}, dy={dy_local:.3f}] → '
            f'Global[{dx_global:.3f}, {dy_global:.3f}] → '
            f'Position({self.x:.2f}, {self.y:.2f}) Yaw={np.rad2deg(self.yaw):.1f}°'
        )

    def republish_path(self):
        """Periodically republish path for RViz"""
        if len(self.path.poses) > 0:
            self.path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self.path)
            self.get_logger().debug(
                f'Path: {len(self.path.poses)} poses, '
                f'end: ({self.x:.2f}, {self.y:.2f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ViNTPathVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
