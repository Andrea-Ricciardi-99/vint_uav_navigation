#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3Stamped, PointStamped
from nav_msgs.msg import Path

class PositionToPathNode(Node):
    def __init__(self):
        super().__init__('position_to_path')

        # Try subscribing as PointStamped first
        self.position_sub = self.create_subscription(
            PointStamped,
            '/leica/position',
            self.point_callback,
            10
        )

        # Fallback to Vector3Stamped if needed
        self.position_sub2 = self.create_subscription(
            Vector3Stamped,
            '/leica/position',
            self.vector_callback,
            10
        )

        # Publish path
        self.path_pub = self.create_publisher(Path, '/drone_path', 10)

        # Store path
        self.path = Path()
        self.path.header.frame_id = 'world'

        self.get_logger().info('Position to Path node started (handles PointStamped and Vector3Stamped)')

    def point_callback(self, msg):
        # Create a PoseStamped from PointStamped
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'world'
        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.position.z = msg.point.z
        pose.pose.orientation.w = 1.0

        # Add to path
        self.path.poses.append(pose)
        self.path.header.stamp = msg.header.stamp

        # Publish path
        self.path_pub.publish(self.path)
        self.get_logger().debug(f'Added point to path (PointStamped): [{msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f}]')

    def vector_callback(self, msg):
        # Create a PoseStamped from Vector3Stamped
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'world'
        pose.pose.position.x = msg.vector.x
        pose.pose.position.y = msg.vector.y
        pose.pose.position.z = msg.vector.z
        pose.pose.orientation.w = 1.0

        # Add to path
        self.path.poses.append(pose)
        self.path.header.stamp = msg.header.stamp

        # Publish path
        self.path_pub.publish(self.path)
        self.get_logger().debug(f'Added point to path (Vector3Stamped): [{msg.vector.x:.2f}, {msg.vector.y:.2f}, {msg.vector.z:.2f}]')

def main(args=None):
    rclpy.init(args=args)
    node = PositionToPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
