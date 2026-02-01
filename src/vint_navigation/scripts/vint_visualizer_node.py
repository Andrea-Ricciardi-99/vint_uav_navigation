#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Int32
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point
import numpy as np
import cv2
from cv_bridge import CvBridge
from collections import deque


class ViNTVisualizerNode(Node):
    """
    Visualization node for ViNT navigation.
    Shows the drone's traveled path as a trail.
    """
    
    def __init__(self):
        super().__init__('vint_visualizer')
        
        # Declare parameters
        self.declare_parameter('uav_name', 'uav1')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('waypoint_topic', 'vint/waypoint')
        self.declare_parameter('closest_node_topic', 'vint/closest_node')
        self.declare_parameter('goal_node', -1)
        self.declare_parameter('frame_id', 'uav1/fixed_origin')
        self.declare_parameter('trail_length', 1000)  # Number of poses to keep
        self.declare_parameter('trail_topic', 'vint/viz/trail')
        self.declare_parameter('trail_markers_topic', 'vint/viz/trail_markers')
        self.declare_parameter('annotated_image_topic', 'vint/viz/annotated_image')
        
        # Get parameters
        self.uav_name = self.get_parameter('uav_name').value
        self.frame_id = self.get_parameter('frame_id').value
        self.goal_node = self.get_parameter('goal_node').value
        self.trail_length = self.get_parameter('trail_length').value
        
        # State
        self.latest_image = None
        self.latest_waypoint = None
        self.current_odom = None
        self.closest_node = 0
        self.cv_bridge = CvBridge()
        
        # Trail history (deque for efficient append/pop)
        self.position_history = deque(maxlen=self.trail_length)
        self.last_recorded_pos = None
        self.min_distance_threshold = 0.05  # Record every 5cm
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.image_callback,
            10
        )
        
        self.waypoint_sub = self.create_subscription(
            Float32MultiArray,
            self.get_parameter('waypoint_topic').value,
            self.waypoint_callback,
            10
        )
        
        self.closest_node_sub = self.create_subscription(
            Int32,
            self.get_parameter('closest_node_topic').value,
            self.closest_node_callback,
            10
        )
        
        # Odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.uav_name}/estimation_manager/odom_main',
            self.odom_callback,
            10
        )
        
        # Publishers
        self.trail_pub = self.create_publisher(
            Path,
            self.get_parameter('trail_topic').value,
            10
        )
        
        self.trail_markers_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter('trail_markers_topic').value,
            10
        )
        
        self.annotated_image_pub = self.create_publisher(
            Image,
            self.get_parameter('annotated_image_topic').value,
            10
        )
        
        # Timer for publishing (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_visualizations)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("ViNT Trail Visualizer Node initialized")
        self.get_logger().info(f"  UAV: {self.uav_name}")
        self.get_logger().info(f"  Visualization frame: {self.frame_id}")
        self.get_logger().info(f"  Trail length: {self.trail_length} poses")
        self.get_logger().info("=" * 60)
    
    def image_callback(self, msg):
        """Store latest image"""
        self.latest_image = msg
    
    def waypoint_callback(self, msg):
        """Store latest waypoint"""
        self.latest_waypoint = np.array(msg.data)
    
    def closest_node_callback(self, msg):
        """Update closest node"""
        self.closest_node = msg.data
    
    def odom_callback(self, msg):
        """Store odometry and update position history"""
        self.current_odom = msg
        
        # Record position if moved enough
        current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        if self.last_recorded_pos is None:
            # First position
            self.position_history.append(msg.pose.pose)
            self.last_recorded_pos = current_pos
        else:
            # Check distance from last recorded position
            distance = np.linalg.norm(current_pos - self.last_recorded_pos)
            if distance >= self.min_distance_threshold:
                self.position_history.append(msg.pose.pose)
                self.last_recorded_pos = current_pos
    
    def publish_visualizations(self):
        """Main visualization publishing loop"""
        if self.current_odom is None:
            return
        
        # Publish trail
        self.publish_trail()
        
        # Publish trail markers
        self.publish_trail_markers()
        
        # Publish annotated image
        if self.latest_image is not None:
            self.publish_annotated_image()
    
    def publish_trail(self):
        """Publish the drone's traveled path"""
        if len(self.position_history) == 0:
            return
        
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.frame_id
        
        # Add all historical positions
        for pose in self.position_history:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose = pose
            path_msg.poses.append(pose_stamped)
        
        self.trail_pub.publish(path_msg)
    
    def publish_trail_markers(self):
        """Publish markers along the trail"""
        if len(self.position_history) < 2:
            return
        
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        
        # Current drone position marker (small blue sphere)
        drone_marker = Marker()
        drone_marker.header.stamp = now
        drone_marker.header.frame_id = self.frame_id
        drone_marker.ns = "current_position"
        drone_marker.id = 0
        drone_marker.type = Marker.SPHERE
        drone_marker.action = Marker.ADD
        
        drone_marker.pose = self.current_odom.pose.pose
        
        drone_marker.scale.x = 0.15  # Much smaller (was 0.6)
        drone_marker.scale.y = 0.15
        drone_marker.scale.z = 0.15
        
        drone_marker.color.r = 0.0
        drone_marker.color.g = 0.5
        drone_marker.color.b = 1.0
        drone_marker.color.a = 1.0
        
        marker_array.markers.append(drone_marker)
        
        # Start position marker (small green sphere)
        start_marker = Marker()
        start_marker.header.stamp = now
        start_marker.header.frame_id = self.frame_id
        start_marker.ns = "start_position"
        start_marker.id = 1
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        
        start_marker.pose = self.position_history[0]
        
        start_marker.scale.x = 0.12  # Smaller (was 0.4)
        start_marker.scale.y = 0.12
        start_marker.scale.z = 0.12
        
        start_marker.color.r = 0.0
        start_marker.color.g = 1.0
        start_marker.color.b = 0.0
        start_marker.color.a = 1.0
        
        marker_array.markers.append(start_marker)
        
        # Breadcrumb markers every N positions (tiny spheres)
        breadcrumb_interval = max(1, len(self.position_history) // 20)
        
        marker_id = 2
        for i in range(0, len(self.position_history), breadcrumb_interval):
            if i == 0:
                continue
            
            breadcrumb = Marker()
            breadcrumb.header.stamp = now
            breadcrumb.header.frame_id = self.frame_id
            breadcrumb.ns = "breadcrumbs"
            breadcrumb.id = marker_id
            breadcrumb.type = Marker.SPHERE
            breadcrumb.action = Marker.ADD
            
            breadcrumb.pose = self.position_history[i]
            
            breadcrumb.scale.x = 0.08  # Very small (was 0.2)
            breadcrumb.scale.y = 0.08
            breadcrumb.scale.z = 0.08
            
            # Gradient color: old=red, new=yellow
            ratio = i / len(self.position_history)
            breadcrumb.color.r = 1.0
            breadcrumb.color.g = ratio
            breadcrumb.color.b = 0.0
            breadcrumb.color.a = 0.8
            
            marker_array.markers.append(breadcrumb)
            marker_id += 1
        
        # Info text (BLACK and smaller)
        text_marker = Marker()
        text_marker.header.stamp = now
        text_marker.header.frame_id = self.frame_id
        text_marker.ns = "info_text"
        text_marker.id = 999
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        text_marker.pose.position.x = self.current_odom.pose.pose.position.x
        text_marker.pose.position.y = self.current_odom.pose.pose.position.y
        text_marker.pose.position.z = self.current_odom.pose.pose.position.z + 1.0  # Closer to drone
        text_marker.pose.orientation.w = 1.0
        
        text_marker.scale.z = 0.3  # Smaller text (was 0.5)
        
        text_marker.color.r = 0.0  # BLACK text
        text_marker.color.g = 0.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0
        
        # Calculate traveled distance
        total_distance = 0.0
        for i in range(1, len(self.position_history)):
            p1 = self.position_history[i-1].position
            p2 = self.position_history[i].position
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            dz = p2.z - p1.z
            total_distance += np.sqrt(dx*dx + dy*dy + dz*dz)
        
        if self.goal_node > 0:
            text_marker.text = f"Node {self.closest_node}/{self.goal_node}\nDist: {total_distance:.1f}m"
        else:
            text_marker.text = f"Node {self.closest_node}\nDist: {total_distance:.1f}m"
        
        marker_array.markers.append(text_marker)
        
        self.trail_markers_pub.publish(marker_array)

    
    def publish_annotated_image(self):
        """Publish camera image with waypoint overlay and ViNT outputs"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            h, w = cv_image.shape[:2]
            
            # Draw waypoint if available
            if self.latest_waypoint is not None and len(self.latest_waypoint) >= 2:
                scale_forward = 150
                scale_lateral = 150
                
                x_forward = self.latest_waypoint[0]
                y_lateral = self.latest_waypoint[1]
                
                img_x = int(w / 2 + y_lateral * scale_lateral)
                img_y = int(h - x_forward * scale_forward)
                
                img_x = max(0, min(w - 1, img_x))
                img_y = max(0, min(h - 1, img_y))
                
                # Draw target circle
                cv2.circle(cv_image, (img_x, img_y), 10, (0, 255, 255), -1)
                cv2.circle(cv_image, (img_x, img_y), 12, (0, 0, 0), 2)
                
                # Draw arrow from bottom center to target
                cv2.arrowedLine(
                    cv_image,
                    (w // 2, h - 20),
                    (img_x, img_y),
                    (0, 255, 255),
                    3,
                    tipLength=0.3
                )
            
            # Info text - Node progress
            info_text = f"Node: {self.closest_node}"
            if self.goal_node > 0:
                info_text += f"/{self.goal_node}"
            
            cv2.putText(
                cv_image,
                info_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2
            )
            
            # Waypoint position (x, y)
            if self.latest_waypoint is not None and len(self.latest_waypoint) >= 2:
                wp_text = f"x:{self.latest_waypoint[0]:.2f} y:{self.latest_waypoint[1]:.2f}"
                cv2.putText(
                    cv_image,
                    wp_text,
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2
                )
            
            # Heading outputs (cos, sin)
            if self.latest_waypoint is not None and len(self.latest_waypoint) >= 4:
                cos_h = self.latest_waypoint[2]
                sin_h = self.latest_waypoint[3]
                heading_angle = np.rad2deg(np.arctan2(sin_h, cos_h))
                
                heading_text = f"cos:{cos_h:.2f} sin:{sin_h:.2f} ({heading_angle:.0f}deg)"
                cv2.putText(
                    cv_image,
                    heading_text,
                    (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 128, 0),  # Orange
                    2
                )
            
            # Trail counter
            trail_text = f"Trail: {len(self.position_history)} points"
            cv2.putText(
                cv_image,
                trail_text,
                (10, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 0),  # Yellow
                2
            )
            
            # Convert back to ROS Image
            annotated_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            annotated_msg.header.stamp = self.get_clock().now().to_msg()
            annotated_msg.header.frame_id = 'camera'
            
            self.annotated_image_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error creating annotated image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = ViNTVisualizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
