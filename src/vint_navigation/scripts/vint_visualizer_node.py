#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Int32, Bool
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
import numpy as np
import cv2
from cv_bridge import CvBridge



class ViNTVisualizerNode(Node):
    """
    Visualization node for ViNT navigation.
    Subscribes to navigation outputs and publishes RViz visualizations.
    """
    
    def __init__(self):
        super().__init__('vint_visualizer')
        
        # Declare parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('waypoint_topic', '/vint/waypoint')
        self.declare_parameter('closest_node_topic', '/vint/closest_node')
        self.declare_parameter('goal_node', -1)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('num_waypoints_to_show', 3)
        self.declare_parameter('predicted_path_topic', '/vint/viz/predicted_path')
        self.declare_parameter('markers_topic', '/vint/viz/markers')
        self.declare_parameter('annotated_image_topic', '/vint/viz/annotated_image')
        
        # Get parameters
        self.frame_id = self.get_parameter('frame_id').value
        self.num_waypoints_show = self.get_parameter('num_waypoints_to_show').value
        self.goal_node = self.get_parameter('goal_node').value
        
        # State
        self.latest_image = None
        self.latest_waypoint = None
        self.closest_node = 0
        self.cv_bridge = CvBridge()
        
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
        
        # Publishers
        self.path_pub = self.create_publisher(
            Path,
            self.get_parameter('predicted_path_topic').value,
            10
        )
        
        self.markers_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter('markers_topic').value,
            10
        )
        
        self.annotated_image_pub = self.create_publisher(
            Image,
            self.get_parameter('annotated_image_topic').value,
            10
        )
        
        # Timer for publishing (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_visualizations)
        
        self.get_logger().info("ViNT Visualizer Node initialized")
    
    def image_callback(self, msg):
        """Store latest image"""
        self.latest_image = msg
    
    def waypoint_callback(self, msg):
        """Store latest waypoint"""
        self.latest_waypoint = np.array(msg.data)
    
    def closest_node_callback(self, msg):
        """Update closest node"""
        self.closest_node = msg.data
    
    def publish_visualizations(self):
        """Main visualization publishing loop"""
        if self.latest_waypoint is None:
            return
        
        # Publish path
        self.publish_path()
        
        # Publish markers
        self.publish_markers()
        
        # Publish annotated image
        if self.latest_image is not None:
            self.publish_annotated_image()
    
    def publish_path(self):
        """Publish predicted waypoint as a path"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.frame_id
        
        # Waypoint format: [x, y, theta, distance]
        if len(self.latest_waypoint) >= 2:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(self.latest_waypoint[0])
            pose.pose.position.y = float(self.latest_waypoint[1])
            pose.pose.position.z = 0.0
            
            # Convert theta to quaternion if available
            if len(self.latest_waypoint) >= 3:
                theta = float(self.latest_waypoint[2])
                pose.pose.orientation.z = np.sin(theta / 2.0)
                pose.pose.orientation.w = np.cos(theta / 2.0)
            else:
                pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def publish_markers(self):
        """Publish markers for current and goal nodes"""
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        
        # Waypoint arrow marker
        arrow_marker = Marker()
        arrow_marker.header.stamp = now
        arrow_marker.header.frame_id = self.frame_id
        arrow_marker.ns = "vint_waypoint"
        arrow_marker.id = 0
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        
        # Start point (robot position)
        start_point = Point()
        start_point.x = 0.0
        start_point.y = 0.0
        start_point.z = 0.0
        arrow_marker.points.append(start_point)

        # End point (waypoint)
        if len(self.latest_waypoint) >= 2:
            end_point = Point()
            end_point.x = float(self.latest_waypoint[0])
            end_point.y = float(self.latest_waypoint[1])
            end_point.z = 0.0
            arrow_marker.points.append(end_point)
        
        arrow_marker.scale.x = 0.1  # Shaft diameter
        arrow_marker.scale.y = 0.2  # Head diameter
        arrow_marker.scale.z = 0.3  # Head length
        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 1.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 0.8
        marker_array.markers.append(arrow_marker)
        
        # Progress text marker
        text_marker = Marker()
        text_marker.header.stamp = now
        text_marker.header.frame_id = self.frame_id
        text_marker.ns = "vint_progress"
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = 0.0
        text_marker.pose.position.y = 0.0
        text_marker.pose.position.z = 1.0
        text_marker.scale.z = 0.3
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        
        if self.goal_node > 0:
            text_marker.text = f"Node {self.closest_node}/{self.goal_node}"
        else:
            text_marker.text = f"Node {self.closest_node}"
        
        marker_array.markers.append(text_marker)
        
        # Waypoint sphere
        sphere_marker = Marker()
        sphere_marker.header.stamp = now
        sphere_marker.header.frame_id = self.frame_id
        sphere_marker.ns = "vint_target"
        sphere_marker.id = 2
        sphere_marker.type = Marker.SPHERE
        sphere_marker.action = Marker.ADD
        
        if len(self.latest_waypoint) >= 2:
            sphere_marker.pose.position.x = float(self.latest_waypoint[0])
            sphere_marker.pose.position.y = float(self.latest_waypoint[1])
            sphere_marker.pose.position.z = 0.0
        
        sphere_marker.scale.x = 0.2
        sphere_marker.scale.y = 0.2
        sphere_marker.scale.z = 0.2
        sphere_marker.color.r = 0.0
        sphere_marker.color.g = 1.0
        sphere_marker.color.b = 0.0
        sphere_marker.color.a = 0.8
        marker_array.markers.append(sphere_marker)
        
        self.markers_pub.publish(marker_array)
    
    def publish_annotated_image(self):
        """Publish camera image with waypoint overlay"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            h, w = cv_image.shape[:2]
            
            # Draw waypoint
            if len(self.latest_waypoint) >= 2:
                # Simple projection: assume x is forward, y is lateral
                # Scale factors (adjust these based on your camera FOV)
                scale_forward = 150  # pixels per meter forward
                scale_lateral = 150  # pixels per meter lateral
                
                x_forward = self.latest_waypoint[0]
                y_lateral = self.latest_waypoint[1]
                
                # Project to image coordinates
                # Forward distance -> vertical position (bottom to top)
                # Lateral offset -> horizontal position (center +/- lateral)
                img_x = int(w / 2 + y_lateral * scale_lateral)
                img_y = int(h - x_forward * scale_forward)
                
                # Clamp to image bounds
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
            
            # Draw info text
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
            
            # Draw waypoint values
            if len(self.latest_waypoint) >= 2:
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
