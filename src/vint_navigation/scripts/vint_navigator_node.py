#!/usr/bin/env python3

import os
import numpy as np
import torch
import yaml
from PIL import Image as PILImage
from pathlib import Path

# ROS2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray, Int32
from ament_index_python.packages import get_package_share_directory

# Local imports
from utils import load_model, msg_to_pil, to_numpy, transform_images


class ViNTNavigatorNode(Node):
    """ViNT Navigator Node - Direct adaptation from navigate.py"""
    
    def __init__(self):
        super().__init__('vint_navigator')
        
        # Declare all parameters
        self.declare_parameter('model', 'vint')
        self.declare_parameter('waypoint_index', 2)
        self.declare_parameter('topomap_dir', '')
        self.declare_parameter('goal_node', -1)
        self.declare_parameter('close_threshold', 3)
        self.declare_parameter('radius', 4)
        self.declare_parameter('num_samples', 8)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('waypoint_topic', '/vint/waypoint')
        self.declare_parameter('sampled_actions_topic', '/vint/sampled_actions')
        self.declare_parameter('goal_reached_topic', '/vint/goal_reached')
        self.declare_parameter('closest_node_topic', '/vint/closest_node')
        self.declare_parameter('debug', True)
        
        # Get parameters
        self.model_name = self.get_parameter('model').value
        self.waypoint_idx = self.get_parameter('waypoint_index').value
        self.topomap_dir_param = self.get_parameter('topomap_dir').value
        self.goal_node_param = self.get_parameter('goal_node').value
        self.close_threshold = self.get_parameter('close_threshold').value
        self.radius = self.get_parameter('radius').value
        self.num_samples = self.get_parameter('num_samples').value
        self.debug = self.get_parameter('debug').value
        
        # Get package paths
        pkg_share = get_package_share_directory('vint_navigation')
        
        # Load configs
        self.robot_config = self._load_yaml(os.path.join(pkg_share, 'config', 'robot.yaml'))
        self.models_config = self._load_yaml(os.path.join(pkg_share, 'config', 'models.yaml'))
        
        # Extract robot config
        self.MAX_V = self.robot_config['max_v']
        self.MAX_W = self.robot_config['max_w']
        self.RATE = self.robot_config['frame_rate']
        
        # Load model config
        model_config_path = self.models_config[self.model_name]['config_path']
        if not os.path.isabs(model_config_path):
            model_config_path = os.path.join(pkg_share, model_config_path)
        self.model_params = self._load_yaml(model_config_path)
        
        # Setup device
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Using device: {self.device}")
        
        # Load model weights
        ckpt_path = self.models_config[self.model_name]['ckpt_path']
        if not os.path.isabs(ckpt_path):
            ckpt_path = os.path.join(pkg_share, ckpt_path)
        
        if os.path.exists(ckpt_path):
            self.get_logger().info(f"Loading model from {ckpt_path}")
        else:
            raise FileNotFoundError(f"Model weights not found at {ckpt_path}")
        
        self.model = load_model(ckpt_path, self.model_params, self.device)
        self.model = self.model.to(self.device)
        self.model.eval()
        
        # Load topomap
        if not self.topomap_dir_param:
            raise ValueError("topomap_dir parameter must be set!")
        
        topomap_filenames = sorted(
            os.listdir(self.topomap_dir_param),
            key=lambda x: int(x.split(".")[0])
        )
        self.topomap = []
        for filename in topomap_filenames:
            image_path = os.path.join(self.topomap_dir_param, filename)
            self.topomap.append(PILImage.open(image_path))
        
        self.get_logger().info(f"Loaded topomap with {len(self.topomap)} nodes")
        
        # Navigation state
        self.context_queue = []
        self.context_size = self.model_params['context_size']
        self.closest_node = 0
        
        if self.goal_node_param == -1:
            self.goal_node = len(self.topomap) - 1
        else:
            self.goal_node = self.goal_node_param
        
        assert -1 <= self.goal_node_param < len(self.topomap), "Invalid goal index"
        
        self.reached_goal = False
        
        # Subscribers
        image_topic = self.get_parameter('image_topic').value
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.callback_obs,
            1
        )
        
        # Publishers
        waypoint_topic = self.get_parameter('waypoint_topic').value
        sampled_actions_topic = self.get_parameter('sampled_actions_topic').value
        goal_reached_topic = self.get_parameter('goal_reached_topic').value
        closest_node_topic = self.get_parameter('closest_node_topic').value
        
        self.waypoint_pub = self.create_publisher(Float32MultiArray, waypoint_topic, 1)
        self.sampled_actions_pub = self.create_publisher(Float32MultiArray, sampled_actions_topic, 1)
        self.goal_pub = self.create_publisher(Bool, goal_reached_topic, 1)
        self.closest_node_pub = self.create_publisher(Int32, closest_node_topic, 10)
        
        # Timer for navigation loop
        timer_period = 1.0 / self.RATE
        self.timer = self.create_timer(timer_period, self.navigation_loop)
        
        self.get_logger().info("ViNT Navigator initialized. Waiting for images...")
    
    def _load_yaml(self, path):
        """Load YAML file"""
        with open(path, 'r') as f:
            return yaml.safe_load(f)
    
    def callback_obs(self, msg):
        """Image callback - maintains context queue (EXACT same as ROS1)"""
        obs_img = msg_to_pil(msg)  # No bridge needed!
        if len(self.context_queue) < self.context_size + 1:
            self.context_queue.append(obs_img)
        else:
            self.context_queue.pop(0)
            self.context_queue.append(obs_img)
    
    def navigation_loop(self):
        """Main navigation loop - runs at RATE Hz"""
        
        # Default zero waypoint
        chosen_waypoint = np.zeros(4)
        
        # Check if we have enough context
        if len(self.context_queue) > self.context_size:
            
            # ViNT/GNM model (non-diffusion)
            start = max(self.closest_node - self.radius, 0)
            end = min(self.closest_node + self.radius + 1, self.goal_node)
            
            batch_obs_imgs = []
            batch_goal_data = []
            
            for sg_img in self.topomap[start:end + 1]:
                transf_obs_img = transform_images(
                    self.context_queue, 
                    self.model_params["image_size"]
                )
                goal_data = transform_images(
                    sg_img, 
                    self.model_params["image_size"]
                )
                batch_obs_imgs.append(transf_obs_img)
                batch_goal_data.append(goal_data)
            
            # Predict distances and waypoints
            batch_obs_imgs = torch.cat(batch_obs_imgs, dim=0).to(self.device)
            batch_goal_data = torch.cat(batch_goal_data, dim=0).to(self.device)
            
            with torch.no_grad():
                distances, waypoints = self.model(batch_obs_imgs, batch_goal_data)
            
            distances = to_numpy(distances)
            waypoints = to_numpy(waypoints)

            # Find closest node
            min_dist_idx = np.argmin(distances)

            if (self.debug):
                self.get_logger().info(
                    f"Nodes [{start}:{end+1}] Dist: {distances.flatten()} Min@{min_dist_idx}",
                    throttle_duration_sec=0.5
                )
            
            # Choose subgoal and output waypoint
            if distances[min_dist_idx] > self.close_threshold:
                chosen_waypoint = waypoints[min_dist_idx][self.waypoint_idx]
                self.closest_node = start + min_dist_idx
            else:
                chosen_waypoint = waypoints[
                    min(min_dist_idx + 1, len(waypoints) - 1)
                ][self.waypoint_idx]
                self.closest_node = min(start + min_dist_idx + 1, self.goal_node)
            
            self.get_logger().info(
                f"Closest node: {self.closest_node}/{self.goal_node}",
                throttle_duration_sec=1.0
            )
        
        # Normalize waypoint if needed
        if self.model_params.get("normalize", False):
            chosen_waypoint[:2] *= (self.MAX_V / self.RATE)
        
        # Publish waypoint
        waypoint_msg = Float32MultiArray()
        waypoint_msg.data = chosen_waypoint.tolist()
        self.waypoint_pub.publish(waypoint_msg)

        # Publish closest node
        node_msg = Int32()
        node_msg.data = int(self.closest_node)
        self.closest_node_pub.publish(node_msg)

        # Check goal reached
        self.reached_goal = bool(self.closest_node == self.goal_node)
        goal_msg = Bool()
        goal_msg.data = self.reached_goal
        self.goal_pub.publish(goal_msg)
        
        if self.reached_goal:
            self.get_logger().info("=" * 60)
            self.get_logger().info("🎯 GOAL REACHED! Shutting down navigator...")
            self.get_logger().info("=" * 60)
            
            # Publish zero waypoint before shutdown
            zero_waypoint = Float32MultiArray()
            zero_waypoint.data = [0.0, 0.0, 0.0, 0.0]
            self.waypoint_pub.publish(zero_waypoint)
            
            # Cancel timer to stop loop
            self.timer.cancel()
            
            # Trigger shutdown
            raise SystemExit("Goal reached")



def main(args=None):
    rclpy.init(args=args)
    node = ViNTNavigatorNode()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit) as e:
        if isinstance(e, SystemExit):
            node.get_logger().info("Navigator shutdown complete.")
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
