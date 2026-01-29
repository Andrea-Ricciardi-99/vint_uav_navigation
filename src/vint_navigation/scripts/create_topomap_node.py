#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, SetBool
import os
import shutil
import time
from pathlib import Path
from PIL import Image as PILImage
import numpy as np
from utils import msg_to_pil


class TopomapCreatorNode(Node):
    """
    Node to create topological maps by saving camera images at regular intervals.
    Control via ROS2 services for start/stop.
    """
    
    def __init__(self):
        super().__init__('topomap_creator')
        
        # Declare parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('topomap_dir', '')
        self.declare_parameter('topomap_name', 'topomap')
        self.declare_parameter('dt', 1.0)  # Time between saved images (seconds)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('timeout', 5.0)  # Timeout for no images
        
        # Get parameters
        self.image_topic = self.get_parameter('image_topic').value
        self.topomap_base_dir = self.get_parameter('topomap_dir').value
        self.topomap_name = self.get_parameter('topomap_name').value
        self.dt = self.get_parameter('dt').value
        self.auto_start = self.get_parameter('auto_start').value
        self.timeout = self.get_parameter('timeout').value
        
        # Validate parameters
        assert self.dt > 0, "dt must be positive"
        
        if not self.topomap_base_dir:
            # Default to package directory
            self.topomap_base_dir = str(Path.home() / 'topomaps')
        
        # State
        self.is_recording = False
        self.latest_image = None
        self.image_count = 0
        self.last_image_time = None
        self.topomap_path = None
        
        # Create base directory if needed
        Path(self.topomap_base_dir).mkdir(parents=True, exist_ok=True)
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        
        # Services
        self.start_service = self.create_service(
            Trigger,
            '~/start_recording',
            self.start_recording_callback
        )
        
        self.stop_service = self.create_service(
            Trigger,
            '~/stop_recording',
            self.stop_recording_callback
        )
        
        # Timer for saving images
        self.save_timer = self.create_timer(self.dt, self.save_image_callback)
        
        # Timer for timeout check
        self.timeout_timer = self.create_timer(1.0, self.check_timeout)
        
        self.get_logger().info(f"Topomap Creator initialized")
        self.get_logger().info(f"  Image topic: {self.image_topic}")
        self.get_logger().info(f"  Base directory: {self.topomap_base_dir}")
        self.get_logger().info(f"  Topomap name: {self.topomap_name}")
        self.get_logger().info(f"  Sampling interval: {self.dt}s")
        
        if self.auto_start:
            self.get_logger().info("Auto-start enabled. Starting recording...")
            self._start_recording()
        else:
            self.get_logger().info("Waiting for start_recording service call...")
    
    # def image_callback(self, msg):
    #     """Store latest image"""
    #     # Convert ROS Image to PIL
    #     img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(
    #         msg.height, msg.width, -1
    #     )
    #     self.latest_image = PILImage.fromarray(img_array)
    #     self.last_image_time = time.time()
    
    def image_callback(self, msg):
        """Store latest image"""
        try:
            self.latest_image = msg_to_pil(msg)
            self.last_image_time = time.time()
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")
        
    def _start_recording(self):
        """Internal method to start recording"""
        # Create topomap directory
        self.topomap_path = Path(self.topomap_base_dir) / self.topomap_name
        
        if self.topomap_path.exists():
            self.get_logger().warn(
                f"Topomap '{self.topomap_name}' already exists. Removing old images..."
            )
            shutil.rmtree(self.topomap_path)
        
        self.topomap_path.mkdir(parents=True, exist_ok=True)
        
        # Reset state
        self.image_count = 0
        self.is_recording = True
        self.last_image_time = time.time()
        
        self.get_logger().info(f"Started recording to: {self.topomap_path}")
    
    def start_recording_callback(self, request, response):
        """Service callback to start recording"""
        if self.is_recording:
            response.success = False
            response.message = "Already recording"
        else:
            self._start_recording()
            response.success = True
            response.message = f"Started recording to {self.topomap_path}"
        
        return response
    
    def stop_recording_callback(self, request, response):
        """Service callback to stop recording"""
        if not self.is_recording:
            response.success = False
            response.message = "Not recording"
        else:
            self.is_recording = False
            response.success = True
            response.message = f"Stopped recording. Saved {self.image_count} images to {self.topomap_path}"
            
            self.get_logger().info(f"Stopped recording. Total images: {self.image_count}")
        
        return response
    
    def save_image_callback(self):
        """Timer callback to save images at regular intervals"""
        if not self.is_recording:
            return
        
        if self.latest_image is None:
            self.get_logger().warn(
                "No images received yet...",
                throttle_duration_sec=2.0
            )
            return
        
        # Save image
        filename = f"{self.image_count:04d}.png"
        filepath = self.topomap_path / filename
        
        try:
            self.latest_image.save(filepath)
            self.get_logger().info(f"Saved image {self.image_count}: {filename}")
            self.image_count += 1
            self.latest_image = None  # Clear to avoid saving duplicates
        except Exception as e:
            self.get_logger().error(f"Failed to save image: {str(e)}")
    
    def check_timeout(self):
        """Check if images stopped coming"""
        if not self.is_recording:
            return
        
        if self.last_image_time is None:
            return
        
        time_since_last = time.time() - self.last_image_time
        
        if time_since_last > self.timeout:
            self.get_logger().warn(
                f"No images received for {time_since_last:.1f}s (timeout: {self.timeout}s). "
                f"Stopping recording..."
            )
            self.is_recording = False
            self.get_logger().info(f"Recording stopped. Total images: {self.image_count}")


def main(args=None):
    rclpy.init(args=args)
    node = TopomapCreatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.is_recording:
            node.get_logger().info(f"Interrupted. Saved {node.image_count} images.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
