# ViNT Navigation for UAV

Vision-based Navigation Transformer (ViNT) implementation for ROS2 UAV navigation with MRS UAV System integration.

**Based on:** [Visual Navigation Transformer (ViNT)](https://github.com/robodhruv/visualnav-transformer)

## Repository Structure

```
.
├── aws-robomaker-hospital-world/     # Hospital environment for Gazebo simulation
├── aws-robomaker-small-warehouse-world/  # Warehouse environment for Gazebo simulation
└── vint_navigation/                  # Main navigation package
    ├── config/                       # Configuration files
    ├── launch/                       # ROS2 launch files
    ├── tmux/                         # Simulation startup scripts
    │   ├── gazebo/                   # Gazebo simulation sessions
    │   └── rosbags/                  # Rosbag replay sessions
    └── vint_navigation/              # Python nodes
        ├── create_topomap_node.py
        ├── utils.py
        ├── vint_navigator_node.py
        ├── vint_visualizer_node.py
        ├── vint_to_mrs_waypoint_node.py
        └── vint_velocity_reference_generator.py
```

## Overview

This repository implements visual navigation for UAVs using the ViNT model. The system uses topological maps created from camera images and generates waypoints for autonomous navigation in indoor environments.

## Prerequisites

### System Requirements

- Ubuntu 24.04 (recommended)
- ROS2 Jazzy
- Python 3.10+
- CUDA-capable GPU (optional, for faster inference)

### Required Libraries

Install Python dependencies:

```bash
# Core deep learning and vision libraries
pip install torch==2.6.0 torchvision==0.21.0 torchaudio==2.6.0
pip install numpy==1.26.4 pillow==12.0.0 opencv-python==4.8.1.78

# ViNT model dependencies
pip install transformers==4.57.6 diffusers==0.35.2 einops==0.8.1
pip install efficientnet_pytorch==0.7.1 datasets==4.1.1

# Training utilities (if fine-tuning)
pip install git+https://github.com/Tony-Y/pytorch_warmup.git
pip install warmup_scheduler==0.3

# Utilities
pip install PyYAML==6.0.3 tqdm==4.67.1 scipy==1.17.0

# ViNT training repository
pip install -e git+https://github.com/robodhruv/visualnav-transformer.git#egg=vint_train&subdirectory=train
```

### MRS UAV System Installation

This package requires the MRS UAV System for UAV control and simulation. Follow the installation instructions:

```bash
# Clone the MRS UAV System repository
git clone https://github.com/ctu-mrs/mrs_uav_system.git -b ros2

# Follow the installation guide in the repository
cd mrs_uav_system
./installation/install.sh
```

For detailed installation instructions, visit: [https://github.com/ctu-mrs/mrs_uav_system/tree/ros2](https://github.com/ctu-mrs/mrs_uav_system/tree/ros2)

## Installation

1. Clone this repository into your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone <repository-url> vint_uav
cd ~/ros2_ws
colcon build --packages-select vint_navigation aws_robomaker_hospital_world aws_robomaker_small_warehouse_world
source install/setup.bash
```

2. Download ViNT model weights and place them in the appropriate directory as specified in your config files.

## ViNT Navigation Package

### Nodes Description

#### 1. `create_topomap_node.py`

Creates topological maps by recording camera images at regular intervals during a manual flight.

**Subscribed Topics:**
- `/camera/image_raw` - Camera feed

**Services:**
- `~/start_recording` - Start recording images
- `~/stop_recording` - Stop recording and save topomap

**Parameters:**
- `image_topic` - Camera image topic (default: `/camera/image_raw`)
- `topomap_dir` - Directory to save topomaps
- `topomap_name` - Name of the topomap
- `dt` - Time interval between saved images (default: 1.0s)
- `auto_start` - Start recording automatically (default: false)

#### 2. `vint_navigator_node.py`

Main navigation node that processes camera images and generates waypoints using the ViNT model.

**Subscribed Topics:**
- `/camera/image_raw` - Camera feed

**Published Topics:**
- `/vint/waypoint` (Float32MultiArray) - Predicted waypoint [x, y, theta, distance]
- `/vint/closest_node` (Int32) - Current closest topomap node
- `/vint/goal_reached` (Bool) - Goal reached flag

**Parameters:**
- `model` - Model type (default: 'vint')
- `waypoint_index` - Which waypoint from trajectory to use (default: 2)
- `topomap_dir` - Path to topomap directory
- `goal_node` - Target node index (-1 for last node)
- `close_threshold` - Distance threshold to advance to next node
- `radius` - Search radius for closest node
- `image_topic` - Camera image topic

#### 3. `vint_visualizer_node.py`

Visualization node for RViz displaying navigation progress and predicted waypoints.

**Subscribed Topics:**
- `/camera/image_raw` - Camera feed
- `/vint/waypoint` - Current waypoint
- `/vint/closest_node` - Current node index

**Published Topics:**
- `/vint/viz/predicted_path` (Path) - Predicted path for RViz
- `/vint/viz/markers` (MarkerArray) - Visualization markers
- `/vint/viz/annotated_image` (Image) - Camera image with waypoint overlay

#### 4. `vint_velocity_reference_generator.py`

Converts ViNT spatial waypoints to velocity references for MRS UAV controllers. Handles frame transformations from camera frame to world frame.

**Subscribed Topics:**
- `/vint/waypoint` - ViNT waypoint commands
- `/{uav_name}/estimation_manager/odom_main` - UAV odometry

**Published Topics:**
- `/{uav_name}/control_manager/velocity_reference` - MRS velocity reference

**Parameters:**
- `uav_name` - UAV namespace (default: 'uav1')
- `max_horizontal_speed` - Maximum horizontal velocity (default: 1.5 m/s)
- `max_vertical_speed` - Maximum vertical velocity (default: 0.5 m/s)
- `max_yaw_rate` - Maximum yaw rate (default: 0.8 rad/s)
- `altitude_hold` - Target altitude (default: 1.5 m)
- `vint_is_normalized` - Whether ViNT outputs are normalized (default: true)
- `velocity_damping` - Velocity smoothing factor (default: 0.7)

#### 5. `vint_to_mrs_waypoint_node.py`

Alternative to velocity reference generator - converts ViNT waypoints to MRS goto commands for position-based control.

#### 6. `utils.py`

Utility functions including:
- `load_model()` - Load ViNT model from checkpoint
- `msg_to_pil()` - Convert ROS Image messages to PIL format
- `transform_images()` - Preprocess images for model inference
- `clip_angle()` - Normalize angles to [-π, π]

## Running the System

### 1. Start Simulation

Navigate to one of the tmux directories and run the start script:

```bash
# For Gazebo simulation
cd vint_navigation/tmux/gazebo
./start

# OR for rosbag replay
cd vint_navigation/tmux/rosbags
./start
```

This will launch the MRS UAV system with the selected environment in a tmux session.

### 2. Create Topomap (First Time)

Fly the UAV manually along your desired path while recording images:

```bash
# In a new terminal
ros2 run vint_navigation create_topomap_node --ros-args \
    -p topomap_dir:=/path/to/topomaps \
    -p topomap_name:=my_route \
    -p dt:=1.0

# Start recording
ros2 service call /create_topomap_node/start_recording std_srvs/srv/Trigger

# Fly the route manually...

# Stop recording
ros2 service call /create_topomap_node/stop_recording std_srvs/srv/Trigger
```

### 3. Run Navigation

Launch the ViNT navigation system:

```bash
ros2 launch vint_navigation vint_navigation.launch.py \
    topomap_dir:=/path/to/topomaps/my_route \
    uav_name:=uav1
```

Or run nodes individually:

```bash
# Start navigator
ros2 run vint_navigation vint_navigator_node --ros-args \
    -p topomap_dir:=/path/to/topomaps/my_route \
    -p goal_node:=-1

# Start velocity reference generator
ros2 run vint_navigation vint_velocity_reference_generator --ros-args \
    -p uav_name:=uav1 \
    -p altitude_hold:=1.5

# Start visualizer (optional)
ros2 run vint_navigation vint_visualizer_node
```

### 4. Monitor in RViz

```bash
rviz2 -d vint_navigation/rviz/vint_navigation.rviz
```

## Configuration

Key configuration files in `vint_navigation/config/`:

- `robot.yaml` - Robot parameters (max velocities, frame rate)
- `models.yaml` - Model paths and configurations
- `vint_config.yaml` - ViNT model hyperparameters

## Environments

### Hospital World

Large indoor hospital environment with corridors, rooms, and medical equipment. Suitable for long-range navigation testing.

### Warehouse World

Smaller warehouse environment with shelving and obstacles. Good for testing in confined spaces.

## Troubleshooting

**No images received:**
- Check camera topic name matches configuration
- Verify camera is publishing: `ros2 topic echo /camera/image_raw`

**Model not loading:**
- Verify model checkpoint path in `config/models.yaml`
- Check CUDA availability if using GPU

**UAV not moving:**
- Ensure MRS UAV system is properly initialized
- Check that control manager is in correct mode
- Verify velocity/goto commands are being published

**Poor navigation performance:**
- Ensure topomap was created in similar lighting conditions
- Adjust `waypoint_index` parameter (try 1-3)
- Tune velocity limits and damping parameters

## Citation

If you use this code, please cite the original ViNT paper:

```bibtex
@article{vint2023,
  title={ViNT: A Foundation Model for Visual Navigation},
  author={Shah, Dhruv and Sridhar, Ajay and Dashora, Nitish and Stachowicz, Kyle and Black, Kevin and Hirose, Noriaki and Levine, Sergey},
  journal={arXiv preprint arXiv:2306.14846},
  year={2023}
}
```

## License

This project integrates multiple components:
- ViNT model: See original ViNT repository license
- AWS RoboMaker worlds: Apache 2.0
- MRS UAV System: BSD 3-Clause

## Acknowledgments

- [ViNT: Visual Navigation Transformer](https://general-navigation-models.github.io/vint/)
- [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system)
- [AWS RoboMaker](https://github.com/aws-robotics)
