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

This repository implements visual navigation for UAVs using the ViNT model. The system uses topological maps created from camera images and generates waypoints for autonomous navigation.

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

1. Install the Robot Operating System (Jazzy):
```bash
curl https://ctu-mrs.github.io/ppa2-stable/add_ros_ppa.sh | bash
sudo apt install ros-jazzy-desktop-full
```

2. Configure your ROS environment according to [https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html#setup-environment](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html#setup-environment)

3. Add the **[stable](https://github.com/ctu-mrs/ppa2-stable)** PPA into your apt-get repository:
```bash
curl https://ctu-mrs.github.io/ppa2-stable/add_ppa.sh | bash
```
  * <details>
    <summary>>>> Special instructions for the MRS System developers <<<</summary>

      * Instead of the stable PPA, you can add the **[unstable](https://github.com/ctu-mrs/ppa2-unstable)** PPA, for which the packages are build immediatelly after being pushed to **ros2**.
      * If you have both PPAs, the **unstable** has a priority.
      * Beware! The **unstable** PPA might be internally inconsistent, buggy and dangerous!

    </details>

4. Install the MRS UAV System:
```bash
sudo apt install ros-jazzy-mrs-uav-system-full
```

5. Start the example MRS simulation session:
```bash
cd /opt/ros/jazzy/share/mrs_multirotor_simulator/tmux/mrs_one_drone
./start.sh
```

For detailed info about the MRS UAV System, visit: [https://github.com/ctu-mrs/mrs_uav_system/tree/ros2](https://github.com/ctu-mrs/mrs_uav_system/tree/ros2)

## Installation

1. Clone this repository into your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone <repository-url> vint_uav
cd ~/ros2_ws
colcon build --packages-select vint_navigation aws_robomaker_hospital_world aws_robomaker_small_warehouse_world
source install/setup.bash
```

2. Download [ViNT model weights](https://drive.google.com/drive/folders/1a9yWR2iooXFAqjQHetz263--4_2FFggg?usp=sharing) and place them in the appropriate directory as specified in your config files.

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

Visualization node for RViz displaying navigation progress and the drone's traveled path as a trail.

**Subscribed Topics:**
- `/camera/image_raw` - Camera feed for annotated image
- `/vint/waypoint` - Current waypoint from ViNT
- `/vint/closest_node` - Current node index in topomap
- `/{uav_name}/estimation_manager/odom_main` - UAV odometry for trail recording

**Published Topics:**
- `/vint/viz/trail` (Path) - Drone's traveled path trail
- `/vint/viz/trail_markers` (MarkerArray) - Start/current position markers and breadcrumbs
- `/vint/viz/annotated_image` (Image) - Camera image with waypoint overlay and ViNT outputs

**Parameters:**
- `uav_name` - UAV namespace (default: 'uav1')
- `frame_id` - Visualization frame (default: 'uav1/fixed_origin')
- `trail_length` - Number of positions to keep in trail history (default: 1000)
- `goal_node` - Target node index for progress display (default: -1)

**Visualization Features:**
- **Trail Path:** Continuous cyan line showing where the drone has traveled
- **Position Markers:** 
  - Green sphere at start position
  - Blue sphere at current drone position
  - Orange-yellow breadcrumbs along the path (color gradient shows time)
- **Info Display:** Black text showing current node and total distance traveled
- **Annotated Image:** Camera view with:
  - Node progress (green text)
  - Waypoint x, y coordinates (cyan text)
  - Heading cos, sin + angle in degrees (orange text)
  - Trail point count (yellow text)
  - Visual arrow pointing to waypoint target


#### 4. `vint_velocity_reference_generator.py`

Converts ViNT spatial waypoints to velocity references for MRS UAV controllers. Handles frame transformations from camera frame to world frame and includes optional depth-based collision avoidance.

**Subscribed Topics:**
- `/vint/waypoint` - ViNT waypoint commands (x, y, cos_heading, sin_heading)
- `/{uav_name}/estimation_manager/odom_main` - UAV odometry for frame transformation
- `/{uav_name}/rgbd_front/depth/image_raw` (optional) - Depth image for collision avoidance

**Published Topics:**
- `/{uav_name}/control_manager/velocity_reference` - MRS velocity reference commands

**Parameters:**
- `uav_name` - UAV namespace (default: 'uav1')
- `max_horizontal_speed` - Maximum horizontal velocity (default: 2.5 m/s)
- `max_vertical_speed` - Maximum vertical velocity (default: 1.0 m/s)
- `max_yaw_rate` - Maximum yaw rate (default: 0.8 rad/s)
- `altitude_hold` - Target altitude (default: 2.0 m)
- `vint_is_normalized` - Whether ViNT outputs are normalized (default: true)
- `vint_max_v` - MAX_V from ViNT training config (default: 0.5)
- `vint_rate` - Frame rate from ViNT training config (default: 10.0)
- `use_heading` - Use ViNT heading predictions (default: false)
- `velocity_damping` - Velocity smoothing factor 0-1 (default: 0.7)
- `dt` - Control timestep (default: 0.1 s)
- `use_collision_avoidance` - Enable depth-based collision avoidance (default: true)
- `collision_threshold` - Minimum safe distance to obstacles (default: 1.0 m)

**Features:**
- Body-frame to world-frame transformation
- Velocity scaling and clamping
- Exponential smoothing for stable control
- Optional heading control from ViNT predictions
- Depth-based collision avoidance with automatic stopping


#### 5. `utils.py`

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
ros2 launch vint_navigation create_topomap.launch.py \
             topomap_dir:=/path/to/your/topomaps/dir \
             topomap_name:=topomap_name \
             image_topic:=/camera/image_raw \
             dt:=1.0

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
    uav_name:=uav1 \
    enable_viz:=true \
    enable_control:=true \
    use_heading:=true \
    altitude_hold:=1.5 \
    use_collision_avoidance:=true 
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
    -p enable_control:=true 
    -p use_heading:=true 
    -p use_collision_avoidance:=true 

# Start visualizer (optional)
ros2 run vint_navigation vint_visualizer_node
    -p enable_viz:=true 
```

### 4. Monitor in RViz

```bash
ros2 run rviz2 rviz2 -d ./rviz.rviz --ros-args -p use_sim_time:=true
```

## Configuration

Key configuration files in `vint_navigation/config/`:

- `robot.yaml` - Robot parameters (max velocities, frame rate)
- `models.yaml` - Model paths and configurations
- `vint_params.yaml` - ViNT navigation hyperparameters
- `velocity_reference_generator.yaml` - ViNT velocity reference hyperparameters

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
