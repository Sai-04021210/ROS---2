# Comprehensive Usage Guide

## Getting Started

### System Requirements
- OS: Ubuntu 22.04 LTS
- ROS: ROS 2 Humble
- RAM: Minimum 4GB (8GB recommended)
- GPU: Optional (for advanced Gazebo rendering)

### Installation Steps

1. Install ROS 2 Humble:
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update
sudo apt install ros-humble-desktop-full
```

2. Install Additional Dependencies:
```bash
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro
```

3. Setup Workspace:
```bash
mkdir -p ~/conveyor_robot_ws/src
cd ~/conveyor_robot_ws
git clone <repository-url> src/
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Launch Configurations

### 1. Basic Robot Testing
Purpose: Test robot movement without simulation

```bash
# Terminal 1: Launch robot state publisher and GUI
ros2 launch conveyor_belt_system simple_robot_test.launch.py

# Terminal 2: Monitor joint states
ros2 topic echo /joint_states

# Terminal 3: Manual joint control
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

### 2. Automated Demo
**Purpose:** Watch automated pick-and-place sequence

```bash
# Launch robot with automated demo
ros2 launch conveyor_belt_system robot_only_test.launch.py

# The robot will automatically:
# 1. Move to home position
# 2. Move to pick position
# 3. Close gripper
# 4. Lift object
# 5. Move to drop position
# 6. Open gripper
# 7. Return home
```

### 3. Full Gazebo Simulation
**Purpose:** Complete physics simulation environment

```bash
# Launch Gazebo simulation
ros2 launch conveyor_belt_system gazebo_simulation.launch.py

# Optional: Launch without GUI (headless)
ros2 launch conveyor_belt_system gazebo_simulation.launch.py gui:=false
```

### 4. RViz Visualization
**Purpose:** Visualize robot and environment

```bash
# Launch with RViz
ros2 launch conveyor_belt_system robot_conveyor_demo.launch.py

# RViz will show:
# - Robot model with current joint positions
# - TF frames
# - Conveyor belt and objects
# - Drop zones
```

## 🎮 Manual Control

### Joint Position Control
```bash
# Direct joint position commands
ros2 topic pub /joint_group_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, -0.3, 0.2, -0.1]"

# Joint positions correspond to:
# [baseplate_joint, forward_drivearm_joint, horizontal_arm_joint, gripper_right_joint]
```

### Predefined Positions
```bash
# Home position
ros2 topic pub /joint_group_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0]"

# Pick position (open gripper)
ros2 topic pub /joint_group_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, -0.3, 0.2, -0.5]"

# Pick position (closed gripper)
ros2 topic pub /joint_group_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, -0.3, 0.2, -0.05]"

# Drop position
ros2 topic pub /joint_group_position_controller/commands std_msgs/msg/Float64MultiArray "data: [-0.5, -0.2, 0.1, -0.05]"
```

## 🔧 Configuration Options

### Robot Controller Parameters
```yaml
# Edit: src/conveyor_manipulation/conveyor_manipulation/robot_controller.py

# Movement parameters
self.control_rate = 50  # Hz
self.movement_duration = 3.0  # seconds

# Position definitions
self.home_position = [0.0, 0.0, 0.0, 0.0]
self.pick_position_1 = [0.5, -0.3, 0.2, -0.5]  # Open gripper
self.pick_position_2 = [0.5, -0.3, 0.2, -0.05]  # Closed gripper
self.drop_position = [-0.5, -0.2, 0.1, -0.05]  # Drop location
```

### Conveyor Belt Parameters
```yaml
# Edit: src/conveyor_belt_system/urdf/conveyor_belt.xacro

<xacro:property name="belt_length" value="2.0"/>
<xacro:property name="belt_width" value="0.5"/>
<xacro:property name="belt_height" value="0.05"/>
<xacro:property name="belt_speed" value="0.1"/>
```

### ROS 2 Control Settings
```yaml
# Edit: src/conveyor_belt_system/config/robot_controllers.yaml

controller_manager:
  ros__parameters:
    update_rate: 100  # Hz (increase for smoother control)

joint_group_position_controller:
  ros__parameters:
    state_publish_rate: 50.0  # Hz
    action_monitor_rate: 20.0  # Hz
```

## 📊 Monitoring and Debugging

### Essential Topics to Monitor
```bash
# Joint states
ros2 topic echo /joint_states

# Joint commands
ros2 topic echo /joint_group_position_controller/commands

# Robot description
ros2 topic echo /robot_description

# TF transforms
ros2 run tf2_tools view_frames.py
```

### Useful Commands
```bash
# List all topics
ros2 topic list

# Check node status
ros2 node list

# Monitor system performance
ros2 topic hz /joint_states

# Check controller status
ros2 control list_controllers

# View robot model
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro src/basic_description/urdf/robot_clean.xacro)"
```

## 🐛 Troubleshooting

### Common Issues and Solutions

#### 1. Robot Not Moving
```bash
# Check if controllers are loaded
ros2 control list_controllers

# Manually load controllers
ros2 control load_controller joint_state_broadcaster
ros2 control load_controller joint_group_position_controller

# Set controllers active
ros2 control set_controller_state joint_state_broadcaster active
ros2 control set_controller_state joint_group_position_controller active
```

#### 2. Gazebo Not Starting
```bash
# Check Gazebo installation
gazebo --version

# Clear Gazebo cache
rm -rf ~/.gazebo/

# Launch Gazebo separately
gazebo --verbose
```

#### 3. XACRO Parsing Errors
```bash
# Test XACRO file directly
xacro src/basic_description/urdf/robot_clean.xacro

# Check for syntax errors
xmllint --noout src/basic_description/urdf/robot_clean.xacro
```

#### 4. Missing Dependencies
```bash
# Install missing packages
rosdep install --from-paths src --ignore-src -r -y

# Check specific package
apt search ros-humble-<package-name>
```

## 🎯 Advanced Usage

### Custom Pick-and-Place Sequences
```python
# Create custom controller node
import rclpy
from conveyor_manipulation.robot_controller import RobotController

class CustomController(RobotController):
    def custom_sequence(self):
        # Define your custom positions
        custom_position = [0.3, -0.4, 0.1, -0.2]
        self.move_to_position(custom_position, duration=2.0)
```

### Adding New Objects
```xml
<!-- Edit: src/conveyor_belt_system/urdf/robot_gazebo.xacro -->
<link name="new_object">
    <visual>
        <geometry>
            <sphere radius="0.03"/>
        </geometry>
        <material name="object_green">
            <color rgba="0.0 1.0 0.0 1.0"/>
        </material>
    </visual>
    <!-- Add collision and inertial properties -->
</link>
```

### Performance Tuning
```yaml
# Increase control frequency for smoother motion
update_rate: 200  # Hz

# Adjust trajectory constraints
constraints:
    goal_time: 0.3  # Faster movements
    stopped_velocity_tolerance: 0.005  # Higher precision
```

## 📈 Performance Metrics

### Expected Performance
- **Joint Movement Accuracy:** ±0.01 radians
- **Pick-and-Place Cycle Time:** ~15 seconds
- **Control Frequency:** 50-100 Hz
- **Simulation Real-time Factor:** 0.8-1.0x

### Benchmarking Commands
```bash
# Monitor control loop frequency
ros2 topic hz /joint_states

# Check CPU usage
htop

# Monitor memory usage
free -h

# Gazebo performance
gz stats
```

This guide provides comprehensive instructions for using the conveyor belt pick-and-place robot system. For additional help, refer to the main README.md or contact the developer.
