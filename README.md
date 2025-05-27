# Conveyor Belt Pick-and-Place Robot System

**Developer:** Sai-04021210 (inthedarkshades00008@gmail.com)
**Timeline:** January 2025 - March 2025
**Framework:** ROS 2 Humble, Gazebo, MoveIt, RViz2

## Project Overview

This project implements an automated pick-and-place robot system operating on a conveyor belt using ROS 2. The system features a 4-DOF robotic arm with gripper that can autonomously pick objects from a moving conveyor belt and place them in designated drop zones.

### Key Features

- 4-DOF Robotic Arm with synchronized gripper
- Conveyor Belt Simulation with physics-based object movement
- Automated Pick-and-Place with trajectory planning
- Gazebo Integration for realistic simulation
- RViz2 Visualization for monitoring and debugging
- MoveIt Integration for advanced motion planning
- Modular Architecture for easy extension and customization

## System Architecture

```
├── src/
│   ├── basic_description/          # Robot URDF/XACRO descriptions
│   ├── basic_py_examples/          # Basic ROS 2 Python examples
│   ├── conveyor_belt_system/       # Conveyor belt and integration
│   └── conveyor_manipulation/      # Pick-and-place logic
```

### Package Descriptions

#### basic_description
- Purpose: Robot URDF/XACRO descriptions with collision and inertial properties
- Key Files:
  - urdf/robot_clean.xacro - Clean robot description
  - meshes/ - STL mesh files for visualization

#### basic_py_examples
- Purpose: Educational ROS 2 Python examples
- Key Files:
  - simple_publisher.py - Basic publisher example
  - simple_subscriber.py - Basic subscriber example
  - parameters.py - Parameter handling demonstration

#### conveyor_belt_system
- Purpose: Conveyor belt simulation and system integration
- Key Files:
  - urdf/conveyor_belt.xacro - Conveyor belt description
  - urdf/robot_gazebo.xacro - Complete Gazebo-ready system
  - launch/gazebo_simulation.launch.py - Gazebo simulation launcher
  - config/robot_controllers.yaml - ROS 2 control configuration

#### conveyor_manipulation
- Purpose: Robot control and pick-and-place logic
- Key Files:
  - robot_controller.py - Main robot control node
  - Advanced manipulation algorithms (planned for future commits)

## Quick Start Guide

### Prerequisites

```bash
# ROS 2 Humble installation required
sudo apt update
sudo apt install ros-humble-desktop-full
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-gazebo-ros2-control
```

### Installation

1. Clone the repository:
```bash
git clone <repository-url>
cd ROS---2
```

2. Install dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the workspace:
```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### Usage Examples

#### Basic Robot Movement Test
```bash
# Terminal 1: Launch robot with GUI control
ros2 launch conveyor_belt_system simple_robot_test.launch.py

# Terminal 2: Run automated demo
ros2 run conveyor_manipulation robot_controller
```

#### Full Gazebo Simulation
```bash
# Launch complete simulation environment
ros2 launch conveyor_belt_system gazebo_simulation.launch.py
```

#### RViz Visualization Only
```bash
# Launch robot visualization
ros2 launch conveyor_belt_system robot_conveyor_demo.launch.py use_rviz:=true
```

## Development Timeline (30 Commits)

### Phase 1: Foundation Setup (January 2025)
- [x] Commit 1-2: Enhanced robot description with Gazebo integration (COMPLETED)
- [ ] Commit 3: MoveIt configuration for motion planning
- [ ] Commit 4: Basic conveyor belt physics in Gazebo
- [ ] Commit 5: Robot-conveyor integration testing
- [ ] Commit 6: Joint controllers and hardware interface
- [ ] Commit 7: Basic pick-and-place validation
- [ ] Commit 8: RViz visualization improvements
- [ ] Commit 9: Error handling and safety features
- [ ] Commit 10: Phase 1 documentation and testing

### Phase 2: Conveyor System (February 2025)
- [ ] Commit 11: Dynamic object spawning system
- [ ] Commit 12: Conveyor belt movement simulation
- [ ] Commit 13: Object detection and tracking
- [ ] Commit 14: Vision-based pick point calculation
- [ ] Commit 15: Trajectory planning for moving objects
- [ ] Commit 16: Multi-object handling logic
- [ ] Commit 17: Drop zone management
- [ ] Commit 18: Performance optimization
- [ ] Commit 19: Failure recovery mechanisms
- [ ] Commit 20: Phase 2 integration testing

### Phase 3: Automation & Polish (March 2025)
- [ ] Commit 21: Fully automated workflow
- [ ] Commit 22: Multiple object types support
- [ ] Commit 23: Real-time performance monitoring
- [ ] Commit 24: Advanced motion planning
- [ ] Commit 25: User interface for control
- [ ] Commit 26: Comprehensive testing suite
- [ ] Commit 27: Documentation and tutorials
- [ ] Commit 28: Demo scenarios and examples
- [ ] Commit 29: Performance benchmarking
- [ ] Commit 30: Final integration and deployment

## Configuration

### Robot Parameters
```yaml
# Joint limits and capabilities
baseplate_joint: ±90° (±1.57 rad)
forward_drivearm_joint: ±90° (±1.57 rad)
horizontal_arm_joint: ±45° (±0.785 rad)
gripper_right_joint: -90° to 0° (-1.57 to 0 rad)
```

### Conveyor Belt Settings
```yaml
# Conveyor specifications
length: 2.0m
width: 0.5m
height: 0.05m
speed: 0.1 m/s (configurable)
```

## Current Capabilities

### Working Features:
- Smooth robot joint movement with cosine interpolation
- Automated pick-and-place demo sequences
- Gazebo simulation environment
- ROS 2 control integration
- RViz visualization
- Modular launch system

### In Development:
- MoveIt motion planning integration
- Dynamic object spawning
- Conveyor belt movement physics
- Vision-based object detection

## API Reference

### Robot Controller Node

```python
# Main control interface
class RobotController(Node):
    def move_to_position(target_positions, duration=3.0)
    def execute_pick_and_place_demo()
    def emergency_stop()
```

### Key Topics

```bash
# Joint control
/joint_group_position_controller/commands  # Float64MultiArray
/joint_states                              # sensor_msgs/JointState

# Visualization
/robot_description                         # std_msgs/String
/tf                                       # tf2_msgs/TFMessage
```

## Troubleshooting

### Common Issues

1. XACRO parsing errors:
   ```bash
   # Use the clean robot description
   ros2 launch conveyor_belt_system simple_robot_test.launch.py
   ```

2. Missing dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Gazebo not starting:
   ```bash
   # Check Gazebo installation
   gazebo --version
   ```

## Contributing

This project follows the development timeline from January to March 2025. Each commit represents a specific milestone in the development process.

### Development Guidelines
- Follow ROS 2 coding standards
- Add comprehensive documentation for new features
- Include unit tests for critical functionality
- Update this README for major changes

## License

TODO: Add license information

## Contact

**Developer:** Sai-04021210
**Email:** inthedarkshades00008@gmail.com
**Project Timeline:** January 2025 - March 2025

---

This project demonstrates advanced robotics concepts including autonomous manipulation, simulation, and real-time control using modern ROS 2 frameworks.
