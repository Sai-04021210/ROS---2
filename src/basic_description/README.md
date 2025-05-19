# Basic Robot Description Package

This package contains the URDF/XACRO description of a basic robotic arm with a gripper. The robot model is designed for simulation and visualization purposes in ROS 2.

## Robot Structure

The robot consists of the following main components:

1. **Base**: A fixed base connected to the world frame
2. **Baseplate**: Rotates around the Z-axis (yaw) relative to the base
3. **Forward Drive Arm**: Rotates around the X-axis (pitch) relative to the baseplate
4. **Horizontal Arm**: Extends outward and can rotate around the X-axis
5. **Claw Support**: Fixed attachment to the horizontal arm
6. **Gripper**: Two-finger gripper with synchronized movement

## Joint Configuration

The robot has 5 revolute joints:
- `baseplate_joint`: Rotation of the baseplate (±90°)
- `forward_drivearm_joint`: Rotation of the forward arm (±90°)
- `horizontal_arm_joint`: Rotation of the horizontal arm (±45°)
- `gripper_left_joint`: Left gripper finger (0° to 90°)
- `gripper_right_joint`: Right gripper finger (-90° to 0°)

The gripper fingers are synchronized through a mimic joint configuration.

## Mesh Files

The robot uses STL mesh files for visualization, located in the `meshes` directory:
- basement.STL
- base_plate.STL
- forward_drive_arm.STL
- horizontal_arm.STL
- claw_support.STL
- left_finger.STL
- right_finger.STL

## Usage

To visualize the robot model:

```bash
# Launch with RViz using the urdf_tutorial package
ros2 launch urdf_tutorial display.launch.py model:=/path/to/ros2/src/basic_description/urdf/basic_urdf.xacro
```

For example:
```bash
ros2 launch urdf_tutorial display.launch.py model:=/home/ram/Documents/ros2/src/basic_description/urdf/basic_urdf.xacro
```

## License

TODO: Add license information
