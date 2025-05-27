#!/usr/bin/env python3

"""
Robot Controller for Conveyor Belt Pick and Place System
This node controls the robot arm movements for pick and place operations.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import time
import math

class RobotController(Node):
    """
    Controls the robot arm for pick and place operations on a conveyor belt.
    """
    
    def __init__(self):
        super().__init__('robot_controller')
        
        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray, 
            '/joint_group_position_controller/commands', 
            10
        )
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Robot state
        self.current_joint_positions = None
        self.joint_names = [
            'baseplate_joint',
            'forward_drivearm_joint',
            'horizontal_arm_joint',
            'gripper_right_joint'
        ]
        
        # Predefined positions
        self.home_position = [0.0, 0.0, 0.0, 0.0]
        self.pick_position_1 = [0.5, -0.3, 0.2, -0.5]  # Open gripper
        self.pick_position_2 = [0.5, -0.3, 0.2, -0.05]  # Closed gripper
        self.drop_position = [-0.5, -0.2, 0.1, -0.05]  # Drop location
        
        # Control parameters
        self.control_rate = 50  # Hz
        self.movement_duration = 3.0  # seconds
        
        # Demo state
        self.demo_running = False
        self.demo_step = 0
        
        # Timer for demo
        self.demo_timer = self.create_timer(5.0, self.demo_callback)
        
        self.get_logger().info('Robot Controller initialized')
        self.get_logger().info('Waiting for joint states...')
    
    def joint_state_callback(self, msg):
        """Process joint state messages."""
        if self.current_joint_positions is None:
            self.get_logger().info('Received first joint states')
        
        # Extract positions for our controlled joints
        positions = []
        for joint_name in self.joint_names:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                positions.append(msg.position[idx])
            else:
                positions.append(0.0)
        
        self.current_joint_positions = positions
    
    def move_to_position(self, target_positions, duration=None):
        """
        Move robot to target joint positions with smooth interpolation.
        
        Args:
            target_positions: List of target joint positions
            duration: Time to complete movement (seconds)
        """
        if self.current_joint_positions is None:
            self.get_logger().warn('No joint states received yet')
            return False
        
        if duration is None:
            duration = self.movement_duration
        
        start_positions = self.current_joint_positions.copy()
        start_time = time.time()
        
        rate = self.create_rate(self.control_rate)
        
        self.get_logger().info(f'Moving to position: {target_positions}')
        
        while rclpy.ok():
            elapsed = time.time() - start_time
            if elapsed >= duration:
                # Movement complete
                self.publish_joint_command(target_positions)
                break
            
            # Smooth interpolation using cosine function
            t = elapsed / duration
            smooth_t = 0.5 * (1 - math.cos(math.pi * t))
            
            current_targets = [
                start_pos + smooth_t * (target_pos - start_pos)
                for start_pos, target_pos in zip(start_positions, target_positions)
            ]
            
            self.publish_joint_command(current_targets)
            
            try:
                rate.sleep()
            except:
                break
        
        self.get_logger().info('Movement completed')
        return True
    
    def publish_joint_command(self, positions):
        """Publish joint position commands."""
        msg = Float64MultiArray()
        msg.data = positions
        self.joint_cmd_pub.publish(msg)
    
    def demo_callback(self):
        """Execute demo sequence."""
        if not self.demo_running and self.current_joint_positions is not None:
            self.demo_running = True
            self.execute_pick_and_place_demo()
    
    def execute_pick_and_place_demo(self):
        """Execute a complete pick and place demonstration."""
        self.get_logger().info('Starting pick and place demo...')
        
        try:
            # Step 1: Move to home position
            self.get_logger().info('Step 1: Moving to home position')
            self.move_to_position(self.home_position, 2.0)
            time.sleep(1.0)
            
            # Step 2: Move to pick position (open gripper)
            self.get_logger().info('Step 2: Moving to pick position')
            self.move_to_position(self.pick_position_1, 3.0)
            time.sleep(1.0)
            
            # Step 3: Close gripper
            self.get_logger().info('Step 3: Closing gripper')
            self.move_to_position(self.pick_position_2, 1.0)
            time.sleep(1.0)
            
            # Step 4: Lift object
            lift_position = self.pick_position_2.copy()
            lift_position[1] += 0.2  # Lift arm
            self.get_logger().info('Step 4: Lifting object')
            self.move_to_position(lift_position, 2.0)
            time.sleep(1.0)
            
            # Step 5: Move to drop position
            self.get_logger().info('Step 5: Moving to drop position')
            self.move_to_position(self.drop_position, 4.0)
            time.sleep(1.0)
            
            # Step 6: Open gripper (drop object)
            drop_open = self.drop_position.copy()
            drop_open[3] = -0.5  # Open gripper
            self.get_logger().info('Step 6: Dropping object')
            self.move_to_position(drop_open, 1.0)
            time.sleep(1.0)
            
            # Step 7: Return to home
            self.get_logger().info('Step 7: Returning to home')
            self.move_to_position(self.home_position, 3.0)
            time.sleep(2.0)
            
            self.get_logger().info('Pick and place demo completed!')
            
        except Exception as e:
            self.get_logger().error(f'Demo failed: {str(e)}')
        
        finally:
            self.demo_running = False
    
    def move_to_home(self):
        """Move robot to home position."""
        return self.move_to_position(self.home_position)
    
    def emergency_stop(self):
        """Emergency stop - hold current position."""
        if self.current_joint_positions is not None:
            self.publish_joint_command(self.current_joint_positions)
            self.get_logger().warn('Emergency stop activated')

def main(args=None):
    rclpy.init(args=args)
    
    robot_controller = RobotController()
    
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        robot_controller.get_logger().info('Shutting down robot controller...')
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
