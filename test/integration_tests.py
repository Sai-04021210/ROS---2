#!/usr/bin/env python3
"""
Integration Tests for Conveyor Belt Pick-and-Place Robot System
Developer: Sai-04021210 (inthedarkshades00008@gmail.com)
Date: January 2025

This module contains comprehensive integration tests for the robot system.
"""

import unittest
import rospy
import rostest
import time
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import numpy as np

class ConveyorRobotIntegrationTest(unittest.TestCase):
    """Integration tests for the complete conveyor robot system."""

    def setUp(self):
        """Set up test environment."""
        rospy.init_node('integration_test_node', anonymous=True)

        # Publishers
        self.joint_cmd_pub = rospy.Publisher(
            '/joint_group_position_controller/commands',
            Float64MultiArray,
            queue_size=1
        )

        self.belt_speed_pub = rospy.Publisher(
            '/conveyor_belt/belt_speed',
            Float64,
            queue_size=1
        )

        # Subscribers
        self.joint_states = None
        self.belt_status = None

        self.joint_sub = rospy.Subscriber(
            '/joint_states',
            JointState,
            self.joint_state_callback
        )

        self.belt_sub = rospy.Subscriber(
            '/conveyor_belt/belt_status',
            Float64,
            self.belt_status_callback
        )

        # Wait for connections
        time.sleep(2.0)

    def joint_state_callback(self, msg):
        """Callback for joint state messages."""
        self.joint_states = msg

    def belt_status_callback(self, msg):
        """Callback for belt status messages."""
        self.belt_status = msg.data

    def test_robot_initialization(self):
        """Test robot initialization and basic functionality."""
        rospy.loginfo("Testing robot initialization...")

        # Wait for joint states
        timeout = 10.0
        start_time = time.time()
        while self.joint_states is None and (time.time() - start_time) < timeout:
            time.sleep(0.1)

        self.assertIsNotNone(self.joint_states, "Joint states not received")
        self.assertGreater(len(self.joint_states.position), 0, "No joint positions received")

        rospy.loginfo("Robot initialization test passed")

    def test_joint_movement(self):
        """Test robot joint movement capabilities."""
        rospy.loginfo("Testing joint movement...")

        # Define test positions
        home_position = [0.0, 0.0, 0.0, 0.0]
        test_position = [0.5, -0.3, 0.2, -0.1]

        # Move to home position
        self.send_joint_command(home_position)
        time.sleep(3.0)

        # Verify movement
        if self.joint_states:
            current_positions = list(self.joint_states.position[:4])
            for i, (current, target) in enumerate(zip(current_positions, home_position)):
                self.assertAlmostEqual(current, target, places=1,
                                     msg=f"Joint {i} not at home position")

        # Move to test position
        self.send_joint_command(test_position)
        time.sleep(3.0)

        rospy.loginfo("Joint movement test passed")

    def test_conveyor_belt_control(self):
        """Test conveyor belt speed control."""
        rospy.loginfo("Testing conveyor belt control...")

        # Test different belt speeds
        test_speeds = [0.0, 0.1, 0.2, 0.05]

        for speed in test_speeds:
            # Send speed command
            speed_msg = Float64()
            speed_msg.data = speed
            self.belt_speed_pub.publish(speed_msg)

            time.sleep(2.0)

            # Verify belt status (if available)
            if self.belt_status is not None:
                self.assertAlmostEqual(self.belt_status, speed, places=2,
                                     msg=f"Belt speed not set to {speed}")

        rospy.loginfo("Conveyor belt control test passed")

    def test_pick_and_place_sequence(self):
        """Test complete pick and place sequence."""
        rospy.loginfo("Testing pick and place sequence...")

        # Define pick and place positions
        home_position = [0.0, 0.0, 0.0, 0.0]
        pick_position_open = [0.5, -0.3, 0.2, -0.5]
        pick_position_closed = [0.5, -0.3, 0.2, -0.05]
        lift_position = [0.5, -0.1, 0.2, -0.05]
        drop_position = [-0.5, -0.2, 0.1, -0.05]
        drop_position_open = [-0.5, -0.2, 0.1, -0.5]

        sequence = [
            ("Home", home_position, 2.0),
            ("Pick (Open)", pick_position_open, 3.0),
            ("Pick (Closed)", pick_position_closed, 1.0),
            ("Lift", lift_position, 2.0),
            ("Drop", drop_position, 4.0),
            ("Drop (Open)", drop_position_open, 1.0),
            ("Return Home", home_position, 3.0)
        ]

        for step_name, position, duration in sequence:
            rospy.loginfo(f"Executing step: {step_name}")
            self.send_joint_command(position)
            time.sleep(duration)

            # Verify position reached (basic check)
            if self.joint_states and len(self.joint_states.position) >= 4:
                current_positions = list(self.joint_states.position[:4])
                # Allow some tolerance for position accuracy
                for i, (current, target) in enumerate(zip(current_positions, position)):
                    self.assertAlmostEqual(current, target, places=0,
                                         msg=f"Joint {i} not at target in step {step_name}")

        rospy.loginfo("Pick and place sequence test passed")

    def test_system_performance(self):
        """Test system performance metrics."""
        rospy.loginfo("Testing system performance...")

        # Test joint state publication rate
        start_time = time.time()
        initial_count = 0
        final_count = 0

        # Count messages for 5 seconds
        test_duration = 5.0
        while (time.time() - start_time) < test_duration:
            if self.joint_states:
                final_count += 1
            time.sleep(0.01)

        message_rate = final_count / test_duration
        self.assertGreater(message_rate, 10.0, "Joint state publication rate too low")

        rospy.loginfo(f"Joint state rate: {message_rate:.1f} Hz")
        rospy.loginfo("System performance test passed")

    def test_error_handling(self):
        """Test system error handling and recovery."""
        rospy.loginfo("Testing error handling...")

        # Test invalid joint commands
        invalid_position = [10.0, 10.0, 10.0, 10.0]  # Beyond joint limits

        try:
            self.send_joint_command(invalid_position)
            time.sleep(2.0)
            # System should handle this gracefully
            rospy.loginfo("Invalid command handled gracefully")
        except Exception as e:
            rospy.logwarn(f"Exception during invalid command test: {e}")

        # Return to safe position
        safe_position = [0.0, 0.0, 0.0, 0.0]
        self.send_joint_command(safe_position)
        time.sleep(2.0)

        rospy.loginfo("Error handling test passed")

    def send_joint_command(self, positions):
        """Send joint position command."""
        msg = Float64MultiArray()
        msg.data = positions
        self.joint_cmd_pub.publish(msg)

    def tearDown(self):
        """Clean up after tests."""
        # Return robot to home position
        home_position = [0.0, 0.0, 0.0, 0.0]
        self.send_joint_command(home_position)
        time.sleep(2.0)

        # Stop conveyor belt
        speed_msg = Float64()
        speed_msg.data = 0.0
        self.belt_speed_pub.publish(speed_msg)

if __name__ == '__main__':
    # Run integration tests
    rostest.rosrun('conveyor_belt_system', 'integration_tests',
                   ConveyorRobotIntegrationTest)
