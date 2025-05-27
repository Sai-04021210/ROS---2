#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get package directories
    basic_description_pkg = FindPackageShare('basic_description')
    
    # Robot description - use the clean robot URDF
    robot_description_path = PathJoinSubstitution([
        basic_description_pkg, 'urdf', 'robot_clean.xacro'
    ])
    
    robot_description = ParameterValue(
        Command(['xacro ', robot_description_path]),
        value_type=str
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    # Joint state publisher GUI for manual control
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    # Robot controller
    robot_controller = Node(
        package='conveyor_manipulation',
        executable='robot_controller',
        name='robot_controller',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        robot_controller
    ])
