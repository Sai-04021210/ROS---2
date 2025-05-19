from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directories
    lego_description_dir = get_package_share_directory('lego_description')
    lego_manipulation_dir = get_package_share_directory('lego_manipulation')

    # Set up robot description parameter
    robot_description_path = os.path.join(
        lego_description_dir, 'urdf', 'robot_with_lego.xacro'
    )

    # Set up RViz configuration path
    rviz_config_path = os.path.join(
        lego_manipulation_dir, 'rviz', 'lego_demo.rviz'
    )

    robot_description = ParameterValue(
        Command(['xacro ', robot_description_path]),
        value_type=str
    )

    # Launch robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Launch joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Launch joint state publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Launch RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Launch LEGO visualizer
    lego_visualizer = Node(
        package='lego_manipulation',
        executable='visualize_lego',
        name='visualize_lego',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        rviz,
        lego_visualizer
    ])
