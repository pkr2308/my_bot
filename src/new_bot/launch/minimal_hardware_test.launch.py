#!/usr/bin/env python3
import launch
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the minimal hardware test launch description."""
    
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("new_bot"), "description", "robot.urdf.xacro"]),
    ])
    
    robot_description = {"robot_description": robot_description_content}
    
    robot_controllers = PathJoinSubstitution([
        FindPackageShare("new_bot"), "config", "controllers.yaml"
    ])
    
    # Robot state publisher for robot description
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )
    
    # Controller manager for hardware interface
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
        remappings=[("~/robot_description", "/robot_description")],
    )
    
    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    return launch.LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
    ])
