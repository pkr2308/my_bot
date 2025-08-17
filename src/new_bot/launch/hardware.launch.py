#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    
    package_name = 'new_bot'
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('rviz', default='true')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyACM0')
    
    # Get package paths
    pkg_path = get_package_share_directory(package_name)
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    
    # Process the URDF file
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # Controller config file path
    controller_config_file = os.path.join(pkg_path, 'config', 'controllers.yaml')
    
    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )
    
    # Controller Manager Node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            robot_description, 
            controller_config_file, 
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('~/robot_description', '/robot_description'),
        ]
    )
    
    # Joint State Broadcaster Spawner (delayed)
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )
    
    # Ackermann Controller Spawner (delayed)
    ackermann_controller_spawner = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['ackermann_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )
    
    # IMU Broadcaster Spawner (delayed)
    imu_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['imu_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )
    
    # Range Sensor Broadcaster Spawner (delayed)
    range_broadcaster_spawner = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['range_sensor_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )
    
    # RPLidar Node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'lidar_frame',
            'inverted': False,
            'angle_compensate': True,
        }]
    )
    
    # Camera Node (PiCamera)
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'pixel_format': 'YUYV',
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link'
        }]
    )
    
    # Robot Localization (EKF) for sensor fusion
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frequency': 30.0,
            'sensor_timeout': 0.1,
            'two_d_mode': True,
            'transform_time_offset': 0.0,
            'transform_timeout': 0.0,
            'print_diagnostics': True,
            'debug': False,
            
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'world_frame': 'odom',
            
            'odom0': '/ackermann_controller/odometry',
            'odom0_config': [False, False, False,
                           False, False, False,
                           True, True, False,
                           False, False, True,
                           False, False, False],
            
            'imu0': '/imu/data',
            'imu0_config': [False, False, False,
                          True, True, True,
                          False, False, False,
                          True, True, True,
                          True, True, True],
        }]
    )
    
    # RViz Node (optional)
    rviz_config_file = os.path.join(pkg_path, 'config', 'robot_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz if true'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Serial port for RP2040 communication'
        ),
        
        # Core nodes
        robot_state_publisher,
        controller_manager,
        
        # Controller spawners (delayed)
        joint_state_broadcaster_spawner,
        ackermann_controller_spawner,
        imu_broadcaster_spawner,
        range_broadcaster_spawner,
        
        # Sensor nodes
        rplidar_node,
        camera_node,
        
        # Localization
        ekf_node,
        
        # Visualization
        rviz_node,
    ])
