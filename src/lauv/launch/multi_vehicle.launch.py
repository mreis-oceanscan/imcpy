#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution
import os

# Configuration: Vehicle names and their corresponding ports
VEHICLE_CONFIG = [
    {'name': 'lauv-simulator-1', 'port': 6004},
    {'name': 'lauv-simulator-2', 'port': 6005},
    # {'name': 'lauv-simulator-3', 'port': 6006},
]

def generate_launch_description():
    # Declare launch arguments
    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='10.0',
        description='Node update frequency in Hz'
    )
    
    # Declare origin argument
    origin_arg = DeclareLaunchArgument(
    'origin',
    default_value='0.0, 0.0, 0.0',
    description='Origin for the vehicle'
    )

    # Find package directory
    pkg_share = FindPackageShare('lauv_sim')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'lauv_ros2.xacro'])
    
    # Create nodes for each vehicle
    nodes = []
    
    for i, vehicle_config in enumerate(VEHICLE_CONFIG):
        vehicle_name = vehicle_config['name']
        port = vehicle_config['port']
        namespace = vehicle_name.replace('-', '_')  # Convert to ROS2-friendly namespace
        color = 'blue' if i % 2 == 0 else 'red'  # Alternate colors for vehicles
        
        # Vehicle updater node for this vehicle
        update_vehicle_node = Node(
            package='lauv_sim',
            executable='update_vehicle.py',
            name=f'vehicle_updater_{i+1}',
            parameters=[{
                'frequency': LaunchConfiguration('frequency'),
                'port': port,
                'vehicle': vehicle_name
            }],
            output='screen'
        )
        
        # Robot state publisher for this vehicle
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'robot_state_publisher_{i+1}',
            namespace=namespace,
            parameters=[{
                'robot_description': Command([
                    'xacro ', urdf_file,
                    f' namespace:={namespace}',
                    f' color_scheme:={color}'
                ]),
                'use_sim_time': True,
                'publish_frequency': LaunchConfiguration('frequency')
            }],
            output='screen'
        )

        nodes.extend([update_vehicle_node, robot_state_publisher])

    # RViz2 for multi-vehicle visualization
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'multi_vehicle.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        frequency_arg,
        origin_arg,
        *nodes,
        rviz_node
    ])