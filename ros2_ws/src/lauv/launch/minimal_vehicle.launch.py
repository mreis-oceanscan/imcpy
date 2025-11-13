#!/usr/bin/env python3

from pathlib import Path
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """
    Minimal launch file for LAUV vehicle system.
    Only launches update_vehicle node and optionally rviz2.
    Skips robot_state_publisher due to missing UUV dependencies.
    """
    
    # Get package directory for RViz config
    pkg_share = FindPackageShare('lauv')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'lauv_ros2.xacro'])
    rviz_file = PathJoinSubstitution([pkg_share, 'rviz', LaunchConfiguration('rviz_config')])

    vehicle_conf = LaunchConfiguration('vehicle_name')
    frequency_conf = LaunchConfiguration('frequency')
    # use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value='lauv_rviz2.rviz',
        description='RViz configuration file name'
    )

    vehicle_name = DeclareLaunchArgument(
            'vehicle_name',
            default_value='lauv_simulator_1',
            description='Vehicle name for IMC connection')

    frequency = DeclareLaunchArgument(
            'frequency',
            default_value='10.0',
            description='Update frequency in Hz')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    update_vehicle = Node(
            package='lauv',
            executable='update_vehicle.py',
            name='update_vehicle',
            output='screen',
            parameters=[{
                'vehicle': vehicle_conf,
                'frequency': LaunchConfiguration('frequency'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'origin': [0.0, 0.0, 0.0]
            }]
            )

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=vehicle_conf,
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file,' namespace:=', vehicle_conf]),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_frequency': LaunchConfiguration('frequency')           
            }],
            output='screen'
            )

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
                }],
            output='screen'
        )

    return LaunchDescription([
        rviz_config,
        vehicle_name,
        frequency,
        use_sim_time_arg,
        update_vehicle,
        robot_state_publisher,
        rviz
    ])