#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='10.0',
        description='Node update frequency in Hz'
    )
    
    ports_arg = DeclareLaunchArgument(
        'ports',
        default_value='[6004]',
        description='List of port numbers for localhost connections'
    )
    
    namespace_prefix_arg = DeclareLaunchArgument(
        'namespace_prefix',
        default_value='lauv',
        description='Vehicle namespace prefix (e.g., lauv -> lauv1, lauv2, etc.)'
    )
    
    # Node definition
    update_vehicle_node = Node(
        package='lauv_sim',
        executable='update_vehicle.py',
        name='vehicle_updater',
        parameters=[{
            'frequency': LaunchConfiguration('frequency'),
            'ports': LaunchConfiguration('ports'),
            'namespace_prefix': LaunchConfiguration('namespace_prefix')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        frequency_arg,
        ports_arg,
        namespace_prefix_arg,
        update_vehicle_node
    ])