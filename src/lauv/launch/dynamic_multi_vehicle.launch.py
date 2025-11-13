#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def create_vehicle_nodes(context, *args, **kwargs):
    """Dynamically create robot_state_publisher nodes based on ports parameter"""
    
    # Get launch configuration values
    ports_str = LaunchConfiguration('ports').perform(context)
    namespace_prefix = LaunchConfiguration('namespace_prefix').perform(context)
    
    # Parse ports list (simple parsing for demonstration)
    # In production, you might want more robust parsing
    try:
        # Remove brackets and split by comma
        ports_clean = ports_str.strip('[]')
        if ports_clean:
            ports = [int(p.strip()) for p in ports_clean.split(',')]
        else:
            ports = [6004]  # Default
    except:
        ports = [6004]  # Fallback
    
    # Find package directory
    pkg_share = FindPackageShare('lauv_sim').find('lauv_sim')
    urdf_file = os.path.join(pkg_share, 'urdf', 'lauv_ros2.xacro')
    
    # Create nodes for each vehicle
    nodes = []
    
    for i, port in enumerate(ports, 1):
        vehicle_name = f"{namespace_prefix}{i}"
        
        # Robot state publisher for this vehicle
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=vehicle_name,
            parameters=[{
                'robot_description': {
                    'type': 'xacro',
                    'file': urdf_file,
                    'args': {'namespace': vehicle_name}
                },
                'use_sim_time': False,
                'publish_frequency': 30.0
            }],
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
            ],
            output='screen'
        )
        
        nodes.append(robot_state_publisher)
    
    return nodes

def generate_launch_description():
    # Declare launch arguments
    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='10.0',
        description='Node update frequency in Hz'
    )
    
    ports_arg = DeclareLaunchArgument(
        'ports',
        default_value='[6004, 6005]',
        description='List of port numbers for localhost connections'
    )
    
    namespace_prefix_arg = DeclareLaunchArgument(
        'namespace_prefix',
        default_value='lauv',
        description='Vehicle namespace prefix (e.g., lauv -> lauv1, lauv2, etc.)'
    )
    
    # Vehicle updater node
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
    
    # Dynamically create vehicle nodes
    vehicle_nodes = OpaqueFunction(function=create_vehicle_nodes)
    
    return LaunchDescription([
        frequency_arg,
        ports_arg,
        namespace_prefix_arg,
        update_vehicle_node,
        vehicle_nodes
    ])