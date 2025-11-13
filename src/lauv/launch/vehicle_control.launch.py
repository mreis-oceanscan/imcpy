import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    pkg_lauv_description = get_package_share_directory('lauv_sim')
    
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='lauv',
        description='Namespace for the robot'
    )
    
    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='interactive',
        description='Control mode: dynamic, service, or interactive'
    )
    
    # Path to the URDF file
    urdf_file = os.path.join(pkg_lauv_description, 'urdf', 'lauv_ros2.xacro')
    
    # Robot State Publisher (for robot description)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file, ' namespace:=', LaunchConfiguration('namespace')]),
            'use_sim_time': False
        }]
    )
    
    # Vehicle position controller (conditional based on mode)
    dynamic_mover = Node(
        package='lauv_sim',
        executable='move_vehicle.py',
        name='vehicle_mover',
        condition=lambda context: context.get_launch_configuration('control_mode') == 'dynamic'
    )
    
    service_controller = Node(
        package='lauv_sim',
        executable='vehicle_position_service.py',
        name='vehicle_position_service',
        condition=lambda context: context.get_launch_configuration('control_mode') == 'service'
    )
    
    interactive_controller = Node(
        package='lauv_sim',
        executable='vehicle_controller.py',
        name='vehicle_controller',
        condition=lambda context: context.get_launch_configuration('control_mode') == 'interactive'
    )
    
    # Joint state demo (for fin/propeller animation)
    joint_demo = Node(
        package='lauv_sim',
        executable='lauv_demo.py',
        name='lauv_demo'
    )
    
    # RViz2
    rviz_config_file = os.path.join(pkg_lauv_description, 'rviz', 'lauv_rviz2.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}]
    )
    
    return LaunchDescription([
        namespace_arg,
        control_mode_arg,
        robot_state_publisher,
        dynamic_mover,
        service_controller,
        interactive_controller,
        joint_demo,
        rviz2
    ])