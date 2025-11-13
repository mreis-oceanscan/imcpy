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
        default_value='lauv-sim',
        description='Namespace for the robot'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Path to the URDF file
    urdf_file = os.path.join(pkg_lauv_description, 'urdf', 'lauv_ros2.xacro')

    # RViz2 for visualization
    rviz_config_file = os.path.join(pkg_lauv_description, 'rviz', 'lauv_rviz2.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        # namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file, 
                                          ' namespace:=', LaunchConfiguration('namespace')
                                          ]),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Vehicle mover node (for dynamic vehicle positioning)
    vehicle_updater = Node(
        package='lauv_sim',
        executable='update_vehicle.py',
        name='vehicle_updater',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        robot_state_publisher,
        vehicle_updater,
        rviz2
    ])