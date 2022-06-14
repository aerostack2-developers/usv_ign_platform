from os.path import join

import launch
from launch import LaunchDescription, conditions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = join(
        get_package_share_directory('usv_ign_platform'),
        'config',
        'control_modes.yaml'
    )
    config_controller = join(
        get_package_share_directory('usv_ign_platform'),
        'config',
        'default_controller.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='drone0'),
        DeclareLaunchArgument('mass', default_value='1.0'),
        DeclareLaunchArgument('sensors', default_value='none'),
        DeclareLaunchArgument('control_modes_file', default_value=config),
        DeclareLaunchArgument('controller_config_file', default_value=config_controller),
        
        Node(
            package="usv_ign_platform",
            executable="usv_ign_platform_node",
            namespace=LaunchConfiguration('drone_id'),
            output="screen",
            emulate_tty=True,
            parameters=[
                {"control_modes_file": LaunchConfiguration('control_modes_file'),
                "controller_config_file": LaunchConfiguration('controller_config_file'), 
                "sensors": LaunchConfiguration('sensors')
                }],
            remappings=[("sensor_measurements/odometry", "self_localization/odom")]
        ),
    ])
