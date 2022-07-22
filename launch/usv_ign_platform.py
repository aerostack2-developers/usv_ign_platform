from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare

import subprocess


def get_world():
    
    cmd = f"ign topic -l"
    output = subprocess.run(cmd.split(), capture_output=True, text=True)
    for line in output.stdout.split('\n'):
        words = line.split('/')
        if len(words) != 5:
            continue
        
        if (words[1] == 'world' and words[3] == 'pose' and words[4] == 'info'):
            world_name = words[2]
            return world_name
    return ""


def get_platform_node(context, *args, **kwargs):
    drone_namespace = LaunchConfiguration('drone_id').perform(context)

    node = Node(
        package="usv_ign_platform",
        executable="usv_ign_platform_node",
        namespace=LaunchConfiguration('drone_id'),
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('controller_config_file'),
            {"control_modes_file": LaunchConfiguration('control_modes_file'),
            "simulation_mode": True,
            "mass": LaunchConfiguration('mass'),
            "max_thrust": LaunchConfiguration('max_thrust'),
            "min_thrust":  LaunchConfiguration('min_thrust'),
            "imu_topic": LaunchConfiguration('imu_topic'),
            "use_odom_plugin": LaunchConfiguration('use_odom_plugin'),
            "use_ground_truth": LaunchConfiguration('use_ground_truth'),
            "world": get_world()
            }]
    )
    return [node]


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('usv_ign_platform'),
        'config', 'control_modes.yaml'
    ])
    
    controller_config_file = PathJoinSubstitution([
        FindPackageShare('usv_ign_platform'),
        'config', 'default_controller.yaml'
    ])
    
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value=EnvironmentVariable('usv')),
        DeclareLaunchArgument('mass', default_value='1.5'),
        DeclareLaunchArgument('max_thrust', default_value='15.0'),
        DeclareLaunchArgument('min_thrust', default_value='0.15'),
        DeclareLaunchArgument('control_modes_file', default_value=config),
        DeclareLaunchArgument('controller_config_file', default_value=controller_config_file),
        DeclareLaunchArgument('imu_topic', default_value='imu/data'),
        DeclareLaunchArgument('use_odom_plugin', default_value='false'),
        DeclareLaunchArgument('use_ground_truth', default_value='true'),
        
        OpaqueFunction(function=get_platform_node)
    ])
