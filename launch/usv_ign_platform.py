from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare

import subprocess


def get_sensors(drone_namespace):
    sensors  = set()

    cmd = f"ign topic -l"
    output = subprocess.run(cmd.split(), capture_output=True, text=True)
    for line in output.stdout.split('\n'):
        if f"{drone_namespace}/model" in line:
            tokens = line.split('/')
            sensors.add(f"{tokens[2]},{tokens[4]},{tokens[6]},{tokens[8]},{tokens[10]}")
    return repr(sensors).replace("', '", ":")[2:-2] if len(sensors) else ""


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
            "sensors": get_sensors(drone_namespace)
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
        DeclareLaunchArgument('control_modes_file', default_value=config),
        DeclareLaunchArgument('controller_config_file', default_value=controller_config_file),
        
        OpaqueFunction(function=get_platform_node)
    ])
