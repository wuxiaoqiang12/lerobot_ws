import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation (Gazebo) or real hardware')

    use_sim_time = LaunchConfiguration('use_sim')

    # 1. Load MoveIt config
    moveit_config = (
        MoveItConfigsBuilder("so101", package_name="lerobot_moveit")
        .to_moveit_configs()
    )

    # 2. Load Servo YAML file
    servo_yaml = yaml.safe_load(open(os.path.join(get_package_share_directory('so101_servo'), 'config', 'so101_servo_config.yaml')))
    servo_params = {"moveit_servo": servo_yaml}

    # 3. Start the core MoveIt Servo node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': use_sim_time},
        ],
        output="screen",
    )

    # 4. Get the absolute path to the joystick config file
    joy_config_path = PathJoinSubstitution(
        [FindPackageShare("so101_servo"), "config", "so101_xbox_config.yaml"]
    )

    # 5. Start the joy node
    joy_node = Node(
        package="joy", 
        executable="joy_node", 
        name="joy_node", 
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # 6. Start the Python script
    joy_to_servo_node = Node(
        package="so101_servo",
        executable="joy_to_servo_node",
        name="joy_to_servo_node",
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[("delta_twist_cmds", "/servo_node/delta_twist_cmds")]
    )

    return LaunchDescription([
        use_sim_arg,
        servo_node,
        joy_node,
        joy_to_servo_node,
    ])