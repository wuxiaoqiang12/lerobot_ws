import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # 1. 加载 MoveIt 配置
    moveit_config = (
        MoveItConfigsBuilder("so101", package_name="lerobot_moveit")
        .to_moveit_configs()
    )

    # 2. 加载我们修改后的 Servo YAML 文件
    servo_yaml = load_yaml("so101_servo", "config/so101_servo_config.yaml")
    # 将加载的参数封装在 'moveit_servo' 命名空间下
    servo_params = {"moveit_servo": servo_yaml}

    # 3. 启动 MoveIt Servo 核心节点
    # 这是最关键的修正：我们传递的是 Python 字典，而不是文件路径
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,  # <-- 传递字典
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
        output="screen",
    )

    # 4. 启动 joy 节点
    joy_node = Node(package="joy", executable="joy_node", name="joy_node", parameters=[{'use_sim_time': True}],)
    
    # 5. 启动我们的 Python 脚本 (这个不需要改动)
    joy_to_servo_node = Node(
        package="so101_servo",
        executable="joy_to_servo_node",
        name="joy_to_servo_node",
        parameters=[{'use_sim_time': True}],
        # 这是修正的关键部分
        remappings=[
            ("delta_twist_cmds", "/servo_node/delta_twist_cmds")
        ]
    )

    return LaunchDescription([
        servo_node,
        joy_node,
        joy_to_servo_node,
    ])