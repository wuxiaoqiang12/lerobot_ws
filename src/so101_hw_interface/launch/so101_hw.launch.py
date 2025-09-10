from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

# --- 1. 添加这个 import ---
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('so101_hw_interface')

    # 声明 use_sim launch 参数，并设置默认值为 false
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation (Gazebo) or real hardware')

    # --- 2. 将 Command(...) 包装在 ParameterValue 中 ---
    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("lerobot_description"),
                        "urdf",
                        "so101.urdf.xacro", # 使用顶层 xacro 文件
                    ]
                ),
                " ",
                "use_sim:=",
                LaunchConfiguration('use_sim'), # 从 launch 参数获取值
            ]
        ),
        value_type=str # 明确告诉 launch 系统最终的值是一个字符串
    )
    robot_description = {"robot_description": robot_description_content}

    # 这个节点就是你之前写的 motor_bridge, 现在作为硬件通信的桥梁
    motor_bridge_node = Node(
        package="so101_hw_interface",
        executable="so101_motor_bridge",
        name="so101_motor_bridge",
        output="screen"
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            os.path.join(pkg_share, "config", "so101_controllers.yaml"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        use_sim_arg, # 将参数添加到 launch description
        robot_state_publisher,
        controller_manager,
        motor_bridge_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
    ])