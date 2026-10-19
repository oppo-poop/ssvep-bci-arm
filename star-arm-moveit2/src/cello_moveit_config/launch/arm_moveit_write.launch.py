from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Set MoveIt configuration for cello robot
    moveit_config = MoveItConfigsBuilder(
        "cello_description", package_name="cello_moveit_config"
    ).to_moveit_configs()

    # 2026-10-17：原来这里把 config 目录当成参数文件传给节点（会报
    # "Parameter file path is not a file"），实际没起作用，已移除。
    # 速度快慢由节点参数控制：-1 = 沿用 joint_limits.yaml 的 default_*_scaling_factor。
    dec_vel = DeclareLaunchArgument(
        'velocity_scaling', default_value='-1.0',
        description='速度缩放 0~1；-1 = 沿用 joint_limits.yaml')
    dec_acc = DeclareLaunchArgument(
        'acceleration_scaling', default_value='-1.0',
        description='加速度缩放 0~1；-1 = 沿用 joint_limits.yaml')

    # Create and return the launch description
    return LaunchDescription(
        [
            dec_vel,
            dec_acc,
            Node(
                package="arm_moveit_write",
                executable="arm_moveit_write",
                name="arm_moveit_control",
                output="screen",
                parameters=[
                    moveit_config.robot_description,  # Load URDF
                    moveit_config.robot_description_semantic,  # Load SRDF
                    moveit_config.robot_description_kinematics,  # Load kinematics.yaml
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                    {
                        'velocity_scaling': LaunchConfiguration('velocity_scaling'),
                        'acceleration_scaling': LaunchConfiguration('acceleration_scaling'),
                    },
                ],
                # Remap topics if needed
                # remappings=[
                #     ("/position_orientation_topic", "/custom_position_orientation_topic"),
                #     ("/gripper_command_topic", "/custom_gripper_command_topic"),
                # ],
            )
        ]
    )