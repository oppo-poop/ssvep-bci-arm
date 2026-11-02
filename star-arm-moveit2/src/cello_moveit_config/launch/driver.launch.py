from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # 启动时是否驱到零位。默认 false —— 只保持当前姿态，不做任何运动。
    #
    # 为什么改了这个默认值
    #   原来 driver 一启动就 arm_move_by_time() 驱到零位，interval 固定 1.5 s。
    #   臂若停在折叠态（j2/j3 各差 90°），等于 1.5 秒内满速展开整臂；
    #   2026-11-02 展开到一半整臂失力摔了一次。
    #
    # 需要回零怎么办
    #   用 scripts/roarm_cli.py home —— 那是 MoveIt 规划的路径，速度受
    #   velocity_scaling 控制，比"上电就冲"安全得多。
    #   task 流程也会自己先回位（sequence 第一步就是 go_home）。
    #
    # 想恢复旧行为：给 driver.launch.py 传 zero_on_start:=true，
    # 或用 robot_up.sh 的 ZERO_ON_START=true。
    zero_on_start = LaunchConfiguration("zero_on_start")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "zero_on_start", default_value="false",
                description="启动时是否驱到零位（true=旧行为，臂会大幅运动）"),
            Node(
                package="robo_driver",
                executable="driver",
                name="cello_driver",
                output="screen",
                parameters=[{"zero_on_start": zero_on_start}],
            ),
            Node(
                package="cello_controller",
                executable="controller",
                name="cello_controller",
                output="screen",
            )
        ]
    )
