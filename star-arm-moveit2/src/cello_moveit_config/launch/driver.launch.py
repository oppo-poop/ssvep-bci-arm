from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 启动时是否驱到零位。默认 true —— 一条命令就到位，不必再补一条 home。
    #
    # 2026-11-02 的调整
    #   原来 interval 固定 1500 ms，臂停折叠态（j2/j3 各差 90°）时等于
    #   1.5 秒满速展开，上升到一半失力回落。现在驱动侧改成 3000 ms，
    #   加速段拉平、冲击电流小一半。
    #
    # 想上电完全不动（比如臂停在很奇怪的位置，想先人工摆好）
    #   zero_on_start:=false  → 只上力矩、保持当前姿态；
    #   回零再走 scripts/roarm_cli.py home（MoveIt 规划，更慢但更稳）。
    zero_on_start = LaunchConfiguration("zero_on_start")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "zero_on_start", default_value="true",
                description="启动时是否驱到零位（3 s 慢速；false=只上力矩保持原姿态）"),
            Node(
                package="robo_driver",
                executable="driver",
                name="cello_driver",
                output="screen",
                # 显式声明类型：LaunchConfiguration 求值出来是字符串，
                # 而节点把 zero_on_start 声明成 bool。不指定 value_type 时
                # 依赖 launch_ros 的隐式 yaml 解析，换版本可能就 TypeMismatch。
                parameters=[{"zero_on_start": ParameterValue(
                    zero_on_start, value_type=bool)}],
            ),
            Node(
                package="cello_controller",
                executable="controller",
                name="cello_controller",
                output="screen",
            )
        ]
    )
