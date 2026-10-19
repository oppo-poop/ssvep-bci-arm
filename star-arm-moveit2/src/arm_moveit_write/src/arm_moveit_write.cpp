/**
 * arm_moveit_write.cpp
 * 功能：接收位姿指令并控制机械臂运动。
 * 改进点：
 * 1. 修复 setMaxAccelerationScalingFactor(0) 的严重 Bug。
 * 2. 采用【生产者-消费者】模型：回调函数只更新目标，独立线程负责规划执行。
 * 3. 解决回调阻塞导致 MoveIt 无法获取 TF/JointStates 从而规划失败的问题。
 */

#include <memory>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <algorithm>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "robo_interfaces/msg/position_orientation.hpp"
#include "robo_interfaces/msg/set_angle.hpp"
#include "robo_interfaces/msg/gripper_command.hpp"
#include "geometry_msgs/msg/pose.hpp"

using moveit::planning_interface::MoveGroupInterface;

class ArmMoveitControl : public rclcpp::Node
{
public:
  ArmMoveitControl() : Node("arm_moveit_control")
  {
    // 1. 订阅目标位姿
    pose_sub_ = create_subscription<robo_interfaces::msg::PositionOrientation>(
      "position_orientation_topic", 10,
      std::bind(&ArmMoveitControl::pose_callback, this, std::placeholders::_1));
    
    // 2. 订阅夹爪命令
    gripper_sub_ = create_subscription<robo_interfaces::msg::GripperCommand>(
      "gripper_command_topic", 10,
      std::bind(&ArmMoveitControl::gripper_callback, this, std::placeholders::_1));
    
    // 3. 发布舵机角度
    set_angle_pub_ = this->create_publisher<robo_interfaces::msg::SetAngle>("set_angle_topic", 10);

    // 3.5 速度/加速度缩放（显式参数，默认 -1 = 沿用 yaml）
    // 原先这里硬编码 setMaxVelocityScalingFactor(1.0)，会把 joint_limits.yaml 的
    // default_velocity_scaling_factor: 0.1 顶掉，等于满速运行。
    this->declare_parameter<double>("velocity_scaling", -1.0);
    this->declare_parameter<double>("acceleration_scaling", -1.0);

    // 4. 启动控制线程 (MoveIt 操作都在这里进行)
    motion_thread_ = std::thread(&ArmMoveitControl::motion_loop, this);

    RCLCPP_INFO(this->get_logger(), "ArmMoveitControl ready. Listening for commands...");
  }

  ~ArmMoveitControl()
  {
    // 优雅退出
    stop_signal_ = true;
    cv_.notify_all(); // 唤醒沉睡的线程
    if (motion_thread_.joinable()) {
      motion_thread_.join();
    }
  }

private:
  // ROS 通讯组件
  rclcpp::Subscription<robo_interfaces::msg::PositionOrientation>::SharedPtr pose_sub_;
  rclcpp::Subscription<robo_interfaces::msg::GripperCommand>::SharedPtr gripper_sub_;
  rclcpp::Publisher<robo_interfaces::msg::SetAngle>::SharedPtr set_angle_pub_;
  
  // MoveIt 组件 (将在独立线程初始化)
  std::shared_ptr<MoveGroupInterface> move_group_;

  // 线程同步与状态管理
  std::thread motion_thread_;
  std::mutex mutex_;
  std::condition_variable cv_;
  geometry_msgs::msg::Pose latest_target_pose_;
  bool has_new_target_ = false;
  std::atomic<bool> stop_signal_{false};
  std::string last_gripper_state_ = "unknown";

  // 解析缩放因子：显式参数 >= 0 时优先；否则读 joint_limits.yaml 的默认缩放；再兜底 0.1
  double resolve_scaling(const std::string & param_name, const std::string & yaml_key)
  {
    const double explicit_val = this->get_parameter(param_name).as_double();
    if (explicit_val >= 0.0) {
      return std::min(explicit_val, 1.0);
    }
    const auto overrides = this->get_node_parameters_interface()->get_parameter_overrides();
    const auto it = overrides.find(yaml_key);
    if (it != overrides.end() &&
      it->second.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      return std::min(std::max(it->second.get<double>(), 0.0), 1.0);
    }
    return 0.1;
  }

  // --- 回调函数 (生产者) ---
  // 这些函数必须非常快，不能阻塞
  void pose_callback(const robo_interfaces::msg::PositionOrientation::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 转换消息到 Geometry Pose
    latest_target_pose_.position.x = msg->position_x;
    latest_target_pose_.position.y = msg->position_y;
    latest_target_pose_.position.z = msg->position_z;
    latest_target_pose_.orientation.x = msg->orientation_x;
    latest_target_pose_.orientation.y = msg->orientation_y;
    latest_target_pose_.orientation.z = msg->orientation_z;
    latest_target_pose_.orientation.w = msg->orientation_w;
    
    // 标记有新任务
    has_new_target_ = true;
    
    // 唤醒工作线程
    cv_.notify_one(); 
    // RCLCPP_INFO(this->get_logger(), "New target received, added to queue.");
  }

  void gripper_callback(const robo_interfaces::msg::GripperCommand::SharedPtr msg)
  {
    // 不做「与上次相同就跳过」的去重。夹爪是绝对位置命令，重复发同目标是幂等的；
    // 而一旦去重，夹爪没走到位时重发就完全失效（2026-10-18 示教时踩过：
    // 连续发两次映射到同一档的 grip 命令，第二次被静默吞掉，看起来像"没反应"）。
    control_gripper(msg->command);
    last_gripper_state_ = msg->command;
    RCLCPP_INFO(this->get_logger(), "Gripper command: %s", msg->command.c_str());
  }

  void control_gripper(const std::string& command)
  {
    auto msg = robo_interfaces::msg::SetAngle();
    msg.servo_id = {6};
    msg.time = {1000}; 

    // ⚠ 角度换算：joint7_left = (deg - 100) / 100 * 0.032
    //   deg=100 ->  0.000 m  两指合拢（0 mm 间隙），正好是 URDF 上限 upper=0
    //   deg= 50 -> -0.016 m  半开（约 32 mm）
    //   deg= 21.9 -> -0.025 m  完全张开（50 mm），正好是 URDF 下限 lower=-0.025
    //
    // 原来 else 分支发 0.0°，算出 -0.032 m，**超出 URDF 下限 28%**。
    // 夹爪一旦超限，MoveIt 的 planning scene 就把整个机器人状态判为 invalid，
    // OMPL 报 "Skipping invalid start state"，之后所有规划全部失败——
    // 表现为"发任何目标都不动"，且与目标点无关。
    //
    // 注意命令名与物理动作相反（历史遗留，调用方用 GRIP_CMD 表映射）：
    //   "open"  -> 100°  -> 物理闭合
    //   "close" -> 21.9° -> 物理张开
    constexpr double kServoClosed = 100.0;   // 物理：合拢
    constexpr double kServoOpened = 21.9;    // 物理：张开（URDF 下限对应的值，不能再小）

    if (command == "open") {
      msg.target_angle = {kServoClosed};
    } else {
      msg.target_angle = {kServoOpened};
    }
    set_angle_pub_->publish(msg);
  }

  // --- 工作线程 (消费者) ---
  // 所有 MoveIt 的 Plan 和 Execute 都在这里，不会阻塞 ROS 消息接收
  void motion_loop()
  {
    // 等待节点完全启动
    rclcpp::sleep_for(std::chrono::seconds(1));

    // 初始化 MoveGroup
    // 使用 shared_from_this() 确保和 Node 绑定
    move_group_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "arm");

    // 配置参数
    move_group_->setPlanningTime(10.0);

    // 缩放因子解析顺序：显式参数(>=0) > joint_limits.yaml 的 default_*_scaling_factor > 0.1
    const double vel_scale =
      resolve_scaling("velocity_scaling",
                      "robot_description_planning.default_velocity_scaling_factor");
    const double acc_scale =
      resolve_scaling("acceleration_scaling",
                      "robot_description_planning.default_acceleration_scaling_factor");
    move_group_->setMaxVelocityScalingFactor(vel_scale);
    move_group_->setMaxAccelerationScalingFactor(acc_scale);
    move_group_->setNumPlanningAttempts(10);
    move_group_->allowReplanning(true);
    RCLCPP_INFO(this->get_logger(),
                "Scaling resolved: velocity=%.3f acceleration=%.3f", vel_scale, acc_scale);

    geometry_msgs::msg::Pose current_goal;

    while (rclcpp::ok() && !stop_signal_) {
      
      // 1. 等待新目标
      {
        std::unique_lock<std::mutex> lock(mutex_);
        // 等待条件：收到停止信号 或者 有新目标
        cv_.wait(lock, [this]{ return stop_signal_ || has_new_target_; });

        if (stop_signal_) break;

        // 取出最新目标，并重置标志位
        // 这样做的好处是：如果在机械臂运动期间来了10个新点，
        // 等它动完，我们只取这10个点里最新的那个，忽略中间的，实现“最新点跟随”
        current_goal = latest_target_pose_;
        has_new_target_ = false;
      }

      // 2. 执行运动规划
      // 归一化四元数 (防止非法数据导致规划器崩溃)
      double norm = std::sqrt(
        current_goal.orientation.x*current_goal.orientation.x + 
        current_goal.orientation.y*current_goal.orientation.y + 
        current_goal.orientation.z*current_goal.orientation.z + 
        current_goal.orientation.w*current_goal.orientation.w);
      
      if (std::abs(norm - 1.0) > 1e-3 && norm > 1e-6) {
        current_goal.orientation.x /= norm;
        current_goal.orientation.y /= norm;
        current_goal.orientation.z /= norm;
        current_goal.orientation.w /= norm;
      } else if (norm <= 1e-6) {
        current_goal.orientation.w = 1.0; // 默认
      }

      move_group_->setStartStateToCurrentState();
      move_group_->setPoseTarget(current_goal);

      // 使用 Move() 它是 Plan + Execute 的封装
      // 如果你想只在规划成功时移动，可以用 plan() then execute()
      // moveit::planning_interface::MoveGroupInterface::Plan plan;
      // if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      //   move_group_->execute(plan);
      // }
      
      // 直接 Move，简单有效，失败会返回错误码但不会崩溃
      auto error_code = move_group_->move();

      if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Move executed successfully.");
      } else {
        RCLCPP_WARN(this->get_logger(), "Move failed with error code: %d", error_code.val);
      }
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ArmMoveitControl>();
  
  // 使用 MultiThreadedExecutor，这对 MoveIt 非常重要
  // 它允许节点在执行回调的同时，MoveIt 可以在后台接收 TF 和 JointStates
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}