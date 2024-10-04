// move_arm_action_server.cpp

#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces/action/move_arm.hpp"  
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"


using namespace std::chrono_literals;

class MoveArmActionServer : public rclcpp::Node
{
public:
  using MoveArm = interfaces::action::MoveArm;
  using GoalHandleMoveArm = rclcpp_action::ServerGoalHandle<MoveArm>;

  MoveArmActionServer() : Node("move_arm_action_server")
  {
    // 创建 Action 服务器
    action_server_ = rclcpp_action::create_server<MoveArm>(
      this,
      "move_arm",  // Action 名称
      std::bind(&MoveArmActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveArmActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveArmActionServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Move Arm Action Server 已启动.");
  }

  void init_move_group()
  {
    try {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "abb_gofa_arm");
      RCLCPP_INFO(this->get_logger(), "MoveGroupInterface 初始化成功。");
    }
    catch (const std::runtime_error &e) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface 初始化失败: %s", e.what());
      throw;
    }
  }

private:
  rclcpp_action::Server<MoveArm>::SharedPtr action_server_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_; 

  // 处理接收到的目标请求
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveArm::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "接收到目标请求: 目标位姿 - Position(x: %.2f, y: %.2f, z: %.2f)",
                goal->target_pose.position.x,
                goal->target_pose.position.y,
                goal->target_pose.position.z);
    // 在这里可以添加更多的目标验证逻辑
    (void)uuid;  // 避免未使用警告
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // 处理取消请求
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveArm> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "收到取消目标请求.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // 当目标被接受时，启动执行
  void handle_accepted(const std::shared_ptr<GoalHandleMoveArm> goal_handle)
  {
    // 在新的线程中执行，以避免阻塞服务器
    std::thread{std::bind(&MoveArmActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  // 执行目标
  void execute(const std::shared_ptr<GoalHandleMoveArm> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "开始执行目标...");

    // 获取目标
    const auto goal = goal_handle->get_goal();
    const geometry_msgs::msg::Pose target_pose = goal->target_pose;

    // 使用moveit2库中的move_group接口移动机械臂到目标位置
    move_group_->setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    auto result = std::make_shared<MoveArm::Result>();
    if (!success) {
      RCLCPP_WARN(this->get_logger(), "轨迹规划失败。");
      result->success = false;
      result->message = "机械臂规划失败";
      goal_handle->abort(result);
      return;
    }else {
      RCLCPP_INFO(this->get_logger(), "轨迹规划成功。");
    }

    // 执行目标
    moveit::planning_interface::MoveItErrorCode exec_status = move_group_->execute(plan);
    if (exec_status != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "轨迹执行失败.");
      result->success = false;
      result->message = "机械臂执行轨迹失败";
      goal_handle->abort(result);
      return;
    }else
    {
      RCLCPP_INFO(this->get_logger(), "轨迹执行成功。");
      result->success = true;
      result->message = "机械臂执行轨迹成功";
      goal_handle->succeed(result);
    }
    
    // 打印机械臂已到达目标位置的消息
    RCLCPP_INFO(this->get_logger(), "机械臂已到达目标位置: Position(x: %.2f, y: %.2f, z: %.2f, qx: %.2f, qy: %.2f, qz: %.2f, qw: %.2f)",
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z,
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.x,
                target_pose.orientation.w);
    // 成功完成目标
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MoveArmActionServer>();

  node->init_move_group();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
