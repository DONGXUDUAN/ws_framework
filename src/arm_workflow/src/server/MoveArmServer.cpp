// move_arm_action_server.cpp

#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces/action/move_arm.hpp"  
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include <moveit_msgs/msg/collision_object.hpp>
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
    const auto is_constrain = goal->is_constrain;
    const auto x_axis_tolerance = goal->x_axis_tolerance;
    const auto y_axis_tolerance = goal->y_axis_tolerance;
    const auto z_axis_tolerance = goal->z_axis_tolerance;

    if (!is_constrain) {
      RCLCPP_INFO(this->get_logger(), "不约束轨迹的姿态.");
    }else{
      RCLCPP_INFO(this->get_logger(), "约束轨迹的姿态.");
      moveit_msgs::msg::OrientationConstraint ocm;
      ocm.link_name = "tool0";
      ocm.header.frame_id = "base_link";
      ocm.orientation.w = target_pose.orientation.w;
      ocm.orientation.x = target_pose.orientation.x;
      ocm.orientation.y = target_pose.orientation.y;
      ocm.orientation.z = target_pose.orientation.z;
      ocm.absolute_x_axis_tolerance = x_axis_tolerance;
      ocm.absolute_y_axis_tolerance = y_axis_tolerance;
      ocm.absolute_z_axis_tolerance = z_axis_tolerance;
      ocm.weight = 1.0;

      moveit_msgs::msg::Constraints path_constraints;
      path_constraints.orientation_constraints.push_back(ocm);
      move_group_->setPathConstraints(path_constraints);
    }

    // 使用moveit2库中的move_group接口移动机械臂到目标位置
    move_group_->setPoseTarget(target_pose);

    const int max_attempts = 10;
    int attempt_count = 0;
    bool plan_success = false;
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    while (attempt_count < max_attempts) {
      bool success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (success) {
        // RCLCPP_INFO(this->get_logger(), "轨迹规划成功。");
        plan_success = true;
        break;
      } else {
        RCLCPP_INFO(this->get_logger(), "轨迹规划失败 %d, 重新尝试...", attempt_count + 1);
        attempt_count++;
      }
    }
    
    auto result = std::make_shared<MoveArm::Result>();
    if (!plan_success) {
      RCLCPP_INFO(this->get_logger(), "重试10次后, 轨迹规划失败。");
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
      RCLCPP_INFO(this->get_logger(), "轨迹执行失败.");
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
    geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose("tool0");
    // 打印机械臂已到达目标位置的消息
    RCLCPP_INFO(this->get_logger(), "机械臂已到达目标位置: Position(x: %.2f, y: %.2f, z: %.2f, qx: %.2f, qy: %.2f, qz: %.2f, qw: %.2f)",
                current_pose.pose.position.x,
                current_pose.pose.position.y,
                current_pose.pose.position.z,
                current_pose.pose.orientation.x,
                current_pose.pose.orientation.y,
                current_pose.pose.orientation.z,
                current_pose.pose.orientation.w);
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
