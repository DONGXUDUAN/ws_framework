#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces/action/move_arm_joint.hpp"  
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include <moveit_msgs/msg/collision_object.hpp>
#include "moveit/planning_scene_interface/planning_scene_interface.h"


using namespace std::chrono_literals;

class MoveArmActionJointServer : public rclcpp::Node
{
public:
  using MoveArmJoint = interfaces::action::MoveArmJoint;
  using GoalHandleMoveArmJoint = rclcpp_action::ServerGoalHandle<MoveArmJoint>;

  MoveArmActionJointServer() : Node("move_arm_joint_action_server")
  {
    // 创建 Action 服务器
    action_server_ = rclcpp_action::create_server<MoveArmJoint>(
      this,
      "move_arm_joint", 
      std::bind(&MoveArmActionJointServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveArmActionJointServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveArmActionJointServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Move Arm Joint Action Server 已启动.");
  }

  void init_move_group()
  {
    try {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "abb_gofa_arm");
      RCLCPP_INFO(this->get_logger(), "MoveGroupInterface for Cartesian 初始化成功。");
    }
    catch (const std::runtime_error &e) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface for Cartesian 初始化失败: %s", e.what());
      throw;
    }
  }

private:
  rclcpp_action::Server<MoveArmJoint>::SharedPtr action_server_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_; 

  // 处理接收到的目标请求
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveArmJoint::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "接收到目标请求: 目标关节角 - (joint_0: %.2f, joint_0: %.2f, joint_0: %.2f, joint_3: %.2f, joint_4: %.2f, joint_5: %.2f)",
                goal->joint_0,
                goal->joint_1,
                goal->joint_2,
                goal->joint_3,
                goal->joint_4,
                goal->joint_5);
    // 在这里可以添加更多的目标验证逻辑
    (void)uuid;  // 避免未使用警告
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // 处理取消请求
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveArmJoint> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "收到取消目标请求.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // 当目标被接受时，启动执行
  void handle_accepted(const std::shared_ptr<GoalHandleMoveArmJoint> goal_handle)
  {
    // 在新的线程中执行，以避免阻塞服务器
    std::thread{std::bind(&MoveArmActionJointServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  // 执行目标
  void execute(const std::shared_ptr<GoalHandleMoveArmJoint> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "开始执行目标...");

    // 获取目标
    const auto goal = goal_handle->get_goal();
    const auto is_constrain = goal->is_constrain;
    const auto joint_0 = goal->joint_0;
    const auto joint_1 = goal->joint_1;
    const auto joint_2 = goal->joint_2;
    const auto joint_3 = goal->joint_3;
    const auto joint_4 = goal->joint_4;
    const auto joint_5 = goal->joint_5;
    const auto x_axis_tolerance = goal->x_axis_tolerance;
    const auto y_axis_tolerance = goal->y_axis_tolerance;
    const auto z_axis_tolerance = goal->z_axis_tolerance;

    // 使用moveit2库中的move_group接口移动机械臂到目标位置
    geometry_msgs::msg::PoseStamped now_pose_stamp = move_group_->getCurrentPose("tool0");
    geometry_msgs::msg::Pose now_pose = now_pose_stamp.pose;

    const moveit::core::JointModelGroup* joint_model_group = move_group_->getCurrentState()->getJointModelGroup("abb_gofa_arm");
    moveit::core::RobotStatePtr current_state = move_group_->getCurrentState(5);
    std::vector<double> joint_group_positions;
    if (current_state)
    {
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
      // 设置关节目标，修改每个关节的目标值
      joint_group_positions[0] = joint_0; 
      joint_group_positions[1] = joint_1;  
      joint_group_positions[2] = joint_2;  
      joint_group_positions[3] = joint_3; 
      joint_group_positions[4] = joint_4; 
      joint_group_positions[5] = joint_5; 
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "获取机械臂当前状态失败");
      rclcpp::shutdown();
      return;
    }

    if (!is_constrain) {
      RCLCPP_INFO(this->get_logger(), "不约束轨迹的姿态.");
    }else{
      RCLCPP_INFO(this->get_logger(), "约束轨迹的姿态.");
      moveit_msgs::msg::OrientationConstraint ocm;
      ocm.link_name = "tool0";
      ocm.header.frame_id = "base_link";
      ocm.orientation.w = now_pose.orientation.w;
      ocm.orientation.x = now_pose.orientation.x;
      ocm.orientation.y = now_pose.orientation.y;
      ocm.orientation.z = now_pose.orientation.z;
      ocm.absolute_x_axis_tolerance = x_axis_tolerance;
      ocm.absolute_y_axis_tolerance = y_axis_tolerance;
      ocm.absolute_z_axis_tolerance = z_axis_tolerance;
      ocm.weight = 1.0;

      moveit_msgs::msg::Constraints path_constraints;
      path_constraints.orientation_constraints.push_back(ocm);
      move_group_->setPathConstraints(path_constraints);
    }

    move_group_->setJointValueTarget(joint_group_positions);

    const int max_attempts = 20;
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
    
    auto result = std::make_shared<MoveArmJoint::Result>();
    // 执行目标
    moveit::planning_interface::MoveItErrorCode exec_status = move_group_->execute(plan);
    if (exec_status != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "重试20次后, 轨迹执行失败.");
      result->success = false;
      result->message = "机械臂执行轨迹失败";
      goal_handle->abort(result);
      move_group_->clearPathConstraints(); // 清除约束
      return;
    }else
    {
      RCLCPP_INFO(this->get_logger(), "轨迹执行成功。");
      result->success = true;
      result->message = "机械臂执行轨迹成功";
      goal_handle->succeed(result);
      move_group_->clearPathConstraints(); // 清除约束
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MoveArmActionJointServer>();
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));

  node->init_move_group();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
