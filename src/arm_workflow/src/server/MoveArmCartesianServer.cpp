#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces/action/move_arm_cartesian.hpp"  
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include <moveit_msgs/msg/collision_object.hpp>
#include "moveit/planning_scene_interface/planning_scene_interface.h"


using namespace std::chrono_literals;

class MoveArmActionCartesianServer : public rclcpp::Node
{
public:
  using MoveArmCartesian = interfaces::action::MoveArmCartesian;
  using GoalHandleMoveArmCartesian = rclcpp_action::ServerGoalHandle<MoveArmCartesian>;

  MoveArmActionCartesianServer() : Node("move_arm_cartesian_action_server")
  {
    // 创建 Action 服务器
    action_server_ = rclcpp_action::create_server<MoveArmCartesian>(
      this,
      "move_arm_cartesian", 
      std::bind(&MoveArmActionCartesianServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveArmActionCartesianServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveArmActionCartesianServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Move Arm Cartesian Action Server 已启动.");
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
  rclcpp_action::Server<MoveArmCartesian>::SharedPtr action_server_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_; 

  // 处理接收到的目标请求
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveArmCartesian::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "接收到目标请求: 目标位移 - (delta_x: %.2f, delta_y: %.2f, delta_z: %.2f)",
                goal->delta_x,
                goal->delta_y,
                goal->delta_z);
    // 在这里可以添加更多的目标验证逻辑
    (void)uuid;  // 避免未使用警告
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // 处理取消请求
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveArmCartesian> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "收到取消目标请求.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // 当目标被接受时，启动执行
  void handle_accepted(const std::shared_ptr<GoalHandleMoveArmCartesian> goal_handle)
  {
    // 在新的线程中执行，以避免阻塞服务器
    std::thread{std::bind(&MoveArmActionCartesianServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  // 执行目标
  void execute(const std::shared_ptr<GoalHandleMoveArmCartesian> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "开始执行目标...");

    // 获取目标
    const auto goal = goal_handle->get_goal();
    const auto is_constrain = goal->is_constrain;
    const auto delta_x = goal->delta_x;
    const auto delta_y = goal->delta_y;
    const auto delta_z = goal->delta_z;
    const auto x_axis_tolerance = goal->x_axis_tolerance;
    const auto y_axis_tolerance = goal->y_axis_tolerance;
    const auto z_axis_tolerance = goal->z_axis_tolerance;

    // 使用moveit2库中的move_group接口移动机械臂到目标位置
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose("tool0");
    geometry_msgs::msg::Pose start_pose = current_pose.pose;

    RCLCPP_INFO(this->get_logger(), "当前机械臂末端的位置 - Position(x: %.2f, y: %.2f, z: %.2f)",
            start_pose.position.x,
            start_pose.position.y,
            start_pose.position.z);

    waypoints.push_back(start_pose);
    geometry_msgs::msg::Pose end_pose = start_pose;
    end_pose.position.x += delta_x;
    end_pose.position.y += delta_y;
    end_pose.position.z += delta_z;
    waypoints.push_back(end_pose);
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(this->get_logger(), "(Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    if (!is_constrain) {
      RCLCPP_INFO(this->get_logger(), "不约束轨迹的姿态.");
    }else{
      RCLCPP_INFO(this->get_logger(), "约束轨迹的姿态.");
      moveit_msgs::msg::OrientationConstraint ocm;
      ocm.link_name = "tool0";
      ocm.header.frame_id = "base_link";
      ocm.orientation.w = start_pose.orientation.w;
      ocm.orientation.x = start_pose.orientation.x;
      ocm.orientation.y = start_pose.orientation.y;
      ocm.orientation.z = start_pose.orientation.z;
      ocm.absolute_x_axis_tolerance = x_axis_tolerance;
      ocm.absolute_y_axis_tolerance = y_axis_tolerance;
      ocm.absolute_z_axis_tolerance = z_axis_tolerance;
      ocm.weight = 1.0;

      moveit_msgs::msg::Constraints path_constraints;
      path_constraints.orientation_constraints.push_back(ocm);
      move_group_->setPathConstraints(path_constraints);
    }

    auto result = std::make_shared<MoveArmCartesian::Result>();
    // 执行目标
    moveit::planning_interface::MoveItErrorCode exec_status = move_group_->execute(trajectory);
    if (exec_status != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "轨迹执行失败.");
      result->success = false;
      result->message = "机械臂执行轨迹失败";
      move_group_->clearPathConstraints(); // 清除约束
      goal_handle->abort(result);
      return;
    }else
    {
      RCLCPP_INFO(this->get_logger(), "轨迹执行成功。");
      result->success = true;
      result->message = "机械臂执行轨迹成功";
      move_group_->clearPathConstraints(); // 清除约束
      goal_handle->succeed(result);
    }

    // geometry_msgs::msg::PoseStamped now_pose_stamp = move_group_->getCurrentPose("tool0");
    // geometry_msgs::msg::Pose now_pose = now_pose_stamp.pose;
    // RCLCPP_INFO(this->get_logger(), "机械臂到达目标位置 - Position(x: %.2f, y: %.2f, z: %.2f)",
    //     now_pose.position.x,
    //     now_pose.position.y,
    //     now_pose.position.z);
    return;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MoveArmActionCartesianServer>();
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));

  node->init_move_group();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
