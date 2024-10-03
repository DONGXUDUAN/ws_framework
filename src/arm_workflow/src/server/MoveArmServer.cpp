// move_arm_action_server.cpp

#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces/action/move_arm.hpp"  
#include "geometry_msgs/msg/pose.hpp"

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

private:
  rclcpp_action::Server<MoveArm>::SharedPtr action_server_;

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

    // 在这里，你可以添加实际控制机械臂移动的逻辑。
    // 目前，只是模拟执行过程。

    // 模拟机械臂移动到目标位置所需的时间
    std::this_thread::sleep_for(2s);  // 模拟延迟

    // 打印机械臂已到达目标位置的消息
    RCLCPP_INFO(this->get_logger(), "机械臂已到达目标位置: Position(x: %.2f, y: %.2f, z: %.2f)",
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z);

    // 设置结果
    auto result = std::make_shared<MoveArm::Result>();
    result->success = true;
    result->message = "机械臂成功到达目标位置。";

    // 成功完成目标
    goal_handle->succeed(result);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MoveArmActionServer>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
