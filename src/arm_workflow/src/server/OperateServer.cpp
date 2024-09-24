// OperateActionServer.cpp

#include <memory>
#include <string>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces/action/operate.hpp"  // 确保路径正确

using namespace std::chrono_literals;

class OperateActionServer : public rclcpp::Node
{
public:
  using Operate = interfaces::action::Operate;
  using GoalHandleOperate = rclcpp_action::ServerGoalHandle<Operate>;

  OperateActionServer() : Node("operate_action_server")
  {
    // 创建 Action 服务器
    action_server_ = rclcpp_action::create_server<Operate>(
      this,
      "operate",
      std::bind(&OperateActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&OperateActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&OperateActionServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Operate Openner Action Server 已启动.");
  }

private:
  rclcpp_action::Server<Operate>::SharedPtr action_server_;

  // 处理接收到的目标请求
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Operate::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "接收到操作请求: operation = '%s'", goal->operation.c_str());
    // 在这里可以添加操作验证逻辑
    (void)uuid;  // 避免未使用警告
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // 处理取消请求
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleOperate> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "收到取消操作请求.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // 当目标被接受时，启动执行
  void handle_accepted(const std::shared_ptr<GoalHandleOperate> goal_handle)
  {
    // 在新的线程中执行，以避免阻塞服务器
    std::thread{std::bind(&OperateActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  // 执行目标
  void execute(const std::shared_ptr<GoalHandleOperate> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "开始执行操作...");

    const auto goal = goal_handle->get_goal();
    const std::string operation = goal->operation;

    // 模拟执行过程
    // 这里仅仅是打印操作，不执行实际的操作
    // 你可以在这里添加实际的操作逻辑
    RCLCPP_INFO(this->get_logger(), "执行操作: %s", operation.c_str());

    // 模拟操作所需的时间
    std::this_thread::sleep_for(2s);  // 模拟延迟

    // 打印操作完成的消息
    RCLCPP_INFO(this->get_logger(), "操作 '%s' 已完成。", operation.c_str());

    // 设置结果
    auto result = std::make_shared<Operate::Result>();
    result->success = true;
    result->message = "操作 '" + operation + "' 成功完成。";

    // 成功完成目标
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "目标已完成: %s", result->message.c_str());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<OperateActionServer>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
