// LinkActionServer.cpp

#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces/action/attach.hpp"  // 确保路径正确
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class AttachActionServer : public rclcpp::Node
{
public:
  using Attach = interfaces::action::Attach;
  using GoalHandleLink = rclcpp_action::ServerGoalHandle<Attach>;

  AttachActionServer() : Node("attach_action_server")
  {
    // 创建 Action 服务器
    action_server_ = rclcpp_action::create_server<Attach>(
      this,
      "attach_links",  // Action 名称，与客户端对应
      std::bind(&AttachActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&AttachActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&AttachActionServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Link Attach Server 已启动.");
  }

private:
  rclcpp_action::Server<Attach>::SharedPtr action_server_;

  // 处理接收到的目标请求
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Attach::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "接收到目标请求: link1 = %s, link2 = %s",
                goal->link1.c_str(),
                goal->link2.c_str());
    // 在这里可以添加目标验证逻辑
    (void)uuid;  // 避免未使用警告
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // 处理取消请求
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleLink> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "收到取消目标请求.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // 当目标被接受时，启动执行
  void handle_accepted(const std::shared_ptr<GoalHandleLink> goal_handle)
  {
    // 在新的线程中执行，以避免阻塞服务器
    std::thread{std::bind(&AttachActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  // 执行目标
  void execute(const std::shared_ptr<GoalHandleLink> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "开始执行目标...");

    const auto goal = goal_handle->get_goal();
    const std::string link1 = goal->link1;
    const std::string link2 = goal->link2;

    // 模拟将两个链接连接起来的过程
    // 这里只是打印信息，不执行实际的连接
    RCLCPP_INFO(this->get_logger(), "连接链接: %s 和 %s", link1.c_str(), link2.c_str());

    // 设置结果
    auto result = std::make_shared<Attach::Result>();
    result->success = true;
    result->message = "成功连接链接 " + link1 + " 和 " + link2 + "。";

    // 成功完成目标
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "目标已完成: %s", result->message.c_str());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<AttachActionServer>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
