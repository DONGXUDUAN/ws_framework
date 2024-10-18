#include "rclcpp/rclcpp.hpp"
#include "gazebo_attach_interfaces/action/attach.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <iostream>

class AttachClient : public rclcpp::Node
{
public:
  using Attach = gazebo_attach_interfaces::action::Attach;
  using GoalHandleAttach = rclcpp_action::ClientGoalHandle<Attach>;

  AttachClient() : Node("attach_client")
  {
    client_ptr_ = rclcpp_action::create_client<Attach>(this, "attach");

    // 声明参数
    this->declare_parameter<std::string>("model_name_1", "ground_plane");
    this->declare_parameter<std::string>("link_name_1", "link");
    this->declare_parameter<std::string>("model_name_2", "ground_plane");
    this->declare_parameter<std::string>("link_name_2", "link");

    // 发送请求
    this->send_goal();
  }

private:
  void send_goal()
  {
    if (!client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action服务未准备好.");
      rclcpp::shutdown();
      return;
    }

    // 创建目标
    auto goal_msg = Attach::Goal();

    // 获取参数
    this->get_parameter("model_name_1", goal_msg.model_name_1);
    this->get_parameter("link_name_1", goal_msg.link_name_1);
    this->get_parameter("model_name_2", goal_msg.model_name_2);
    this->get_parameter("link_name_2", goal_msg.link_name_2);

    RCLCPP_INFO(this->get_logger(), "发送Attach目标请求...");

    // 发送目标请求
    auto send_goal_options = rclcpp_action::Client<Attach>::SendGoalOptions();
    send_goal_options.result_callback =
      std::bind(&AttachClient::result_callback, this, std::placeholders::_1);
    client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void result_callback(const GoalHandleAttach::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Action成功执行");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Action被中止");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Action被取消");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "未知的结果代码");
        return;
    }

    if (result.result->success) {
      RCLCPP_INFO(this->get_logger(), "附着成功: %s", result.result->message.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "附着失败: %s", result.result->message.c_str());
    }

    // 结果返回后关闭节点
    rclcpp::shutdown();
  }

  rclcpp_action::Client<Attach>::SharedPtr client_ptr_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto attach_client = std::make_shared<AttachClient>();
  rclcpp::spin(attach_client);
  return 0;
}
