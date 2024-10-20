#include "rclcpp/rclcpp.hpp"
#include "gazebo_attach_interfaces/action/detach.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <iostream>

class DetachClient : public rclcpp::Node
{
public:
  using Detach = gazebo_attach_interfaces::action::Detach;
  using GoalHandleDetach = rclcpp_action::ClientGoalHandle<Detach>;

  DetachClient() : Node("detach_client"), goal_sent_(false)
  {
    client_ptr_ = rclcpp_action::create_client<Detach>(this, "detach");

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
    auto goal_msg = Detach::Goal();

    // 获取参数
    this->get_parameter("model_name_1", goal_msg.model_name_1);
    this->get_parameter("link_name_1", goal_msg.link_name_1);
    this->get_parameter("model_name_2", goal_msg.model_name_2);
    this->get_parameter("link_name_2", goal_msg.link_name_2);

    RCLCPP_INFO(this->get_logger(), "发送Detach目标请求...");

    // 发送目标请求
    auto send_goal_options = rclcpp_action::Client<Detach>::SendGoalOptions();
    send_goal_options.result_callback =
      std::bind(&DetachClient::result_callback, this, std::placeholders::_1);

    this->goal_sent_ = false;  // 重置状态
    future_goal_handle_ = client_ptr_->async_send_goal(goal_msg, send_goal_options);

    // 添加定时器以检查目标响应
    this->timer_ = this->create_wall_timer(
      std::chrono::seconds(5), std::bind(&DetachClient::check_goal_response, this));
  }

  void check_goal_response()
  {
    if (!goal_sent_) {
      RCLCPP_WARN(this->get_logger(), "未收到目标响应，重新发送目标...");
      this->send_goal();  // 重新发送目标
    }
  }

  void result_callback(const GoalHandleDetach::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Action成功执行");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Action被中止");
        rclcpp::shutdown();
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Action被取消");
        rclcpp::shutdown();
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "未知的结果代码");
        rclcpp::shutdown();
        return;
    }

    if (result.result->success) {
      RCLCPP_INFO(this->get_logger(), "分离成功: %s", result.result->message.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "分离失败: %s", result.result->message.c_str());
    }

    // 成功收到结果后取消定时器并关闭节点
    goal_sent_ = true;
    timer_->cancel();
    rclcpp::shutdown();
  }

  rclcpp_action::Client<Detach>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_future<GoalHandleDetach::SharedPtr> future_goal_handle_;  // 存储目标的 future 句柄
  bool goal_sent_;  // 用于标记目标是否已被服务器响应
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto detach_client = std::make_shared<DetachClient>();
  rclcpp::spin(detach_client);
  return 0;
}
