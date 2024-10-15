#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "interfaces/action/move_arm_cartesian.hpp"

using namespace std::placeholders;

class MoveArmActionClient : public rclcpp::Node
{
public:
  using MoveArmCartesian = interfaces::action::MoveArmCartesian;
  using GoalHandleMoveArmCartesian = rclcpp_action::ClientGoalHandle<MoveArmCartesian>;

  explicit MoveArmActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("move_arm_cartesian_action_client", node_options)
  {
    this->client_ptr_ = rclcpp_action::create_client<MoveArmCartesian>(
      this,
      "move_arm_cartesian");
    // 读取参数
    this->declare_parameter<double>("delta_x", 0.0);
    this->declare_parameter<double>("delta_y", 0.0);
    this->declare_parameter<double>("delta_z", 0.0);
    this->declare_parameter<bool>("is_constrain", false);
    this->declare_parameter<double>("x_axis_tolerance", 0.0);
    this->declare_parameter<double>("y_axis_tolerance", 0.0);
    this->declare_parameter<double>("z_axis_tolerance", 0.0);

    this->send_goal();
  }

void send_goal()
{
    using namespace std::chrono_literals;

    if (!this->client_ptr_->wait_for_action_server(10s)) {
        RCLCPP_ERROR(this->get_logger(), "在等待后操作服务器不可用");
        rclcpp::shutdown();
        return;
    }

    auto goal_msg = MoveArmCartesian::Goal();
    // 设置目标参数
    this->get_parameter("delta_x", goal_msg.delta_x);
    this->get_parameter("delta_y", goal_msg.delta_y);
    this->get_parameter("delta_z", goal_msg.delta_z);
    this->get_parameter("is_constrain", goal_msg.is_constrain);
    this->get_parameter("x_axis_tolerance", goal_msg.x_axis_tolerance);
    this->get_parameter("y_axis_tolerance", goal_msg.y_axis_tolerance);
    this->get_parameter("z_axis_tolerance", goal_msg.z_axis_tolerance);

    RCLCPP_INFO(this->get_logger(), "发送目标");

    auto send_goal_options = rclcpp_action::Client<MoveArmCartesian>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&MoveArmActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&MoveArmActionClient::result_callback, this, std::placeholders::_1);

    auto future_goal_handle = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);  
}

private:
  rclcpp_action::Client<MoveArmCartesian>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

void goal_response_callback(GoalHandleMoveArmCartesian::SharedPtr goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "目标被服务器拒绝");
    } else {
        RCLCPP_INFO(this->get_logger(), "目标已被服务器接受，等待结果");
    }
}


  void result_callback(const GoalHandleMoveArmCartesian::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received");
    RCLCPP_INFO(this->get_logger(), "Success: %s", result.result->success ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Message: %s", result.result->message.c_str());

    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MoveArmActionClient>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
