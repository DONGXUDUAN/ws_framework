#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "interfaces/action/move_arm_joint.hpp"

using namespace std::placeholders;

class MoveArmJointActionClient : public rclcpp::Node
{
public:
  using MoveArmJoint = interfaces::action::MoveArmJoint;
  using GoalHandleMoveArmJoint = rclcpp_action::ClientGoalHandle<MoveArmJoint>;

  explicit MoveArmJointActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("move_arm_joint_client", node_options), goal_sent_(false)
  {
    this->client_ptr_ = rclcpp_action::create_client<MoveArmJoint>(
      this,
      "move_arm_joint");

    // 读取参数
    this->declare_parameter<double>("joint_0", 0.0);
    this->declare_parameter<double>("joint_1", 0.0);
    this->declare_parameter<double>("joint_2", 0.0);
    this->declare_parameter<double>("joint_3", 0.0);
    this->declare_parameter<double>("joint_4", 0.0);
    this->declare_parameter<double>("joint_5", 0.0);
    this->declare_parameter<bool>("is_constrain", false);
    this->declare_parameter<double>("x_axis_tolerance", 0.1);
    this->declare_parameter<double>("y_axis_tolerance", 0.1);
    this->declare_parameter<double>("z_axis_tolerance", 0.1);

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

    auto goal_msg = MoveArmJoint::Goal();
    // 设置目标参数
    this->get_parameter("joint_0", goal_msg.joint_0);
    this->get_parameter("joint_1", goal_msg.joint_1);
    this->get_parameter("joint_2", goal_msg.joint_2);
    this->get_parameter("joint_3", goal_msg.joint_3);
    this->get_parameter("joint_4", goal_msg.joint_4);
    this->get_parameter("joint_5", goal_msg.joint_5);
    this->get_parameter("is_constrain", goal_msg.is_constrain);
    this->get_parameter("x_axis_tolerance", goal_msg.x_axis_tolerance);
    this->get_parameter("y_axis_tolerance", goal_msg.y_axis_tolerance);
    this->get_parameter("z_axis_tolerance", goal_msg.z_axis_tolerance);

    RCLCPP_INFO(this->get_logger(), "发送目标");

    auto send_goal_options = rclcpp_action::Client<MoveArmJoint>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&MoveArmJointActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&MoveArmJointActionClient::result_callback, this, std::placeholders::_1);

    goal_sent_ = false;  // 重置目标状态
    future_goal_handle_ = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);  

    // 添加定时器以检查目标响应
    this->timer_ = this->create_wall_timer(
        5s, std::bind(&MoveArmJointActionClient::check_goal_response, this));
}

void check_goal_response()
{
    if (!goal_sent_) {
        RCLCPP_WARN(this->get_logger(), "未收到目标响应，重新发送目标...");
        this->send_goal();  // 重新发送目标
    }
}

private:
  rclcpp_action::Client<MoveArmJoint>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_sent_;  // 用于标记目标是否已被服务器响应
  std::shared_future<GoalHandleMoveArmJoint::SharedPtr> future_goal_handle_;  // 存储目标的 future 句柄

void goal_response_callback(GoalHandleMoveArmJoint::SharedPtr goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "目标被服务器拒绝");
    } else {
        RCLCPP_INFO(this->get_logger(), "目标已被服务器接受，等待结果");
        goal_sent_ = true;  // 设置为 true，标记目标已发送成功
        timer_->cancel();  // 取消定时器
    }
}

void result_callback(const GoalHandleMoveArmJoint::WrappedResult & result)
{
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        rclcpp::shutdown();
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        rclcpp::shutdown();
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        rclcpp::shutdown();
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

  auto node = std::make_shared<MoveArmJointActionClient>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
