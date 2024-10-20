#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class PipettleActionClient : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  PipettleActionClient()
  : Node("pipettle_action_client"), goal_sent_(false)
  {
    client_ptr_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this,
      "/pipettle/pipettle_joint_controllers/follow_joint_trajectory");

    this->declare_parameter<double>("pipettle_base2link", -0.01);

    // 发送目标
    this->send_goal();
  }

private:
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_sent_;  // 用于跟踪是否发送成功
  std::shared_future<GoalHandleFollowJointTrajectory::SharedPtr> future_goal_handle_;  // 存储异步目标句柄

  void send_goal()
  {
    if (!client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "pipettle_joint_controllers 服务器不可用！");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = FollowJointTrajectory::Goal();

    // 设置关节名称
    goal_msg.trajectory.joint_names = {
      "pipettle_base2link",
    };

    // 创建轨迹点
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.0};  // 设置目标关节位置
    this->get_parameter("pipettle_base2link", point.positions[0]);
    point.time_from_start = rclcpp::Duration::from_seconds(1.0);

    // 添加轨迹点到轨迹中
    goal_msg.trajectory.points.push_back(point);

    // 发送目标
    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&PipettleActionClient::result_callback, this, std::placeholders::_1);

    goal_sent_ = false;  // 重置目标状态
    future_goal_handle_ = client_ptr_->async_send_goal(goal_msg, send_goal_options);

    // 添加定时器以检查目标响应
    timer_ = this->create_wall_timer(
        std::chrono::seconds(5), std::bind(&PipettleActionClient::check_goal_response, this));
  }

  void check_goal_response()
  {
    if (!goal_sent_) {
      RCLCPP_WARN(this->get_logger(), "未收到目标响应，重新发送目标...");
      this->send_goal();  // 重新发送目标
    }
  }

  void result_callback(const GoalHandleFollowJointTrajectory::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "目标执行成功！");
      goal_sent_ = true;  // 标记目标已成功执行
      timer_->cancel();  // 成功后取消定时器
    } else {
      RCLCPP_ERROR(this->get_logger(), "目标执行失败，代码：%d", static_cast<int>(result.code));
    }
    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PipettleActionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
