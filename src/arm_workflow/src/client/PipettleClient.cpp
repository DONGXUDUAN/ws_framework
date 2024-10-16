#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class pipettleActionClient : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  pipettleActionClient()
  : Node("pipettle_action_client")
  {
    client_ptr_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this,
      "/pipettle/pipettle_joint_controllers/follow_joint_trajectory");
    
    this->declare_parameter<double>("pipettle_base2link", -0.01);
    this->send_goal();
  }

private:
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_ptr_;

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
    point.positions = {0.0};  // 在此设置目标关节位置
    this->get_parameter("pipettle_base2link", point.positions[0]);
    point.time_from_start = rclcpp::Duration::from_seconds(1.0);

    // 添加轨迹点到轨迹中
    goal_msg.trajectory.points.push_back(point);

    // 发送目标
    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&pipettleActionClient::result_callback, this, std::placeholders::_1);

    client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void result_callback(const GoalHandleFollowJointTrajectory::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "目标执行成功！");
    } else {
      RCLCPP_ERROR(this->get_logger(), "目标执行失败，代码：%d", static_cast<int>(result.code));
    }
    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pipettleActionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
