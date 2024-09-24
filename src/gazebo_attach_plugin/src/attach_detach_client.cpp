#include <rclcpp/rclcpp.hpp>
#include "gazebo_attach_interfaces/srv/attach.hpp"
#include "gazebo_attach_interfaces/srv/detach.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class AttachDetachClient : public rclcpp::Node
{
public:
  AttachDetachClient() : Node("attach_detach_client")
  {
    attach_client_ = this->create_client<gazebo_attach_interfaces::srv::Attach>("attach");
    detach_client_ = this->create_client<gazebo_attach_interfaces::srv::Detach>("detach");

    // 等待服务可用
    while (!attach_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for attach service...");
    }

    while (!detach_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for detach service...");
    }
  }

  void attach(const std::string &model1, const std::string &link1,
              const std::string &model2, const std::string &link2)
  {
    auto request = std::make_shared<gazebo_attach_interfaces::srv::Attach::Request>();
    request->model_name_1 = model1;
    request->link_name_1 = link1;
    request->model_name_2 = model2;
    request->link_name_2 = link2;

    auto result = attach_client_->async_send_request(request);

    // 等待结果
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      if (result.get()->success)
      {
        RCLCPP_INFO(this->get_logger(), "Successfully attached.");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to attach.");
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Service call failed.");
    }
  }

  void detach(const std::string &model1, const std::string &link1,
              const std::string &model2, const std::string &link2)
  {
    auto request = std::make_shared<gazebo_attach_interfaces::srv::Detach::Request>();
    request->model_name_1 = model1;
    request->link_name_1 = link1;
    request->model_name_2 = model2;
    request->link_name_2 = link2;

    auto result = detach_client_->async_send_request(request);

    // 等待结果
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      if (result.get()->success)
      {
        RCLCPP_INFO(this->get_logger(), "Successfully detached.");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to detach.");
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Service call failed.");
    }
  }

private:
  rclcpp::Client<gazebo_attach_interfaces::srv::Attach>::SharedPtr attach_client_;
  rclcpp::Client<gazebo_attach_interfaces::srv::Detach>::SharedPtr detach_client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto client = std::make_shared<AttachDetachClient>();

  // 示例：附着
  client->attach("robot", "end_effector", "object", "link");

  // 示例：分离
  // client->detach("robot", "end_effector", "object", "link");

  rclcpp::shutdown();
  return 0;
}
