#include "rclcpp/rclcpp.hpp"
#include "gazebo_attach_interfaces/srv/attach.hpp"
#include <memory>
#include <iostream>

class AttachClient : public rclcpp::Node
{
public:
  AttachClient() : Node("attach_client")
  {
    client_ = this->create_client<gazebo_attach_interfaces::srv::Attach>("attach");

    // 提取参数
    model_name_1_ = this->declare_parameter<std::string>("model_name_1", "default_model_1");
    link_name_1_ = this->declare_parameter<std::string>("link_name_1", "default_link_1");
    model_name_2_ = this->declare_parameter<std::string>("model_name_2", "default_model_2");
    link_name_2_ = this->declare_parameter<std::string>("link_name_2", "default_link_2");

    // 发送请求
    this->send_request();
  }

private:
  void send_request()
  {
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "服务未准备好.");
      return;
    }

    auto request = std::make_shared<gazebo_attach_interfaces::srv::Attach::Request>();
    request->model_name_1 = model_name_1_;
    request->link_name_1 = link_name_1_;
    request->model_name_2 = model_name_2_;
    request->link_name_2 = link_name_2_;

    auto result_future = client_->async_send_request(request);

    // 等待响应
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto result = result_future.get();
      if (result->success) {
        RCLCPP_INFO(this->get_logger(), "模型 %s 的 %s 链接已与模型 %s 的 %s 链接附着成功.",
                    model_name_1_.c_str(), link_name_1_.c_str(),
                    model_name_2_.c_str(), link_name_2_.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "附着失败");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "服务调用失败");
    }
  }

  std::string model_name_1_;
  std::string link_name_1_;
  std::string model_name_2_;
  std::string link_name_2_;
  rclcpp::Client<gazebo_attach_interfaces::srv::Attach>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto attach_client = std::make_shared<AttachClient>();
  rclcpp::spin(attach_client);  // 进入事件循环
  rclcpp::shutdown();
  return 0;
}
