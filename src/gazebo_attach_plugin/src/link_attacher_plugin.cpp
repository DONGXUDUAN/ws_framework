#include "gazebo_attach_plugin/link_attacher_plugin.h"

#include <gazebo/transport/transport.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
  GZ_REGISTER_MODEL_PLUGIN(LinkAttacherPlugin)  // 注册插件

  LinkAttacherPlugin::LinkAttacherPlugin() : ModelPlugin() {}

  LinkAttacherPlugin::~LinkAttacherPlugin() {}

  void LinkAttacherPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // 获取世界和物理引擎
    world_ = _model->GetWorld();
    physics_engine_ = world_->Physics();

    // 初始化 ROS 节点
    ros_node_ = gazebo_ros::Node::Get(_sdf);

    // 创建服务
    attach_service_ = ros_node_->create_service<gazebo_attach_interfaces::srv::Attach>(
      "attach",
      std::bind(&LinkAttacherPlugin::OnAttachRequest, this, std::placeholders::_1, std::placeholders::_2));

    detach_service_ = ros_node_->create_service<gazebo_attach_interfaces::srv::Detach>(
      "detach",
      std::bind(&LinkAttacherPlugin::OnDetachRequest, this, std::placeholders::_1, std::placeholders::_2));

      // 创建列出所有链接的服务
    list_links_service_ = ros_node_->create_service<std_srvs::srv::Trigger>(
      "list_links",
      std::bind(&LinkAttacherPlugin::OnListLinksRequest, this, std::placeholders::_1, std::placeholders::_2));

    // 创建自定义 SetStatic 服务
    set_static_service_ = ros_node_->create_service<gazebo_attach_interfaces::srv::SetStatic>(
      "set_static",
      std::bind(&LinkAttacherPlugin::OnSetStaticRequest, this, std::placeholders::_1, std::placeholders::_2));


    RCLCPP_INFO(ros_node_->get_logger(), "LinkAttacherPlugin loaded and services are ready.");
  }

  void LinkAttacherPlugin::OnAttachRequest(
    const std::shared_ptr<gazebo_attach_interfaces::srv::Attach::Request> request,
    std::shared_ptr<gazebo_attach_interfaces::srv::Attach::Response> response)
  {
    // 获取模型
    auto model1 = world_->ModelByName(request->model_name_1);
    auto model2 = world_->ModelByName(request->model_name_2);

    if (!model1 || !model2)
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "One of the models not found.");
      response->success = false;
      return;
    }

    // 获取链接
    auto link1 = model1->GetLink(request->link_name_1);
    auto link2 = model2->GetLink(request->link_name_2);

    if (!link1 || !link2)
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "One of the links not found.");
      response->success = false;
      return;
    }

    // 创建固定关节
    physics::JointPtr joint = physics_engine_->CreateJoint("fixed", model1);
    auto pose1 = link1->WorldPose();
    auto pose2 = link2->WorldPose();
    auto relative_pose = pose2 * pose1.Inverse();

    joint->Load(link1, link2, relative_pose);
    joint->Attach(link1, link2);
    joint->SetModel(model1);
    joint->Init();

    attached_joints_.push_back(joint);

    RCLCPP_INFO(ros_node_->get_logger(), "Attached %s::%s and %s::%s.",
                request->model_name_1.c_str(), request->link_name_1.c_str(),
                request->model_name_2.c_str(), request->link_name_2.c_str());

    response->success = true;
  }

  void LinkAttacherPlugin::OnDetachRequest(
    const std::shared_ptr<gazebo_attach_interfaces::srv::Detach::Request> request,
    std::shared_ptr<gazebo_attach_interfaces::srv::Detach::Response> response)
  {
    for (auto it = attached_joints_.begin(); it != attached_joints_.end(); ++it)
    {
      auto joint = *it;
      auto parent_link = joint->GetParent();
      auto child_link = joint->GetChild();

      if ((parent_link->GetName() == request->link_name_1 && child_link->GetName() == request->link_name_2) ||
          (parent_link->GetName() == request->link_name_2 && child_link->GetName() == request->link_name_1))
      {
        // 分离关节
        joint->Detach();

        // 从已附着关节列表中删除
        attached_joints_.erase(it);
        RCLCPP_INFO(ros_node_->get_logger(), "Detached %s::%s and %s::%s.",
                    request->model_name_1.c_str(), request->link_name_1.c_str(),
                    request->model_name_2.c_str(), request->link_name_2.c_str());

        response->success = true;
        return;
      }
    }

    RCLCPP_WARN(ros_node_->get_logger(), "No joint found between %s::%s and %s::%s.",
                request->model_name_1.c_str(), request->link_name_1.c_str(),
                request->model_name_2.c_str(), request->link_name_2.c_str());

    response->success = false;
  }
  
  void LinkAttacherPlugin::OnListLinksRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::string link_list = "Available models and links:\n";

    // 遍历世界中的所有模型
    for (auto model : world_->Models())
    {
      link_list += "Model: " + model->GetName() + "\n";
      
      // 遍历每个模型的链接
      for (auto link : model->GetLinks())
      {
        link_list += "  Link: " + link->GetName() + "\n";
      }
    }
    // 输出链接信息到日志
    RCLCPP_INFO(ros_node_->get_logger(), link_list.c_str());

    // 返回链接信息
    response->success = true;
    response->message = link_list;
  }

  void LinkAttacherPlugin::OnSetStaticRequest(
  const std::shared_ptr<gazebo_attach_interfaces::srv::SetStatic::Request> request,
  std::shared_ptr<gazebo_attach_interfaces::srv::SetStatic::Response> response)
  {
    auto model = world_->ModelByName(request->model_name);

    if (!model)
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Model %s not found.", request->model_name.c_str());
      response->success = false;
      response->message = "Model not found.";
      return;
    }

    model->SetStatic(request->is_static);
    model->SetWorldPose(model->WorldPose()); // 强制更新模型状态
    RCLCPP_INFO(ros_node_->get_logger(), "Set model %s to static: %s.",
                request->model_name.c_str(), request->is_static ? "true" : "false");

    response->success = true;
    response->message = "Model static state updated.";
  }
}
