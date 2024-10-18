#include "gazebo_attach_plugin/link_attacher_plugin.h"

#include <gazebo/transport/transport.hh>
#include <ignition/math/Pose3.hh>

// Include action messages
#include "gazebo_attach_interfaces/action/attach.hpp"
#include "gazebo_attach_interfaces/action/detach.hpp"
#include "gazebo_attach_interfaces/action/set_static.hpp"

namespace gazebo
{
  GZ_REGISTER_MODEL_PLUGIN(LinkAttacherPlugin)

  LinkAttacherPlugin::LinkAttacherPlugin() : ModelPlugin() {}

  LinkAttacherPlugin::~LinkAttacherPlugin() {}

  void LinkAttacherPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Get the world and physics engine
    world_ = _model->GetWorld();
    physics_engine_ = world_->Physics();

    // Initialize ROS node
    ros_node_ = gazebo_ros::Node::Get(_sdf);

    // Create action servers
    attach_action_server_ = rclcpp_action::create_server<gazebo_attach_interfaces::action::Attach>(
      ros_node_,
      "attach",
      std::bind(&LinkAttacherPlugin::HandleAttachGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&LinkAttacherPlugin::HandleAttachCancel, this, std::placeholders::_1),
      std::bind(&LinkAttacherPlugin::HandleAttachAccepted, this, std::placeholders::_1));

    detach_action_server_ = rclcpp_action::create_server<gazebo_attach_interfaces::action::Detach>(
      ros_node_,
      "detach",
      std::bind(&LinkAttacherPlugin::HandleDetachGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&LinkAttacherPlugin::HandleDetachCancel, this, std::placeholders::_1),
      std::bind(&LinkAttacherPlugin::HandleDetachAccepted, this, std::placeholders::_1));

    set_static_action_server_ = rclcpp_action::create_server<gazebo_attach_interfaces::action::SetStatic>(
      ros_node_,
      "set_static",
      std::bind(&LinkAttacherPlugin::HandleSetStaticGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&LinkAttacherPlugin::HandleSetStaticCancel, this, std::placeholders::_1),
      std::bind(&LinkAttacherPlugin::HandleSetStaticAccepted, this, std::placeholders::_1));

    list_links_action_server_ = rclcpp_action::create_server<gazebo_attach_interfaces::action::ListLinks>(
      ros_node_,
      "list_links",
      std::bind(&LinkAttacherPlugin::HandleListLinksGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&LinkAttacherPlugin::HandleListLinksCancel, this, std::placeholders::_1),
      std::bind(&LinkAttacherPlugin::HandleListLinksAccepted, this, std::placeholders::_1));


    RCLCPP_INFO(ros_node_->get_logger(), "LinkAttacherPlugin loaded and action servers are ready.");
  }

  // Attach Action Server Callbacks
  rclcpp_action::GoalResponse LinkAttacherPlugin::HandleAttachGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const gazebo_attach_interfaces::action::Attach::Goal> goal)
  {
    RCLCPP_INFO(ros_node_->get_logger(), "Received attach goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse LinkAttacherPlugin::HandleAttachCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<gazebo_attach_interfaces::action::Attach>> goal_handle)
  {
    RCLCPP_INFO(ros_node_->get_logger(), "Received request to cancel attach action");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void LinkAttacherPlugin::HandleAttachAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<gazebo_attach_interfaces::action::Attach>> goal_handle)
  {
    std::thread{std::bind(&LinkAttacherPlugin::ExecuteAttach, this, goal_handle)}.detach();
  }

  void LinkAttacherPlugin::ExecuteAttach(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<gazebo_attach_interfaces::action::Attach>> goal_handle)
  {
    auto result = std::make_shared<gazebo_attach_interfaces::action::Attach::Result>();
    const auto goal = goal_handle->get_goal();

    RCLCPP_INFO(ros_node_->get_logger(), "Executing attach action");

    // Get models
    auto model1 = world_->ModelByName(goal->model_name_1);
    auto model2 = world_->ModelByName(goal->model_name_2);

    if (!model1 || !model2)
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "One of the models not found.");
      result->success = false;
      result->message = "One of the models not found.";
      goal_handle->succeed(result);
      return;
    }

    // Get links
    auto link1 = model1->GetLink(goal->link_name_1);
    auto link2 = model2->GetLink(goal->link_name_2);

    if (!link1 || !link2)
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "One of the links not found.");
      result->success = false;
      result->message = "One of the links not found.";
      goal_handle->succeed(result);
      return;
    }

    // Create fixed joint
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
                goal->model_name_1.c_str(), goal->link_name_1.c_str(),
                goal->model_name_2.c_str(), goal->link_name_2.c_str());

    result->success = true;
    result->message = "Attach action completed successfully.";
    goal_handle->succeed(result);
  }

  // Detach Action Server Callbacks
  rclcpp_action::GoalResponse LinkAttacherPlugin::HandleDetachGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const gazebo_attach_interfaces::action::Detach::Goal> goal)
  {
    RCLCPP_INFO(ros_node_->get_logger(), "Received detach goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse LinkAttacherPlugin::HandleDetachCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<gazebo_attach_interfaces::action::Detach>> goal_handle)
  {
    RCLCPP_INFO(ros_node_->get_logger(), "Received request to cancel detach action");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void LinkAttacherPlugin::HandleDetachAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<gazebo_attach_interfaces::action::Detach>> goal_handle)
  {
    std::thread{std::bind(&LinkAttacherPlugin::ExecuteDetach, this, goal_handle)}.detach();
  }

  void LinkAttacherPlugin::ExecuteDetach(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<gazebo_attach_interfaces::action::Detach>> goal_handle)
  {
    auto result = std::make_shared<gazebo_attach_interfaces::action::Detach::Result>();
    const auto goal = goal_handle->get_goal();

    RCLCPP_INFO(ros_node_->get_logger(), "Executing detach action");

    for (auto it = attached_joints_.begin(); it != attached_joints_.end(); ++it)
    {
      auto joint = *it;
      auto parent_link = joint->GetParent();
      auto child_link = joint->GetChild();

      if ((parent_link->GetName() == goal->link_name_1 && child_link->GetName() == goal->link_name_2) ||
          (parent_link->GetName() == goal->link_name_2 && child_link->GetName() == goal->link_name_1))
      {
        // Detach joint
        joint->Detach();

        // Remove from attached joints list
        attached_joints_.erase(it);
        RCLCPP_INFO(ros_node_->get_logger(), "Detached %s::%s and %s::%s.",
                    goal->model_name_1.c_str(), goal->link_name_1.c_str(),
                    goal->model_name_2.c_str(), goal->link_name_2.c_str());

        result->success = true;
        result->message = "Detach action completed successfully.";
        goal_handle->succeed(result);
        return;
      }
    }

    RCLCPP_WARN(ros_node_->get_logger(), "No joint found between %s::%s and %s::%s.",
                goal->model_name_1.c_str(), goal->link_name_1.c_str(),
                goal->model_name_2.c_str(), goal->link_name_2.c_str());

    result->success = false;
    result->message = "No joint found between the specified links.";
    goal_handle->succeed(result);
  }

  // SetStatic Action Server Callbacks
  rclcpp_action::GoalResponse LinkAttacherPlugin::HandleSetStaticGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const gazebo_attach_interfaces::action::SetStatic::Goal> goal)
  {
    RCLCPP_INFO(ros_node_->get_logger(), "Received set_static goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse LinkAttacherPlugin::HandleSetStaticCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<gazebo_attach_interfaces::action::SetStatic>> goal_handle)
  {
    RCLCPP_INFO(ros_node_->get_logger(), "Received request to cancel set_static action");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void LinkAttacherPlugin::HandleSetStaticAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<gazebo_attach_interfaces::action::SetStatic>> goal_handle)
  {
    std::thread{std::bind(&LinkAttacherPlugin::ExecuteSetStatic, this, goal_handle)}.detach();
  }

  void LinkAttacherPlugin::ExecuteSetStatic(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<gazebo_attach_interfaces::action::SetStatic>> goal_handle)
  {
    auto result = std::make_shared<gazebo_attach_interfaces::action::SetStatic::Result>();
    const auto goal = goal_handle->get_goal();

    RCLCPP_INFO(ros_node_->get_logger(), "Executing set_static action");

    auto model = world_->ModelByName(goal->model_name);

    if (!model)
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Model %s not found.", goal->model_name.c_str());
      result->success = false;
      result->message = "Model not found.";
      goal_handle->succeed(result);
      return;
    }

    model->SetStatic(goal->is_static);
    model->SetWorldPose(model->WorldPose()); // Force update model state

    RCLCPP_INFO(ros_node_->get_logger(), "Set model %s to static: %s.",
                goal->model_name.c_str(), goal->is_static ? "true" : "false");

    result->success = true;
    result->message = "Model static state updated.";
    goal_handle->succeed(result);
  }

  // ListLinks Action的回调函数
  rclcpp_action::GoalResponse LinkAttacherPlugin::HandleListLinksGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ListLinksAction::Goal> goal)
  {
    RCLCPP_INFO(ros_node_->get_logger(), "Received list_links goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse LinkAttacherPlugin::HandleListLinksCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ListLinksAction>> goal_handle)
  {
    RCLCPP_INFO(ros_node_->get_logger(), "Received request to cancel list_links action");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void LinkAttacherPlugin::HandleListLinksAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ListLinksAction>> goal_handle)
  {
    std::thread{std::bind(&LinkAttacherPlugin::ExecuteListLinks, this, goal_handle)}.detach();
  }

  void LinkAttacherPlugin::ExecuteListLinks(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ListLinksAction>> goal_handle)
  {
    auto result = std::make_shared<ListLinksAction::Result>();

    RCLCPP_INFO(ros_node_->get_logger(), "Executing list_links action");

    std::vector<std::string> models_and_links;

    // 遍历世界中的所有模型
    for (auto model : world_->Models())
    {
      std::string model_info = "Model: " + model->GetName();
      models_and_links.push_back(model_info);
      
      // 遍历每个模型的链接
      for (auto link : model->GetLinks())
      {
        std::string link_info = "  Link: " + link->GetName();
        models_and_links.push_back(link_info);
      }
    }

    // 输出链接信息到日志
    for (const auto & info : models_and_links)
    {
      RCLCPP_INFO(ros_node_->get_logger(), info.c_str());
    }

    // 返回链接信息
    result->success = true;
    result->message = "ListLinks action completed successfully.";
    result->models_and_links = models_and_links;
    goal_handle->succeed(result);
  }

} // namespace gazebo
