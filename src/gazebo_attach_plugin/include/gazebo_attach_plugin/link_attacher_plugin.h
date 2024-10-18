#ifndef LINK_ATTACHER_PLUGIN_H
#define LINK_ATTACHER_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo/physics/physics.hh>

// 包含Action消息头文件
#include "gazebo_attach_interfaces/action/attach.hpp"
#include "gazebo_attach_interfaces/action/detach.hpp"
#include "gazebo_attach_interfaces/action/set_static.hpp"
#include "gazebo_attach_interfaces/action/list_links.hpp"

#include <rclcpp_action/rclcpp_action.hpp>

namespace gazebo
{
  class LinkAttacherPlugin : public ModelPlugin
  {
  public:
    LinkAttacherPlugin();
    virtual ~LinkAttacherPlugin();

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  private:
    // 定义Action类型别名
    using AttachAction = gazebo_attach_interfaces::action::Attach;
    using DetachAction = gazebo_attach_interfaces::action::Detach;
    using SetStaticAction = gazebo_attach_interfaces::action::SetStatic;
    using ListLinksAction = gazebo_attach_interfaces::action::ListLinks;

    // Action服务器成员变量
    rclcpp_action::Server<AttachAction>::SharedPtr attach_action_server_;
    rclcpp_action::Server<DetachAction>::SharedPtr detach_action_server_;
    rclcpp_action::Server<SetStaticAction>::SharedPtr set_static_action_server_;
    rclcpp_action::Server<ListLinksAction>::SharedPtr list_links_action_server_;

    // Attach Action的回调函数
    rclcpp_action::GoalResponse HandleAttachGoal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const AttachAction::Goal> goal);

    rclcpp_action::CancelResponse HandleAttachCancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<AttachAction>> goal_handle);

    void HandleAttachAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<AttachAction>> goal_handle);

    void ExecuteAttach(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<AttachAction>> goal_handle);

    // Detach Action的回调函数
    rclcpp_action::GoalResponse HandleDetachGoal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const DetachAction::Goal> goal);

    rclcpp_action::CancelResponse HandleDetachCancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<DetachAction>> goal_handle);

    void HandleDetachAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<DetachAction>> goal_handle);

    void ExecuteDetach(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<DetachAction>> goal_handle);

    // SetStatic Action的回调函数
    rclcpp_action::GoalResponse HandleSetStaticGoal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const SetStaticAction::Goal> goal);

    rclcpp_action::CancelResponse HandleSetStaticCancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetStaticAction>> goal_handle);

    void HandleSetStaticAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetStaticAction>> goal_handle);

    void ExecuteSetStatic(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<SetStaticAction>> goal_handle);

    // ListLinks Action的回调函数
    rclcpp_action::GoalResponse HandleListLinksGoal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const ListLinksAction::Goal> goal);

    rclcpp_action::CancelResponse HandleListLinksCancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ListLinksAction>> goal_handle);

    void HandleListLinksAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ListLinksAction>> goal_handle);

    void ExecuteListLinks(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ListLinksAction>> goal_handle);

    // ROS节点
    gazebo_ros::Node::SharedPtr ros_node_;

    // Gazebo指针
    physics::WorldPtr world_;
    physics::PhysicsEnginePtr physics_engine_;

    // 已附着的关节列表
    std::vector<physics::JointPtr> attached_joints_;
  };
}

#endif // LINK_ATTACHER_PLUGIN_H
