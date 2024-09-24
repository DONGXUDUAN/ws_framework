#ifndef GAZEBO_ATTACH_PLUGIN__LINK_ATTACHER_PLUGIN_H_
#define GAZEBO_ATTACH_PLUGIN__LINK_ATTACHER_PLUGIN_H_

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <vector>

#include "gazebo_attach_interfaces/srv/attach.hpp"
#include "gazebo_attach_interfaces/srv/detach.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "gazebo_attach_interfaces/srv/set_static.hpp"

namespace gazebo
{
  class LinkAttacherPlugin : public ModelPlugin
  {
  public:
    LinkAttacherPlugin();
    virtual ~LinkAttacherPlugin();

    // 插件加载函数
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  private:
    // 附着服务回调
    void OnAttachRequest(
      const std::shared_ptr<gazebo_attach_interfaces::srv::Attach::Request> request,
      std::shared_ptr<gazebo_attach_interfaces::srv::Attach::Response> response);

    // 分离服务回调
    void OnDetachRequest(
      const std::shared_ptr<gazebo_attach_interfaces::srv::Detach::Request> request,
      std::shared_ptr<gazebo_attach_interfaces::srv::Detach::Response> response);

    // 列出模型和链接的服务回调
    void OnListLinksRequest(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    // 设置is_static的服务回调
    void OnSetStaticRequest(
      const std::shared_ptr<gazebo_attach_interfaces::srv::SetStatic::Request> request,
      std::shared_ptr<gazebo_attach_interfaces::srv::SetStatic::Response> response);

    // Gazebo 物理世界和引擎的指针
    physics::WorldPtr world_;
    physics::PhysicsEnginePtr physics_engine_;

    // ROS2 节点的指针
    gazebo_ros::Node::SharedPtr ros_node_;

    // 服务的共享指针
    rclcpp::Service<gazebo_attach_interfaces::srv::Attach>::SharedPtr attach_service_;
    rclcpp::Service<gazebo_attach_interfaces::srv::Detach>::SharedPtr detach_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr list_links_service_;
    rclcpp::Service<gazebo_attach_interfaces::srv::SetStatic>::SharedPtr set_static_service_;

    // 存储已附着的关节列表
    std::vector<physics::JointPtr> attached_joints_;
  };
}

#endif  // GAZEBO_ATTACH_PLUGIN__LINK_ATTACHER_PLUGIN_H_
