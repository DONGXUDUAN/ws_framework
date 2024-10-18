cmake_minimum_required(VERSION 3.8)
project(arm_workflow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(gazebo_attach_interfaces REQUIRED)

add_executable(move_arm_server src/server/MoveArmServer.cpp)
ament_target_dependencies(move_arm_server rclcpp rclcpp_action geometry_msgs std_msgs interfaces   
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_ros_move_group)

add_executable(move_arm_client src/client/MoveArmClient.cpp)
ament_target_dependencies(move_arm_client rclcpp rclcpp_action geometry_msgs std_msgs interfaces)

add_executable(move_arm_cartesian_server src/server/MoveArmCartesianServer.cpp)
ament_target_dependencies(move_arm_cartesian_server rclcpp rclcpp_action geometry_msgs std_msgs interfaces   
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_ros_move_group)

add_executable(move_arm_cartesian_client src/client/MoveArmCartesianClient.cpp)
  ament_target_dependencies(move_arm_cartesian_client rclcpp rclcpp_action geometry_msgs std_msgs interfaces)


add_executable(move_arm_joint_server src/server/MoveArmJointServer.cpp)
ament_target_dependencies(move_arm_joint_server rclcpp rclcpp_action geometry_msgs std_msgs interfaces   
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_ros_move_group)

add_executable(move_arm_joint_client src/client/MoveArmJointClient.cpp)
ament_target_dependencies(move_arm_joint_client rclcpp rclcpp_action geometry_msgs std_msgs interfaces)

add_executable(openner_client src/client/OpennerClient.cpp)
ament_target_dependencies(openner_client rclcpp rclcpp_action geometry_msgs std_msgs interfaces control_msgs trajectory_msgs)

add_executable(pipettle_client src/client/PipettleClient.cpp)
ament_target_dependencies(pipettle_client rclcpp rclcpp_action geometry_msgs std_msgs interfaces control_msgs trajectory_msgs)

add_executable(egp64_client src/client/Egp64Client.cpp)
ament_target_dependencies(egp64_client rclcpp rclcpp_action geometry_msgs std_msgs interfaces control_msgs trajectory_msgs)

add_executable(attach_client src/client/AttachClient.cpp)
ament_target_dependencies(attach_client rclcpp rclcpp_action geometry_msgs std_msgs gazebo_attach_interfaces)

add_executable(detach_client src/client/DetachClient.cpp)
ament_target_dependencies(detach_client rclcpp rclcpp_action geometry_msgs std_msgs gazebo_attach_interfaces)

add_executable(work_flow_controller 
  src/WorkFlowController.cpp
  src/utiles.cpp)
ament_target_dependencies(work_flow_controller rclcpp rclcpp_action std_msgs interfaces yaml-cpp)

# Install executables
install(TARGETS
  move_arm_server
  move_arm_cartesian_server
  move_arm_joint_server
  work_flow_controller
  move_arm_client
  move_arm_cartesian_client
  move_arm_joint_client
  openner_client
  pipettle_client
  egp64_client
  attach_client
  detach_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers (if any)
install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY 
    config
    launch
  DESTINATION 
    share/${PROJECT_NAME}
)

ament_package()
