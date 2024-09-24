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

add_executable(move_arm_server src/server/MoveArmServer.cpp)
ament_target_dependencies(move_arm_server rclcpp rclcpp_action geometry_msgs std_msgs interfaces)

add_executable(attach_server src/server/AttachServer.cpp)
ament_target_dependencies(attach_server rclcpp rclcpp_action std_msgs interfaces)

add_executable(detach_server src/server/DetachServer.cpp)
ament_target_dependencies(detach_server rclcpp rclcpp_action std_msgs interfaces)

add_executable(operate_server src/server/OperateServer.cpp)
ament_target_dependencies(operate_server rclcpp rclcpp_action std_msgs interfaces)

add_executable(work_flow_controller 
  src/WorkFlowController.cpp
  src/client/MoveArmClient.cpp
  src/client/AttachClient.cpp
  src/client/DetachClient.cpp
  src/client/OpennerClient.cpp)
ament_target_dependencies(work_flow_controller rclcpp rclcpp_action std_msgs interfaces yaml-cpp)

# Install executables
install(TARGETS
  move_arm_server
  attach_server
  detach_server
  operate_server
  work_flow_controller
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
