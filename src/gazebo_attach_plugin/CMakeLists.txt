cmake_minimum_required(VERSION 3.8)
project(gazebo_attach_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_attach_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

add_library(link_attacher_plugin SHARED
  src/link_attacher_plugin.cpp
)

ament_target_dependencies(link_attacher_plugin
  rclcpp
  gazebo_ros
  gazebo_dev
  gazebo_msgs
  gazebo_attach_interfaces
  std_srvs
)

target_link_libraries(link_attacher_plugin
  ${GAZEBO_LIBRARIES}
)

install(TARGETS link_attacher_plugin
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
