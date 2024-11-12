# 项目概述
本项目是一个基于ROS2-humble的机械臂（Gofa CRB 15000）工作流与附件管理解决方案，使用json文件定义工作流程。它包含多个软件包，用于处理机械臂的运动规划、控制以及与Gazebo仿真环境中的交互。核心功能包括通过动作服务器实现机器人臂的移动、gazebo中link的挂载与卸载，以及通过MoveIt进行路径规划。

# 快速开始
## 安装
### 1.安装ROS2-humble
请参考[官方文档](https://docs.ros.org/en/humble/Installation.html)进行安装。

### 2.安装MoveIt
请参考[官方文档](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html)进行安装。

注意，截至2024年10月15日，直接使用`sudo apt install ros-humble-moveit`安装moveit会导致PathConstraint无法使用，若无需使用PathConstraint，直接安装即可。建议参考官方文档使用源码安装。

### 3.安装Gazebo
```bash
sudo apt install gazebo     
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-gazebo-ros-pkgs
```
### 4.安装控制器相关
```bash
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-gripper-controllers
sudo apt install ros-humble-xacro
```
## 测试
### 1.导入模型文件
将`ws_framework/gazebo_models`中的模型文件复制到`/usr/share/gazebo-11/models`目录下。

### 2.编译工作空间
```bash
cd ws_framework
colcon build
source install/setup.bash
```
### 3.启动gazebo仿真环境，以及控制器节点
```bash
ros2 launch abb_description brinch_gofa.launch.py
```
检查并确认所有控制器节点是否启动。
### 4.启动工作流中涉及到的action server
```bash
ros2 launch arm_workflow launch_actions.launch.py
```
### 5.运行工作流脚本
以下三个工作流分别为：机械臂抓取瓶子，执行开瓶操作；机械臂抓去移液枪执行移液操作；机械臂抓取试管执行核磁测试操作。
```bash
ros2 run arm_workflow work_flow_controller --ros-args -p json_name:=egp64
ros2 run arm_workflow work_flow_controller --ros-args -p json_name:=pipettle
ros2 run arm_workflow work_flow_controller --ros-args -p json_name:=tube
```
