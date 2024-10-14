#include "arm_workflow/Egp64Client.hpp"
#include <iostream>

void execute_egp64(const std::map<std::string, std::shared_ptr<BaseParameter>>& parameters)
{
    std::cout << "正在控制egp64:" << std::endl;
    
    // 动态转换参数为 PoseParameter 类型
    auto it_joints = parameters.find("egp64_joints");
    if (it_joints == parameters.end()) {
        std::cout << "  参数中缺少 'egp64_joints' 键." << std::endl;
        return;
    }

    auto* joints_param = dynamic_cast<Egp64JointParameter*>(it_joints->second.get());
    if (joints_param) {

        // 构建命令行字符串
        std::string command = "ros2 action send_goal /egp64/egp64_joint_controllers/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \"{ trajectory: { joint_names: [";
        command += "'egp64_finger_left_joint', 'egp64_finger_right_joint'], points: [{ positions: [";
        command += std::to_string(joints_param->egp64_finger_left_joint) + ", " + std::to_string(joints_param->egp64_finger_right_joint) + "]";
        command += ", time_from_start: {sec: 1, nanosec: 0} }] } }\"";
        // 执行命令
        int ret = system(command.c_str());;
        if (ret != 0) {
            std::cerr << "  执行命令失败，返回码: " << ret << std::endl;
        }
    } else {
        std::cout << "  未知的参数类型 或者参数没有内容" << std::endl;
    }
}