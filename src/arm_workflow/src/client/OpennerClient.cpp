#include "arm_workflow/OpennerClient.hpp"
#include <iostream>

void execute_openner(const std::map<std::string, std::shared_ptr<BaseParameter>> parameters) {
    std::cout << "正在进行开瓶操作:" << std::endl;

    auto it_open = parameters.find("openner_joints");
    if (it_open == parameters.end()) {
        std::cout << "  参数中缺少 'openner_joints' 键." << std::endl;
        return;
    }

    auto* open_param = dynamic_cast<OpennerJointParameter*>(it_open->second.get());
    if (open_param) {
        // 构建命令行字符串
        std::string command = "ros2 action send_goal /openner/openner_joint_controllers/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \"{ trajectory: { joint_names: [";
        command += "'base2link', 'link2base', 'left_gripper2base', 'right_gripper2base'], points: [{ positions: [";
        command += std::to_string(open_param->base2link) + ", " + std::to_string(open_param->link2base) + ", ";
        command += std::to_string(open_param->left_gripper2base) + ", " + std::to_string(open_param->right_gripper2base) + "]";
        command += ", time_from_start: {sec: 2, nanosec: 0} }] } }\"";

        // 执行命令
        int ret = system(command.c_str());;
        if (ret != 0) {
            std::cerr << "  执行命令失败，返回码: " << ret << std::endl;
        }
    } else {
        std::cout << "  未知的参数类型 或者参数没有内容" << std::endl;
    }
}
