#include "arm_workflow/MoveArmClient.hpp"
#include <iostream>

void execute_move_arm(const std::map<std::string, std::shared_ptr<BaseParameter>>& parameters)
{
    std::cout << "正在执行move_arm:" << std::endl;
    
    // 动态转换参数为 PoseParameter 类型
    auto it = parameters.find("position");
    if (it == parameters.end()) {
        std::cout << "  参数中缺少 'position' 键." << std::endl;
        return;
    }

    auto* pose_param = dynamic_cast<PoseParameter*>(it->second.get());
    if (pose_param) {
        // 构建命令行字符串
        std::string command = "ros2 action send_goal /move_arm interfaces/action/MoveArm \"{target_pose: {position: {x: ";
        command += std::to_string(pose_param->x) + ", y: " + std::to_string(pose_param->y) + ", z: " + std::to_string(pose_param->z) + "}, orientation: {x: ";
        command += std::to_string(pose_param->qx) + ", y: " + std::to_string(pose_param->qy) + ", z: " + std::to_string(pose_param->qz) + ", w: " + std::to_string(pose_param->qw) + "}}}\"";

        std::cout << "执行命令: " << command << std::endl;

        // 执行命令
        int ret = system(command.c_str());
        if (ret != 0) {
            std::cerr << "  执行命令失败，返回码: " << ret << std::endl;
        }
    } else {
        std::cout << "  未知的参数类型 或者参数没有内容" << std::endl;
    }
}