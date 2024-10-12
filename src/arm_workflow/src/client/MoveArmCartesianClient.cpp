#include "arm_workflow/MoveArmCartesianClient.hpp"
#include <iostream>

void execute_move_arm_cartesian(const std::map<std::string, std::shared_ptr<BaseParameter>>& parameters)
{
    std::cout << "正在执行move_arm_cartesian:" << std::endl;
    
    // 动态转换参数为 PoseParameter 类型
    auto it_pose = parameters.find("position");
    if (it_pose == parameters.end()) {
        std::cout << "  参数中缺少 'position' 键." << std::endl;
        return;
    }

    auto it_cons = parameters.find("constrain");
    if (it_cons == parameters.end()) {
        std::cout << "  参数中缺少 'constrain' 键." << std::endl;
        return;
    }

    auto* pose_param = dynamic_cast<PoseParameter*>(it_pose->second.get());
    auto* cons_param = dynamic_cast<ConstrainParameter*>(it_cons->second.get());
    if (pose_param) {

        // 构建命令行字符串
        std::string command = "ros2 action send_goal /move_arm interfaces/action/MoveArm \"{target_pose: {position: {x: ";
        command += std::to_string(pose_param->x) + ", y: " + std::to_string(pose_param->y) + ", z: " + std::to_string(pose_param->z) + "}, orientation: {x: ";
        command += std::to_string(pose_param->qx) + ", y: " + std::to_string(pose_param->qy) + ", z: " + std::to_string(pose_param->qz) + ", w: " + std::to_string(pose_param->qw) + "}}" + ", is_constrain: "; 
        command += std::to_string(cons_param->is_constrain) + ", x_axis_tolerance: " + std::to_string(cons_param->x_axis_tolerance) + ", y_axis_tolerance: ";
        command += std::to_string(cons_param->y_axis_tolerance) + ", z_axis_tolerance: " + std::to_string(cons_param->z_axis_tolerance) + "}\"";

        std::cout << "执行命令: " << command << std::endl;

        // 执行命令
        int ret = system(command.c_str());;
        if (ret != 0) {
            std::cerr << "  执行命令失败，返回码: " << ret << std::endl;
        }
    } else {
        std::cout << "  未知的参数类型 或者参数没有内容" << std::endl;
    }
}