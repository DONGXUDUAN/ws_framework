#include "arm_workflow/MoveArmJointClient.hpp"
#include <iostream>

void execute_move_arm_joint(const std::map<std::string, std::shared_ptr<BaseParameter>>& parameters)
{
    std::cout << "正在执行move_arm_cartesian:" << std::endl;
    
    // 动态转换参数为 PoseParameter 类型
    auto it_dis = parameters.find("joints_value");
    if (it_dis == parameters.end()) {
        std::cout << "  参数中缺少 'joints_value' 键." << std::endl;
        return;
    }

    auto it_cons = parameters.find("constrain");
    if (it_cons == parameters.end()) {
        std::cout << "  参数中缺少 'constrain' 键." << std::endl;
        return;
    }

    auto* dis_param = dynamic_cast<JointPathParameter*>(it_dis->second.get());
    auto* cons_param = dynamic_cast<ConstrainParameter*>(it_cons->second.get());
    if (dis_param) {

        // 构建命令行字符串
        std::string command = "ros2 action send_goal /move_arm_joint interfaces/action/MoveArmJoint \"{joint_0: ";
        command += std::to_string(dis_param->joint_0) + ", joint_1: " + std::to_string(dis_param->joint_1) + ", joint_2: ";
        command += std::to_string(dis_param->joint_2) + ", joint_3: " + std::to_string(dis_param->joint_3) + ", joint_4: ";
        command += std::to_string(dis_param->joint_4) + ", joint_5: " + std::to_string(dis_param->joint_5) + ", is_constrain: " ;
        command += std::to_string(cons_param->is_constrain) + ", x_axis_tolerance: " + std::to_string(cons_param->x_axis_tolerance) + ", y_axis_tolerance: ";
        command += std::to_string(cons_param->y_axis_tolerance) + ", z_axis_tolerance: " + std::to_string(cons_param->z_axis_tolerance) + "}\"";

        // 执行命令
        int ret = system(command.c_str());;
        if (ret != 0) {
            std::cerr << "  执行命令失败，返回码: " << ret << std::endl;
        }
    } else {
        std::cout << "  未知的参数类型 或者参数没有内容" << std::endl;
    }
}