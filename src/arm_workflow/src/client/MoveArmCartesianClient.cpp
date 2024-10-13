#include "arm_workflow/MoveArmCartesianClient.hpp"
#include <iostream>

void execute_move_arm_cartesian(const std::map<std::string, std::shared_ptr<BaseParameter>>& parameters)
{
    std::cout << "正在执行move_arm_cartesian:" << std::endl;
    
    // 动态转换参数为 PoseParameter 类型
    auto it_dis = parameters.find("displacement");
    if (it_dis == parameters.end()) {
        std::cout << "  参数中缺少 'displacement' 键." << std::endl;
        return;
    }

    auto it_cons = parameters.find("constrain");
    if (it_cons == parameters.end()) {
        std::cout << "  参数中缺少 'constrain' 键." << std::endl;
        return;
    }

    auto* dis_param = dynamic_cast<CartesianPathParameter*>(it_dis->second.get());
    auto* cons_param = dynamic_cast<ConstrainParameter*>(it_cons->second.get());
    if (dis_param) {

        // 构建命令行字符串
        std::string command = "ros2 action send_goal /move_arm_cartesian interfaces/action/MoveArmCartesian \"{delta_x: ";
        command += std::to_string(dis_param->delta_x) + ", delta_y: " + std::to_string(dis_param->delta_y);
        command += ", delta_z: " + std::to_string(dis_param->delta_z) + ", is_constrain: " ;
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