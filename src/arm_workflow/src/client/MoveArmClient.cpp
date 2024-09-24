#include "arm_workflow/MoveArmClient.hpp"
#include <iostream>

// 函数实现
void execute_move_arm(const std::map<std::string, std::shared_ptr<BaseParameter>>& parameters)
{
    std::cout << "正在执行move_arm:" << std::endl;
    
    // 动态转换参数为 PoseParameter 类型
    auto* pose_param = dynamic_cast<PoseParameter*>(parameters.at("position").get());
    if (pose_param) {
        // 输出位姿信息
        std::cout << "  Pose: (" << pose_param->x << ", " << pose_param->y << ", " 
                  << pose_param->z << ") 旋转: (" 
                  << pose_param->qx << ", " << pose_param->qy << ", " 
                  << pose_param->qz << ", " << pose_param->qw << ")" << std::endl;
    } else {
        std::cout << "  未知的参数类型 或者参数没有内容" << std::endl;
    }
}
