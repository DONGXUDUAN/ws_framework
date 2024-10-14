#ifndef MOVE_ARM_ACTION_JOINT_HPP
#define MOVE_ARM_ACTION_JOINT_HPP

#include <map>
#include <memory>
#include <string>
#include "arm_workflow/workflow_step.hpp"

// 函数声明
void execute_move_arm_joint(const std::map<std::string, std::shared_ptr<BaseParameter>>& parameters);

#endif 
