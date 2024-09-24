#ifndef MOVE_ARM_ACTION_HPP
#define MOVE_ARM_ACTION_HPP

#include <map>
#include <memory>
#include <string>
#include "arm_workflow/workflow_step.hpp"

// 函数声明
void execute_move_arm(const std::map<std::string, std::shared_ptr<BaseParameter>>& parameters);

#endif // MOVE_ARM_ACTION_HPP
