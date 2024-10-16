#ifndef UTILES_HPP
#define UTILES_HPP

#include <map>
#include <memory>
#include <string>
#include "arm_workflow/workflow_step.hpp"

// 函数声明
bool execute_move_arm(const std::map<std::string, std::shared_ptr<BaseParameter>>& parameters);
bool execute_move_arm_cartesian(const std::map<std::string, std::shared_ptr<BaseParameter>>& parameters);
bool execute_move_arm_joint(const std::map<std::string, std::shared_ptr<BaseParameter>>& parameters);
bool execute_openner(const std::map<std::string, std::shared_ptr<BaseParameter>> parameters);
bool execute_pipettle(const std::map<std::string, std::shared_ptr<BaseParameter>>& parameters);
bool execute_egp64(const std::map<std::string, std::shared_ptr<BaseParameter>>& parameters);
bool execute_attach(std::map<std::string, std::shared_ptr<BaseParameter>> parameters);
#endif 
