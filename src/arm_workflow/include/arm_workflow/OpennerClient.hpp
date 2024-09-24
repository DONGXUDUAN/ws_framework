#ifndef OPERATE_ACTION_HPP
#define OPERATE_ACTION_HPP

#include <map>
#include <memory>
#include "arm_workflow/workflow_step.hpp" 

void execute_openner(std::map<std::string, std::shared_ptr<BaseParameter>> parameters);

#endif  // OPERATE_ACTION_HPP
