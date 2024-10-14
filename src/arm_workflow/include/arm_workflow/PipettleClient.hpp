#ifndef PIPETTLE_ACTION_HPP
#define PIPETTLE_ACTION_HPP

#include <map>
#include <memory>
#include "arm_workflow/workflow_step.hpp" 

void execute_pipettle(const std::map<std::string, std::shared_ptr<BaseParameter>>& parameters);

#endif  
