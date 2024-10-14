#ifndef EGP64_ACTION_HPP
#define EGP64_ACTION_HPP

#include <map>
#include <memory>
#include "arm_workflow/workflow_step.hpp" 

void execute_egp64(const std::map<std::string, std::shared_ptr<BaseParameter>>& parameters);

#endif  
