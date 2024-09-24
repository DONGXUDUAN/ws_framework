#ifndef DETACH_ACTION_HPP
#define DETACH_ACTION_HPP

#include <map>
#include <memory>
#include "arm_workflow/workflow_step.hpp" 

void execute_detach(std::map<std::string, std::shared_ptr<BaseParameter>> parameters);

#endif  // DETACH_ACTION_HPP
