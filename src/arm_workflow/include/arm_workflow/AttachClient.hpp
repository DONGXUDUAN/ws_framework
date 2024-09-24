#ifndef ATTACH_ACTION_HPP
#define ATTACH_ACTION_HPP

#include <map>
#include <memory>
#include "arm_workflow/workflow_step.hpp" 

void execute_attach(std::map<std::string, std::shared_ptr<BaseParameter>> parameters);

#endif  // ATTACH_ACTION_HPP
