// workflow_step.hpp

#ifndef WORKFLOW_STEP_HPP
#define WORKFLOW_STEP_HPP

// 参数中只有string类型的简单版本
#include <string>
#include <map>

struct WorkflowStep {
    std::string action_name; // 例如: "move_arm", "attach", "detach", "operate"
    std::map<std::string, std::string> parameters; // 参数键值对
};

#endif // WORKFLOW_STEP_HPP