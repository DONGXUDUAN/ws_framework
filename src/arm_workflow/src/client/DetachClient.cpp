#include "arm_workflow/DetachClient.hpp"
#include <iostream>

void execute_detach(std::map<std::string, std::shared_ptr<BaseParameter>> parameters) {
    std::cout << "正在执行detach:" << std::endl;
    auto* link1_param = dynamic_cast<StringParameter*>(parameters["link1"].get());
    auto* link2_param = dynamic_cast<StringParameter*>(parameters["link2"].get());
    if (link1_param && link2_param) {
        std::cout << "  解开: " << link1_param->value << " 和 " << link2_param->value << std::endl;
    } else {
        std::cout << "  未知的参数类型 或者参数没有内容" << std::endl;
    }
}
