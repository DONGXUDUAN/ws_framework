#include "arm_workflow/AttachClient.hpp"
#include <iostream>

void execute_attach(std::map<std::string, std::shared_ptr<BaseParameter>> parameters) {
    std::cout << "正在执行attach:" << std::endl;
    auto* link1_param = dynamic_cast<StringParameter*>(parameters["link1"].get());
    auto* link2_param = dynamic_cast<StringParameter*>(parameters["link2"].get());
    if (link1_param && link2_param) {
        std::cout << "  连接: " << link1_param->value << " 和 " << link2_param->value << std::endl;
    } else {
        std::cout << "  未知的参数类型 或者参数没有内容" << std::endl;
    }
}
