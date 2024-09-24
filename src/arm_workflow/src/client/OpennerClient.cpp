#include "arm_workflow/OpennerClient.hpp"
#include <iostream>

void execute_openner(std::map<std::string, std::shared_ptr<BaseParameter>> parameters) {
    std::cout << "正在执行operate:" << std::endl;
    auto* start_flag = dynamic_cast<StringParameter*>(parameters["start_flag"].get());
    if (start_flag) {
        std::cout << "  接收到的指令是: " << start_flag->value << std::endl;
    } else {
        std::cout << "  未知的参数类型 或者参数没有内容" << std::endl;
    }
}
