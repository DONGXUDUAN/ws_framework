#include "arm_workflow/AttachClient.hpp"
#include <iostream>

void execute_attach(std::map<std::string, std::shared_ptr<BaseParameter>> parameters)
{
        std::cout << "正在执行 attach:" << std::endl;

    // 查找 "model_1" 和 "model_2" 参数
    auto it_model_1 = parameters.find("model_1");
    auto it_model_2 = parameters.find("model_2");

    if (it_model_1 == parameters.end() || it_model_2 == parameters.end()) {
        std::cerr << "  参数中缺少 'mdoel_1' 或 'model_2' 键." << std::endl;
        return;
    }

    // 查找 "link1" 和 "link2" 参数
    auto it_link1 = parameters.find("link_1");
    auto it_link2 = parameters.find("link_2");

    if (it_link1 == parameters.end() || it_link2 == parameters.end()) {
        std::cerr << "  参数中缺少 'link1' 或 'link2' 键." << std::endl;
        return;
    }


    // 尝试将参数转换为 StringParameter
    auto* model_1_param = dynamic_cast<StringParameter*>(it_model_1->second.get());
    auto* model_2_param = dynamic_cast<StringParameter*>(it_model_2->second.get());
    auto* link1_param = dynamic_cast<StringParameter*>(it_link1->second.get());
    auto* link2_param = dynamic_cast<StringParameter*>(it_link2->second.get());

    if (link1_param && link2_param) {
        // 构建命令行字符串
        std::string command = "ros2 service call /attach gazebo_attach_interfaces/srv/Attach \"{model_name_1: ";
        command += model_1_param->value + ", link_name_1: " + link1_param->value + ", model_name_2: ";
        command += model_2_param->value + ", link_name_2: " + link2_param->value + "}\"";

        std::cout << "  执行命令: " << command << std::endl;

        // 执行命令
        int ret = system(command.c_str());
        if (ret != 0) {
            std::cerr << "  执行命令失败，返回码: " << ret << std::endl;
        }
    } else {
        std::cerr << "  参数 'link1' 或 'link2' 类型无效或为空." << std::endl;
    }
}