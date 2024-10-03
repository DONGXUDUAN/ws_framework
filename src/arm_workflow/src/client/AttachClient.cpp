#include "arm_workflow/AttachClient.hpp"
#include <iostream>

void execute_attach(std::map<std::string, std::shared_ptr<BaseParameter>> parameters)
{
        std::cout << "正在执行 attach:" << std::endl;

    // 查找 "link1" 和 "link2" 参数
    auto it_link1 = parameters.find("link1");
    auto it_link2 = parameters.find("link2");

    if (it_link1 == parameters.end() || it_link2 == parameters.end()) {
        std::cerr << "  参数中缺少 'link1' 或 'link2' 键." << std::endl;
        return;
    }

    // 尝试将参数转换为 StringParameter
    auto* link1_param = dynamic_cast<StringParameter*>(it_link1->second.get());
    auto* link2_param = dynamic_cast<StringParameter*>(it_link2->second.get());

    if (link1_param && link2_param) {
        // 构建命令行字符串
        std::string command = "ros2 action send_goal /attach_links interfaces/action/Attach \"{link1: '";
        command += link1_param->value + "', link2: '" + link2_param->value + "'}\"";

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