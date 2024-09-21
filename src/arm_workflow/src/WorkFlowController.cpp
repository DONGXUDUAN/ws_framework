// workflow_controller.cpp
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// 引入你的 Action 接口
#include "interfaces/action/move_arm.hpp"
#include "interfaces/action/attach.hpp"
#include "interfaces/action/detach.hpp"
#include "interfaces/action/operate.hpp"

#include "arm_workflow/workflow_step.hpp"
#include <nlohmann/json.hpp>
#include <fstream>

using json = nlohmann::json;

using namespace std::chrono_literals;

class WorkflowController : public rclcpp::Node
{
public:
    WorkflowController() : Node("workflow_controller")
    {
        // 加载工作流程步骤
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("arm_workflow");
        if (!load_workflow_steps(package_share_directory + "/config/workflow_steps.json")) {
            RCLCPP_ERROR(this->get_logger(), "无法加载工作流程步骤配置文件");
            rclcpp::shutdown();
            return;
        }

        // 开始执行工作流程
        this->timer_ = this->create_wall_timer(
            500ms,
            std::bind(&WorkflowController::execute_workflow, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    size_t current_step_index_ = 0;
    std::vector<WorkflowStep> workflow_steps_;

    // 初始化工作流程步骤
    bool load_workflow_steps(const std::string& config_file_path) {
        try {
            // 打开并读取 JSON 文件
            std::ifstream file(config_file_path);
            if (!file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "无法打开配置文件: %s", config_file_path.c_str());
                return false;
            }

            json config;
            file >> config;

            // 检查是否存在 "workflow_steps" 节点
            if (config.find("workflow_steps") == config.end()) {
                RCLCPP_ERROR(this->get_logger(), "配置文件中缺少 'workflow_steps' 节点");
                return false;
            }

            // 遍历 workflow_steps 数组
            for (const auto& step : config["workflow_steps"]) {
                WorkflowStep workflow_step;
                workflow_step.action_name = step["action_name"].get<std::string>();

                // 遍历 parameters 节点并将其加入到 workflow_step 中
                for (const auto& param : step["parameters"].items()) {
                    workflow_step.parameters[param.key()] = param.value().get<std::string>();
                }

                workflow_steps_.push_back(workflow_step);
            }
            return true;
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "JSON解析错误: %s", e.what());
            return false;
        }
        }

    // 执行工作流程
    void execute_workflow()
    {
        if (current_step_index_ >= workflow_steps_.size()) {
            RCLCPP_INFO(this->get_logger(), "工作流程已完成。");
            this->timer_->cancel();
            return;
        }
        WorkflowStep step = workflow_steps_[current_step_index_];
        RCLCPP_INFO(this->get_logger(), "执行步骤 %zu: %s", current_step_index_ + 1, step.action_name.c_str());
        if (step.action_name == "move_arm") {
            execute_move_arm(step.parameters);
            current_step_index_ += 1;
        }
        else if (step.action_name == "attach") {
            execute_attach(step.parameters);
            current_step_index_ += 1;
        }
        else if (step.action_name == "detach") {
            execute_detach(step.parameters);
            current_step_index_ += 1;
        }
        else if (step.action_name == "operate") {
            execute_operate(step.parameters);
            current_step_index_ += 1;
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "未知的Action类型: %s", step.action_name.c_str());
            this->timer_->cancel();
        }
    }
    void execute_move_arm(const std::map<std::string, std::string>& params)
    {
        std::cout << "正在执行move_arm:" << std::endl;
        for (const auto& param : params) {
            std::cout << "Parameter: " << param.first << " = " << param.second << std::endl;
        }
    }

    void execute_attach(const std::map<std::string, std::string>& params)
    {
        std::cout << "正在执行attach:" << std::endl;
        for (const auto& param : params) {
            std::cout << "Parameter: " << param.first << " = " << param.second << std::endl;
        }
    }   

    void execute_detach(const std::map<std::string, std::string>& params)
    {
        std::cout << "正在执行detach:" << std::endl;
        for (const auto& param : params) {
            std::cout << "Parameter: " << param.first << " = " << param.second << std::endl;
        }
    }
    void execute_operate(const std::map<std::string, std::string>& params)
    {
        std::cout << "正在执行operate:" << std::endl;
        for (const auto& param : params) {
            std::cout << "Parameter: " << param.first << " = " << param.second << std::endl;
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WorkflowController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


