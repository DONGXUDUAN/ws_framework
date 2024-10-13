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
#include "arm_workflow/MoveArmClient.hpp" 
#include "arm_workflow/AttachClient.hpp" 
#include "arm_workflow/DetachClient.hpp"
#include "arm_workflow/OpennerClient.hpp"
#include "arm_workflow/MoveArmCartesianClient.hpp"
#include <nlohmann/json.hpp>
#include <fstream>

using json = nlohmann::json;

using namespace std::chrono_literals;

class WorkflowController : public rclcpp::Node
{
public:
    WorkflowController(const rclcpp::NodeOptions & options) : Node("workflow_controller", options)
    {
        // 加载工作流程步骤
        this->declare_parameter<std::string>("json_name", "default_value");
        std::string json_name = this->get_parameter("json_name").as_string();
        std::cout << "json_name: " << json_name << std::endl;

        std::string package_share_directory = ament_index_cpp::get_package_share_directory("arm_workflow");
        std::string config_file_path = package_share_directory + "/config/workflow_steps_" + json_name + ".json";

        if (!load_workflow_steps(config_file_path)) {
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
                for (const auto& param_pair : step["parameters"].items()) {
                const std::string& param_name = param_pair.key();
                const nlohmann::json& param_json = param_pair.value();
                std::string type = param_json["type"];
                    if (type == "PoseParameter") {
                        double x = param_json["x"].get<double>();
                        double y = param_json["y"].get<double>();
                        double z = param_json["z"].get<double>();
                        double qx = param_json["qx"].get<double>();
                        double qy = param_json["qy"].get<double>();
                        double qz = param_json["qz"].get<double>();
                        double qw = param_json["qw"].get<double>();
                        workflow_step.parameters[param_name] = std::make_shared<PoseParameter>(x, y, z, qx, qy, qz, qw);
                    }else if (type == "ConstrainParameter") {
                        bool is_constrain = param_json["is_constrain"].get<bool>();
                        double x_axis_tolerance = param_json["x_axis_tolerance"].get<double>();
                        double y_axis_tolerance = param_json["y_axis_tolerance"].get<double>();
                        double z_axis_tolerance = param_json["z_axis_tolerance"].get<double>();
                        workflow_step.parameters[param_name] = std::make_shared<ConstrainParameter>(is_constrain, x_axis_tolerance, y_axis_tolerance, z_axis_tolerance);
                    }else if (type == "CartesianPathParameter") {
                        double delta_x = param_json["delta_x"].get<double>();
                        double delta_y = param_json["delta_y"].get<double>();
                        double delta_z = param_json["delta_z"].get<double>();
                        workflow_step.parameters[param_name] = std::make_shared<CartesianPathParameter>(delta_x, delta_y, delta_z);
                    }else if (type== "StringParameter") {
                        std::string value = param_json["value"];
                        workflow_step.parameters[param_name] = std::make_shared<StringParameter>(value);
                    }else {
                        RCLCPP_ERROR(this->get_logger(), "未知参数类型: %s", type.c_str());
                    }
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
            rclcpp::shutdown();
            return;
        }
        WorkflowStep step = workflow_steps_[current_step_index_];
        RCLCPP_INFO(this->get_logger(), "执行步骤 %zu: %s", current_step_index_ + 1, step.action_name.c_str());
        if (step.action_name == "move_arm") {
            execute_move_arm(step.parameters);
            current_step_index_ += 1;
        }
        else if (step.action_name == "move_arm_cartesian") {
            execute_move_arm_cartesian(step.parameters);
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
            execute_openner(step.parameters);
            current_step_index_ += 1;
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "未知的Action类型: %s", step.action_name.c_str());
            this->timer_->cancel();
        }
    } 

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args"});
    auto node = std::make_shared<WorkflowController>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

