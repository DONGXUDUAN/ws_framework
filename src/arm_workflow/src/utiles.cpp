#include "arm_workflow/utiles.hpp"
#include <iostream>

bool execute_move_arm(const std::map<std::string, std::shared_ptr<BaseParameter>>& parameters)
{
    std::cout << "正在执行move_arm:" << std::endl;
    
    // 动态转换参数为 PoseParameter 类型
    auto it_pose = parameters.find("position");
    if (it_pose == parameters.end()) {
        std::cout << "  参数中缺少 'position' 键." << std::endl;
        return false;
    }

    auto it_cons = parameters.find("constrain");
    if (it_cons == parameters.end()) {
        std::cout << "  参数中缺少 'constrain' 键." << std::endl;
        return false;
    }

    auto* pose_param = dynamic_cast<PoseParameter*>(it_pose->second.get());
    auto* cons_param = dynamic_cast<ConstrainParameter*>(it_cons->second.get());
    if (pose_param) {
        //  构建命令
        std::string command = "ros2 run arm_workflow move_arm_client --ros-args";
        command += " -p position_x:=" + std::to_string(pose_param->x);
        command += " -p position_y:=" + std::to_string(pose_param->y);
        command += " -p position_z:=" + std::to_string(pose_param->z);
        command += " -p orientation_x:=" + std::to_string(pose_param->qx);
        command += " -p orientation_y:=" + std::to_string(pose_param->qy);
        command += " -p orientation_z:=" + std::to_string(pose_param->qz);
        command += " -p orientation_w:=" + std::to_string(pose_param->qw) + " -p is_constrain:=";
        command += (cons_param->is_constrain ? "true" : "false");
        command += " -p x_axis_tolerance:=" + std::to_string(cons_param->x_axis_tolerance);
        command += " -p y_axis_tolerance:=" + std::to_string(cons_param->y_axis_tolerance);
        command += " -p z_axis_tolerance:=" + std::to_string(cons_param->z_axis_tolerance);

        std::string full_command = "bash -c 'source /opt/ros/humble/setup.bash && ";
        full_command += command + " 2>&1'";

        FILE* pipe = popen(full_command.c_str(), "r");
        if (!pipe) {
            std::cerr << "无法执行命令！" << std::endl;
            return false;
        }

        char buffer[128];
        std::string result = "";
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
        }

        int returnCode = pclose(pipe);
        if (returnCode != 0) {
            std::cerr << "执行命令失败，返回码: " << returnCode << std::endl;
        }
        // 输出命令执行的结果
        // std::cout << "mvoe arm 命令输出：" << std::endl;
        // std::cout << result << std::endl;

        std::string success_marker = "Success: true";
        if (result.find(success_marker) != std::string::npos) {
            return true; 
        } else {
            return false; 
        }
    } else {
        std::cout << "  未知的参数类型 或者参数没有内容" << std::endl;
        return false;
    }
}
bool execute_move_arm_cartesian(const std::map<std::string, std::shared_ptr<BaseParameter>>& parameters)
{
    std::cout << "正在执行move_arm_cartesian:" << std::endl;
    
    // 动态转换参数为 PoseParameter 类型
    auto it_dis = parameters.find("displacement");
    if (it_dis == parameters.end()) {
        std::cout << "  参数中缺少 'displacement' 键." << std::endl;
        return false;
    }

    auto it_cons = parameters.find("constrain");
    if (it_cons == parameters.end()) {
        std::cout << "  参数中缺少 'constrain' 键." << std::endl;
        return false;
    }

    auto* dis_param = dynamic_cast<CartesianPathParameter*>(it_dis->second.get());
    auto* cons_param = dynamic_cast<ConstrainParameter*>(it_cons->second.get());
    if (dis_param) {

        // 构建命令行字符串
        std::string command = "ros2 run arm_workflow move_arm_cartesian_client --ros-args -p delta_x:=";
        command += std::to_string(dis_param->delta_x) + " -p delta_y:=" + std::to_string(dis_param->delta_y);
        command += " -p delta_z:=" + std::to_string(dis_param->delta_z) + " -p is_constrain:=";
        command += (cons_param->is_constrain ? "true" : "false");
        command += " -p x_axis_tolerance:=" + std::to_string(cons_param->x_axis_tolerance);
        command += " -p y_axis_tolerance:=" + std::to_string(cons_param->y_axis_tolerance);
        command += " -p z_axis_tolerance:=" + std::to_string(cons_param->z_axis_tolerance);

        std::string full_command = "bash -c 'source /opt/ros/humble/setup.bash && ";
        full_command += command + " 2>&1'";

        FILE* pipe = popen(full_command.c_str(), "r");
        if (!pipe) {
            std::cerr << "无法执行命令！" << std::endl;
            return false;
        }

        char buffer[128];
        std::string result = "";
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
        }

        int returnCode = pclose(pipe);
        if (returnCode != 0) {
            std::cerr << "执行命令失败，返回码: " << returnCode << std::endl;
        }
        // 输出命令执行的结果
        // std::cout << "mvoe arm cartesian 命令输出：" << std::endl;
        // std::cout << result << std::endl;

        std::string success_marker = "Success: true";
        if (result.find(success_marker) != std::string::npos) {
            return true; 
        } else {
            return false; 
        }

    } else {
        std::cout << "  未知的参数类型 或者参数没有内容" << std::endl;
        return false;
    }
}

bool execute_move_arm_joint(const std::map<std::string, std::shared_ptr<BaseParameter>>& parameters)
{
    std::cout << "正在执行move_arm_joint:" << std::endl;
    
    // 动态转换参数为 PoseParameter 类型
    auto it_dis = parameters.find("joints_value");
    if (it_dis == parameters.end()) {
        std::cout << "  参数中缺少 'joints_value' 键." << std::endl;
        return false;
    }

    auto it_cons = parameters.find("constrain");
    if (it_cons == parameters.end()) {
        std::cout << "  参数中缺少 'constrain' 键." << std::endl;
        return false;
    }

    auto* dis_param = dynamic_cast<JointPathParameter*>(it_dis->second.get());
    auto* cons_param = dynamic_cast<ConstrainParameter*>(it_cons->second.get());
    if (dis_param) {

        // 构建命令行字符串
        std::string command = "ros2 run arm_workflow move_arm_joint_client --ros-args";
        command += " -p joint_0:=" + std::to_string(dis_param->joint_0) + " -p joint_1:=" + std::to_string(dis_param->joint_1);
        command += " -p joint_2:=" + std::to_string(dis_param->joint_2) + " -p joint_3:=" + std::to_string(dis_param->joint_3);
        command += " -p joint_4:=" + std::to_string(dis_param->joint_4) + " -p joint_5:=" + std::to_string(dis_param->joint_5) + " -p is_constrain:=";
        command += (cons_param->is_constrain ? "true" : "false");
        command += " -p x_axis_tolerance:=" + std::to_string(cons_param->x_axis_tolerance);
        command += " -p y_axis_tolerance:=" + std::to_string(cons_param->y_axis_tolerance);
        command += " -p z_axis_tolerance:=" + std::to_string(cons_param->z_axis_tolerance);

        std::string full_command = "bash -c 'source /opt/ros/humble/setup.bash && ";
        full_command += command + " 2>&1'";

        // 执行命令
        FILE* pipe = popen(full_command.c_str(), "r");
        if (!pipe) {
            std::cerr << "无法执行命令！" << std::endl;
            return false;
        }

        char buffer[128];
        std::string result = "";
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
        }

        int returnCode = pclose(pipe);
        if (returnCode != 0) {
            std::cerr << "执行命令失败，返回码: " << returnCode << std::endl;
        }
        // 输出命令执行的结果
        // std::cout << "mvoe arm joint 命令输出：" << std::endl;
        // std::cout << result << std::endl;

        std::string success_marker = "Success: true";
        if (result.find(success_marker) != std::string::npos) {
            return true; 
        } else {
            return false; 
        }
    } else {
        std::cout << "  未知的参数类型 或者参数没有内容" << std::endl;
        return false;
    }
}

bool execute_openner(const std::map<std::string, std::shared_ptr<BaseParameter>> parameters) 
{
    std::cout << "正在进行开瓶操作:" << std::endl;

    auto it_open = parameters.find("openner_joints");
    if (it_open == parameters.end()) {
        std::cout << "  参数中缺少 'openner_joints' 键." << std::endl;
        return false;
    }

    auto* open_param = dynamic_cast<OpennerJointParameter*>(it_open->second.get());
    if (open_param) {
        // 构建命令行字符串
        std::string command = "ros2 run arm_workflow openner_client --ros-args ";
        command += " -p base2link:=" + std::to_string(open_param->base2link);
        command += " -p link2base:=" + std::to_string(open_param->link2base);
        command += " -p left_gripper2base:=" + std::to_string(open_param->left_gripper2base);
        command += " -p right_gripper2base:=" + std::to_string(open_param->right_gripper2base);
        command += " -p duration:=" + std::to_string(open_param->duration); 

        // 执行命令
        std::string full_command = "bash -c 'source /opt/ros/humble/setup.bash && ";
        full_command += command + " 2>&1'";

        // 执行命令
        FILE* pipe = popen(full_command.c_str(), "r");
        if (!pipe) {
            std::cerr << "无法执行命令！" << std::endl;
            return false;
        }

        char buffer[128];
        std::string result = "";
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
        }

        int returnCode = pclose(pipe);
        if (returnCode != 0) {
            std::cerr << "执行命令失败，返回码: " << returnCode << std::endl;
        }
        // 输出命令执行的结果
        // std::cout << "execute openner 命令输出：" << std::endl;
        // std::cout << result << std::endl;

        std::string success_marker = "成功";
        if (result.find(success_marker) != std::string::npos) {
            return true; 
        } else {
            return false; 
        }
    } else {
        std::cout << "  未知的参数类型 或者参数没有内容" << std::endl;
        return false;
    }
}

bool execute_pipettle(const std::map<std::string, std::shared_ptr<BaseParameter>>& parameters)
{
    std::cout << "正在控制pipettle:" << std::endl;
    
    // 动态转换参数为 PoseParameter 类型
    auto it_joints = parameters.find("pipettle_joints");
    if (it_joints == parameters.end()) {
        std::cout << "  参数中缺少 'pipettle_joints' 键." << std::endl;
        return false;
    }

    auto* joints_param = dynamic_cast<PipettleJointParameter*>(it_joints->second.get());
    if (joints_param) {

        // 构建命令行字符串
        std::string command = "ros2 run arm_workflow pipettle_client --ros-args ";
        command += " -p pipettle_base2link:=" + std::to_string(joints_param->pipettle_base2link);

        std::string full_command = "bash -c 'source /opt/ros/humble/setup.bash && ";
        full_command += command + " 2>&1'";

        // 执行命令
        FILE* pipe = popen(full_command.c_str(), "r");
        if (!pipe) {
            std::cerr << "无法执行命令！" << std::endl;
            return false;
        }

        char buffer[128];
        std::string result = "";
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
        }

        int returnCode = pclose(pipe);
        if (returnCode != 0) {
            std::cerr << "执行命令失败，返回码: " << returnCode << std::endl;
        }
        // 输出命令执行的结果
        // std::cout << "execute pipettle 命令输出：" << std::endl;
        // std::cout << result << std::endl;

        std::string success_marker = "成功";
        if (result.find(success_marker) != std::string::npos) {
            return true; 
        } else {
            return false; 
        }
    } else {
        std::cout << "  未知的参数类型 或者参数没有内容" << std::endl;
        return false;
    }
}

bool execute_egp64(const std::map<std::string, std::shared_ptr<BaseParameter>>& parameters)
{
    std::cout << "正在控制egp64:" << std::endl;
    
    // 动态转换参数为 PoseParameter 类型
    auto it_joints = parameters.find("egp64_joints");
    if (it_joints == parameters.end()) {
        std::cout << "  参数中缺少 'egp64_joints' 键." << std::endl;
        return false;
    }

    auto* joints_param = dynamic_cast<Egp64JointParameter*>(it_joints->second.get());
    if (joints_param) {

        // 构建命令行字符串
        std::string command = "ros2 run arm_workflow egp64_client --ros-args ";
        command += " -p egp64_finger_left_joint:=" + std::to_string(joints_param->egp64_finger_left_joint);
        command += " -p egp64_finger_right_joint:=" + std::to_string(joints_param->egp64_finger_right_joint);

        std::string full_command = "bash -c 'source /opt/ros/humble/setup.bash && ";
        full_command += command + " 2>&1'";

        // 执行命令
        FILE* pipe = popen(full_command.c_str(), "r");
        if (!pipe) {
            std::cerr << "无法执行命令！" << std::endl;
            return false;
        }

        char buffer[128];
        std::string result = "";
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
        }

        int returnCode = pclose(pipe);
        if (returnCode != 0) {
            std::cerr << "执行命令失败，返回码: " << returnCode << std::endl;
        }
        // 输出命令执行的结果
        // std::cout << "execute pipettle 命令输出：" << std::endl;
        // std::cout << result << std::endl;

        std::string success_marker = "成功";
        if (result.find(success_marker) != std::string::npos) {
            return true; 
        } else {
            return false; 
        }
    } else {
        std::cout << "  未知的参数类型 或者参数没有内容" << std::endl;
        return false;
    }
}

bool execute_attach(std::map<std::string, std::shared_ptr<BaseParameter>> parameters)
{
    std::cout << "正在执行 attach:" << std::endl;
    // 查找 "model_1" 和 "model_2" 参数
    auto it_model_1 = parameters.find("model_1");
    auto it_model_2 = parameters.find("model_2");

    if (it_model_1 == parameters.end() || it_model_2 == parameters.end()) {
        std::cerr << "  参数中缺少 'mdoel_1' 或 'model_2' 键." << std::endl;
        return false;
    }

    // 查找 "link1" 和 "link2" 参数
    auto it_link1 = parameters.find("link_1");
    auto it_link2 = parameters.find("link_2");

    if (it_link1 == parameters.end() || it_link2 == parameters.end()) {
        std::cerr << "  参数中缺少 'link1' 或 'link2' 键." << std::endl;
        return false;
    }

    // 尝试将参数转换为 StringParameter
    auto* model_1_param = dynamic_cast<StringParameter*>(it_model_1->second.get());
    auto* model_2_param = dynamic_cast<StringParameter*>(it_model_2->second.get());
    auto* link1_param = dynamic_cast<StringParameter*>(it_link1->second.get());
    auto* link2_param = dynamic_cast<StringParameter*>(it_link2->second.get());

    if (link1_param && link2_param) {
        // 构建命令行字符串
        std::string command = "ros2 run arm_workflow attach_client --ros-args ";
        command += " -p model_name_1:=" + model_1_param->value;
        command += " -p model_name_2:=" + model_2_param->value;
        command += " -p link_name_1:=" + link1_param->value;
        command += " -p link_name_2:=" + link2_param->value;

        std::string full_command = "bash -c 'source /opt/ros/humble/setup.bash && ";
        full_command += command + " 2>&1'";


        // 执行命令
        FILE* pipe = popen(full_command.c_str(), "r");
        if (!pipe) {
            std::cerr << "无法执行命令！" << std::endl;
            return false;
        }

        char buffer[128];
        std::string result = "";
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
        }

        int returnCode = pclose(pipe);
        if (returnCode != 0) {
            std::cerr << "执行命令失败，返回码: " << returnCode << std::endl;
        }
        // 输出命令执行的结果
        std::cout << "execute attach 命令输出：" << std::endl;
        std::cout << result << std::endl;

        std::string success_marker = "成功";
        if (result.find(success_marker) != std::string::npos) {
            return true; 
        } else {
            return false; 
        }
    } else {
        std::cerr << "  参数 'link1' 或 'link2' 类型无效或为空." << std::endl;
        return false;
    }
}

bool execute_detach(std::map<std::string, std::shared_ptr<BaseParameter>> parameters)
{
    std::cout << "正在执行 detach:" << std::endl;

    // 查找 "model_1" 和 "model_2" 参数
    auto it_model_1 = parameters.find("model_1");
    auto it_model_2 = parameters.find("model_2");

    if (it_model_1 == parameters.end() || it_model_2 == parameters.end()) {
        std::cerr << "  参数中缺少 'mdoel_1' 或 'model_2' 键." << std::endl;
        return false;
    }

    // 查找 "link1" 和 "link2" 参数
    auto it_link1 = parameters.find("link_1");
    auto it_link2 = parameters.find("link_2");

    if (it_link1 == parameters.end() || it_link2 == parameters.end()) {
        std::cerr << "  参数中缺少 'link1' 或 'link2' 键." << std::endl;
        return false;
    }


    // 尝试将参数转换为 StringParameter
    auto* model_1_param = dynamic_cast<StringParameter*>(it_model_1->second.get());
    auto* model_2_param = dynamic_cast<StringParameter*>(it_model_2->second.get());
    auto* link1_param = dynamic_cast<StringParameter*>(it_link1->second.get());
    auto* link2_param = dynamic_cast<StringParameter*>(it_link2->second.get());

    if (link1_param && link2_param) {
        // 构建命令行字符串
        std::string command = "ros2 run arm_workflow detach_client --ros-args ";
        command += " -p model_name_1:=" + model_1_param->value;
        command += " -p model_name_2:=" + model_2_param->value;
        command += " -p link_name_1:=" + link1_param->value;
        command += " -p link_name_2:=" + link2_param->value;

        std::string full_command = "bash -c 'source /opt/ros/humble/setup.bash && ";
        full_command += command + " 2>&1'";


        // 执行命令
        FILE* pipe = popen(full_command.c_str(), "r");
        if (!pipe) {
            std::cerr << "无法执行命令！" << std::endl;
            return false;
        }

        char buffer[128];
        std::string result = "";
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
        }

        int returnCode = pclose(pipe);
        if (returnCode != 0) {
            std::cerr << "执行命令失败，返回码: " << returnCode << std::endl;
        }
        // 输出命令执行的结果
        std::cout << "execute detach 命令输出：" << std::endl;
        std::cout << result << std::endl;

        std::string success_marker = "成功";
        if (result.find(success_marker) != std::string::npos) {
            return true; 
        } else {
            return false; 
        }
    } else {
        std::cerr << "  参数 'link1' 或 'link2' 类型无效或为空." << std::endl;
        return false;
    }
}