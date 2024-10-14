// workflow_step.hpp

#ifndef WORKFLOW_STEP_HPP
#define WORKFLOW_STEP_HPP

// 参数中只有string类型的简单版本
#include <memory>
#include <string>
#include <map>

class BaseParameter {
public:
    virtual ~BaseParameter() = default;
};

class PoseParameter : public BaseParameter {
public:
    double x, y, z, qx, qy, qz, qw;
    PoseParameter(double _x, double _y, double _z, double _qx, double _qy, double _qz, double _qw) 
        : x(_x), y(_y), z(_z), qx(_qx), qy(_qy), qz(_qz), qw(_qw) {}
};

class ConstrainParameter : public BaseParameter {
public:
    bool is_constrain;
    double x_axis_tolerance, y_axis_tolerance, z_axis_tolerance;
    ConstrainParameter(bool _is_constrain, double _x_axis_tolerance, double _y_axis_tolerance, double _z_axis_tolerance) 
        : is_constrain(_is_constrain), x_axis_tolerance(_x_axis_tolerance), y_axis_tolerance(_y_axis_tolerance), z_axis_tolerance(_z_axis_tolerance) {}
};

class CartesianPathParameter : public BaseParameter {
public:
    double delta_x, delta_y, delta_z;
    CartesianPathParameter(double _delta_x, double _delta_y, double _delta_z) 
        : delta_x(_delta_x), delta_y(_delta_y), delta_z(_delta_z) {}
};

class JointPathParameter : public BaseParameter {
public:
    double joint_0, joint_1, joint_2, joint_3, joint_4, joint_5;
    JointPathParameter(double _joint_0, double _joint_1, double _joint_2, double _joint_3, double _joint_4, double _joint_5)
        :joint_0(_joint_0), joint_1(_joint_1), joint_2(_joint_2), joint_3(_joint_3), joint_4(_joint_4), joint_5(_joint_5) {}
};

class StringParameter : public BaseParameter {
public:
    std::string value;
    StringParameter(const std::string& v) : value(v) {}
};

// 定义工作流步骤结构体
struct WorkflowStep {
    std::string action_name;
    std::map<std::string, std::shared_ptr<BaseParameter>> parameters;
};

#endif // WORKFLOW_STEP_HPP