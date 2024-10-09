#!/usr/bin/python3

import launch 
import launch_ros
from ament_index_python.packages import get_package_share_directory
import launch_ros.parameter_descriptions 

def generate_launch_description():
    urdf_path = get_package_share_directory('abb_gofa_description')
    default_model_path = urdf_path + "/urdf/pipette/pipettle.urdf"

    # 申明一个参数model 表示xacro的路径
    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path), description='URDF 的绝对路径')
    # 根据上一步得到的xacro路径读取xacro文件的内容 并传给robot_description
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')])
        ,value_type=str)

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
    )

    joint_state_publisher_gui_node = launch_ros.actions.Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher',
    output='screen'
)

    return launch.LaunchDescription([
        action_declare_arg_model_path,
        robot_state_publisher_node,
        rviz_node,
        joint_state_publisher_gui_node,
    ])