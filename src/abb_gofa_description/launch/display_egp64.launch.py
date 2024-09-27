#!/usr/bin/python3
import launch 
import launch_ros
from ament_index_python.packages import get_package_share_directory
import launch_ros.parameter_descriptions 
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    robot_name_in_model = "egp64"
    urdf_path = get_package_share_directory('abb_gofa_description')
    default_model_path = urdf_path + "/urdf/egp64/egp64.urdf.xacro"

    # 申明一个参数model 表示xacro的路径
    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path), description='URDF 的绝对路径')
    # 根据上一步得到的xacro路径读取xacro文件的内容 并传给robot_description
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')])
        ,value_type=str)

    egp64_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='egp64',
        parameters=[{'robot_description': robot_description}]
    )

    egp64_joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        namespace='egp64',
        output='screen')

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
    )
    return launch.LaunchDescription([
        action_declare_arg_model_path,
        egp64_state_publisher_node,
        egp64_joint_state_publisher_gui_node,
        rviz_node
    ])