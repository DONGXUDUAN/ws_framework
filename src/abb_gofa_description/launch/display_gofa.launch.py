#!/usr/bin/python3
import launch 
import launch_ros
from ament_index_python.packages import get_package_share_directory
import launch_ros.parameter_descriptions 
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    robot_name_in_model = "abb_gofa"
    urdf_path = get_package_share_directory('abb_gofa_description')
    default_model_path = urdf_path + "/urdf/abb_gofa/abb_gofa.urdf.xacro"
    # defult_rviz_config_path = urdf_path +'/config/display_gofa.rviz'
    default_world_path = urdf_path + '/world/gofa.world'

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
        parameters=[{'robot_description': robot_description}],
    )

    # 通过 IncludeLaunchDescription 包含另外一个 launch 文件
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
        # 传递参数
        launch_arguments=[('world', default_world_path), ('verbose', 'true'),("audio", "false")]
    )
    # 请求 Gazebo 加载机器人
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name_in_model, ])

    joint_trajectory_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["abb_gofa_arm_controller", "-c", "/controller_manager"],
    )

    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    return launch.LaunchDescription([
        action_declare_arg_model_path,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[joint_trajectory_controller_spawner]
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=joint_trajectory_controller_spawner,
                on_exit=[joint_state_broadcaster_spawner]
            ),
        ),

        
    ])