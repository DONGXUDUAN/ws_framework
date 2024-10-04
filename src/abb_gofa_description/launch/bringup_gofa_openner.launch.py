#!/usr/bin/python3
import os
import launch 
import launch_ros
from ament_index_python.packages import get_package_share_directory
import launch_ros.parameter_descriptions 
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess

def generate_launch_description():

    # gofa机器人相关的路径
    robot_name_in_model = "abb_gofa"
    urdf_path = get_package_share_directory('abb_gofa_description')
    default_model_path = urdf_path + "/urdf/abb_gofa/abb_gofa.urdf.xacro"
    # defult_rviz_config_path = urdf_path +'/config/moveit_gazebo_gofa.rviz'
    default_world_path = urdf_path + '/world/gofa.world'

    # 开瓶器的文件路径
    openner_path = urdf_path + "/urdf/cap_openner/cap_openner.urdf.xacro"

    # ------------------导入gofa crb15000的xacro文件 ----------------------------------------------
    # 申明一个参数model 表示xacro的路径
    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path), description='URDF 的绝对路径')
    # 根据上一步得到的xacro路径读取xacro文件的内容 并传给robot_description
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')])
        ,value_type=str)

    # ------------------导入cap-openner的xacro文件 ------------------------------------------------
    # 申明一个参数model 表示 openner 的xacro的路径
    action_declare_arg_oepnner_path = launch.actions.DeclareLaunchArgument(
        name='openner', default_value=str(openner_path), description='openner urdf文件的绝对路径')
    # 根据上一步得到的xacro路径读取xacro文件的内容 并传给robot_description
    openner_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('openner')])
        ,value_type=str)
    
    # ------------------导入egp的xacro文件 --------------------------------------------------------
    # 申明一个参数model 表示egp64执行器 xacro的路径
    egp64_model_path = urdf_path + "/urdf/egp64/egp64.urdf.xacro"
    action_declare_arg_egp64_path = launch.actions.DeclareLaunchArgument(
    name='egp64', default_value=str(egp64_model_path), description=' egp64 URDF 的绝对路径')
    # 根据上一步得到的xacro路径读取xacro文件的内容 并传给robot_description
    egp64_description = launch_ros.parameter_descriptions.ParameterValue(
    launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('egp64')])
    ,value_type=str)

    openner_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='openner',
        parameters=[{'robot_description': openner_description},
                    {'use_sim_time': True}],
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description},
                    {'use_sim_time': True}],
    )

    egp64_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='egp64',
        parameters=[{'robot_description': egp64_description},
                    {'use_sim_time': True}],
    )
    
    # 通过 IncludeLaunchDescription 包含另外一个 launch 文件
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
        # 传递参数
        launch_arguments=[('world', default_world_path), ('verbose', 'true'), ("use_sim_time", "true")]
    )
    # 请求 Gazebo 加载机器人
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name_in_model, ])

    # 请求 Gazebo 加载openner
    spawn_openner_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/openner/robot_description',
                   '-entity', "cap_openner", ])
    
    # 请求 Gazebo 加载egp64
    spawn_egp64_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/egp64/robot_description',
                    '-entity', "egp64",
                    '-x', '-0.585',
                    '-y', '0.09',
                    '-z', '1.12',
                    '-R', '3.1415926',
                    '-P', '0.0',
                    '-Y', '0.0',
        ]
    )
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

    openner_joint_state_broadcaster_spawner = launch_ros.actions.Node(
    package="controller_manager",
    executable="spawner",
    namespace="openner",  
    arguments=["openner_joint_state_broadcaster", "--controller-manager", "/openner/controller_manager"],
    )


    openner_position_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        namespace="openner", 
        arguments=["openner_joint_controllers", "-c", "/openner/controller_manager"],
    )

    egp64_joint_state_broadcaster_spawner = launch_ros.actions.Node(
    package="controller_manager",
    executable="spawner",
    namespace="egp64",  
    arguments=["egp64_joint_state_broadcaster", "--controller-manager", "/egp64/controller_manager"],
    )

    egp64_position_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        namespace="egp64", 
        arguments=["egp64_joint_controllers", "--controller-manager", "/egp64/controller_manager"],
    ) 

    moveit_path = get_package_share_directory("abb_gofa_moveit_config")
    moveit_config = (
        MoveItConfigsBuilder("abb_gofa")
        .robot_description(file_path= urdf_path + "/urdf/abb_gofa/abb_gofa.urdf.xacro")
        .trajectory_execution(file_path=moveit_path + "/config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        # .planning_pipelines(
        #     pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        # )
        .to_moveit_configs()
    )

    run_move_group_node = launch_ros.actions.Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
    )

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        # arguments=["-d", defult_rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    return launch.LaunchDescription([
        action_declare_arg_model_path,
        action_declare_arg_oepnner_path,
        action_declare_arg_egp64_path,
        robot_state_publisher_node,
        openner_state_publisher_node,
        egp64_state_publisher_node,
        # oepnner_joint_state_publisher_gui_node,
        launch_gazebo,
        ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call', '/attach', 'gazebo_attach_interfaces/srv/Attach',
            "{model_name_1: 'bottle_1', link_name_1: 'bottle::link', model_name_2: 'bottle_cap_1', link_name_2: 'bottle_cap::link'}"
        ],
        output='screen'),
        ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call', '/attach', 'gazebo_attach_interfaces/srv/Attach',
            "{model_name_1: 'bottle_plate_1', link_name_1: 'bottle_plate::link', model_name_2: 'bottle_1', link_name_2: 'bottle::link'}"
        ],
        output='screen'),
        spawn_entity_node,
        # spawn_openner_node,
        spawn_egp64_node,
        # openner_joint_state_broadcaster_spawner,
        # openner_position_controller_spawner,
        egp64_joint_state_broadcaster_spawner,
        egp64_position_controller_spawner,
        rviz_node,
 
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
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[run_move_group_node]
            ),
        ),
        ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call', '/detach', 'gazebo_attach_interfaces/srv/Detach',
            "{model_name_1: 'bottle_plate_1', link_name_1: 'bottle_plate::link', model_name_2: 'bottle_1', link_name_2: 'bottle::link'}"
        ],
        output='screen'),
    ])