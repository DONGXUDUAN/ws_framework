#!/usr/bin/python3
import launch 
import launch_ros
from ament_index_python.packages import get_package_share_directory
import launch_ros.parameter_descriptions 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder
def generate_launch_description():

    robot_name_in_model = "abb_gofa"
    urdf_path = get_package_share_directory('abb_gofa_description')
    default_model_path = urdf_path + "/urdf/abb_gofa/abb_gofa.urdf.xacro"
    defult_rviz_config_path = urdf_path +'/config/display_gofa.rviz'
    default_world_path = urdf_path + '/world/gofa.world'

    # 申明一个参数model 表示gofa机械臂 xacro的路径
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

    # 申明一个参数model 表示egp64执行器 xacro的路径
    egp64_model_path = urdf_path + "/urdf/egp64/egp64.urdf.xacro"
    action_declare_arg_egp64_path = launch.actions.DeclareLaunchArgument(
    name='egp64', default_value=str(egp64_model_path), description=' egp64 URDF 的绝对路径')
    # 根据上一步得到的xacro路径读取xacro文件的内容 并传给robot_description
    egp64_description = launch_ros.parameter_descriptions.ParameterValue(
    launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('egp64')])
    ,value_type=str)

    egp64_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='egp64',
        parameters=[{'robot_description': egp64_description}],
    )

    # 通过 IncludeLaunchDescription 包含另外一个 launch 文件
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
        # 传递参数
        launch_arguments=[('world', default_world_path), ('verbose', 'true'),("audio", "false")]
    )
    # 请求 Gazebo 加载机器人
    spawn_gofa_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', robot_name_in_model,])

    abb_gofa_joint_trajectory_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["abb_gofa_arm_controller", "--controller-manager", "controller_manager"],
    )

    abb_gofa_joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"],
    )


    # 请求加载egp64执行器以及其控制器
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
        arguments=["-d", defult_rviz_config_path],
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
        launch.actions.DeclareLaunchArgument('attempts', default_value='5'),
        action_declare_arg_model_path,
        action_declare_arg_egp64_path,
        robot_state_publisher_node,
        egp64_state_publisher_node,
        launch_gazebo,
        spawn_gofa_node,
        spawn_egp64_node,
        rviz_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_egp64_node,
                on_exit=[egp64_joint_state_broadcaster_spawner]
            )
        ),
        
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=egp64_joint_state_broadcaster_spawner,
                on_exit=[egp64_position_controller_spawner]
            ),
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=egp64_position_controller_spawner,
                on_exit=[abb_gofa_joint_state_broadcaster_spawner]
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=abb_gofa_joint_state_broadcaster_spawner,
                on_exit=[abb_gofa_joint_trajectory_controller_spawner,]
            ),
        ),
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=abb_gofa_joint_state_broadcaster_spawner,
        #         on_exit=[run_move_group_node]
        #     ),
        # ),
    ])


