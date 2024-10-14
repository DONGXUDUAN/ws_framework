from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_workflow', 
            executable='attach_server', 
            name='attach_server', 
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='arm_workflow',  
            executable='detach_server', 
            name='detach_server', 
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='arm_workflow', 
            executable='move_arm_server',  
            name='move_workflow_server',  
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='arm_workflow', 
            executable='move_arm_cartesian_server',  
            name='move_cartesian_workflow_server',  
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='arm_workflow', 
            executable='move_arm_joint_server',  
            name='move_joint_workflow_server',  
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='arm_workflow',  
            executable='operate_server',  
            name='operate_server', 
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[{'use_sim_time': True}]
        ),
    ])
