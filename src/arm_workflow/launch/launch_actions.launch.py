from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_workflow', 
            executable='attach_server', 
            name='attach_server', 
            output='screen',
            # parameters=[{}]
        ),
        Node(
            package='arm_workflow',  
            executable='detach_server', 
            name='detach_server', 
            output='screen',
            # parameters=[{}]
        ),
        Node(
            package='arm_workflow', 
            executable='move_arm_server',  
            name='move_arm_server',  
            output='screen',
            # parameters=[{}]
        ),
        Node(
            package='arm_workflow',  
            executable='operate_server',  
            name='operate_server', 
            output='screen',
            # parameters=[{}]
        ),
    ])
