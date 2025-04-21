from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='patrol_robot',
            executable='patrol_node',
            name='patrol_node',
            output='screen'
        ),
        Node(
            package='patrol_robot',
            executable='detection_node',
            name='detection_node',
            output='screen'
        ),
        Node(
            package='patrol_robot',
            executable='visualization_node',
            name='visualization_node',
            output='screen'
        )
    ])
