from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='zumo_robot',
            executable='camera_node',
            name='camera_node'
        ),
        Node(
            package='zumo_robot',
            executable='pid_controller_node',
            name='pid_controller_node'
        ),
        Node(
            package='zumo_robot',
            executable='arduino_node',
            name='arduino_node'
        ),
        Node(
            package='zumo_robot',
            executable='path_mapping_node',
            name='mapping_node'
        ),
    ])
