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
            executable='motors_node',
            name='motors_node'
        ),
        Node(
            package='zumo_robot',
            executable='encoder_node',
            name='encoder_node'
        ),
        Node(
            package='zumo_robot',
            executable='path_mapping_node',
            name='path_mapping_node'
        ),
        Node(
            package='zumo_robot',
            executable='tf2_node',
            name='tf2_node'
        ),
        Node(
            package='zumo_robot',
            executable="interface_node",
            name='interface_node',
        ),
        Node(
            package='rviz2',          
            executable='rviz2',       
            name='rviz2',             
            output='screen',         
        ),
    ])
