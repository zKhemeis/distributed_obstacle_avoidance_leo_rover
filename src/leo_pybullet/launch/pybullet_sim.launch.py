from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="leo_pybullet",
            executable="pybullet_sim_node",
            name="leo_pybullet_sim_node",
            output="screen",
        )
    ])
