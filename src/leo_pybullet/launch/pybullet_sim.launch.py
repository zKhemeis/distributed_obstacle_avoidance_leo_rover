from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="boxes",
        description="Path to YAML world file",
    )

    sim_node = Node(
        package="leo_pybullet",
        executable="pybullet_sim_node",
        name="leo_pybullet_sim_node",
        output="screen",
        parameters=[
            {"world": LaunchConfiguration("world")}
        ],
    )

    return LaunchDescription([
        world_arg,
        sim_node,
    ])
