import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="leo_empty.sdf",
        description="Gazebo world file from leo_gz_worlds/worlds",
    )

    world = LaunchConfiguration("world")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": [
                "-r -s ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("leo_gz_worlds"),
                        "worlds",
                        world,
                    ]
                ),
            ]
        }.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("leo_gz_bringup"),
                "launch",
                "spawn_robot.launch.py",
            )
        )
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            world_arg,
            gz_sim,
            spawn_robot,
            clock_bridge,
        ]
    )