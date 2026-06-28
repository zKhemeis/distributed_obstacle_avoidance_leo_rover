# Copyright 2023 Fictionlab sp. z o.o.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_project_gazebo = get_package_share_directory("leo_gz_bringup")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="leo_empty.sdf",
        description="Gazebo world file from leo_gz_worlds/worlds",
    )

    robot_ns = DeclareLaunchArgument(
        "robot_ns",
        default_value="",
        description="Robot namespace",
    )

    world = LaunchConfiguration("world")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PathJoinSubstitution(
                [
                    FindPackageShare("leo_gz_worlds"),
                    "worlds",
                    world,
                ]
            )
        }.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_gazebo, "launch", "spawn_robot.launch.py")
        ),
        launch_arguments={"robot_ns": LaunchConfiguration("robot_ns")}.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        parameters=[
            {
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            world_arg,
            robot_ns,
            gz_sim,
            spawn_robot,
            clock_bridge,
        ]
    )