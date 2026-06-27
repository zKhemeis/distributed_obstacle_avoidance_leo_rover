import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_project_gazebo = get_package_share_directory("leo_gz_bringup")

    sim_world = LaunchConfiguration("sim_world").perform(context)
    robot_ns = LaunchConfiguration("robot_ns").perform(context)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-s -r {sim_world}"
        }.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_gazebo, "launch", "spawn_robot.launch.py")
        ),
        launch_arguments={
            "robot_ns": robot_ns
        }.items(),
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

    return [
        gz_sim,
        spawn_robot,
        clock_bridge,
    ]


def generate_launch_description():
    pkg_project_worlds = get_package_share_directory("leo_gz_worlds")

    return LaunchDescription([
        DeclareLaunchArgument(
            "sim_world",
            default_value=os.path.join(
                pkg_project_worlds,
                "worlds",
                "leo_empty.sdf"
            ),
            description="Path to the Gazebo world file",
        ),

        DeclareLaunchArgument(
            "robot_ns",
            default_value="",
            description="Robot namespace",
        ),

        OpaqueFunction(function=launch_setup),
    ])
