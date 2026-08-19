import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory(
        "leo_obstacle_avoidance"
    )

    default_params_file = os.path.join(
        package_share,
        "config",
        "lidar_calibration_sim.yaml"
    )

    params_file = LaunchConfiguration("params_file")
    scan_topic = LaunchConfiguration("scan_topic")
    reference_distance = LaunchConfiguration(
        "reference_distance_m"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="LiDAR calibration parameter file"
        ),

        DeclareLaunchArgument(
            "scan_topic",
            default_value="/scan",
            description="LaserScan input topic"
        ),

        DeclareLaunchArgument(
            "reference_distance_m",
            default_value="2.0",
            description="Measured distance from LiDAR origin to obstacle"
        ),

        Node(
            package="leo_obstacle_avoidance",
            executable="lidar_range_calibration_node",
            name="lidar_range_calibration_node",
            output="screen",
            parameters=[
                params_file,
                {
                    "reference_distance_m": ParameterValue(
                        reference_distance,
                        value_type=float
                    )
                }
            ],
            remappings=[
                ("/scan", scan_topic)
            ]
        )
    ])
