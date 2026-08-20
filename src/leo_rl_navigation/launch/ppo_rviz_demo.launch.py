"""Launch one immutable benchmark scenario with PPO and RViz."""

from __future__ import annotations

import csv
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _boolean_argument(value: str, name: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {'1', 'true', 'yes', 'on'}:
        return True
    if normalized in {'0', 'false', 'no', 'off'}:
        return False
    raise ValueError(f'{name} must be true or false, got: {value}')


def _resolve_world(manifest: Path, split: str, configured: str) -> Path:
    path = Path(configured).expanduser()
    if path.is_file():
        return path.resolve()
    portable = manifest.parent / split / path.name
    if portable.is_file():
        return portable.resolve()
    raise FileNotFoundError(
        f'World does not exist: {path}; portable candidate: {portable}')


def _launch_scenario(context):
    manifest = Path(
        LaunchConfiguration('manifest').perform(context)
    ).expanduser().resolve()
    split = LaunchConfiguration('split').perform(context)
    world_index = int(LaunchConfiguration('world_index').perform(context))
    model = Path(
        LaunchConfiguration('model').perform(context)
    ).expanduser().resolve()
    python_executable = LaunchConfiguration(
        'python_executable').perform(context)
    detailed_model = _boolean_argument(
        LaunchConfiguration('detailed_model').perform(context),
        'detailed_model',
    )
    explicit_front_clearance = _boolean_argument(
        LaunchConfiguration('explicit_front_clearance').perform(context),
        'explicit_front_clearance',
    )

    if not manifest.is_file():
        raise FileNotFoundError(f'Manifest does not exist: {manifest}')
    if not model.is_file():
        raise FileNotFoundError(f'Model does not exist: {model}')

    with manifest.open(newline='', encoding='utf-8') as stream:
        rows = [
            row for row in csv.DictReader(stream)
            if row['split'] == split
        ]
    if world_index < 0 or world_index >= len(rows):
        raise IndexError(
            f'world_index={world_index} outside [0, {len(rows)})')

    row = rows[world_index]
    world = _resolve_world(manifest, split, row['world_file'])
    start_x = float(row['start_x'])
    start_y = float(row['start_y'])
    start_yaw = float(row['start_yaw'])
    goal_x = float(row['goal_x'])
    goal_y = float(row['goal_y'])

    policy_command = [
        python_executable,
        '-m',
        'leo_rl_navigation.ros_policy_node',
        '--ros-args',
        '-p', f'model_path:={model}',
        '-p', f'goal_x:={goal_x}',
        '-p', f'goal_y:={goal_y}',
        '-p', 'n_sectors:=50',
        '-p', 'number_of_rays:=500',
        '-p', 'range_min:=0.05',
        '-p', 'range_max:=12.0',
        '-p', 'maximum_goal_distance:=14.1421356237',
        '-p', 'front_half_angle_deg:=30.0',
        '-p',
        f'include_front_clearance:={str(explicit_front_clearance).lower()}',
        '-p', 'front_normalization_distance:=0.80',
        '-p', 'linear_speed_max:=0.25',
        '-p', 'angular_speed_max:=0.80',
        '-p', 'goal_tolerance:=0.25',
        '-p', 'maximum_episode_steps:=400',
        '-p', 'control_hz:=10.0',
        '-p', 'sensor_timeout:=0.50',
    ]

    package_share = Path(get_package_share_directory('leo_rl_navigation'))
    rviz_filename = (
        'ppo_demo_detailed.rviz' if detailed_model else 'ppo_demo.rviz')
    rviz_config = package_share / 'rviz' / rviz_filename

    actions = [
        LogInfo(msg=(
            f'Benchmark {split}[{world_index}]: {world.name}; '
            f'start=({start_x:.6f}, {start_y:.6f}, {start_yaw:.6f}); '
            f'goal=({goal_x:.6f}, {goal_y:.6f}); '
            f'detailed_model={detailed_model}; '
            f'explicit_front_clearance={explicit_front_clearance}')),
        Node(
            package='leo_pybullet',
            executable='pybullet_sim_node',
            name='leo_pybullet_sim_node',
            output='screen',
            parameters=[{
                'world': str(world),
                'start_x': start_x,
                'start_y': start_y,
                'start_yaw': start_yaw,
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
            output='screen',
        ),
        ExecuteProcess(cmd=policy_command, output='screen'),
    ]

    if detailed_model:
        visualization_xacro = package_share / 'urdf' / 'leo_visualization.urdf.xacro'
        if not visualization_xacro.is_file():
            raise FileNotFoundError(
                f'Visualization Xacro does not exist: {visualization_xacro}')
        robot_description = xacro.process_file(
            str(visualization_xacro),
            mappings={'mecanum_wheels': 'false'},
        ).toxml()
        actions.extend([
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='leo_visual_robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description}],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_visual_model',
                arguments=[
                    '--x', '0.0', '--y', '0.0', '--z', '0.0',
                    '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                    '--frame-id', 'base_link',
                    '--child-frame-id', 'visual_base_link',
                ],
                output='screen',
            ),
        ])

    actions.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', str(rviz_config)],
            additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        )
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'manifest',
            default_value=(
                '/root/leo_ws/src/leo_pybullet/worlds/'
                'benchmark_v2/manifest.csv'),
        ),
        DeclareLaunchArgument('split', default_value='validation'),
        DeclareLaunchArgument('world_index', default_value='0'),
        DeclareLaunchArgument(
            'model',
            default_value=(
                '/root/leo_ws/results/rl_v2/'
                'ppo_pilot_100k_seed42_002/'
                'best_model/best_model.zip'),
        ),
        DeclareLaunchArgument(
            'python_executable',
            default_value='/root/leo_ws/.venv_rl/bin/python',
        ),
        DeclareLaunchArgument(
            'detailed_model',
            default_value='true',
            description=(
                'Show the detailed Leo mesh through robot_state_publisher. '
                'This changes visualization only.'),
        ),
        DeclareLaunchArgument(
            'explicit_front_clearance',
            default_value='false',
            description=(
                'Use the 54-value observation containing explicit normalized '
                'front clearance. Keep false for legacy 53-value policies.'),
        ),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_launch_scenario),
    ])
