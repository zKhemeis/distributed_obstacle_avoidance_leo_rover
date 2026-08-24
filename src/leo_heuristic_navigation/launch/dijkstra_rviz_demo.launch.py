"""Launch one benchmark scenario with Dijkstra navigation and RViz."""

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
import yaml


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
    config_path = Path(
        LaunchConfiguration('config').perform(context)
    ).expanduser().resolve()
    manifest = Path(
        LaunchConfiguration('manifest').perform(context)
    ).expanduser().resolve()
    split = LaunchConfiguration('split').perform(context)
    world_index = int(LaunchConfiguration('world_index').perform(context))
    python_executable = LaunchConfiguration(
        'python_executable').perform(context)
    detailed_model = _boolean_argument(
        LaunchConfiguration('detailed_model').perform(context),
        'detailed_model',
    )

    if not config_path.is_file():
        raise FileNotFoundError(
            f'Dijkstra configuration does not exist: {config_path}')
    if not manifest.is_file():
        raise FileNotFoundError(f'Manifest does not exist: {manifest}')
    with config_path.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError(f'Configuration is not a mapping: {config_path}')
    planner = config.get('planner')
    mapping = config.get('mapping')
    controller = config.get('controller')
    evaluation = config.get('evaluation')
    if not all(isinstance(section, dict) for section in (
            planner, mapping, controller, evaluation)):
        raise ValueError(
            'LiDAR-only configuration needs planner, mapping, '
            'controller, and evaluation sections')

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

    parameter_arguments = [
        '-p', f'goal_x:={goal_x}',
        '-p', f'goal_y:={goal_y}',
    ]
    for name, value in planner.items():
        rendered = (
            str(value).lower() if isinstance(value, bool) else str(value))
        parameter_arguments.extend(['-p', f'{name}:={rendered}'])
    for name, value in mapping.items():
        rendered = (
            str(value).lower() if isinstance(value, bool) else str(value))
        parameter_arguments.extend(['-p', f'{name}:={rendered}'])
    for name, value in controller.items():
        rendered = (
            str(value).lower() if isinstance(value, bool) else str(value))
        parameter_arguments.extend(['-p', f'{name}:={rendered}'])
    parameter_arguments.extend([
        '-p',
        'maximum_episode_steps:='
        f'{evaluation["maximum_episode_steps"]}',
    ])
    navigation_command = [
        python_executable,
        '-m',
        'leo_heuristic_navigation.ros_dijkstra_node',
        '--ros-args',
        *parameter_arguments,
    ]

    heuristic_share = Path(
        get_package_share_directory('leo_heuristic_navigation'))
    rl_share = Path(get_package_share_directory('leo_rl_navigation'))
    rviz_config = heuristic_share / 'rviz' / 'dijkstra_demo_detailed.rviz'

    actions = [
        LogInfo(msg=(
            f'Dijkstra benchmark {split}[{world_index}]: {world.name}; '
            f'start=({start_x:.6f}, {start_y:.6f}, {start_yaw:.6f}); '
            f'goal=({goal_x:.6f}, {goal_y:.6f}); '
            f'detailed_model={detailed_model}; obstacle_source=lidar_only')),
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
        ExecuteProcess(cmd=navigation_command, output='screen'),
    ]

    if detailed_model:
        visualization_xacro = (
            rl_share / 'urdf' / 'leo_visualization.urdf.xacro')
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

    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_config)],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    ))
    return actions


def generate_launch_description() -> LaunchDescription:
    heuristic_share = Path(
        get_package_share_directory('leo_heuristic_navigation'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=str(
                heuristic_share / 'config' / 'dijkstra_lidar_v2.yaml'),
        ),
        DeclareLaunchArgument(
            'manifest',
            default_value=(
                '/root/leo_ws/src/leo_pybullet/worlds/'
                'benchmark_v4_hard/manifest.csv'),
        ),
        DeclareLaunchArgument('split', default_value='validation'),
        DeclareLaunchArgument('world_index', default_value='0'),
        DeclareLaunchArgument(
            'python_executable',
            default_value='/root/leo_ws/.venv_rl/bin/python',
        ),
        DeclareLaunchArgument('detailed_model', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_launch_scenario),
    ])
