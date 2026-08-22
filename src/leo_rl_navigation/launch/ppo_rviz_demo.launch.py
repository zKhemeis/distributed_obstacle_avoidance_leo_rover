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
    config_argument = LaunchConfiguration('config').perform(context).strip()
    environment_config = {}
    if config_argument:
        config_path = Path(config_argument).expanduser().resolve()
        if not config_path.is_file():
            raise FileNotFoundError(f'Configuration does not exist: {config_path}')
        with config_path.open(encoding='utf-8') as stream:
            loaded = yaml.safe_load(stream)
        if not isinstance(loaded, dict):
            raise ValueError(f'Configuration is not a mapping: {config_path}')
        environment_config = loaded.get('environment', {})
        if not isinstance(environment_config, dict):
            raise ValueError('Configuration environment must be a mapping')

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
    if config_argument:
        explicit_front_clearance = bool(environment_config.get(
            'include_front_clearance', explicit_front_clearance))
    shield_override = LaunchConfiguration('safety_shield').perform(context)
    if shield_override.strip().lower() == 'auto':
        safety_shield = bool(environment_config.get(
            'enable_safety_shield', False))
    else:
        safety_shield = _boolean_argument(shield_override, 'safety_shield')

    def configured(name: str, default):
        return environment_config.get(name, default)

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
        '-p', f'n_sectors:={configured("n_sectors", 50)}',
        '-p', 'number_of_rays:=500',
        '-p', 'range_min:=0.05',
        '-p', 'range_max:=12.0',
        '-p', (
            'maximum_goal_distance:='
            f'{configured("maximum_goal_distance", 14.1421356237)}'),
        '-p', f'front_half_angle_deg:={configured("front_half_angle_deg", 30.0)}',
        '-p',
        f'include_front_clearance:={str(explicit_front_clearance).lower()}',
        '-p', (
            'front_normalization_distance:='
            f'{configured("front_normalization_distance", 0.80)}'),
        '-p', (
            'include_directional_clearance:='
            f'{str(bool(configured("include_directional_clearance", False))).lower()}'),
        '-p', (
            'directional_normalization_distance:='
            f'{configured("directional_normalization_distance", 3.0)}'),
        '-p', (
            'directional_inner_angle_deg:='
            f'{configured("directional_inner_angle_deg", 25.0)}'),
        '-p', (
            'directional_outer_angle_deg:='
            f'{configured("directional_outer_angle_deg", 85.0)}'),
        '-p', (
            'use_footprint_clearance:='
            f'{str(bool(configured("use_footprint_clearance", False))).lower()}'),
        '-p', (
            'footprint_half_length:='
            f'{configured("footprint_half_length", 0.215)}'),
        '-p', (
            'footprint_half_width:='
            f'{configured("footprint_half_width", 0.259)}'),
        '-p', f'lidar_offset_x:={configured("lidar_offset_x", 0.10)}',
        '-p', f'lidar_offset_y:={configured("lidar_offset_y", 0.0)}',
        '-p', (
            'footprint_half_angle_deg:='
            f'{configured("footprint_half_angle_deg", 90.0)}'),
        '-p', f'enable_safety_shield:={str(safety_shield).lower()}',
        '-p', (
            'safety_stop_distance:='
            f'{configured("safety_stop_distance", 0.08)}'),
        '-p', (
            'safety_slowdown_distance:='
            f'{configured("safety_slowdown_distance", 0.35)}'),
        '-p', (
            'safety_minimum_turn_speed:='
            f'{configured("safety_minimum_turn_speed", 0.40)}'),
        '-p', f'linear_speed_max:={configured("linear_speed_max", 0.25)}',
        '-p', (
            'linear_reverse_speed_max:='
            f'{configured("linear_reverse_speed_max", 0.0)}'),
        '-p', f'angular_speed_max:={configured("angular_speed_max", 0.80)}',
        '-p', f'goal_tolerance:={configured("goal_tolerance", 0.25)}',
        '-p', (
            'maximum_episode_steps:='
            f'{configured("maximum_episode_steps", 400)}'),
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
            f'explicit_front_clearance={explicit_front_clearance}; '
            'directional_clearance='
            f'{bool(configured("include_directional_clearance", False))}; '
            f'safety_shield={safety_shield}')),
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
            'config',
            default_value='',
            description='Optional PPO YAML defining the ROS observation contract.',
        ),
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
        DeclareLaunchArgument(
            'safety_shield',
            default_value='auto',
            description='Use the configured safety shield, or override true/false.',
        ),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_launch_scenario),
    ])
