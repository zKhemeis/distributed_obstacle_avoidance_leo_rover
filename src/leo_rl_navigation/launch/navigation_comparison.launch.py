"""Launch LiDAR-only Dijkstra, single-robot PPO, or four-robot PPO."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch.substitutions import (
    LaunchConfiguration,
)

import yaml


def contains_lidar_only(
    value,
) -> bool:
    if isinstance(
        value,
        dict,
    ):
        return any(
            contains_lidar_only(
                item
            )
            for item in value.values()
        )

    if isinstance(
        value,
        list,
    ):
        return any(
            contains_lidar_only(
                item
            )
            for item in value
        )

    return (
        isinstance(
            value,
            str,
        )
        and value.strip().lower() == "lidar_only"
    )


def resolve_dijkstra_config(
    workspace: Path,
) -> Path:
    directory = (
        workspace
        / "src"
        / "leo_heuristic_navigation"
        / "config"
    )

    preferred_config = (
        directory
        / "dijkstra_lidar_v2.yaml"
    )

    if preferred_config.is_file():
        return preferred_config.resolve()

    for candidate in sorted(
        directory.glob(
            "*.yaml"
        )
    ):
        with candidate.open(
            encoding="utf-8",
        ) as stream:
            configuration = yaml.safe_load(
                stream
            )

        if contains_lidar_only(
            configuration
        ):
            return candidate.resolve()

    raise FileNotFoundError(
        "No LiDAR-only Dijkstra configuration was found in "
        f"{directory}. Provide config:=/absolute/path/to/config.yaml."
    )


def launch_selected_method(
    context,
):
    method = LaunchConfiguration(
        "method"
    ).perform(
        context
    ).strip()

    allowed_methods = {
        "dijkstra",
        "ppo-single",
        "ppo-multi",
    }

    if method not in allowed_methods:
        raise ValueError(
            "method must be dijkstra, ppo-single, or ppo-multi."
        )

    workspace = Path(
        LaunchConfiguration(
            "workspace"
        ).perform(
            context
        )
    ).expanduser().resolve()

    world_argument = LaunchConfiguration(
        "map"
    ).perform(
        context
    ).strip()

    manifest_argument = LaunchConfiguration(
        "manifest"
    ).perform(
        context
    ).strip()

    if not world_argument and not manifest_argument:
        raise ValueError(
            "Provide either map:=/path/to/map.yaml "
            "or manifest:=/path/to/manifest.csv."
        )

    if world_argument:
        world_path = Path(
            world_argument
        ).expanduser().resolve()

        if not world_path.is_file():
            raise FileNotFoundError(
                f"Map does not exist: {world_path}"
            )

        if manifest_argument:
            manifest_path = Path(
                manifest_argument
            ).expanduser().resolve()

        else:
            manifest_path = world_path.with_name(
                f"{world_path.stem}_manifest.csv"
            )

    else:
        manifest_path = Path(
            manifest_argument
        ).expanduser().resolve()

    if not manifest_path.is_file():
        raise FileNotFoundError(
            f"Manifest does not exist: {manifest_path}"
        )

    explicit_config = LaunchConfiguration(
        "config"
    ).perform(
        context
    ).strip()

    explicit_model = LaunchConfiguration(
        "model"
    ).perform(
        context
    ).strip()

    launch_arguments = {
        "manifest": str(
            manifest_path
        ),
        "split": LaunchConfiguration(
            "split"
        ).perform(
            context
        ),
        "world_index": LaunchConfiguration(
            "world_index"
        ).perform(
            context
        ),
        "detailed_model": LaunchConfiguration(
            "detailed_model"
        ).perform(
            context
        ),
        "rviz": LaunchConfiguration(
            "rviz"
        ).perform(
            context
        ),
    }

    if method == "dijkstra":
        if explicit_model:
            raise ValueError(
                "The Dijkstra method does not use a PPO model."
            )

        if explicit_config:
            config_path = Path(
                explicit_config
            ).expanduser().resolve()

        else:
            config_path = resolve_dijkstra_config(
                workspace
            )

        launch_arguments[
            "config"
        ] = str(
            config_path
        )

        package = (
            "leo_heuristic_navigation"
        )

        launch_filename = (
            "dijkstra_rviz_demo.launch.py"
        )

        description = (
            "LiDAR-only Dijkstra navigation"
        )

    else:
        if method == "ppo-single":
            policy_name = (
                "ppo_single"
            )

            description = (
                "Single-robot PPO navigation"
            )

        else:
            policy_name = (
                "ppo_multi"
            )

            description = (
                "Four-robot shared PPO navigation"
            )

        policy_directory = (
            workspace
            / "deployment"
            / "policies"
            / policy_name
        )

        if explicit_config:
            config_path = Path(
                explicit_config
            ).expanduser().resolve()

        else:
            config_path = (
                policy_directory
                / "config.yaml"
            )

        if explicit_model:
            model_path = Path(
                explicit_model
            ).expanduser().resolve()

        else:
            model_path = (
                policy_directory
                / "model.zip"
            )

        if not model_path.is_file():
            raise FileNotFoundError(
                f"PPO model does not exist: {model_path}"
            )

        launch_arguments.update(
            {
                "config": str(
                    config_path
                ),
                "model": str(
                    model_path
                ),
                "explicit_front_clearance": "true",
                "safety_shield": "false",
            }
        )

        package = (
            "leo_rl_navigation"
        )

        launch_filename = (
            "ppo_rviz_demo.launch.py"
        )

    if not config_path.is_file():
        raise FileNotFoundError(
            f"Method configuration does not exist: {config_path}"
        )

    launch_file = (
        Path(
            get_package_share_directory(
                package
            )
        )
        / "launch"
        / launch_filename
    )

    if not launch_file.is_file():
        raise FileNotFoundError(
            f"Launch file does not exist: {launch_file}"
        )

    return [
        LogInfo(
            msg=(
                f"Method: {description}; "
                f"manifest={manifest_path}; "
                f"config={config_path}"
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    launch_file
                )
            ),
            launch_arguments=launch_arguments.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "method",
                default_value="ppo-multi",
            ),
            DeclareLaunchArgument(
                "map",
                default_value="",
            ),
            DeclareLaunchArgument(
                "manifest",
                default_value="",
            ),
            DeclareLaunchArgument(
                "model",
                default_value="",
            ),
            DeclareLaunchArgument(
                "config",
                default_value="",
            ),
            DeclareLaunchArgument(
                "workspace",
                default_value=str(
                    Path.home()
                    / "leo_ws"
                ),
            ),
            DeclareLaunchArgument(
                "split",
                default_value="validation",
            ),
            DeclareLaunchArgument(
                "world_index",
                default_value="0",
            ),
            DeclareLaunchArgument(
                "detailed_model",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
            ),
            OpaqueFunction(
                function=launch_selected_method
            ),
        ]
    )
