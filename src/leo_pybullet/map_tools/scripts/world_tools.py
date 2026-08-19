#!/usr/bin/env python3
"""Geometry, generation and validation helpers for leo_pybullet YAML worlds."""

from __future__ import annotations

import copy
import heapq
import json
import math
import random
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable

import yaml


@dataclass(frozen=True)
class Box:
    name: str
    x: float
    y: float
    z: float
    size_x: float
    size_y: float
    size_z: float

    @property
    def x_min(self) -> float:
        return self.x - self.size_x / 2.0

    @property
    def x_max(self) -> float:
        return self.x + self.size_x / 2.0

    @property
    def y_min(self) -> float:
        return self.y - self.size_y / 2.0

    @property
    def y_max(self) -> float:
        return self.y + self.size_y / 2.0


def read_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return data


def load_boxes(path: Path) -> list[Box]:
    data = read_yaml(path)
    obstacles = data.get("obstacles", [])
    if not isinstance(obstacles, list):
        raise ValueError("'obstacles' must be a list")
    required = ("name", "x", "y", "z", "size_x", "size_y", "size_z")
    boxes: list[Box] = []
    for index, item in enumerate(obstacles):
        if not isinstance(item, dict):
            raise ValueError(f"obstacles[{index}] must be a mapping")
        missing = [key for key in required if key not in item]
        if missing:
            raise ValueError(f"obstacles[{index}] is missing: {', '.join(missing)}")
        boxes.append(
            Box(
                name=str(item["name"]),
                x=float(item["x"]),
                y=float(item["y"]),
                z=float(item["z"]),
                size_x=float(item["size_x"]),
                size_y=float(item["size_y"]),
                size_z=float(item["size_z"]),
            )
        )
    return boxes


def write_world(path: Path, boxes: Iterable[Box]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {"obstacles": [asdict(box) for box in boxes]}
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(payload, stream, sort_keys=False, default_flow_style=False)


def write_metadata(path: Path, metadata: dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as stream:
        json.dump(metadata, stream, indent=2, sort_keys=True)
        stream.write("\n")


def boxes_overlap(a: Box, b: Box, gap: float = 0.0) -> bool:
    return (
        abs(a.x - b.x) < (a.size_x + b.size_x) / 2.0 + gap
        and abs(a.y - b.y) < (a.size_y + b.size_y) / 2.0 + gap
    )


def point_to_box_distance(px: float, py: float, box: Box) -> float:
    dx = max(box.x_min - px, 0.0, px - box.x_max)
    dy = max(box.y_min - py, 0.0, py - box.y_max)
    return math.hypot(dx, dy)


def robot_inflation_radius(config: dict[str, Any]) -> float:
    robot = config["robot"]
    return 0.5 * math.hypot(float(robot["length"]), float(robot["width"])) + float(
        robot["safety_margin"]
    )


def box_inside_arena(box: Box, config: dict[str, Any]) -> bool:
    arena = config["arena"]
    margin = float(arena["boundary_margin"])
    return (
        box.x_min >= float(arena["x_min"]) + margin
        and box.x_max <= float(arena["x_max"]) - margin
        and box.y_min >= float(arena["y_min"]) + margin
        and box.y_max <= float(arena["y_max"]) - margin
    )


def is_boundary_wall(box: Box) -> bool:
    return box.name.startswith("boundary_")


def make_boundary_walls(config: dict[str, Any]) -> list[Box]:
    wall_cfg = config.get("boundary_walls", {})
    if not bool(wall_cfg.get("enabled", False)):
        return []
    arena = config["arena"]
    xmin, xmax = float(arena["x_min"]), float(arena["x_max"])
    ymin, ymax = float(arena["y_min"]), float(arena["y_max"])
    thickness = float(wall_cfg["thickness"])
    height = float(wall_cfg["height"])
    if thickness <= 0.0 or height <= 0.0:
        raise ValueError("boundary wall thickness and height must be positive")
    return [
        Box("boundary_left", xmin - thickness / 2.0, (ymin + ymax) / 2.0, height / 2.0, thickness, ymax - ymin, height),
        Box("boundary_right", xmax + thickness / 2.0, (ymin + ymax) / 2.0, height / 2.0, thickness, ymax - ymin, height),
        Box("boundary_bottom", (xmin + xmax) / 2.0, ymin - thickness / 2.0, height / 2.0, xmax - xmin, thickness, height),
        Box("boundary_top", (xmin + xmax) / 2.0, ymax + thickness / 2.0, height / 2.0, xmax - xmin, thickness, height),
    ]


def candidate_is_valid(box: Box, placed: list[Box], config: dict[str, Any]) -> bool:
    if box.size_x <= 0.0 or box.size_y <= 0.0 or box.size_z <= 0.0:
        return False
    if not box_inside_arena(box, config):
        return False
    if not math.isclose(box.z, box.size_z / 2.0, rel_tol=0.0, abs_tol=1e-6):
        return False
    gap = float(config["generation"]["inter_obstacle_gap"])
    if any(boxes_overlap(box, other, gap) for other in placed):
        return False
    for key in ("start", "goal"):
        point = config[key]
        if point_to_box_distance(float(point["x"]), float(point["y"]), box) < float(
            point["exclusion_radius"]
        ):
            return False
    return True


def line_intersects_aabb(
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    box: Box,
    inflation: float = 0.0,
) -> bool:
    """Liang-Barsky segment/AABB intersection in 2D."""
    xmin = box.x_min - inflation
    xmax = box.x_max + inflation
    ymin = box.y_min - inflation
    ymax = box.y_max + inflation
    dx, dy = x1 - x0, y1 - y0
    p = (-dx, dx, -dy, dy)
    q = (x0 - xmin, xmax - x0, y0 - ymin, ymax - y0)
    u1, u2 = 0.0, 1.0
    for pi, qi in zip(p, q):
        if math.isclose(pi, 0.0):
            if qi < 0.0:
                return False
            continue
        t = qi / pi
        if pi < 0.0:
            u1 = max(u1, t)
        else:
            u2 = min(u2, t)
        if u1 > u2:
            return False
    return True


def direct_path_blocked(boxes: Iterable[Box], config: dict[str, Any]) -> bool:
    start, goal = config["start"], config["goal"]
    inflation = robot_inflation_radius(config)
    return any(
        line_intersects_aabb(
            float(start["x"]),
            float(start["y"]),
            float(goal["x"]),
            float(goal["y"]),
            box,
            inflation,
        )
        for box in boxes
    )


def _grid_shape(config: dict[str, Any]) -> tuple[int, int, float]:
    arena = config["arena"]
    resolution = float(config["planner"]["resolution"])
    nx = int(math.floor((float(arena["x_max"]) - float(arena["x_min"])) / resolution)) + 1
    ny = int(math.floor((float(arena["y_max"]) - float(arena["y_min"])) / resolution)) + 1
    return nx, ny, resolution


def _world_to_cell(x: float, y: float, config: dict[str, Any]) -> tuple[int, int]:
    arena = config["arena"]
    _, _, resolution = _grid_shape(config)
    return (
        int(round((x - float(arena["x_min"])) / resolution)),
        int(round((y - float(arena["y_min"])) / resolution)),
    )


def _cell_to_world(ix: int, iy: int, config: dict[str, Any]) -> tuple[float, float]:
    arena = config["arena"]
    _, _, resolution = _grid_shape(config)
    return (
        float(arena["x_min"]) + ix * resolution,
        float(arena["y_min"]) + iy * resolution,
    )


def _blocked_cells(boxes: Iterable[Box], config: dict[str, Any]) -> set[tuple[int, int]]:
    boxes = list(boxes)
    nx, ny, _ = _grid_shape(config)
    inflation = robot_inflation_radius(config)
    blocked: set[tuple[int, int]] = set()
    for ix in range(nx):
        for iy in range(ny):
            x, y = _cell_to_world(ix, iy, config)
            if any(
                box.x_min - inflation <= x <= box.x_max + inflation
                and box.y_min - inflation <= y <= box.y_max + inflation
                for box in boxes
            ):
                blocked.add((ix, iy))
    return blocked


def find_path(boxes: Iterable[Box], config: dict[str, Any]) -> list[tuple[float, float]] | None:
    """Return an A* path for a conservatively circular rover, or None."""
    boxes = list(boxes)
    nx, ny, resolution = _grid_shape(config)
    start_cfg, goal_cfg = config["start"], config["goal"]
    start = _world_to_cell(float(start_cfg["x"]), float(start_cfg["y"]), config)
    goal = _world_to_cell(float(goal_cfg["x"]), float(goal_cfg["y"]), config)
    blocked = _blocked_cells(boxes, config)
    if start in blocked or goal in blocked:
        return None

    cardinal = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0))
    diagonal = ((1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)))
    moves = cardinal + diagonal if bool(config["planner"].get("allow_diagonal", True)) else cardinal

    frontier: list[tuple[float, tuple[int, int]]] = [(0.0, start)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    cost = {start: 0.0}

    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal:
            cells = [current]
            while current != start:
                current = came_from[current]
                cells.append(current)
            cells.reverse()
            return [_cell_to_world(ix, iy, config) for ix, iy in cells]

        for dx, dy, move_cost in moves:
            neighbor = (current[0] + dx, current[1] + dy)
            if not (0 <= neighbor[0] < nx and 0 <= neighbor[1] < ny):
                continue
            if neighbor in blocked:
                continue
            if dx != 0 and dy != 0:
                # Prevent passing diagonally through the corner of two obstacles.
                if (current[0] + dx, current[1]) in blocked or (current[0], current[1] + dy) in blocked:
                    continue
            new_cost = cost[current] + move_cost
            if new_cost >= cost.get(neighbor, math.inf):
                continue
            cost[neighbor] = new_cost
            came_from[neighbor] = current
            heuristic = math.hypot(goal[0] - neighbor[0], goal[1] - neighbor[1])
            heapq.heappush(frontier, (new_cost + heuristic, neighbor))
    return None


def validate_boxes(boxes: list[Box], config: dict[str, Any]) -> tuple[list[str], list[tuple[float, float]] | None]:
    errors: list[str] = []
    names: set[str] = set()
    gap = float(config["generation"]["inter_obstacle_gap"])
    for index, box in enumerate(boxes):
        if box.name in names:
            errors.append(f"duplicate obstacle name: {box.name}")
        names.add(box.name)
        if box.size_x <= 0.0 or box.size_y <= 0.0 or box.size_z <= 0.0:
            errors.append(f"{box.name}: every size must be positive")
        if not math.isclose(box.z, box.size_z / 2.0, rel_tol=0.0, abs_tol=1e-6):
            errors.append(f"{box.name}: z must equal size_z / 2 so it rests on the floor")
        if not is_boundary_wall(box) and not box_inside_arena(box, config):
            errors.append(f"{box.name}: outside the configured arena")
        for key in ("start", "goal"):
            point = config[key]
            if point_to_box_distance(float(point["x"]), float(point["y"]), box) < float(
                point["exclusion_radius"]
            ):
                errors.append(f"{box.name}: violates the {key} exclusion area")
        for other in boxes[:index]:
            pair_gap = 0.0 if is_boundary_wall(box) and is_boundary_wall(other) else gap
            if boxes_overlap(box, other, pair_gap):
                errors.append(f"{box.name}: overlaps/is too close to {other.name}")

    path = find_path(boxes, config) if not errors else None
    if not errors and path is None:
        errors.append("no collision-free A* path exists between start and goal")
    return errors, path


def _uniform_size(rng: random.Random, profile: dict[str, Any], key: str) -> float:
    low, high = map(float, profile[key])
    return rng.uniform(low, high)


def _config_with_episode_geometry(
    rng: random.Random, config: dict[str, Any]
) -> dict[str, Any]:
    """Return a private config copy with an optional random start and goal."""
    active = copy.deepcopy(config)
    settings = active.get("start_goal_randomization", {})
    if not bool(settings.get("enabled", False)):
        return active

    arena = active["arena"]
    start = active["start"]
    goal = active["goal"]
    default_clearance = max(
        robot_inflation_radius(active),
        float(start["exclusion_radius"]),
        float(goal["exclusion_radius"]),
    )
    clearance = float(settings.get("boundary_clearance", default_clearance))
    minimum_distance = float(settings["minimum_distance"])
    maximum_distance = float(settings.get("maximum_distance", math.inf))
    attempts = int(settings.get("max_sampling_attempts", 1000))

    if clearance <= 0.0:
        raise ValueError("start/goal boundary_clearance must be positive")
    if minimum_distance <= 0.0:
        raise ValueError("start/goal minimum_distance must be positive")
    if maximum_distance < minimum_distance:
        raise ValueError("start/goal maximum_distance is smaller than minimum_distance")
    if attempts <= 0:
        raise ValueError("start/goal max_sampling_attempts must be positive")

    x_min = float(arena["x_min"]) + clearance
    x_max = float(arena["x_max"]) - clearance
    y_min = float(arena["y_min"]) + clearance
    y_max = float(arena["y_max"]) - clearance
    if x_min >= x_max or y_min >= y_max:
        raise ValueError("start/goal boundary clearance leaves no usable arena")

    for _ in range(attempts):
        start_x = rng.uniform(x_min, x_max)
        start_y = rng.uniform(y_min, y_max)
        goal_x = rng.uniform(x_min, x_max)
        goal_y = rng.uniform(y_min, y_max)
        distance = math.hypot(goal_x - start_x, goal_y - start_y)
        if minimum_distance <= distance <= maximum_distance:
            start["x"] = round(start_x, 6)
            start["y"] = round(start_y, 6)
            if bool(settings.get("randomize_start_yaw", True)):
                start["yaw"] = round(rng.uniform(-math.pi, math.pi), 6)
            goal["x"] = round(goal_x, 6)
            goal["y"] = round(goal_y, 6)
            return active

    raise RuntimeError(
        f"could not sample start/goal distance in "
        f"[{minimum_distance}, {maximum_distance}] after {attempts} attempts"
    )


def _random_candidate(
    rng: random.Random,
    profile: dict[str, Any],
    config: dict[str, Any],
    name: str,
    direct_blocker: bool,
) -> Box:
    size_x = round(_uniform_size(rng, profile, "size_x"), 6)
    size_y = round(_uniform_size(rng, profile, "size_y"), 6)
    size_z = round(_uniform_size(rng, profile, "size_z"), 6)
    arena = config["arena"]
    margin = float(arena["boundary_margin"])

    if direct_blocker:
        start, goal = config["start"], config["goal"]
        x0, y0 = float(start["x"]), float(start["y"])
        x1, y1 = float(goal["x"]), float(goal["y"])
        t = rng.uniform(0.28, 0.72)
        length = max(math.hypot(x1 - x0, y1 - y0), 1e-9)
        nx, ny = -(y1 - y0) / length, (x1 - x0) / length
        offset = rng.uniform(-0.12, 0.12)
        x = x0 + t * (x1 - x0) + nx * offset
        y = y0 + t * (y1 - y0) + ny * offset
    else:
        x = rng.uniform(
            float(arena["x_min"]) + margin + size_x / 2.0,
            float(arena["x_max"]) - margin - size_x / 2.0,
        )
        y = rng.uniform(
            float(arena["y_min"]) + margin + size_y / 2.0,
            float(arena["y_max"]) - margin - size_y / 2.0,
        )

    return Box(
        name=name,
        x=round(x, 6),
        y=round(y, 6),
        z=size_z / 2.0,
        size_x=size_x,
        size_y=size_y,
        size_z=size_z,
    )


def generate_boxes(
    config: dict[str, Any], seed: int, difficulty: str, count_override: int | None = None
) -> tuple[list[Box], dict[str, Any]]:
    if difficulty not in config["difficulties"]:
        valid = ", ".join(sorted(config["difficulties"]))
        raise ValueError(f"unknown difficulty '{difficulty}'; choose one of: {valid}")

    rng = random.Random(seed)
    profile = config["difficulties"][difficulty]
    count_range = list(map(int, profile["obstacle_count"]))
    count = int(count_override) if count_override is not None else rng.randint(count_range[0], count_range[1])
    if count < 0:
        raise ValueError("obstacle count cannot be negative")

    generation = config["generation"]
    max_world_attempts = int(generation["max_world_attempts"])
    max_placement_attempts = int(generation["max_placement_attempts"])
    require_blocker = count > 0 and rng.random() < float(profile.get("direct_block_probability", 0.0))

    for world_attempt in range(1, max_world_attempts + 1):
        active_config = _config_with_episode_geometry(rng, config)
        boundary_walls = make_boundary_walls(active_config)
        placed: list[Box] = list(boundary_walls)
        success = True
        for index in range(count):
            placed_this_box = False
            direct = require_blocker and index == 0
            for _ in range(max_placement_attempts):
                candidate = _random_candidate(
                    rng,
                    profile,
                    active_config,
                    f"box_{index:03d}",
                    direct_blocker=direct,
                )
                if candidate_is_valid(candidate, placed, active_config):
                    placed.append(candidate)
                    placed_this_box = True
                    break
            if not placed_this_box:
                success = False
                break
        if not success:
            continue

        errors, path = validate_boxes(placed, active_config)
        if errors or path is None:
            continue
        if require_blocker and not direct_path_blocked(placed, active_config):
            continue

        path_length = sum(
            math.hypot(x1 - x0, y1 - y0)
            for (x0, y0), (x1, y1) in zip(path, path[1:])
        )
        metadata = {
            "seed": seed,
            "difficulty": difficulty,
            "random_obstacle_count": count,
            "boundary_wall_count": len(boundary_walls),
            "total_box_count": len(placed),
            "direct_path_blocked": direct_path_blocked(placed, active_config),
            "astar_path_length_m": round(path_length, 4),
            "generation_attempt": world_attempt,
            "start": active_config["start"],
            "goal": active_config["goal"],
            "robot_inflation_radius_m": round(
                robot_inflation_radius(active_config), 4
            ),
        }
        return placed, metadata

    raise RuntimeError(
        f"could not generate a valid {difficulty} world after {max_world_attempts} attempts; "
        "reduce obstacle count/size or enlarge the arena"
    )
