#!/usr/bin/env python3
"""Create a dependency-light SVG preview of a generated world and A* path."""

from __future__ import annotations

import argparse
import html
from pathlib import Path

from world_tools import (
    find_path,
    load_boxes,
    read_yaml,
    robot_inflation_radius,
    validate_boxes,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--delete-invalid",
        action="store_true",
        help="delete the invalid YAML, its .meta.json, and a stale preview",
    )
    args = parser.parse_args()

    config = read_yaml(args.config)
    boxes = load_boxes(args.world)
    errors, validated_path = validate_boxes(boxes, config)
    if errors:
        print(f"INVALID: {args.world}")
        for error in errors:
            print(f"  - {error}")

        if args.delete_invalid:
            if args.world.suffix.lower() not in {".yaml", ".yml"}:
                raise SystemExit("refusing deletion: --world must be a .yaml or .yml file")
            files_to_delete = (
                args.world,
                args.world.with_suffix(".meta.json"),
                args.output,
            )
            for path in files_to_delete:
                if path.is_file():
                    path.unlink()
                    print(f"Deleted: {path}")
        else:
            print("Map was not deleted. Add --delete-invalid to delete invalid artifacts.")
        raise SystemExit(1)

    path = validated_path if validated_path is not None else find_path(boxes, config)
    arena = config["arena"]
    xmin, xmax = float(arena["x_min"]), float(arena["x_max"])
    ymin, ymax = float(arena["y_min"]), float(arena["y_max"])
    width_px = 1000
    height_px = max(300, round(width_px * (ymax - ymin) / (xmax - xmin)))

    def sx(x: float) -> float:
        return (x - xmin) / (xmax - xmin) * width_px

    def sy(y: float) -> float:
        return height_px - (y - ymin) / (ymax - ymin) * height_px

    def sw(value: float) -> float:
        return value / (xmax - xmin) * width_px

    def sh(value: float) -> float:
        return value / (ymax - ymin) * height_px

    inflation = robot_inflation_radius(config)
    svg: list[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width_px}" height="{height_px}" viewBox="0 0 {width_px} {height_px}">',
        '<rect width="100%" height="100%" fill="#f7f7f7" stroke="#222" stroke-width="3"/>',
    ]
    for box in boxes:
        svg.append(
            f'<rect x="{sx(box.x_min - inflation):.2f}" y="{sy(box.y_max + inflation):.2f}" '
            f'width="{sw(box.size_x + 2 * inflation):.2f}" height="{sh(box.size_y + 2 * inflation):.2f}" '
            'fill="#e45756" opacity="0.18"/>'
        )
        svg.append(
            f'<rect x="{sx(box.x_min):.2f}" y="{sy(box.y_max):.2f}" '
            f'width="{sw(box.size_x):.2f}" height="{sh(box.size_y):.2f}" '
            'fill="#e45756" stroke="#8b1e1e" stroke-width="2"/>'
        )
        svg.append(
            f'<text x="{sx(box.x):.2f}" y="{sy(box.y):.2f}" text-anchor="middle" '
            f'font-size="12" fill="white">{html.escape(box.name)}</text>'
        )
    if path:
        points = " ".join(f"{sx(x):.2f},{sy(y):.2f}" for x, y in path)
        svg.append(f'<polyline points="{points}" fill="none" stroke="#3366cc" stroke-width="4"/>')
    for key, color in (("start", "#2ca02c"), ("goal", "#ffbf00")):
        point = config[key]
        svg.append(
            f'<circle cx="{sx(float(point["x"])):.2f}" cy="{sy(float(point["y"])):.2f}" '
            f'r="10" fill="{color}" stroke="#222" stroke-width="2"/>'
        )
        svg.append(
            f'<text x="{sx(float(point["x"])) + 14:.2f}" y="{sy(float(point["y"])) - 12:.2f}" '
            f'font-size="18" fill="#222">{key}</text>'
        )
    svg.append('</svg>')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(svg) + "\n", encoding="utf-8")
    print(f"Preview: {args.output}")


if __name__ == "__main__":
    main()
