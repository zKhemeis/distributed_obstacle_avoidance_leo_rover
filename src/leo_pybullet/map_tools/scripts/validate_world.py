#!/usr/bin/env python3
"""Validate geometry and navigability of a leo_pybullet YAML world."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from world_tools import direct_path_blocked, load_boxes, read_yaml, validate_boxes


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--world", type=Path, required=True)
    args = parser.parse_args()

    config = read_yaml(args.config)
    boxes = load_boxes(args.world)
    errors, path = validate_boxes(boxes, config)
    if errors:
        print(f"INVALID: {args.world}")
        for error in errors:
            print(f"  - {error}")
        raise SystemExit(1)

    assert path is not None
    length = sum(
        math.hypot(x1 - x0, y1 - y0)
        for (x0, y0), (x1, y1) in zip(path, path[1:])
    )
    print(f"VALID: {args.world}")
    print(f"  obstacles: {len(boxes)}")
    print(f"  direct path blocked: {direct_path_blocked(boxes, config)}")
    print(f"  A* path exists: yes ({length:.2f} m)")


if __name__ == "__main__":
    main()

