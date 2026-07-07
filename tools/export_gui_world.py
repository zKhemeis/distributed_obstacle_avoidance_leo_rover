#!/usr/bin/env python3

import argparse
import ast
import re
from pathlib import Path


def extract_data_string(raw_text: str) -> str:
    match = re.search(r'data:\s*"(.*)"\s*$', raw_text, re.S)
    if not match:
        raise RuntimeError("Could not find Gazebo data string in raw file.")

    return ast.literal_eval('"' + match.group(1) + '"')


def extract_box_models(sdf_text: str) -> list[str]:
    return re.findall(
        r"    <model name='box(?:_[0-9]+(?:_[0-9]+)?)?'>.*?    </model>",
        sdf_text,
        re.S,
    )


def main():
    parser = argparse.ArgumentParser(
        description="Convert Gazebo generated_world_sdf output into a clean Leo world."
    )
    parser.add_argument("raw_file", help="Raw file created from generate_world_sdf")
    parser.add_argument("output_file", help="Clean output .sdf world file")
    parser.add_argument(
        "--base-world",
        default="/root/leo_ws/src/leo_simulator-ros2/leo_gz_worlds/worlds/leo_empty.sdf",
        help="Base world file used as template",
    )

    args = parser.parse_args()

    raw_path = Path(args.raw_file)
    out_path = Path(args.output_file)
    base_path = Path(args.base_world)

    raw_text = raw_path.read_text()
    sdf_text = extract_data_string(raw_text)
    box_models = extract_box_models(sdf_text)

    if not box_models:
        raise RuntimeError("No box models found in exported world.")

    clean_boxes = []
    for box in box_models:
        box = re.sub(r"<static>false</static>", "<static>true</static>", box)
        clean_boxes.append(box)

    base_text = base_path.read_text()

    insert_text = (
        "\n\n    <!-- GUI-generated obstacle boxes -->\n\n"
        + "\n\n".join(clean_boxes)
        + "\n"
    )

    if "</world>" not in base_text:
        raise RuntimeError("Could not find </world> in base world.")

    new_world = base_text.replace("</world>", insert_text + "  </world>")

    out_path.write_text(new_world)

    print(f"Found {len(box_models)} box models")
    print(f"Saved clean world: {out_path}")


if __name__ == "__main__":
    main()
