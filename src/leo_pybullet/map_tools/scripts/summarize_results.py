#!/usr/bin/env python3
"""Summarize completed evaluation rows by split and difficulty."""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path


def truthy(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "y"}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results", type=Path, required=True)
    args = parser.parse_args()

    groups: dict[tuple[str, str], list[dict[str, str]]] = defaultdict(list)
    with args.results.open("r", newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if not row.get("success", "").strip():
                continue
            groups[(row.get("split", "unknown"), row.get("difficulty", "unknown"))].append(row)

    if not groups:
        print("No completed evaluation rows found.")
        return
    print("split,difficulty,episodes,success_rate,collision_rate,timeout_rate,mean_duration_s")
    for key in sorted(groups):
        rows = groups[key]
        n = len(rows)
        success = sum(truthy(row["success"]) for row in rows) / n
        collision = sum(truthy(row["collision"]) for row in rows) / n
        timeout = sum(truthy(row["timeout"]) for row in rows) / n
        durations = [float(row["duration_s"]) for row in rows if row.get("duration_s", "").strip()]
        mean_duration = sum(durations) / len(durations) if durations else float("nan")
        print(f"{key[0]},{key[1]},{n},{success:.3f},{collision:.3f},{timeout:.3f},{mean_duration:.3f}")


if __name__ == "__main__":
    main()

