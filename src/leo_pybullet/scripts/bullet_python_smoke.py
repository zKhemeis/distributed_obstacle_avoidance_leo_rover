#!/usr/bin/env python3
"""Smoke-test the direct Python binding without ROS or real-time sleeping."""

import argparse
import math
import time

from leo_bullet_sim import BulletSim, NUMBER_OF_RAYS, PHYSICS_RATE


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('world_file')
    parser.add_argument('--physics-steps', type=int, default=1200)
    parser.add_argument('--linear', type=float, default=0.15)
    parser.add_argument('--angular', type=float, default=0.0)
    args = parser.parse_args()

    if args.physics_steps <= 0:
        parser.error('--physics-steps must be positive')

    simulation = BulletSim()
    simulation.reset(args.world_file)
    simulation.set_command(args.linear, args.angular)

    wall_start = time.perf_counter()
    simulation.step(args.physics_steps)
    wall_seconds = time.perf_counter() - wall_start

    state = simulation.robot_state()
    scan = simulation.laser_scan()
    expected_sim_seconds = args.physics_steps / PHYSICS_RATE
    real_time_factor = expected_sim_seconds / max(wall_seconds, 1e-12)

    if len(scan.ranges) != NUMBER_OF_RAYS:
        raise RuntimeError(
            f'Expected {NUMBER_OF_RAYS} rays, received {len(scan.ranges)}')
    if any(math.isnan(value) or value == -math.inf for value in scan.ranges):
        raise RuntimeError('LiDAR contains NaN or negative infinity')
    finite_ranges = [
        value for value in scan.ranges if math.isfinite(value)
    ]
    if any(
            value < scan.range_min or value > scan.range_max
            for value in finite_ranges):
        raise RuntimeError('LiDAR contains a finite value outside its limits')
    no_return_rays = len(scan.ranges) - len(finite_ranges)

    print(f'sim_time_s={simulation.simulation_time:.6f}')
    print(f'wall_time_s={wall_seconds:.6f}')
    print(f'rtf={real_time_factor:.3f}')
    print(f'pose_x={state.transform.x:.6f}')
    print(f'pose_y={state.transform.y:.6f}')
    print(f'yaw={state.yaw:.6f}')
    print(f'lidar_rays={len(scan.ranges)}')
    print(f'min_range={min(finite_ranges, default=scan.range_max):.6f}')
    print(f'no_return_rays={no_return_rays}')
    print(f'collision={str(simulation.has_collision()).lower()}')
    print(f'obstacles={len(simulation.obstacles)}')


if __name__ == '__main__':
    main()
