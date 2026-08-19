#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

#include "leo_pybullet/bullet_sim_core.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: bullet_core_smoke WORLD.yaml\n";
    return 2;
  }

  try {
    leo_pybullet::BulletSimCore simulator;
    simulator.reset(argv[1]);
    simulator.setCommand(0.15, 0.0);

    const auto wall_start = std::chrono::steady_clock::now();
    constexpr std::size_t control_steps = 50;  // 5 simulated seconds at 10 Hz.
    for (std::size_t index = 0; index < control_steps; ++index) {
      simulator.step(leo_pybullet::BulletSimCore::kControlPhysicsSteps);
      const auto scan = simulator.laserScan();
      if (scan.ranges.size() != leo_pybullet::BulletSimCore::kNumberOfRays) {
        std::cerr << "Unexpected scan size: " << scan.ranges.size() << "\n";
        return 1;
      }
      if (simulator.hasCollision()) {
        std::cerr << "Unexpected collision in smoke test at step " << index << "\n";
        return 1;
      }
    }
    const double wall_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - wall_start).count();
    const auto state = simulator.robotState();
    const double rtf = simulator.simulationTime() / wall_seconds;

    std::cout << "sim_time_s=" << simulator.simulationTime() << "\n";
    std::cout << "wall_time_s=" << wall_seconds << "\n";
    std::cout << "rtf=" << rtf << "\n";
    std::cout << "pose_x=" << state.transform.x << "\n";
    std::cout << "pose_y=" << state.transform.y << "\n";
    std::cout << "yaw=" << state.yaw << "\n";
    std::cout << "obstacles=" << simulator.obstacles().size() << "\n";

    if (!std::isfinite(rtf) || rtf <= 1.0) {
      std::cerr << "Headless core did not run faster than real time\n";
      return 1;
    }
  } catch (const std::exception & error) {
    std::cerr << "Smoke test failed: " << error.what() << "\n";
    return 1;
  }
  return 0;
}
