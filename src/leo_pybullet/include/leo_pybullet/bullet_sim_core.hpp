#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace leo_pybullet
{

struct BoxObstacle
{
  std::string name;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double size_x = 0.0;
  double size_y = 0.0;
  double size_z = 0.0;
};

struct Transform3D
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
};

struct Vector3D
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct RobotState
{
  Transform3D transform;
  Vector3D linear_velocity;
  Vector3D angular_velocity;
  double yaw = 0.0;
};

struct WheelState
{
  Transform3D transform;
  double position = 0.0;
  double target_velocity = 0.0;
};

struct LaserScanData
{
  double angle_min = 0.0;
  double angle_max = 0.0;
  double angle_increment = 0.0;
  double range_min = 0.0;
  double range_max = 0.0;
  double scan_time = 0.0;
  std::vector<float> ranges;
};

class BulletSimCore
{
public:
  static constexpr double kPhysicsRate = 240.0;
  static constexpr std::size_t kControlPhysicsSteps = 24;
  static constexpr std::size_t kNumberOfRays = 500;
  static constexpr double kLidarRangeMin = 0.05;
  static constexpr double kLidarRangeMax = 12.0;

  BulletSimCore();
  ~BulletSimCore();

  BulletSimCore(const BulletSimCore &) = delete;
  BulletSimCore & operator=(const BulletSimCore &) = delete;
  BulletSimCore(BulletSimCore &&) noexcept;
  BulletSimCore & operator=(BulletSimCore &&) noexcept;

  void reset(
    const std::string & world_file,
    double start_x = 0.0,
    double start_y = 0.0,
    double start_yaw = 0.0);

  void setCommand(double linear_velocity, double angular_velocity);
  void step(std::size_t physics_steps = 1);

  RobotState robotState() const;
  std::array<WheelState, 4> wheelStates() const;
  LaserScanData laserScan();
  bool hasCollision() const;

  const std::vector<BoxObstacle> & obstacles() const;
  double simulationTime() const;
  std::size_t stepCount() const;
  bool initialized() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace leo_pybullet
