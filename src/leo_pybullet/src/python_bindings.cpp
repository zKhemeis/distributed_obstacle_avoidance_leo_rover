#include <cstddef>
#include <string>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "leo_pybullet/bullet_sim_core.hpp"

namespace py = pybind11;
using leo_pybullet::BoxObstacle;
using leo_pybullet::BulletSimCore;
using leo_pybullet::LaserScanData;
using leo_pybullet::RobotState;
using leo_pybullet::Transform3D;
using leo_pybullet::Vector3D;
using leo_pybullet::WheelState;

PYBIND11_MODULE(leo_bullet_sim, module)
{
  module.doc() = "Direct Python binding for the Leo Rover C++ Bullet simulation core";

  py::class_<Transform3D>(module, "Transform3D")
  .def_readonly("x", &Transform3D::x)
  .def_readonly("y", &Transform3D::y)
  .def_readonly("z", &Transform3D::z)
  .def_readonly("qx", &Transform3D::qx)
  .def_readonly("qy", &Transform3D::qy)
  .def_readonly("qz", &Transform3D::qz)
  .def_readonly("qw", &Transform3D::qw);

  py::class_<Vector3D>(module, "Vector3D")
  .def_readonly("x", &Vector3D::x)
  .def_readonly("y", &Vector3D::y)
  .def_readonly("z", &Vector3D::z);

  py::class_<RobotState>(module, "RobotState")
  .def_readonly("transform", &RobotState::transform)
  .def_readonly("linear_velocity", &RobotState::linear_velocity)
  .def_readonly("angular_velocity", &RobotState::angular_velocity)
  .def_readonly("yaw", &RobotState::yaw);

  py::class_<WheelState>(module, "WheelState")
  .def_readonly("transform", &WheelState::transform)
  .def_readonly("position", &WheelState::position)
  .def_readonly("target_velocity", &WheelState::target_velocity);

  py::class_<LaserScanData>(module, "LaserScanData")
  .def_readonly("angle_min", &LaserScanData::angle_min)
  .def_readonly("angle_max", &LaserScanData::angle_max)
  .def_readonly("angle_increment", &LaserScanData::angle_increment)
  .def_readonly("range_min", &LaserScanData::range_min)
  .def_readonly("range_max", &LaserScanData::range_max)
  .def_readonly("scan_time", &LaserScanData::scan_time)
  .def_readonly("ranges", &LaserScanData::ranges);

  py::class_<BoxObstacle>(module, "BoxObstacle")
  .def_readonly("name", &BoxObstacle::name)
  .def_readonly("x", &BoxObstacle::x)
  .def_readonly("y", &BoxObstacle::y)
  .def_readonly("z", &BoxObstacle::z)
  .def_readonly("size_x", &BoxObstacle::size_x)
  .def_readonly("size_y", &BoxObstacle::size_y)
  .def_readonly("size_z", &BoxObstacle::size_z);

  py::class_<BulletSimCore>(module, "BulletSim")
  .def(py::init<>())
  .def(
    "reset", &BulletSimCore::reset,
    py::arg("world_file"),
    py::arg("start_x") = 0.0,
    py::arg("start_y") = 0.0,
    py::arg("start_yaw") = 0.0,
    py::call_guard<py::gil_scoped_release>())
  .def(
    "set_command", &BulletSimCore::setCommand,
    py::arg("linear_velocity"), py::arg("angular_velocity"))
  .def(
    "step", &BulletSimCore::step,
    py::arg("physics_steps") = 1,
    py::call_guard<py::gil_scoped_release>())
  .def("robot_state", &BulletSimCore::robotState)
  .def("wheel_states", &BulletSimCore::wheelStates)
  .def("laser_scan", &BulletSimCore::laserScan)
  .def("has_collision", &BulletSimCore::hasCollision)
  .def_property_readonly(
    "obstacles", [](const BulletSimCore & simulation) {
      return simulation.obstacles();
    })
  .def_property_readonly("simulation_time", &BulletSimCore::simulationTime)
  .def_property_readonly("step_count", &BulletSimCore::stepCount)
  .def_property_readonly("initialized", &BulletSimCore::initialized);

  module.attr("PHYSICS_RATE") = BulletSimCore::kPhysicsRate;
  module.attr("CONTROL_PHYSICS_STEPS") = BulletSimCore::kControlPhysicsSteps;
  module.attr("NUMBER_OF_RAYS") = BulletSimCore::kNumberOfRays;
  module.attr("LIDAR_RANGE_MIN") = BulletSimCore::kLidarRangeMin;
  module.attr("LIDAR_RANGE_MAX") = BulletSimCore::kLidarRangeMax;
}
