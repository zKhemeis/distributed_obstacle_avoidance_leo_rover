#include "leo_pybullet/bullet_sim_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "btBulletDynamicsCommon.h"
#include "yaml-cpp/yaml.h"

namespace leo_pybullet
{

namespace
{

Transform3D toTransform3D(const btTransform & transform)
{
  const auto & position = transform.getOrigin();
  const auto & rotation = transform.getRotation();
  return {
    position.x(), position.y(), position.z(),
    rotation.x(), rotation.y(), rotation.z(), rotation.w()
  };
}

Vector3D toVector3D(const btVector3 & value)
{
  return {value.x(), value.y(), value.z()};
}

double yawOf(const btTransform & transform)
{
  btScalar roll;
  btScalar pitch;
  btScalar yaw;
  transform.getBasis().getEulerYPR(yaw, pitch, roll);
  return static_cast<double>(yaw);
}

}  // namespace

class BulletSimCore::Impl
{
public:
  struct Wheel
  {
    btRigidBody * body = nullptr;
    btHingeConstraint * hinge = nullptr;
    bool left_side = true;
    double target_velocity = 0.0;
  };

  ~Impl()
  {
    clear();
  }

  void reset(
    const std::string & world_file,
    double start_x,
    double start_y,
    double start_yaw)
  {
    clear();
    loadWorldFromYaml(world_file);
    initBulletWorld(start_x, start_y, start_yaw);
  }

  void setCommand(double linear_velocity, double angular_velocity)
  {
    linear_velocity_ = linear_velocity;
    angular_velocity_ = angular_velocity;
  }

  void step(std::size_t physics_steps)
  {
    requireInitialized();
    const double dt = 1.0 / BulletSimCore::kPhysicsRate;
    for (std::size_t index = 0; index < physics_steps; ++index) {
      applyWheelMotorCommands(dt);
      dynamics_world_->stepSimulation(dt, 1, dt);
      simulation_time_ += dt;
      ++step_count_;
    }
  }

  RobotState robotState() const
  {
    requireInitialized();
    const btTransform transform = robot_body_->getWorldTransform();
    RobotState state;
    state.transform = toTransform3D(transform);
    state.linear_velocity = toVector3D(robot_body_->getLinearVelocity());
    state.angular_velocity = toVector3D(robot_body_->getAngularVelocity());
    state.yaw = yawOf(transform);
    return state;
  }

  std::array<WheelState, 4> wheelStates() const
  {
    requireInitialized();
    if (wheels_.size() != 4) {
      throw std::runtime_error("Leo simulator must contain exactly four wheels");
    }

    std::array<WheelState, 4> states;
    for (std::size_t index = 0; index < wheels_.size(); ++index) {
      states[index].transform = toTransform3D(wheels_[index].body->getWorldTransform());
      states[index].position = wheel_positions_[index];
      states[index].target_velocity = wheels_[index].target_velocity;
    }
    return states;
  }

  LaserScanData laserScan()
  {
    requireInitialized();

    LaserScanData scan;
    scan.angle_min = 0.0;
    scan.angle_max = 2.0 * M_PI;
    scan.angle_increment =
      (scan.angle_max - scan.angle_min) /
      static_cast<double>(BulletSimCore::kNumberOfRays);
    scan.range_min = BulletSimCore::kLidarRangeMin;
    scan.range_max = BulletSimCore::kLidarRangeMax;
    scan.scan_time =
      static_cast<double>(BulletSimCore::kControlPhysicsSteps) /
      BulletSimCore::kPhysicsRate;
    scan.ranges.assign(
      BulletSimCore::kNumberOfRays,
      std::numeric_limits<float>::infinity());

    const btTransform robot_transform = robot_body_->getWorldTransform();
    const btVector3 robot_position = robot_transform.getOrigin();
    const double yaw = yawOf(robot_transform);

    constexpr double lidar_x = 0.10;
    constexpr double lidar_y = 0.0;
    constexpr double lidar_z = 0.08;
    constexpr double ray_start_offset = 0.08;

    const double lidar_world_x =
      robot_position.x() + std::cos(yaw) * lidar_x - std::sin(yaw) * lidar_y;
    const double lidar_world_y =
      robot_position.y() + std::sin(yaw) * lidar_x + std::cos(yaw) * lidar_y;
    const double lidar_world_z = robot_position.z() + lidar_z;

    for (std::size_t index = 0; index < BulletSimCore::kNumberOfRays; ++index) {
      const double local_angle = scan.angle_min + index * scan.angle_increment;
      const double global_angle = yaw + local_angle;

      const btVector3 ray_from(
        lidar_world_x + ray_start_offset * std::cos(global_angle),
        lidar_world_y + ray_start_offset * std::sin(global_angle),
        lidar_world_z);
      const btVector3 ray_to(
        lidar_world_x + scan.range_max * std::cos(global_angle),
        lidar_world_y + scan.range_max * std::sin(global_angle),
        lidar_world_z);

      btCollisionWorld::ClosestRayResultCallback callback(ray_from, ray_to);
      callback.m_collisionFilterGroup = btBroadphaseProxy::SensorTrigger;
      dynamics_world_->rayTest(ray_from, ray_to, callback);

      if (!callback.hasHit()) {
        continue;
      }

      const double distance =
        ray_start_offset +
        (scan.range_max - ray_start_offset) * callback.m_closestHitFraction;
      if (distance >= scan.range_min && distance <= scan.range_max) {
        scan.ranges[index] = static_cast<float>(distance);
      }
    }
    return scan;
  }

  bool hasCollision() const
  {
    requireInitialized();
    for (int index = 0; index < dispatcher_->getNumManifolds(); ++index) {
      const btPersistentManifold * manifold = dispatcher_->getManifoldByIndexInternal(index);
      const auto * body_a = static_cast<const btCollisionObject *>(manifold->getBody0());
      const auto * body_b = static_cast<const btCollisionObject *>(manifold->getBody1());

      const bool robot_a = isRobotPart(body_a);
      const bool robot_b = isRobotPart(body_b);
      const bool obstacle_a = isObstacle(body_a);
      const bool obstacle_b = isObstacle(body_b);
      if (!((robot_a && obstacle_b) || (robot_b && obstacle_a))) {
        continue;
      }

      for (int point = 0; point < manifold->getNumContacts(); ++point) {
        if (manifold->getContactPoint(point).getDistance() <= 0.0) {
          return true;
        }
      }
    }
    return false;
  }

  bool initialized() const
  {
    return dynamics_world_ != nullptr && robot_body_ != nullptr;
  }

  std::vector<BoxObstacle> obstacles_;
  double simulation_time_ = 0.0;
  std::size_t step_count_ = 0;

private:
  void requireInitialized() const
  {
    if (!initialized()) {
      throw std::runtime_error("BulletSimCore has not been reset with a world");
    }
  }

  void clear()
  {
    if (dynamics_world_ != nullptr) {
      for (auto * constraint : constraints_) {
        dynamics_world_->removeConstraint(constraint);
        delete constraint;
      }
      for (auto * body : rigid_bodies_) {
        dynamics_world_->removeRigidBody(body);
        delete body->getMotionState();
        delete body;
      }
    }

    for (auto * shape : collision_shapes_) {
      delete shape;
    }

    constraints_.clear();
    rigid_bodies_.clear();
    collision_shapes_.clear();
    obstacle_bodies_.clear();
    wheels_.clear();
    obstacles_.clear();
    robot_body_ = nullptr;

    delete dynamics_world_;
    delete solver_;
    delete dispatcher_;
    delete collision_config_;
    delete broadphase_;

    dynamics_world_ = nullptr;
    solver_ = nullptr;
    dispatcher_ = nullptr;
    collision_config_ = nullptr;
    broadphase_ = nullptr;

    linear_velocity_ = 0.0;
    angular_velocity_ = 0.0;
    simulation_time_ = 0.0;
    step_count_ = 0;
    wheel_positions_.fill(0.0);
  }

  void loadWorldFromYaml(const std::string & world_file)
  {
    const YAML::Node config = YAML::LoadFile(world_file);
    const YAML::Node obstacle_nodes = config["obstacles"];
    if (!obstacle_nodes || !obstacle_nodes.IsSequence()) {
      throw std::runtime_error("World YAML must contain an 'obstacles' sequence: " + world_file);
    }

    for (const auto & node : obstacle_nodes) {
      BoxObstacle obstacle;
      obstacle.name = node["name"].as<std::string>();
      obstacle.x = node["x"].as<double>();
      obstacle.y = node["y"].as<double>();
      obstacle.z = node["z"].as<double>();
      obstacle.size_x = node["size_x"].as<double>();
      obstacle.size_y = node["size_y"].as<double>();
      obstacle.size_z = node["size_z"].as<double>();
      if (obstacle.size_x <= 0.0 || obstacle.size_y <= 0.0 || obstacle.size_z <= 0.0) {
        throw std::runtime_error("Obstacle sizes must be positive: " + obstacle.name);
      }
      obstacles_.push_back(std::move(obstacle));
    }
  }

  void initBulletWorld(double start_x, double start_y, double start_yaw)
  {
    broadphase_ = new btDbvtBroadphase();
    collision_config_ = new btDefaultCollisionConfiguration();
    dispatcher_ = new btCollisionDispatcher(collision_config_);
    solver_ = new btSequentialImpulseConstraintSolver();
    dynamics_world_ = new btDiscreteDynamicsWorld(
      dispatcher_, broadphase_, solver_, collision_config_);
    dynamics_world_->setGravity(btVector3(0, 0, -9.81));

    auto * ground_shape = new btStaticPlaneShape(btVector3(0, 0, 1), 0);
    btTransform ground_transform;
    ground_transform.setIdentity();
    createRigidBody(0.0, ground_transform, ground_shape);

    createRobot(start_x, start_y, start_yaw);
    for (const auto & obstacle : obstacles_) {
      addBoxObstacle(obstacle);
    }
  }

  btRigidBody * createRigidBody(
    double mass,
    const btTransform & transform,
    btCollisionShape * shape)
  {
    btVector3 local_inertia(0, 0, 0);
    if (mass > 0.0) {
      shape->calculateLocalInertia(mass, local_inertia);
    }

    auto * motion_state = new btDefaultMotionState(transform);
    btRigidBody::btRigidBodyConstructionInfo info(mass, motion_state, shape, local_inertia);
    auto * body = new btRigidBody(info);
    dynamics_world_->addRigidBody(body);
    rigid_bodies_.push_back(body);
    collision_shapes_.push_back(shape);
    return body;
  }

  void createRobot(double start_x, double start_y, double start_yaw)
  {
    auto * chassis_shape = new btBoxShape(btVector3(0.21, 0.14, 0.07));
    btTransform chassis_transform;
    chassis_transform.setIdentity();
    chassis_transform.setOrigin(btVector3(start_x, start_y, 0.17));
    chassis_transform.setRotation(btQuaternion(btVector3(0, 0, 1), start_yaw));

    robot_body_ = createRigidBody(4.36, chassis_transform, chassis_shape);
    robot_body_->setActivationState(DISABLE_DEACTIVATION);
    robot_body_->setFriction(1.0);
    robot_body_->setDamping(0.05, 0.2);
    robot_body_->setAngularFactor(btVector3(0.0, 0.0, 1.0));

    addWheel(-0.15256, 0.224, true);
    addWheel(0.15256, 0.224, true);
    addWheel(0.15256, -0.224, false);
    addWheel(-0.15256, -0.224, false);
  }

  void addWheel(double local_x, double local_y, bool left_side)
  {
    constexpr double wheel_radius = 0.0625;
    constexpr double wheel_width = 0.07;
    auto * wheel_shape = new btCylinderShape(
      btVector3(wheel_radius, wheel_width / 2.0, wheel_radius));

    const btTransform chassis_transform = robot_body_->getWorldTransform();
    btTransform wheel_transform;
    wheel_transform.setIdentity();
    wheel_transform.setRotation(chassis_transform.getRotation());
    wheel_transform.setOrigin(
      chassis_transform * btVector3(local_x, local_y, -0.10));

    auto * wheel_body = createRigidBody(0.283642, wheel_transform, wheel_shape);
    wheel_body->setActivationState(DISABLE_DEACTIVATION);
    wheel_body->setFriction(2.0);
    wheel_body->setRollingFriction(0.02);
    wheel_body->setDamping(0.01, 0.01);

    const btVector3 pivot_in_chassis(local_x, local_y, -0.1175);
    const btVector3 pivot_in_wheel(0, 0, 0);
    const btVector3 axis_in_chassis(0, 1, 0);
    const btVector3 axis_in_wheel(0, 1, 0);
    auto * hinge = new btHingeConstraint(
      *robot_body_, *wheel_body,
      pivot_in_chassis, pivot_in_wheel,
      axis_in_chassis, axis_in_wheel, true);
    hinge->setLimit(1.0, -1.0);
    hinge->enableAngularMotor(true, 0.0, motor_max_impulse_);

    dynamics_world_->addConstraint(hinge, true);
    constraints_.push_back(hinge);
    wheels_.push_back({wheel_body, hinge, left_side, 0.0});
  }

  void addBoxObstacle(const BoxObstacle & obstacle)
  {
    auto * shape = new btBoxShape(
      btVector3(obstacle.size_x / 2.0, obstacle.size_y / 2.0, obstacle.size_z / 2.0));
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(obstacle.x, obstacle.y, obstacle.z));
    auto * body = createRigidBody(0.0, transform, shape);
    body->setFriction(1.0);
    obstacle_bodies_.push_back(body);
  }

  void applyWheelMotorCommands(double dt)
  {
    const double left_linear =
      linear_velocity_ - angular_velocity_ * track_width_ / 2.0;
    const double right_linear =
      linear_velocity_ + angular_velocity_ * track_width_ / 2.0;
    const double left_speed = left_linear / wheel_radius_;
    const double right_speed = right_linear / wheel_radius_;

    for (std::size_t index = 0; index < wheels_.size(); ++index) {
      auto & wheel = wheels_[index];
      const double target_speed = wheel.left_side ? left_speed : right_speed;
      wheel.target_velocity = target_speed;
      wheel_positions_[index] += target_speed * dt;
      wheel.hinge->enableAngularMotor(true, target_speed, motor_max_impulse_);
    }
  }

  bool isRobotPart(const btCollisionObject * object) const
  {
    if (object == robot_body_) {
      return true;
    }
    return std::any_of(
      wheels_.begin(), wheels_.end(),
      [object](const Wheel & wheel) {return object == wheel.body;});
  }

  bool isObstacle(const btCollisionObject * object) const
  {
    return std::find(obstacle_bodies_.begin(), obstacle_bodies_.end(), object) !=
           obstacle_bodies_.end();
  }

  btBroadphaseInterface * broadphase_ = nullptr;
  btDefaultCollisionConfiguration * collision_config_ = nullptr;
  btCollisionDispatcher * dispatcher_ = nullptr;
  btSequentialImpulseConstraintSolver * solver_ = nullptr;
  btDiscreteDynamicsWorld * dynamics_world_ = nullptr;

  std::vector<btCollisionShape *> collision_shapes_;
  std::vector<btRigidBody *> rigid_bodies_;
  std::vector<btRigidBody *> obstacle_bodies_;
  std::vector<btTypedConstraint *> constraints_;
  std::vector<Wheel> wheels_;
  btRigidBody * robot_body_ = nullptr;

  std::array<double, 4> wheel_positions_ = {0.0, 0.0, 0.0, 0.0};
  double linear_velocity_ = 0.0;
  double angular_velocity_ = 0.0;
  const double wheel_radius_ = 0.0625;
  const double track_width_ = 0.448;
  const double motor_max_impulse_ = 5.0;
};

BulletSimCore::BulletSimCore()
: impl_(std::make_unique<Impl>())
{
}

BulletSimCore::~BulletSimCore() = default;
BulletSimCore::BulletSimCore(BulletSimCore &&) noexcept = default;
BulletSimCore & BulletSimCore::operator=(BulletSimCore &&) noexcept = default;

void BulletSimCore::reset(
  const std::string & world_file,
  double start_x,
  double start_y,
  double start_yaw)
{
  impl_->reset(world_file, start_x, start_y, start_yaw);
}

void BulletSimCore::setCommand(double linear_velocity, double angular_velocity)
{
  impl_->setCommand(linear_velocity, angular_velocity);
}

void BulletSimCore::step(std::size_t physics_steps)
{
  impl_->step(physics_steps);
}

RobotState BulletSimCore::robotState() const
{
  return impl_->robotState();
}

std::array<WheelState, 4> BulletSimCore::wheelStates() const
{
  return impl_->wheelStates();
}

LaserScanData BulletSimCore::laserScan()
{
  return impl_->laserScan();
}

bool BulletSimCore::hasCollision() const
{
  return impl_->hasCollision();
}

const std::vector<BoxObstacle> & BulletSimCore::obstacles() const
{
  return impl_->obstacles_;
}

double BulletSimCore::simulationTime() const
{
  return impl_->simulation_time_;
}

std::size_t BulletSimCore::stepCount() const
{
  return impl_->step_count_;
}

bool BulletSimCore::initialized() const
{
  return impl_->initialized();
}

}  // namespace leo_pybullet
