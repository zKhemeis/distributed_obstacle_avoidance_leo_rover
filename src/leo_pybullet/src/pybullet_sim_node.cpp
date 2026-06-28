#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "btBulletDynamicsCommon.h"

class LeoPybulletSimNode : public rclcpp::Node
{
public:
  LeoPybulletSimNode()
  : Node("leo_pybullet_sim_node")
  {
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&LeoPybulletSimNode::cmdCallback, this, std::placeholders::_1)
    );

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    initBulletWorld();

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / physics_rate_),
      std::bind(&LeoPybulletSimNode::stepSimulation, this)
    );

    start_wall_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "Real Bullet physics simulator started");
  }

  ~LeoPybulletSimNode()
  {
    for (auto body : rigid_bodies_) {
      dynamics_world_->removeRigidBody(body);
      delete body->getMotionState();
      delete body;
    }

    for (auto shape : collision_shapes_) {
      delete shape;
    }

    delete dynamics_world_;
    delete solver_;
    delete dispatcher_;
    delete collision_config_;
    delete broadphase_;
  }

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    linear_velocity_ = msg->linear.x;
    angular_velocity_ = msg->angular.z;
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

    btRigidBody::btRigidBodyConstructionInfo rb_info(
      mass, motion_state, shape, local_inertia
    );

    auto * body = new btRigidBody(rb_info);
    dynamics_world_->addRigidBody(body);

    rigid_bodies_.push_back(body);
    collision_shapes_.push_back(shape);

    return body;
  }

  void initBulletWorld()
  {
    broadphase_ = new btDbvtBroadphase();
    collision_config_ = new btDefaultCollisionConfiguration();
    dispatcher_ = new btCollisionDispatcher(collision_config_);
    solver_ = new btSequentialImpulseConstraintSolver();

    dynamics_world_ = new btDiscreteDynamicsWorld(
      dispatcher_,
      broadphase_,
      solver_,
      collision_config_
    );

    dynamics_world_->setGravity(btVector3(0, 0, -9.81));

    // Ground plane
    auto * ground_shape = new btStaticPlaneShape(btVector3(0, 0, 1), 0);
    btTransform ground_tf;
    ground_tf.setIdentity();
    ground_tf.setOrigin(btVector3(0, 0, 0));
    createRigidBody(0.0, ground_tf, ground_shape);

    // Simple robot body
    auto * robot_shape = new btBoxShape(btVector3(0.25, 0.18, 0.10));
    btTransform robot_tf;
    robot_tf.setIdentity();
    robot_tf.setOrigin(btVector3(0, 0, 0.20));

    robot_body_ = createRigidBody(5.0, robot_tf, robot_shape);
    robot_body_->setActivationState(DISABLE_DEACTIVATION);
    robot_body_->setFriction(1.0);

    // Static obstacle boxes
    addBoxObstacle(2.0, 0.0, 0.25, 0.5, 0.5, 0.5);
    addBoxObstacle(3.0, 1.0, 0.25, 0.6, 0.6, 0.5);
    addBoxObstacle(4.0, -1.0, 0.25, 0.6, 0.6, 0.5);
    addBoxObstacle(5.0, 0.0, 0.25, 0.8, 0.8, 0.5);
  }

  void addBoxObstacle(
    double x,
    double y,
    double z,
    double sx,
    double sy,
    double sz)
  {
    auto * shape = new btBoxShape(btVector3(sx / 2.0, sy / 2.0, sz / 2.0));

    btTransform tf;
    tf.setIdentity();
    tf.setOrigin(btVector3(x, y, z));

    auto * body = createRigidBody(0.0, tf, shape);
    body->setFriction(1.0);
  }

  void stepSimulation()
  {
    const double dt = 1.0 / physics_rate_;

    applyVelocityCommand();

    dynamics_world_->stepSimulation(dt, 1, dt);

    sim_time_ += dt;
    step_count_++;

    if (step_count_ % scan_publish_every_n_steps_ == 0) {
      publishRaycastScan();
    }

    if (step_count_ % static_cast<int>(physics_rate_) == 0) {
      printStatus();
    }
  }

  void applyVelocityCommand()
  {
    btTransform tf;
    robot_body_->getMotionState()->getWorldTransform(tf);

    const double yaw = getYaw(tf);

    const double vx = linear_velocity_ * std::cos(yaw);
    const double vy = linear_velocity_ * std::sin(yaw);

    robot_body_->setLinearVelocity(btVector3(vx, vy, 0.0));
    robot_body_->setAngularVelocity(btVector3(0.0, 0.0, angular_velocity_));
  }

  double getYaw(const btTransform & tf) const
  {
    btScalar roll;
    btScalar pitch;
    btScalar yaw;

    tf.getBasis().getEulerYPR(yaw, pitch, roll);

    return static_cast<double>(yaw);
  }

  void publishRaycastScan()
  {
    sensor_msgs::msg::LaserScan scan;

    scan.header.stamp = this->now();
    scan.header.frame_id = "base_scan";

    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = M_PI / 180.0;

    scan.range_min = 0.12;
    scan.range_max = 3.5;

    const int number_of_rays = static_cast<int>(
      (scan.angle_max - scan.angle_min) / scan.angle_increment
    );

    scan.ranges.resize(number_of_rays);
    scan.intensities.assign(number_of_rays, 0.0);

    btTransform robot_tf;
    robot_body_->getMotionState()->getWorldTransform(robot_tf);

    const btVector3 robot_pos = robot_tf.getOrigin();
    const double yaw = getYaw(robot_tf);

    const double lidar_z = 0.25;

    for (int i = 0; i < number_of_rays; ++i) {
      const double local_angle = scan.angle_min + i * scan.angle_increment;
      const double global_angle = yaw + local_angle;

      const btVector3 ray_from(
        robot_pos.x(),
        robot_pos.y(),
        lidar_z
      );

      const btVector3 ray_to(
        robot_pos.x() + scan.range_max * std::cos(global_angle),
        robot_pos.y() + scan.range_max * std::sin(global_angle),
        lidar_z
      );

      btCollisionWorld::ClosestRayResultCallback ray_callback(ray_from, ray_to);
      dynamics_world_->rayTest(ray_from, ray_to, ray_callback);

      if (ray_callback.hasHit()) {
        const double distance =
          scan.range_max * ray_callback.m_closestHitFraction;

        if (distance >= scan.range_min && distance <= scan.range_max) {
          scan.ranges[i] = static_cast<float>(distance);
        } else {
          scan.ranges[i] = std::numeric_limits<float>::infinity();
        }
      } else {
        scan.ranges[i] = std::numeric_limits<float>::infinity();
      }
    }

    scan_pub_->publish(scan);
  }

  void printStatus()
  {
    btTransform tf;
    robot_body_->getMotionState()->getWorldTransform(tf);

    const auto now = std::chrono::steady_clock::now();
    const double wall_time =
      std::chrono::duration<double>(now - start_wall_time_).count();

    const double rtf = sim_time_ / wall_time;
    const btVector3 pos = tf.getOrigin();
    const double yaw = getYaw(tf);

    RCLCPP_INFO(
      this->get_logger(),
      "Pose: x=%.3f y=%.3f yaw=%.3f sim_time=%.2f RTF=%.2f",
      pos.x(), pos.y(), yaw, sim_time_, rtf
    );
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  btBroadphaseInterface * broadphase_ = nullptr;
  btDefaultCollisionConfiguration * collision_config_ = nullptr;
  btCollisionDispatcher * dispatcher_ = nullptr;
  btSequentialImpulseConstraintSolver * solver_ = nullptr;
  btDiscreteDynamicsWorld * dynamics_world_ = nullptr;

  std::vector<btCollisionShape *> collision_shapes_;
  std::vector<btRigidBody *> rigid_bodies_;

  btRigidBody * robot_body_ = nullptr;

  const double physics_rate_ = 240.0;
  const int scan_publish_every_n_steps_ = 24;

  double linear_velocity_ = 0.0;
  double angular_velocity_ = 0.0;

  double sim_time_ = 0.0;
  int step_count_ = 0;

  std::chrono::steady_clock::time_point start_wall_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeoPybulletSimNode>());
  rclcpp::shutdown();
  return 0;
}
