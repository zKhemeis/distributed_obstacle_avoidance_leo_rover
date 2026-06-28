#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "btBulletDynamicsCommon.h"
#include "visualization_msgs/msg/marker_array.hpp"

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
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/pybullet_markers",
      10
    );

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
      publishMarkers();
      
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
  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray markers;

    btTransform robot_tf;
    robot_body_->getMotionState()->getWorldTransform(robot_tf);

    const btVector3 robot_pos = robot_tf.getOrigin();
    const double yaw = getYaw(robot_tf);

    const auto makeQuatFromYaw = [](double yaw_angle) {
      geometry_msgs::msg::Quaternion q;
      q.x = 0.0;
      q.y = 0.0;
      q.z = std::sin(yaw_angle / 2.0);
      q.w = std::cos(yaw_angle / 2.0);
      return q;
    };

    const auto transformPoint = [&](double lx, double ly, double lz) {
      geometry_msgs::msg::Point p;
      p.x = robot_pos.x() + std::cos(yaw) * lx - std::sin(yaw) * ly;
      p.y = robot_pos.y() + std::sin(yaw) * lx + std::cos(yaw) * ly;
      p.z = robot_pos.z() + lz;
      return p;
    };

    // Chassis
    visualization_msgs::msg::Marker chassis;
    chassis.header.frame_id = "map";
    chassis.header.stamp = this->now();
    chassis.ns = "leo_robot";
    chassis.id = 0;
    chassis.type = visualization_msgs::msg::Marker::CUBE;
    chassis.action = visualization_msgs::msg::Marker::ADD;
    chassis.pose.position = transformPoint(0.0, 0.0, 0.0);
    chassis.pose.orientation = makeQuatFromYaw(yaw);
    chassis.scale.x = 0.42;
    chassis.scale.y = 0.28;
    chassis.scale.z = 0.16;
    chassis.color.r = 0.1;
    chassis.color.g = 0.4;
    chassis.color.b = 1.0;
    chassis.color.a = 1.0;
    markers.markers.push_back(chassis);

    // LiDAR position from URDF scan_joint xyz="0.10 0 0.08"
    visualization_msgs::msg::Marker lidar;
    lidar.header.frame_id = "map";
    lidar.header.stamp = this->now();
    lidar.ns = "leo_robot";
    lidar.id = 1;
    lidar.type = visualization_msgs::msg::Marker::SPHERE;
    lidar.action = visualization_msgs::msg::Marker::ADD;
    lidar.pose.position = transformPoint(0.10, 0.0, 0.08);
    lidar.pose.orientation.w = 1.0;
    lidar.scale.x = 0.06;
    lidar.scale.y = 0.06;
    lidar.scale.z = 0.04;
    lidar.color.r = 0.0;
    lidar.color.g = 1.0;
    lidar.color.b = 0.0;
    lidar.color.a = 1.0;
    markers.markers.push_back(lidar);

  // Wheel positions, approximated from URDF:
  // rocker joints at y= +/-0.14167, wheel offsets x= +/-0.15256, y=-0.08214.
  // This gives approximate wheel y positions around +/-0.224.
    struct WheelVis {
      double x;
      double y;
      int id;
    };

    std::vector<WheelVis> wheels = {
      {-0.15256,  0.224, 2},  // FL
      { 0.15256,  0.224, 3},  // RL
      { 0.15256, -0.224, 4},  // FR
      {-0.15256, -0.224, 5},  // RR
    };

    for (const auto & wheel : wheels) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = this->now();
      marker.ns = "leo_robot";
      marker.id = wheel.id;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = transformPoint(wheel.x, wheel.y, -0.10);

    // Simple wheel cylinder orientation.
    // In RViz a cylinder is along z by default. This rotates it approximately to wheel axis.
      marker.pose.orientation.x = 0.7071;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 0.7071;

      marker.scale.x = 0.125;  // diameter
      marker.scale.y = 0.125;  // diameter
      marker.scale.z = 0.07;   // width
      marker.color.r = 0.02;
      marker.color.g = 0.02;
      marker.color.b = 0.02;
      marker.color.a = 1.0;
      markers.markers.push_back(marker);
    }

    // Obstacles
    std::vector<std::tuple<double, double, double, double, double>> boxes = {
      {2.0, 0.0, 0.25, 0.5, 0.5},
      {3.0, 1.0, 0.25, 0.6, 0.6},
      {4.0, -1.0, 0.25, 0.6, 0.6},
      {5.0, 0.0, 0.25, 0.8, 0.8},
    };

    int id = 100;
    for (const auto & box : boxes) {
      auto [x, y, z, sx, sy] = box;

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = this->now();
      marker.ns = "obstacles";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = z;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = sx;
      marker.scale.y = sy;
      marker.scale.z = 0.5;
      marker.color.r = 1.0;
      marker.color.g = 0.2;
      marker.color.b = 0.1;
      marker.color.a = 1.0;
      markers.markers.push_back(marker);
    }

    marker_pub_->publish(markers);
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
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
