#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include "yaml-cpp/yaml.h"
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "btBulletDynamicsCommon.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

struct Wheel
{
  btRigidBody * body = nullptr;
  btHingeConstraint * hinge = nullptr;
  bool left_side = true;
};

struct BoxObstacle
{
  std::string name;
  double x;
  double y;
  double z;
  double size_x;
  double size_y;
  double size_z;
};

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
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/pybullet_markers", 10
    );
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    loadWorldFromYaml("/root/leo_ws/src/leo_pybullet/worlds/boxes_world.yaml");
    initBulletWorld();

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / physics_rate_),
      std::bind(&LeoPybulletSimNode::stepSimulation, this)
    );

    start_wall_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "Bullet simulator with physical wheel motors started");
  }

  ~LeoPybulletSimNode()
  {
    for (auto constraint : constraints_) {
      dynamics_world_->removeConstraint(constraint);
      delete constraint;
    }

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
  void loadWorldFromYaml(const std::string & world_file)
  {
    obstacles_.clear();

    try {
      YAML::Node config = YAML::LoadFile(world_file);

      if (!config["obstacles"]) {
        RCLCPP_WARN(this->get_logger(), "No obstacles found in world file: %s",
        world_file.c_str());
        return;
      }

      for (const auto & obs : config["obstacles"]) {
        BoxObstacle box;
        box.name = obs["name"].as<std::string>();
        box.x = obs["x"].as<double>();  
        box.y = obs["y"].as<double>();
        box.z = obs["z"].as<double>();
        box.size_x = obs["size_x"].as<double>();
        box.size_y = obs["size_y"].as<double>();
        box.size_z = obs["size_z"].as<double>();

        obstacles_.push_back(box);
      }

      RCLCPP_INFO(this->get_logger(), "Loaded %zu obstacles from %s", obstacles_.size(),
      world_file.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load world file: %s", e.what());
    }
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

    auto * ground_shape = new btStaticPlaneShape(btVector3(0, 0, 1), 0);
    btTransform ground_tf;
    ground_tf.setIdentity();
    createRigidBody(0.0, ground_tf, ground_shape);

    createRobot();
    for (const auto & box : obstacles_) {
      addBoxObstacle(box.x, box.y, box.z, box.size_x, box.size_y, box.size_z);
    }
    //addBoxObstacle(2.0, 0.0, 0.25, 0.5, 0.5, 0.5);
    //addBoxObstacle(3.0, 1.0, 0.25, 0.6, 0.6, 0.5);
    //addBoxObstacle(4.0, -1.0, 0.25, 0.6, 0.6, 0.5);
    //addBoxObstacle(5.0, 0.0, 0.25, 0.8, 0.8, 0.5);
  }

  void createRobot()
  {
    auto * chassis_shape = new btBoxShape(btVector3(0.21, 0.14, 0.07));

    btTransform chassis_tf;
    chassis_tf.setIdentity();
    chassis_tf.setOrigin(btVector3(0, 0, 0.17));

    robot_body_ = createRigidBody(4.36, chassis_tf, chassis_shape);
    robot_body_->setActivationState(DISABLE_DEACTIVATION);
    robot_body_->setFriction(1.0);
    robot_body_->setDamping(0.05, 0.2);

    // Keep the rover mostly planar to avoid unrealistic flipping during this first wheel test.
    robot_body_->setAngularFactor(btVector3(0.0, 0.0, 1.0));

    addWheel(-0.15256,  0.224, true);   // FL
    addWheel( 0.15256,  0.224, true);   // RL
    addWheel( 0.15256, -0.224, false);  // FR
    addWheel(-0.15256, -0.224, false);  // RR
  }

  void addWheel(double local_x, double local_y, bool left_side)
  {
    const double wheel_radius = 0.0625;
    const double wheel_width = 0.07;

    auto * wheel_shape = new btCylinderShape(
      btVector3(wheel_radius, wheel_width / 2.0, wheel_radius)
    );

    btTransform chassis_tf;
    robot_body_->getMotionState()->getWorldTransform(chassis_tf);
    const btVector3 chassis_pos = chassis_tf.getOrigin();

    btTransform wheel_tf;
    wheel_tf.setIdentity();
    wheel_tf.setOrigin(
      chassis_pos + btVector3(local_x, local_y, -0.10)
    );

    auto * wheel_body = createRigidBody(0.283642, wheel_tf, wheel_shape);
    wheel_body->setActivationState(DISABLE_DEACTIVATION);
    wheel_body->setFriction(2.0);
    wheel_body->setRollingFriction(0.02);
    wheel_body->setDamping(0.01, 0.01);

    btVector3 pivot_in_chassis(local_x, local_y, -0.1175);
    btVector3 pivot_in_wheel(0, 0, 0);

    btVector3 axis_in_chassis(0, 1, 0);
    btVector3 axis_in_wheel(0, 1, 0);

    auto * hinge = new btHingeConstraint(
      *robot_body_,
      *wheel_body,
      pivot_in_chassis,
      pivot_in_wheel,
      axis_in_chassis,
      axis_in_wheel,
      true
    );

    hinge->setLimit(1.0, -1.0);
    hinge->enableAngularMotor(true, 0.0, motor_max_impulse_);

    dynamics_world_->addConstraint(hinge, true);
    constraints_.push_back(hinge);

    wheels_.push_back({wheel_body, hinge, left_side});
  }

  void addBoxObstacle(double x, double y, double z, double sx, double sy, double sz)
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

    applyWheelMotorCommands();

    dynamics_world_->stepSimulation(dt, 1, dt);

    sim_time_ += dt;
    step_count_++;

    if (step_count_ % scan_publish_every_n_steps_ == 0) {
      publishRaycastScan();
      publishMarkers();
      publishOdom();
    }

    if (step_count_ % static_cast<int>(physics_rate_) == 0) {
      printStatus();
    }
  }

  void applyWheelMotorCommands()
  {
    const double left_linear =
      linear_velocity_ - angular_velocity_ * track_width_ / 2.0;

    const double right_linear =
      linear_velocity_ + angular_velocity_ * track_width_ / 2.0;

    const double left_wheel_speed =  left_linear / wheel_radius_;
    const double right_wheel_speed = right_linear / wheel_radius_;

    for (auto & wheel : wheels_) {
      const double target_speed =
        wheel.left_side ? left_wheel_speed : right_wheel_speed;

      wheel.hinge->enableAngularMotor(true, target_speed, motor_max_impulse_);
    }
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

    const double lidar_x = 0.10;
    const double lidar_y = 0.0;
    const double lidar_z = 0.08;

    const double lidar_world_x =
      robot_pos.x() + std::cos(yaw) * lidar_x - std::sin(yaw) * lidar_y;
    const double lidar_world_y =
      robot_pos.y() + std::sin(yaw) * lidar_x + std::cos(yaw) * lidar_y;
    const double lidar_world_z = robot_pos.z() + lidar_z;

    for (int i = 0; i < number_of_rays; ++i) {
      const double local_angle = scan.angle_min + i * scan.angle_increment;
      const double global_angle = yaw + local_angle;

      const btVector3 ray_from(lidar_world_x, lidar_world_y, lidar_world_z);

      const btVector3 ray_to(
        lidar_world_x + scan.range_max * std::cos(global_angle),
        lidar_world_y + scan.range_max * std::sin(global_angle),
        lidar_world_z
      );

      btCollisionWorld::ClosestRayResultCallback ray_callback(ray_from, ray_to);
      ray_callback.m_collisionFilterGroup = btBroadphaseProxy::SensorTrigger;
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

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray markers;

    auto makeQuat = [](const btQuaternion & q_bt) {
      geometry_msgs::msg::Quaternion q;
      q.x = q_bt.x();
      q.y = q_bt.y();
      q.z = q_bt.z();
      q.w = q_bt.w();
      return q;
    };

    btTransform chassis_tf;
    robot_body_->getMotionState()->getWorldTransform(chassis_tf);

    visualization_msgs::msg::Marker chassis;
    chassis.header.frame_id = "map";
    chassis.header.stamp = this->now();
    chassis.ns = "leo_robot";
    chassis.id = 0;
    chassis.type = visualization_msgs::msg::Marker::CUBE;
    chassis.action = visualization_msgs::msg::Marker::ADD;
    chassis.pose.position.x = chassis_tf.getOrigin().x();
    chassis.pose.position.y = chassis_tf.getOrigin().y();
    chassis.pose.position.z = chassis_tf.getOrigin().z();
    chassis.pose.orientation = makeQuat(chassis_tf.getRotation());
    chassis.scale.x = 0.50;
    chassis.scale.y = 0.36;
    chassis.scale.z = 0.16;
    chassis.color.r = 0.1;
    chassis.color.g = 0.4;
    chassis.color.b = 1.0;
    chassis.color.a = 1.0;
    markers.markers.push_back(chassis);

    int id = 1;
    for (const auto & wheel : wheels_) {
      btTransform wheel_tf;
      wheel.body->getMotionState()->getWorldTransform(wheel_tf);

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = this->now();
      marker.ns = "leo_robot";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = wheel_tf.getOrigin().x();
      marker.pose.position.y = wheel_tf.getOrigin().y();
      marker.pose.position.z = wheel_tf.getOrigin().z();
      btQuaternion wheel_marker_rotation =
        wheel_tf.getRotation() * btQuaternion(btVector3(1, 0, 0), M_PI / 2.0);

      marker.pose.orientation = makeQuat(wheel_marker_rotation);
      marker.scale.x = 0.125;
      marker.scale.y = 0.125;
      marker.scale.z = 0.07;
      marker.color.r = 0.02;
      marker.color.g = 0.02;
      marker.color.b = 0.02;
      marker.color.a = 1.0;
      markers.markers.push_back(marker);
    }

    int box_id = 100;

for (const auto & box : obstacles_) {

  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = "obstacles";
  marker.id = box_id++;

  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = box.x;
  marker.pose.position.y = box.y;
  marker.pose.position.z = box.z;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = box.size_x;
  marker.scale.y = box.size_y;
  marker.scale.z = box.size_z;

  marker.color.r = 1.0;
  marker.color.g = 0.2;
  marker.color.b = 0.1;
  marker.color.a = 1.0;

  markers.markers.push_back(marker);
}
    

    marker_pub_->publish(markers);
  }
  void publishOdom()
  {
    btTransform tf;
    robot_body_->getMotionState()->getWorldTransform(tf);

    const btVector3 pos = tf.getOrigin();
    const btQuaternion q = tf.getRotation();
    const btVector3 lin_vel = robot_body_->getLinearVelocity();
    const btVector3 ang_vel = robot_body_->getAngularVelocity();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = pos.x();
    odom.pose.pose.position.y = pos.y();
    odom.pose.pose.position.z = pos.z();

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = lin_vel.x();
    odom.twist.twist.linear.y = lin_vel.y();
    odom.twist.twist.linear.z = lin_vel.z();

    odom.twist.twist.angular.x = ang_vel.x();
    odom.twist.twist.angular.y = ang_vel.y();
    odom.twist.twist.angular.z = ang_vel.z();

    odom_pub_->publish(odom);
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = odom.header.stamp;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";

    tf_msg.transform.translation.x = pos.x();
    tf_msg.transform.translation.y = pos.y();
    tf_msg.transform.translation.z = pos.z();

    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
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
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  btBroadphaseInterface * broadphase_ = nullptr;
  btDefaultCollisionConfiguration * collision_config_ = nullptr;
  btCollisionDispatcher * dispatcher_ = nullptr;
  btSequentialImpulseConstraintSolver * solver_ = nullptr;
  btDiscreteDynamicsWorld * dynamics_world_ = nullptr;

  std::vector<btCollisionShape *> collision_shapes_;
  std::vector<btRigidBody *> rigid_bodies_;
  std::vector<btTypedConstraint *> constraints_;
  std::vector<Wheel> wheels_;
  std::vector<BoxObstacle> obstacles_;

  btRigidBody * robot_body_ = nullptr;

  const double physics_rate_ = 240.0;
  const int scan_publish_every_n_steps_ = 24;

  const double wheel_radius_ = 0.0625;
  const double track_width_ = 0.448;
  const double motor_max_impulse_ = 5.0; // oringinal 2.0 

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
