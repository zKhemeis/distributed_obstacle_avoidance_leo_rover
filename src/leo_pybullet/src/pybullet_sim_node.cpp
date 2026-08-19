#include <chrono>
#include <cstddef>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "leo_pybullet/bullet_sim_core.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"

class LeoPybulletSimNode : public rclcpp::Node
{
public:
  LeoPybulletSimNode()
  : Node("leo_pybullet_sim_node")
  {
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        core_.setCommand(message->linear.x, message->angular.z);
      });

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);
    marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/pybullet_markers", 10);
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    std::string world_file = declare_parameter<std::string>("world", "boxes");
    if (world_file == "boxes") {
      world_file = "/root/leo_ws/src/leo_pybullet/worlds/boxes_world.yaml";
    } else if (world_file == "empty") {
      world_file = "/root/leo_ws/src/leo_pybullet/worlds/empty_world.yaml";
    }

    try {
      core_.reset(world_file);
    } catch (const std::exception & error) {
      RCLCPP_FATAL(get_logger(), "Failed to initialize Bullet world: %s", error.what());
      throw;
    }

    publishStaticTransforms();
    start_wall_time_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / leo_pybullet::BulletSimCore::kPhysicsRate),
      std::bind(&LeoPybulletSimNode::stepSimulation, this));

    RCLCPP_INFO(
      get_logger(), "Loaded %zu obstacles from %s",
      core_.obstacles().size(), world_file.c_str());
    RCLCPP_INFO(get_logger(), "Bullet simulator with reusable physics core started");
  }

private:
  static geometry_msgs::msg::Quaternion quaternionOf(
    const leo_pybullet::Transform3D & transform)
  {
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.x = transform.qx;
    quaternion.y = transform.qy;
    quaternion.z = transform.qz;
    quaternion.w = transform.qw;
    return quaternion;
  }

  void stepSimulation()
  {
    core_.step();
    if (core_.stepCount() % leo_pybullet::BulletSimCore::kControlPhysicsSteps == 0) {
      publishRaycastScan();
      publishMarkers();
      publishOdom();
      publishImu();
      publishJointStates();
    }

    const auto physics_rate =
      static_cast<std::size_t>(leo_pybullet::BulletSimCore::kPhysicsRate);
    if (core_.stepCount() % physics_rate == 0) {
      printStatus();
    }
  }

  void publishRaycastScan()
  {
    const auto data = core_.laserScan();
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = now();
    scan.header.frame_id = "base_scan";
    scan.angle_min = static_cast<float>(data.angle_min);
    scan.angle_max = static_cast<float>(data.angle_max);
    scan.angle_increment = static_cast<float>(data.angle_increment);
    scan.range_min = static_cast<float>(data.range_min);
    scan.range_max = static_cast<float>(data.range_max);
    scan.scan_time = static_cast<float>(data.scan_time);
    scan.time_increment =
      static_cast<float>(data.scan_time / static_cast<double>(data.ranges.size()));
    scan.ranges = data.ranges;
    scan.intensities.assign(data.ranges.size(), 0.0F);
    scan_pub_->publish(scan);
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray markers;
    const auto stamp = now();
    const auto robot = core_.robotState();

    visualization_msgs::msg::Marker chassis;
    chassis.header.frame_id = "map";
    chassis.header.stamp = stamp;
    chassis.ns = "leo_robot";
    chassis.id = 0;
    chassis.type = visualization_msgs::msg::Marker::CUBE;
    chassis.action = visualization_msgs::msg::Marker::ADD;
    chassis.pose.position.x = robot.transform.x;
    chassis.pose.position.y = robot.transform.y;
    chassis.pose.position.z = robot.transform.z;
    chassis.pose.orientation = quaternionOf(robot.transform);
    chassis.scale.x = 0.50;
    chassis.scale.y = 0.36;
    chassis.scale.z = 0.16;
    chassis.color.r = 0.1F;
    chassis.color.g = 0.4F;
    chassis.color.b = 1.0F;
    chassis.color.a = 1.0F;
    markers.markers.push_back(chassis);

    const auto wheels = core_.wheelStates();
    int marker_id = 1;
    for (const auto & wheel : wheels) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = stamp;
      marker.ns = "leo_robot";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = wheel.transform.x;
      marker.pose.position.y = wheel.transform.y;
      marker.pose.position.z = wheel.transform.z;

      // The physical wheel cylinder is aligned with Y. RViz's cylinder axis is Z,
      // so rotate the core orientation by +90 degrees about X.
      const double half_angle = M_PI / 4.0;
      const double rx = std::sin(half_angle);
      const double rw = std::cos(half_angle);
      marker.pose.orientation.x = wheel.transform.qw * rx + wheel.transform.qx * rw;
      marker.pose.orientation.y = wheel.transform.qy * rw + wheel.transform.qz * rx;
      marker.pose.orientation.z = wheel.transform.qz * rw - wheel.transform.qy * rx;
      marker.pose.orientation.w = wheel.transform.qw * rw - wheel.transform.qx * rx;
      marker.scale.x = 0.125;
      marker.scale.y = 0.125;
      marker.scale.z = 0.07;
      marker.color.r = 0.02F;
      marker.color.g = 0.02F;
      marker.color.b = 0.02F;
      marker.color.a = 1.0F;
      markers.markers.push_back(marker);
    }

    marker_id = 100;
    for (const auto & obstacle : core_.obstacles()) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = stamp;
      marker.ns = "obstacles";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = obstacle.x;
      marker.pose.position.y = obstacle.y;
      marker.pose.position.z = obstacle.z;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = obstacle.size_x;
      marker.scale.y = obstacle.size_y;
      marker.scale.z = obstacle.size_z;
      marker.color.r = 1.0F;
      marker.color.g = 0.2F;
      marker.color.b = 0.1F;
      marker.color.a = 1.0F;
      markers.markers.push_back(marker);
    }
    marker_pub_->publish(markers);
  }

  void publishOdom()
  {
    const auto robot = core_.robotState();
    const auto stamp = now();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = robot.transform.x;
    odom.pose.pose.position.y = robot.transform.y;
    odom.pose.pose.position.z = robot.transform.z;
    odom.pose.pose.orientation = quaternionOf(robot.transform);
    odom.twist.twist.linear.x = robot.linear_velocity.x;
    odom.twist.twist.linear.y = robot.linear_velocity.y;
    odom.twist.twist.linear.z = robot.linear_velocity.z;
    odom.twist.twist.angular.x = robot.angular_velocity.x;
    odom.twist.twist.angular.y = robot.angular_velocity.y;
    odom.twist.twist.angular.z = robot.angular_velocity.z;
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = robot.transform.x;
    transform.transform.translation.y = robot.transform.y;
    transform.transform.translation.z = robot.transform.z;
    transform.transform.rotation = quaternionOf(robot.transform);
    tf_broadcaster_->sendTransform(transform);
  }

  void publishImu()
  {
    const auto robot = core_.robotState();
    const double dt =
      static_cast<double>(leo_pybullet::BulletSimCore::kControlPhysicsSteps) /
      leo_pybullet::BulletSimCore::kPhysicsRate;

    leo_pybullet::Vector3D acceleration;
    acceleration.x = (robot.linear_velocity.x - previous_linear_velocity_.x) / dt;
    acceleration.y = (robot.linear_velocity.y - previous_linear_velocity_.y) / dt;
    acceleration.z = (robot.linear_velocity.z - previous_linear_velocity_.z) / dt;
    previous_linear_velocity_ = robot.linear_velocity;

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = now();
    imu.header.frame_id = "imu_frame";
    imu.orientation = quaternionOf(robot.transform);
    imu.angular_velocity.x = robot.angular_velocity.x;
    imu.angular_velocity.y = robot.angular_velocity.y;
    imu.angular_velocity.z = robot.angular_velocity.z;
    imu.linear_acceleration.x = acceleration.x;
    imu.linear_acceleration.y = acceleration.y;
    imu.linear_acceleration.z = acceleration.z;
    imu_pub_->publish(imu);
  }

  void publishJointStates()
  {
    const auto wheels = core_.wheelStates();
    sensor_msgs::msg::JointState message;
    message.header.stamp = now();
    message.name = {
      "wheel_FL_joint", "wheel_RL_joint", "wheel_FR_joint", "wheel_RR_joint"};
    message.position.resize(wheels.size());
    message.velocity.resize(wheels.size());
    message.effort.assign(wheels.size(), 0.0);
    for (std::size_t index = 0; index < wheels.size(); ++index) {
      message.position[index] = wheels[index].position;
      message.velocity[index] = wheels[index].target_velocity;
    }
    joint_state_pub_->publish(message);
  }

  void publishStaticTransforms()
  {
    const auto stamp = now();
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    geometry_msgs::msg::TransformStamped scan_transform;
    scan_transform.header.stamp = stamp;
    scan_transform.header.frame_id = "base_link";
    scan_transform.child_frame_id = "base_scan";
    scan_transform.transform.translation.x = 0.10;
    scan_transform.transform.translation.z = 0.08;
    scan_transform.transform.rotation.w = 1.0;
    transforms.push_back(scan_transform);

    geometry_msgs::msg::TransformStamped imu_transform;
    imu_transform.header.stamp = stamp;
    imu_transform.header.frame_id = "base_link";
    imu_transform.child_frame_id = "imu_frame";
    imu_transform.transform.translation.x = 0.0628;
    imu_transform.transform.translation.y = -0.0314;
    imu_transform.transform.translation.z = -0.0393;
    imu_transform.transform.rotation.w = 1.0;
    transforms.push_back(imu_transform);

    static_tf_broadcaster_->sendTransform(transforms);
  }

  void printStatus()
  {
    const auto robot = core_.robotState();
    const auto wall_now = std::chrono::steady_clock::now();
    const double wall_time =
      std::chrono::duration<double>(wall_now - start_wall_time_).count();
    const double rtf = wall_time > 0.0 ? core_.simulationTime() / wall_time : 0.0;
    RCLCPP_INFO(
      get_logger(),
      "Pose: x=%.3f y=%.3f yaw=%.3f sim_time=%.2f RTF=%.2f collision=%s",
      robot.transform.x, robot.transform.y, robot.yaw,
      core_.simulationTime(), rtf, core_.hasCollision() ? "true" : "false");
  }

  leo_pybullet::BulletSimCore core_;
  leo_pybullet::Vector3D previous_linear_velocity_;
  std::chrono::steady_clock::time_point start_wall_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeoPybulletSimNode>());
  rclcpp::shutdown();
  return 0;
}
