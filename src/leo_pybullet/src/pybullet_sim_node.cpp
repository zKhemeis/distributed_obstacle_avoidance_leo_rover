#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class LeoPybulletSimNode : public rclcpp::Node
{
public:
  LeoPybulletSimNode()
  : Node("leo_pybullet_sim_node")
  {
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10,
      std::bind(&LeoPybulletSimNode::cmdCallback, this, std::placeholders::_1)
    );

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / 240.0),
      std::bind(&LeoPybulletSimNode::stepSimulation, this)
    );

    RCLCPP_INFO(this->get_logger(), "Leo PyBullet C++ simulator node started");
  }

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    linear_velocity_ = msg->linear.x;
    angular_velocity_ = msg->angular.z;
  }

  void stepSimulation()
  {
    const double dt = 1.0 / 240.0;

    yaw_ += angular_velocity_ * dt;
    x_ += linear_velocity_ * std::cos(yaw_) * dt;
    y_ += linear_velocity_ * std::sin(yaw_) * dt;

    sim_time_ += dt;
    step_count_++;

    if (step_count_ % 24 == 0) {
      publishFakeScan();
    }

    if (step_count_ % 240 == 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "Pose: x=%.3f y=%.3f yaw=%.3f sim_time=%.2f",
        x_, y_, yaw_, sim_time_
      );
    }
  }

  void publishFakeScan()
  {
    sensor_msgs::msg::LaserScan scan;

    scan.header.stamp = this->now();
    scan.header.frame_id = "base_scan";

    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = M_PI / 180.0;

    scan.range_min = 0.12;
    scan.range_max = 3.5;

    int number_of_rays = static_cast<int>(
      (scan.angle_max - scan.angle_min) / scan.angle_increment
    );

    scan.ranges.assign(number_of_rays, std::numeric_limits<float>::infinity());
    scan.intensities.assign(number_of_rays, 0.0);

    scan_pub_->publish(scan);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;

  double linear_velocity_ = 0.0;
  double angular_velocity_ = 0.0;

  double sim_time_ = 0.0;
  int step_count_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeoPybulletSimNode>());
  rclcpp::shutdown();
  return 0;
}
