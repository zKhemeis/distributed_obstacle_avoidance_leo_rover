#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class SimpleLidarAvoidanceNode : public rclcpp::Node
{
public:
  SimpleLidarAvoidanceNode()
  : Node("simple_lidar_avoidance_node")
  {
    obstacle_threshold_ = this->declare_parameter<double>("obstacle_threshold", 0.7);
    forward_speed_ = this->declare_parameter<double>("forward_speed", 0.2);
    turn_speed_ = this->declare_parameter<double>("turn_speed", -0.6);
    front_angle_deg_ = this->declare_parameter<double>("front_angle_deg", 30.0);
    target_turn_angle_deg_ = this->declare_parameter<double>("target_turn_angle_deg", 90.0);

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&SimpleLidarAvoidanceNode::scanCallback, this, std::placeholders::_1)
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&SimpleLidarAvoidanceNode::odomCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Simple LiDAR obstacle avoidance started");
  }

private:
  double normalizeAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  double yawFromOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & q = msg->pose.pose.orientation;

    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

    return std::atan2(siny_cosp, cosy_cosp);
  }

void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const double new_yaw = yawFromOdom(msg);

  if (has_odom_ && turning_) {
    const double delta_yaw = normalizeAngle(new_yaw - previous_yaw_);
    accumulated_turn_angle_ += std::abs(delta_yaw);
  }

  current_yaw_ = new_yaw;
  previous_yaw_ = new_yaw;
  has_odom_ = true;
}

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    geometry_msgs::msg::Twist cmd;

    if (!has_odom_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Waiting for /odom..."
      );
      return;
    }

    if (turning_) {
      const double turned_angle = accumulated_turn_angle_;
      const double target_angle = target_turn_angle_deg_ * M_PI / 180.0;

      if (turned_angle < target_angle) {
        cmd.linear.x = 0.0;
        cmd.angular.z = turn_speed_;

        RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          500,
          "Turning: %.1f / %.1f deg",
          turned_angle * 180.0 / M_PI,
          target_turn_angle_deg_
        );
      } else {
        turning_ = false;
        cmd.linear.x = forward_speed_;
        cmd.angular.z = 0.0;

        RCLCPP_INFO(this->get_logger(), "90 degree turn finished -> moving forward");
      }

      cmd_pub_->publish(cmd);
      return;
    }

    double closest_front = std::numeric_limits<double>::infinity();
    const double front_angle_rad = front_angle_deg_ * M_PI / 180.0;

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      const double angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;
      const double normalized_angle = normalizeAngle(angle);

      if (std::abs(normalized_angle) <= front_angle_rad) {
        const float r = scan->ranges[i];

        if (std::isfinite(r) && r >= scan->range_min && r <= scan->range_max) {
          closest_front = std::min(closest_front, static_cast<double>(r));
        }
      }
    }

    if (closest_front < obstacle_threshold_) {
      turning_ = true;
      accumulated_turn_angle_ = 0.0;
      previous_yaw_ = current_yaw_;

      cmd.linear.x = 0.0;
      cmd.angular.z = turn_speed_;

      RCLCPP_INFO(
        this->get_logger(),
        "Obstacle ahead: %.2f m -> starting %.1f degree turn",
        closest_front , 
        target_turn_angle_deg_
      );
    } else {
      cmd.linear.x = forward_speed_;
      cmd.angular.z = 0.0;

      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Path clear -> moving forward"
      );
    }

    cmd_pub_->publish(cmd);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  double obstacle_threshold_;
  double forward_speed_;
  double turn_speed_;
  double front_angle_deg_;
  double target_turn_angle_deg_;

  bool turning_ = false;
  bool has_odom_ = false;

  double current_yaw_ = 0.0;
  double previous_yaw_ = 0.0;
  double accumulated_turn_angle_ = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleLidarAvoidanceNode>());
  rclcpp::shutdown();
  return 0;
}
