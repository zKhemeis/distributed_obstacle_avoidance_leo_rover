#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

struct BoxObstacle
{
  double x;
  double y;
  double sx;
  double sy;
};

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

    obstacles_ = {
      {2.0, 0.0, 0.5, 0.5},
      {3.0, 1.0, 0.6, 0.6},
      {4.0, -1.0, 0.6, 0.6},
      {5.0, 0.0, 0.8, 0.8},
    };

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / physics_rate_),
      std::bind(&LeoPybulletSimNode::stepSimulation, this)
    );

    start_wall_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "Leo PyBullet C++ simulator node started");
    RCLCPP_INFO(this->get_logger(), "Using simple 2D ray-cast LiDAR with box obstacles");
  }

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    linear_velocity_ = msg->linear.x;
    angular_velocity_ = msg->angular.z;
  }

  void stepSimulation()
  {
    const double dt = 1.0 / physics_rate_;

    yaw_ += angular_velocity_ * dt;
    x_ += linear_velocity_ * std::cos(yaw_) * dt;
    y_ += linear_velocity_ * std::sin(yaw_) * dt;

    sim_time_ += dt;
    step_count_++;

    if (step_count_ % scan_publish_every_n_steps_ == 0) {
      publishRaycastScan();
    }

    if (step_count_ % static_cast<int>(physics_rate_) == 0) {
      const auto now = std::chrono::steady_clock::now();
      const double wall_time =
        std::chrono::duration<double>(now - start_wall_time_).count();

      const double rtf = sim_time_ / wall_time;

      RCLCPP_INFO(
        this->get_logger(),
        "Pose: x=%.3f y=%.3f yaw=%.3f sim_time=%.2f RTF=%.2f",
        x_, y_, yaw_, sim_time_, rtf
      );
    }
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

    for (int i = 0; i < number_of_rays; ++i) {
      const double local_angle = scan.angle_min + i * scan.angle_increment;
      const double global_angle = yaw_ + local_angle;

      const double dx = std::cos(global_angle);
      const double dy = std::sin(global_angle);

      double closest = std::numeric_limits<double>::infinity();

      for (const auto & box : obstacles_) {
        const double hit = rayBoxIntersection2D(x_, y_, dx, dy, box);
        if (std::isfinite(hit) && hit < closest) {
          closest = hit;
        }
      }

      if (std::isfinite(closest) &&
          closest >= scan.range_min &&
          closest <= scan.range_max)
      {
        scan.ranges[i] = static_cast<float>(closest);
      } else {
        scan.ranges[i] = std::numeric_limits<float>::infinity();
      }
    }
    float min_range = std::numeric_limits<float>::infinity();

    for (const auto & r : scan.ranges) {
      if (std::isfinite(r) && r < min_range) {
        min_range = r;
      }
    }

    if (std::isfinite(min_range)) {
      RCLCPP_INFO(this->get_logger(), "Closest LiDAR hit: %.3f m", min_range);
    } else {
      RCLCPP_INFO(this->get_logger(), "No LiDAR hit");
    }
    scan_pub_->publish(scan);
  }

  double rayBoxIntersection2D(
    double ray_x,
    double ray_y,
    double ray_dx,
    double ray_dy,
    const BoxObstacle & box)
  {
    const double xmin = box.x - box.sx / 2.0;
    const double xmax = box.x + box.sx / 2.0;
    const double ymin = box.y - box.sy / 2.0;
    const double ymax = box.y + box.sy / 2.0;

    double tmin = -std::numeric_limits<double>::infinity();
    double tmax = std::numeric_limits<double>::infinity();

    if (std::abs(ray_dx) < 1e-9) {
      if (ray_x < xmin || ray_x > xmax) {
        return std::numeric_limits<double>::infinity();
      }
    } else {
      const double tx1 = (xmin - ray_x) / ray_dx;
      const double tx2 = (xmax - ray_x) / ray_dx;

      tmin = std::max(tmin, std::min(tx1, tx2));
      tmax = std::min(tmax, std::max(tx1, tx2));
    }

    if (std::abs(ray_dy) < 1e-9) {
      if (ray_y < ymin || ray_y > ymax) {
        return std::numeric_limits<double>::infinity();
      }
    } else {
      const double ty1 = (ymin - ray_y) / ray_dy;
      const double ty2 = (ymax - ray_y) / ray_dy;

      tmin = std::max(tmin, std::min(ty1, ty2));
      tmax = std::min(tmax, std::max(ty1, ty2));
    }

    if (tmax < 0.0 || tmin > tmax) {
      return std::numeric_limits<double>::infinity();
    }

    return tmin >= 0.0 ? tmin : tmax;
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<BoxObstacle> obstacles_;

  const double physics_rate_ = 240.0;
  const int scan_publish_every_n_steps_ = 24;

  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;

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
