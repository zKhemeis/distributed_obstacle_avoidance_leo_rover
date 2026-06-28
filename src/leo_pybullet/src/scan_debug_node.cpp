#include <cmath>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanDebugNode : public rclcpp::Node
{
public:
  ScanDebugNode()
  : Node("scan_debug_node")
  {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      10,
      std::bind(&ScanDebugNode::scanCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Scan debug node started");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    float min_range = std::numeric_limits<float>::infinity();

    for (const auto & r : msg->ranges) {
      if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
        if (r < min_range) {
          min_range = r;
        }
      }
    }

    if (std::isfinite(min_range)) {
      RCLCPP_INFO(this->get_logger(), "Closest obstacle: %.3f m", min_range);
    } else {
      RCLCPP_INFO(this->get_logger(), "No obstacle detected");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanDebugNode>());
  rclcpp::shutdown();
  return 0;
}
