#include <chrono>
#include <fstream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomLoggerNode : public rclcpp::Node
{
public:
  OdomLoggerNode()
  : Node("odom_logger_node")
  {
    output_file_ = "/root/leo_ws/src/leo_pybullet/results/odom_log.csv";

    file_.open(output_file_, std::ios::out);
    file_ << "time_sec,x,y,z,qx,qy,qz,qw,vx,vy,vz,wz\n";

    start_time_ = this->now();

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      10,
      std::bind(&OdomLoggerNode::callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Logging /odom to %s", output_file_.c_str());
  }

  ~OdomLoggerNode()
  {
    if (file_.is_open()) {
      file_.close();
    }
  }

private:
  void callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double t = (this->now() - start_time_).seconds();

    file_
      << t << ","
      << msg->pose.pose.position.x << ","
      << msg->pose.pose.position.y << ","
      << msg->pose.pose.position.z << ","
      << msg->pose.pose.orientation.x << ","
      << msg->pose.pose.orientation.y << ","
      << msg->pose.pose.orientation.z << ","
      << msg->pose.pose.orientation.w << ","
      << msg->twist.twist.linear.x << ","
      << msg->twist.twist.linear.y << ","
      << msg->twist.twist.linear.z << ","
      << msg->twist.twist.angular.z
      << "\n";
  }

  std::string output_file_;
  std::ofstream file_;
  rclcpp::Time start_time_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomLoggerNode>());
  rclcpp::shutdown();
  return 0;
}
