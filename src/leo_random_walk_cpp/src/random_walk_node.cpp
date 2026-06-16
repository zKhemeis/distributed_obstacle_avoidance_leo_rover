#include <chrono>
#include <memory>
#include <random>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class RandomWalkNode : public rclcpp::Node
{
public:
  RandomWalkNode() : Node("random_walk_node")
  {
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

    RCLCPP_INFO(this->get_logger(), "Publishing velocity commands to: %s", cmd_vel_topic.c_str());

    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&RandomWalkNode::timer_callback, this)
    );

    choose_new_motion();

    RCLCPP_INFO(this->get_logger(), "Smooth random exploration started.");
  }

private:
  void choose_new_motion()
  {
    std::uniform_real_distribution<double> linear_dist(0.15, 0.35);
    std::uniform_real_distribution<double> angular_dist(-0.45, 0.45);
    std::uniform_int_distribution<int> duration_dist(30, 70);

    current_cmd_.linear.x = linear_dist(rng_);
    current_cmd_.angular.z = angular_dist(rng_);

    steps_remaining_ = duration_dist(rng_);

    RCLCPP_INFO(
      this->get_logger(),
      "New motion: linear.x=%.2f, angular.z=%.2f",
      current_cmd_.linear.x,
      current_cmd_.angular.z
    );
  }

  void timer_callback()
  {
    if (steps_remaining_ <= 0) {
      choose_new_motion();
    }

    cmd_pub_->publish(current_cmd_);
    steps_remaining_--;
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist current_cmd_;
  std::mt19937 rng_{std::random_device{}()};
  int steps_remaining_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RandomWalkNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
