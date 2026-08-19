#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarRangeCalibrationNode : public rclcpp::Node
{
public:
  LidarRangeCalibrationNode()
  : Node("lidar_range_calibration_node")
  {
    center_angle_deg_ = declare_parameter<double>(
      "center_angle_deg", 0.0);

    sector_half_width_deg_ = declare_parameter<double>(
      "sector_half_width_deg", 3.0);

    sample_scans_ = declare_parameter<int>(
      "sample_scans", 30);

    reference_distance_m_ = declare_parameter<double>(
      "reference_distance_m", -1.0);

    if (sample_scans_ < 1) {
      sample_scans_ = 1;
    }

    scan_sub_ =
      create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS(),
      std::bind(
        &LidarRangeCalibrationNode::scanCallback,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "LiDAR calibration started: center=%.1f deg, "
      "sector=+/-%.1f deg, scans=%d, reference=%.3f m",
      center_angle_deg_,
      sector_half_width_deg_,
      sample_scans_,
      reference_distance_m_);
  }

private:
  static double normalizeAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static double median(std::vector<double> values)
  {
    if (values.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    std::sort(values.begin(), values.end());

    const std::size_t middle = values.size() / 2;

    if (values.size() % 2 == 0) {
      return 0.5 * (values[middle - 1] + values[middle]);
    }

    return values[middle];
  }

  static double mean(const std::vector<double> & values)
  {
    if (values.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    return std::accumulate(
      values.begin(), values.end(), 0.0) /
      static_cast<double>(values.size());
  }

  static double standardDeviation(
    const std::vector<double> & values,
    double average)
  {
    double sum = 0.0;

    for (const double value : values) {
      const double difference = value - average;
      sum += difference * difference;
    }

    return std::sqrt(
      sum / static_cast<double>(values.size()));
  }

  void scanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    constexpr double pi = 3.14159265358979323846;

    const double center_angle_rad =
      center_angle_deg_ * pi / 180.0;

    const double sector_half_width_rad =
      sector_half_width_deg_ * pi / 180.0;

    double closest_angle_difference =
      std::numeric_limits<double>::infinity();

    double center_range =
      std::numeric_limits<double>::quiet_NaN();

    for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
      const double beam_angle =
        static_cast<double>(scan->angle_min) +
        static_cast<double>(i) *
        static_cast<double>(scan->angle_increment);

      const double angle_difference = std::abs(
        normalizeAngle(beam_angle - center_angle_rad));

      const double range =
        static_cast<double>(scan->ranges[i]);

      const bool valid =
        std::isfinite(range) &&
        range >= static_cast<double>(scan->range_min) &&
        range <= static_cast<double>(scan->range_max);

      if (angle_difference < closest_angle_difference) {
        closest_angle_difference = angle_difference;

        center_range = valid
          ? range
          : std::numeric_limits<double>::quiet_NaN();
      }

      if (valid &&
          angle_difference <= sector_half_width_rad)
      {
        sector_samples_.push_back(range);
      }
    }

    if (std::isfinite(center_range)) {
      center_samples_.push_back(center_range);
    }

    ++received_scans_;

    if (received_scans_ >= sample_scans_) {
      printStatistics();

      received_scans_ = 0;
      sector_samples_.clear();
      center_samples_.clear();
    }
  }

  void printStatistics()
  {
    if (sector_samples_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "No valid measurements in the selected sector");
      return;
    }

    const double sector_median =
      median(sector_samples_);

    const double sector_mean =
      mean(sector_samples_);

    const double sector_stddev =
      standardDeviation(
        sector_samples_, sector_mean);

    const auto min_max = std::minmax_element(
      sector_samples_.begin(),
      sector_samples_.end());

    RCLCPP_INFO(
      get_logger(),
      "----------------------------------------");

    RCLCPP_INFO(
      get_logger(),
      "Results from %d scans",
      sample_scans_);

    RCLCPP_INFO(
      get_logger(),
      "Valid sector samples: %zu",
      sector_samples_.size());

    RCLCPP_INFO(
      get_logger(),
      "Sector median: %.4f m",
      sector_median);

    RCLCPP_INFO(
      get_logger(),
      "Sector mean:   %.4f m",
      sector_mean);

    RCLCPP_INFO(
      get_logger(),
      "Standard deviation: %.4f m",
      sector_stddev);

    RCLCPP_INFO(
      get_logger(),
      "Minimum: %.4f m, maximum: %.4f m",
      *min_max.first,
      *min_max.second);

    if (!center_samples_.empty()) {
      const double center_median =
        median(center_samples_);

      RCLCPP_INFO(
        get_logger(),
        "Center-beam median: %.4f m",
        center_median);

      if (reference_distance_m_ > 0.0) {
        RCLCPP_INFO(
          get_logger(),
          "Center-beam error: %+.4f m",
          center_median - reference_distance_m_);
      }
    } else {
      RCLCPP_WARN(
        get_logger(),
        "No valid center-beam measurements");
    }

    if (reference_distance_m_ > 0.0) {
      RCLCPP_INFO(
        get_logger(),
        "Reference distance: %.4f m",
        reference_distance_m_);

      RCLCPP_INFO(
        get_logger(),
        "Sector median error: %+.4f m",
        sector_median - reference_distance_m_);
    }
  }

  rclcpp::Subscription<
    sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  double center_angle_deg_;
  double sector_half_width_deg_;
  int sample_scans_;
  double reference_distance_m_;

  int received_scans_ = 0;

  std::vector<double> sector_samples_;
  std::vector<double> center_samples_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<LidarRangeCalibrationNode>());
  rclcpp::shutdown();

  return 0;
}
