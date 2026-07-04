#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"

class UrdfInspectorNode : public rclcpp::Node
{
public:
  UrdfInspectorNode()
  : Node("urdf_inspector_node")
  {
    const std::string urdf_path =
      "/root/leo_ws/src/leo_pybullet/models/leo_bullet.urdf";

    urdf::Model model;

    if (!model.initFile(urdf_path)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load URDF: %s", urdf_path.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded URDF robot: %s", model.getName().c_str());

    for (const auto & joint_pair : model.joints_) {
      const auto & joint = joint_pair.second;

      RCLCPP_INFO(this->get_logger(), "--------------------------------");
      RCLCPP_INFO(this->get_logger(), "Joint: %s", joint->name.c_str());
      RCLCPP_INFO(this->get_logger(), "Parent: %s", joint->parent_link_name.c_str());
      RCLCPP_INFO(this->get_logger(), "Child: %s", joint->child_link_name.c_str());
      RCLCPP_INFO(this->get_logger(), "Type: %d", joint->type);

      RCLCPP_INFO(
        this->get_logger(),
        "Origin xyz: %.5f %.5f %.5f",
        joint->parent_to_joint_origin_transform.position.x,
        joint->parent_to_joint_origin_transform.position.y,
        joint->parent_to_joint_origin_transform.position.z
      );

      RCLCPP_INFO(
        this->get_logger(),
        "Axis xyz: %.5f %.5f %.5f",
        joint->axis.x,
        joint->axis.y,
        joint->axis.z
      );
    }

    RCLCPP_INFO(this->get_logger(), "========== LINK MASSES ==========");

    for (const auto & link_pair : model.links_) {
      const auto & link = link_pair.second;

      if (link->inertial) {
        RCLCPP_INFO(
          this->get_logger(),
          "Link: %s | mass: %.6f",
          link->name.c_str(),
          link->inertial->mass
        );
      } else {
        RCLCPP_INFO(
          this->get_logger(),
          "Link: %s | no inertial data",
          link->name.c_str()
        );
      }
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UrdfInspectorNode>();
  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}
