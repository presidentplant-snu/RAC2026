#include "rclcpp/rclcpp.hpp"

#include "vision_tracker/vision_tracker_mode.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <std_msgs/msg/u_int8.hpp>

using MyNodeWithMode = px4_ros2::NodeWithMode<VisionTrackerMode>;

static const std::string kNodeName = "Vision_Tracker";
static const bool kEnableDebugOutput = true;

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  // NodeWithMode registers the mode with the FMU in its constructor, so the
  // assigned mode ID is known here. Latch it for the mission runner.
  auto node = std::make_shared<MyNodeWithMode>(kNodeName, kEnableDebugOutput);
  auto mode_id_pub = node->create_publisher<std_msgs::msg::UInt8>(
      kModeIdTopicName, rclcpp::QoS(1).transient_local());
  std_msgs::msg::UInt8 mode_id_msg;
  mode_id_msg.data = node->getMode().id();
  mode_id_pub->publish(mode_id_msg);
  RCLCPP_INFO(node->get_logger(), "Registered with mode ID %u, published on %s",
      mode_id_msg.data, kModeIdTopicName.c_str());

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

