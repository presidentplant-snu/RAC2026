#pragma once
#include <rclcpp/rclcpp.hpp>

#include <optional>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/mission/mission_executor.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <vision_tracker/vision_tracker_mode.hpp>

// Runs the externally registered vision tracker mode. The mode ID is assigned
// by the FMU at registration (order-dependent), so it is received from the
// vision tracker node via a latched topic rather than hardcoded.
class VisionTrackerModeAction : public px4_ros2::ActionInterface
{
public:
  explicit VisionTrackerModeAction(px4_ros2::ModeBase & mode)
  : _node(mode.node())
  {
    _mode_id_sub = _node.create_subscription<std_msgs::msg::UInt8>(
      kModeIdTopicName, rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        RCLCPP_INFO(_node.get_logger(),
          "Received vision tracker mode ID: %u", msg->data);
        _mode_id = msg->data;
      });
  }
  std::string name() const override {return "visionTrackerMode";}

  bool canRun(const px4_ros2::ActionArguments &, std::vector<std::string> & errors) override
  {
    if (!_mode_id) {
      errors.push_back(
        "visionTrackerMode: mode ID not yet received on " + kModeIdTopicName +
        " (is the vision_tracker node running and registered?)");
      return false;
    }
    return true;
  }

  void run(
    const std::shared_ptr<px4_ros2::ActionHandler> & handler,
    const px4_ros2::ActionArguments & arguments,
    const std::function<void()> & on_completed) override
  {
    RCLCPP_INFO(_node.get_logger(), "Running vision tracker mode (ID %u)", *_mode_id);
    handler->runMode(*_mode_id, on_completed);
  }

private:
  rclcpp::Node & _node;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr _mode_id_sub;
  std::optional<px4_ros2::ModeBase::ModeID> _mode_id;
};
