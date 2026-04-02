#pragma once
#include <rclcpp/rclcpp.hpp>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/mission/mission_executor.hpp>
#include <vision_tracker/vision_tracker_mode.hpp>

class VisionTrackerModeAction : public px4_ros2::ActionInterface
{
public:
  explicit VisionTrackerModeAction(px4_ros2::ModeBase & mode, 
		  px4_ros2::ModeBase::ModeID mode_id)
  : _node(mode.node()), _mode_id(mode_id)
  {
  }
  std::string name() const override {return "visionTrackerMode";}

  void run(
    const std::shared_ptr<px4_ros2::ActionHandler> & handler,
    const px4_ros2::ActionArguments & arguments,
    const std::function<void()> & on_completed) override
  {
    RCLCPP_INFO(_node.get_logger(), "Running custom mode");
    handler->runMode(_mode_id, on_completed);
  }

private:
  rclcpp::Node & _node;
  px4_ros2::ModeBase::ModeID _mode_id;
};
