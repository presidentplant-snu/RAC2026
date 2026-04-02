#pragma once 

#include <rclcpp/rclcpp.hpp>

#include <px4_ros2/mission/mission.hpp>
#include <px4_ros2/mission/mission_executor.hpp>


static const std::string kNodeName = "mission_runner";

class MissionRunnerNode : public rclcpp::Node
{
public:
  // Private constructor — use create() instead so shared_ptr exists before
  // any shared_from_this() call.
  static std::shared_ptr<MissionRunnerNode> create();

private:
  MissionRunnerNode()
  : Node(kNodeName)
  {
    this->declare_parameter<std::string>("mission_file", "");
  }

  void init();

  std::unique_ptr<px4_ros2::MissionExecutor> _executor;
  std::unique_ptr<px4_ros2::MissionFileMonitor> _monitor;
};


int main(int argc, char * argv[]);
