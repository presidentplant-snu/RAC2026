#pragma once
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/mission/trajectory/trajectory_executor.hpp>
#include <px4_ros2/mission/mission.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>

class FixedWingTrajectoryExecutor : public px4_ros2::TrajectoryExecutorInterface
{
public:
  explicit FixedWingTrajectoryExecutor(px4_ros2::ModeBase & mode, float acceptance_radius = 2.0f);

  ~FixedWingTrajectoryExecutor() override = default;

  bool navigationItemTypeSupported(px4_ros2::NavigationItemType type) override;
  bool frameSupported(px4_ros2::MissionFrame frame) override;
  void runTrajectory(const px4_ros2::TrajectoryExecutorInterface::TrajectoryConfig & config) override;
  void updateSetpoint() override;

private:
  void continueNextItem();
  bool positionReached(const Eigen::Vector3d & target_position_m, float acceptance_radius) const;
  bool headingReached(float target_heading_rad) const;

  TrajectoryConfig _current_trajectory;
  const float _acceptance_radius;
  std::shared_ptr<px4_ros2::OdometryGlobalPosition> _vehicle_global_position;
  std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
  std::shared_ptr<px4_ros2::MulticopterGotoGlobalSetpointType> _setpoint;
  std::optional<int> _current_index;
  rclcpp::Node & _node;
};

