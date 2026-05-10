
#pragma once

#include <optional>

#include <rclcpp/rclcpp.hpp>

#include <px4_ros2/mission/trajectory/trajectory_executor.hpp>
#include <px4_ros2/mission/mission.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include <px4_ros2/control/setpoint_types/fixedwing/lateral_longitudinal.hpp>
#include <px4_ros2/utils/geodesic.hpp>

class FixedWingTrajectoryExecutor : public px4_ros2::TrajectoryExecutorInterface
{
public:
  explicit FixedWingTrajectoryExecutor(px4_ros2::ModeBase & mode, 
		  float acceptance_radius_xy = 4.0f,
		  float acceptance_radius_z = 4.0f);

  ~FixedWingTrajectoryExecutor() override = default;

  bool navigationItemTypeSupported(px4_ros2::NavigationItemType type) override;
  bool frameSupported(px4_ros2::MissionFrame frame) override;
  void runTrajectory(const px4_ros2::TrajectoryExecutorInterface::TrajectoryConfig & config) override;
  void updateSetpoint() override;

private:
  void continueNextItem();
  bool positionReached(const Eigen::Vector3d & target_position_m, 
		  float acceptance_radius_xy, float acceptance_radius_z) const;

  TrajectoryConfig _current_trajectory;
  const float _acceptance_radius_xy;
  const float _acceptance_radius_z;

  std::shared_ptr<px4_ros2::OdometryGlobalPosition> _vehicle_global_position;
  std::shared_ptr<px4_ros2::FwLateralLongitudinalSetpointType> _setpoint;
  std::optional<int> _current_index;
  rclcpp::Node & _node;
};

