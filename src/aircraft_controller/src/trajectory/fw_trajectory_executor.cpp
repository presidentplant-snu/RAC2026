#include "trajectory/fw_trajectory_executor.hpp"

FixedWingTrajectoryExecutor::FixedWingTrajectoryExecutor(
		px4_ros2::ModeBase & mode,
		float acceptance_radius_xy, float acceptance_radius_z)
	: _acceptance_radius_xy(acceptance_radius_xy),
	_acceptance_radius_z(acceptance_radius_z),
	_node(mode.node())
{
	_setpoint = std::make_shared<px4_ros2::FwLateralLongitudinalSetpointType>(mode);
	_vehicle_global_position = std::make_shared<px4_ros2::OdometryGlobalPosition>(mode);
}

bool FixedWingTrajectoryExecutor::navigationItemTypeSupported(px4_ros2::NavigationItemType type)
{
	return type == px4_ros2::NavigationItemType::Waypoint;
}

bool FixedWingTrajectoryExecutor::frameSupported(px4_ros2::MissionFrame frame)
{
	return frame == px4_ros2::MissionFrame::Global;
}

void FixedWingTrajectoryExecutor::runTrajectory(const TrajectoryConfig & config)
{
	_current_trajectory = config;
	_current_index = config.start_index;
}

void FixedWingTrajectoryExecutor::updateSetpoint()
{
	if (!_current_index) {
		return;
	}
	const auto * navigation_item = std::get_if<px4_ros2::NavigationItem>(
			&_current_trajectory.trajectory->items()[*_current_index]);
	if (!navigation_item) {
		// Not expected to happen
		continueNextItem();
		return;
	}

	if (!_vehicle_global_position->positionValid()) {
		RCLCPP_ERROR(_node.get_logger(), "Global position not valid, aborting");
		_current_trajectory.on_failure();
		return;
	}

	const auto & waypoint = std::get<px4_ros2::Waypoint>(navigation_item->data);

	const Eigen::Vector3d & target = waypoint.coordinate;
	const float course = px4_ros2::headingToGlobalPosition(
			_vehicle_global_position->position(), target);
	const float altitude_amsl = static_cast<float>(target.z());

	_setpoint->updateWithAltitude(altitude_amsl, course);

	if (positionReached(target, _acceptance_radius_xy, _acceptance_radius_z)) {
		continueNextItem();
	}
}

void FixedWingTrajectoryExecutor::continueNextItem()
{
	const int index_reached = *_current_index;
	_current_index = *_current_index + 1;
	if (*_current_index > _current_trajectory.end_index) {
		_current_index.reset();
	}
	// Call the callback after updating the state (as it might set a new trajectory already)
	_current_trajectory.on_index_reached(index_reached);
}

bool FixedWingTrajectoryExecutor::positionReached(
		const Eigen::Vector3d & target_position_m,
		float acceptance_radius_xy, float acceptance_radius_z) const
{
	const float position_error = px4_ros2::horizontalDistanceToGlobalPosition(
			_vehicle_global_position->position(), target_position_m);

	const float altitude_error = std::abs(_vehicle_global_position->position().z() - target_position_m.z());

	return position_error < acceptance_radius_xy && altitude_error < acceptance_radius_z;
}

