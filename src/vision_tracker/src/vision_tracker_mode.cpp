#include "vision_tracker_mode.hpp"

#include <px4_ros2/utils/geometry.hpp>

using namespace px4_ros2::literals; // NOLINT

VisionTrackerMode::VisionTrackerMode(rclcpp::Node & node)
	: ModeBase(node, Settings{mode_name}.preventArming(true))
	, _node(node)
{
	_goto_setpoint = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(*this);
	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

	_cam_tracked_obj_sub = _node.create_subscription<aircraft_msgs::msg::CameraTrackedObject>(
			_cam_tracked_obj_topic_name, 10, 
			[this](const aircraft_msgs::msg::CameraTrackedObject::SharedPtr msg) {this->trackedObjectCallback(msg);}
			);
}

void VisionTrackerMode::onActivate() {
	_state = State::Init;
	// Activation Code
}

void VisionTrackerMode::onDeactivate() {
	// Deactivation Code
	isTargetValid = false;
}

void VisionTrackerMode::updateSetpoint(float dt_s)   {
	// Setpoint Update Code
	// TODO: Implement State Machine
	Eigen::Vector3f target;
	switch(_state) {
		case State::Init:
			if (!checkTargetTimeout() && isTargetValid){
				_initial_heading = _vehicle_local_position->heading();

				switchToState(State::Approach);
			}
			break;
		case State::Approach:
			if (checkTargetTimeout()){
				RCLCPP_ERROR(_node.get_logger(), "Target track timeout, returning to previous mode");
				completed(px4_ros2::Result::Timeout);
				return;
			}
			
			target = _target_pos_ned;
			target(2) -= approach_alt;

			_goto_setpoint->update(target, _initial_heading, 
					horizontal_approach_vel, descend_vel);

			if (positionReached(target)) {
				switchToState(State::Descend);
			}
			break;
		case State::Descend:
			// Don't check for timeout since we are already on the target, just keep descending even if target's not visible
			// Have to edit logic depending on mission.

			target = _target_pos_ned;
			target(2) -= descend_alt;

			_goto_setpoint->update(target, _initial_heading, 
					horizontal_approach_vel_slow, descend_vel);

			if (positionReached(target)) {
				switchToState(State::Done);
			}
			break;
		case State::Done:
			completed(px4_ros2::Result::Success);
			break;
		default:
			return;
	}

}

void VisionTrackerMode::switchToState(State state){
	RCLCPP_INFO(_node.get_logger(), "Switched to state %s", stateName(state).c_str());
	_state = state;
}



void VisionTrackerMode::trackedObjectCallback(const aircraft_msgs::msg::CameraTrackedObject::SharedPtr msg ){
	// Distance the drone is above the target 
	float height{0};

	Eigen::Vector3f vehicle_pos = _vehicle_local_position->positionNed();

	float yaw = _vehicle_local_position->heading();

	if(std::isnan(msg->tan_x)){
		isTargetValid = false;
		return;
	}


	if (use_distance_sensor) {
		height = _vehicle_local_position->distanceGround();
	}
	else {
		height = -vehicle_pos(2);
	}

	// Get delta x / delta y from camera frame
	// +x is right, +y is forwards 

	// horizontal position delta, in local frame (target - current) 
	Eigen::Vector2f delta_cam_xy = height * Eigen::Vector2f(msg->tan_x, msg->tan_y) + Eigen::Vector2f(msg->offset_x, msg->offset_y);

	// Local to NED transformation matrix (assume pitch=roll=0 for now... Don't want to do quaternions..)
	Eigen::Matrix2f local_to_ned_xy;
	local_to_ned_xy << -sinf(yaw), cosf(yaw), 
					cosf(yaw), sinf(yaw);

	Eigen::Vector2f delta_ned_xy = local_to_ned_xy * delta_cam_xy;

	// Assume target is on ground (for now). Use vehicle_pos(2) + height instead of 0 because of distance sensor.
	_target_pos_ned << (delta_ned_xy + vehicle_pos.head<2>()), vehicle_pos(2) + height;
	targetTimestamp = _node.now();
	isTargetValid = true;
}

bool VisionTrackerMode::checkTargetTimeout(){
	return (_node.now() - targetTimestamp).seconds() > timeoutLimit;
}

bool VisionTrackerMode::positionReached(const Eigen::Vector3f & target_position_m) const {
	static constexpr float kPositionErrorThreshold = 0.1f; // [m]
	static constexpr float kVelocityErrorThreshold = 0.1f; // [m/s]
	const Eigen::Vector3f position_error_m = target_position_m -
		_vehicle_local_position->positionNed();
	return (position_error_m.norm() < kPositionErrorThreshold) &&
		(_vehicle_local_position->velocityNed().norm() < kVelocityErrorThreshold);
}

bool VisionTrackerMode::headingReached(float target_heading_rad) const {
	static constexpr float kHeadingErrorThreshold = 5.0_deg;
	const float heading_error_wrapped = px4_ros2::wrapPi(
			target_heading_rad - _vehicle_local_position->heading());
	return fabsf(heading_error_wrapped) < kHeadingErrorThreshold;
}
