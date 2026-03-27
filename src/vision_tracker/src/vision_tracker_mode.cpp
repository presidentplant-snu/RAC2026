#include "vision_tracker_mode.hpp"
#include <px4_ros2/utils/geometry.hpp>

using namespace px4_ros2::literals; // NOLINT

VisionTrackerMode::VisionTrackerMode(rclcpp::Node & node)
    : ModeBase(node, Settings{mode_name}.preventArming(true))
    , _node(node)
{
    _trajectory_setpoint  = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

    _cam_tracked_obj_sub = _node.create_subscription<aircraft_msgs::msg::CameraTrackedObject>(
        _cam_tracked_obj_topic_name, 10,
        [this](const aircraft_msgs::msg::CameraTrackedObject::SharedPtr msg) {
            this->trackedObjectCallback(msg);
        });
}

void VisionTrackerMode::onActivate()
{
    _state = State::Init;
    _slew.reset();
    RCLCPP_INFO(_node.get_logger(), "Tracking Mode activated!");
}

void VisionTrackerMode::onDeactivate()
{
    isTargetValid = false;
}

// ------------------------------------------------------------------
// Sends a trajectory setpoint with slew-limited XY and a fixed Z descent rate.
// raw_target  : the live _target_pos_ned (XY used for slew input and braking)
// z_target_ned: the Z position setpoint in NED (negative = higher altitude)
// ------------------------------------------------------------------
void VisionTrackerMode::sendTrackedSetpoint(const Eigen::Vector3f & raw_target,
                                             float z_target_ned)
{
    const Eigen::Vector2f current_xy = _vehicle_local_position->positionNed().head<2>();
    const Eigen::Vector2f slewed_xy  = _slew.update(current_xy, raw_target.head<2>());

    const float current_z   = _vehicle_local_position->positionNed()(2);
    const float z_error     = z_target_ned - current_z;
    // Clamp descent velocity: only descend (positive NED = down), don't climb here
	const float vz = std::clamp(z_error * 0.5f, -descend_vel, descend_vel);

    px4_ros2::TrajectorySetpoint sp;
    sp.withHorizontalPosition(slewed_xy)
      .withPositionZ(z_target_ned)
      .withVelocityZ(vz)
      .withYaw(_initial_heading);

    _trajectory_setpoint->update(sp);
}

void VisionTrackerMode::updateSetpoint(float dt_s)
{
    if (!_vehicle_local_position->lastValid()) {
        RCLCPP_WARN(_node.get_logger(), "No valid local position received");
        return;
    }

    // Keep slew dt in sync with actual update rate
    _slew.dt = dt_s;

    switch (_state) {

        // ----------------------------------------------------------------
        case State::Init:
            if (!isTargetValid) return;
            if (checkTargetTimeout()) return;

            _initial_heading = _vehicle_local_position->heading();
            switchToState(State::Approach);
            break;

        // ----------------------------------------------------------------
        // Fast approach at high altitude, tracking target XY.
        case State::Approach: {
            if (checkTargetTimeout()) {
                RCLCPP_ERROR(_node.get_logger(), "Target track timeout in Approach");
                completed(px4_ros2::Result::Timeout);
                return;
            }

            const float z_target = _target_pos_ned(2) - approach_alt;
            sendTrackedSetpoint(_target_pos_ned, z_target);

            Eigen::Vector3f target3 = _target_pos_ned;
            target3(2) = z_target;
            if (positionReached(target3, approach_pos_threshold)) {
                switchToState(State::DescendToFine);
            }
            break;
        }

        // ----------------------------------------------------------------
        // Descend to fine_alt while continuing to track target XY.
        case State::DescendToFine: {
            if (checkTargetTimeout()) {
                RCLCPP_ERROR(_node.get_logger(), "Target track timeout in DescendToFine");
                completed(px4_ros2::Result::Timeout);
                return;
            }

            const float z_target = _target_pos_ned(2) - fine_alt;
            sendTrackedSetpoint(_target_pos_ned, z_target);

            Eigen::Vector3f target3 = _target_pos_ned;
            target3(2) = z_target;
            if (positionReached(target3, approach_pos_threshold)) {
                switchToState(State::FineApproach);
            }
            break;
        }

        // ----------------------------------------------------------------
        // Slow precise XY correction at fine_alt, still tracking.
        case State::FineApproach: {
            if (!isTargetValid &&
                (_node.get_clock()->now() - targetTimestamp).seconds() > fineApproachTimeoutLimit)
            {
                RCLCPP_ERROR(_node.get_logger(), "Target lost during FineApproach");
                completed(px4_ros2::Result::Timeout);
                return;
            }

            const float z_target = _target_pos_ned(2) - fine_alt;
            sendTrackedSetpoint(_target_pos_ned, z_target);

            Eigen::Vector3f target3 = _target_pos_ned;
            target3(2) = z_target;
            if (positionReached(target3, fine_pos_threshold)) {
                switchToState(State::Descend);
            }
            break;
        }

        // ----------------------------------------------------------------
        // Blind final descend to snapshotted position. No tracking.
        case State::Descend: {
            px4_ros2::TrajectorySetpoint sp;
            sp.withHorizontalPosition(_descend_target_pos_ned.head<2>())
              .withPositionZ(_descend_target_pos_ned(2))
              .withYaw(_initial_heading);
            _trajectory_setpoint->update(sp);

            if (std::abs(_vehicle_local_position->positionNed()(2) - _descend_target_pos_ned(2)) < final_alt_threshold ) {
                switchToState(State::Done);
            }
            break;
        }

        // ----------------------------------------------------------------
        case State::Done:
            completed(px4_ros2::Result::Success);
            break;

        default:
			break;
    }
}

void VisionTrackerMode::switchToState(State state)
{
    // Configure slew rate profile for the incoming state
    switch (state) {
        case State::Approach:
        case State::DescendToFine:
            _slew.v_max = fast_v_max;
            _slew.a_max = fast_a_max;
            break;
        case State::FineApproach:
            _slew.v_max = slow_v_max;
            _slew.a_max = slow_a_max;
            break;
        default:
            break;
    }

    // Snapshot target position on entry to Descend
    if (state == State::Descend) {
        _descend_target_pos_ned      = _target_pos_ned;
        _descend_target_pos_ned(2)  -= descend_alt;
    }

    // Reset slew on every transition so there's no jerk from stale history
    _slew.reset();

    RCLCPP_INFO(_node.get_logger(), "Switched to state %s", stateName(state).c_str());
    _state = state;
}

void VisionTrackerMode::trackedObjectCallback(
    const aircraft_msgs::msg::CameraTrackedObject::SharedPtr msg)
{
    if (!_vehicle_local_position->lastValid()) {
        RCLCPP_WARN(_node.get_logger(), "No valid local position received");
        return;
    }

    if (std::isnan(msg->tan_x)) {
        isTargetValid = false;
        return;
    }

    Eigen::Vector3f vehicle_pos = _vehicle_local_position->positionNed();
    float yaw = _vehicle_local_position->heading();

    float height = use_distance_sensor
        ? _vehicle_local_position->distanceGround()
        : -vehicle_pos(2);

    Eigen::Vector2f delta_cam_xy =
        height * Eigen::Vector2f(msg->tan_x, msg->tan_y)
        + Eigen::Vector2f(msg->offset_x, msg->offset_y);

    Eigen::Matrix2f local_to_ned_xy;
    local_to_ned_xy << -sinf(yaw), cosf(yaw),
                        cosf(yaw), sinf(yaw);

    Eigen::Vector2f delta_ned_xy = local_to_ned_xy * delta_cam_xy;

    _target_pos_ned << (delta_ned_xy + vehicle_pos.head<2>()),
                       vehicle_pos(2) + height;

    targetTimestamp = _node.get_clock()->now();
    isTargetValid   = true;
}

bool VisionTrackerMode::checkTargetTimeout()
{
    if (targetTimestamp.nanoseconds() == 0) return true;
    return (_node.get_clock()->now() - targetTimestamp).seconds() > timeoutLimit;
}

bool VisionTrackerMode::positionReached(
    const Eigen::Vector3f & target_position_m,
    const float pos_error_threshold,
	const float vel_error_threshold) const
{
    const Eigen::Vector3f position_error_m =
        target_position_m - _vehicle_local_position->positionNed();

    RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 100,
        "Position error: %.3f m, Velocity: %.3f m/s",
        position_error_m.norm(),
        _vehicle_local_position->velocityNed().norm());

    return (position_error_m.norm() < pos_error_threshold) &&
           (_vehicle_local_position->velocityNed().norm() < vel_error_threshold);
}

bool VisionTrackerMode::headingReached(float target_heading_rad) const
{
    static constexpr float kHeadingErrorThreshold = 5.0_deg;
    const float heading_error_wrapped = px4_ros2::wrapPi(
        target_heading_rad - _vehicle_local_position->heading());
    return fabsf(heading_error_wrapped) < kHeadingErrorThreshold;
}
