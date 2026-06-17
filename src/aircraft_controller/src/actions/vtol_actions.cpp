#include "actions/vtol_actions.hpp"

#include <cmath>

#include <px4_ros2/utils/geometry.hpp>

using namespace std::chrono_literals;

VTOLTransitionAction::VTOLTransitionAction(px4_ros2::ModeBase & mode)
: _node(mode.node())
, _vtol(std::make_shared<px4_ros2::VTOL>(mode))
, _trajectory_setpoint(std::make_shared<px4_ros2::TrajectorySetpointType>(mode))
, _fixed_wing_setpoint(std::make_shared<px4_ros2::FwLateralLongitudinalSetpointType>(mode))
, _vehicle_local_position(std::make_shared<px4_ros2::OdometryLocalPosition>(mode))
{}

void VTOLTransitionAction::deactivate()
{
    if (_timer) {
        _timer->cancel();
        _timer.reset();
    }
}

void VTOLTransitionAction::sendTransitionSetpoint(float course_sp,
    const Eigen::Vector3f & acceleration_sp)
{
    // During the transition PX4 expects an acceleration setpoint together with a
    // fixed-wing lateral/longitudinal setpoint that holds altitude and aligns the
    // vehicle to the desired course.
    const Eigen::Vector3f velocity_sp{NAN, NAN, 0.f};
    const float height_rate_sp = 0.f;

    _trajectory_setpoint->update(velocity_sp, acceleration_sp);
    _fixed_wing_setpoint->updateWithHeightRate(height_rate_sp, course_sp);
}

void VTOLTransitionAction::runTransition(
    const std::function<void()> & on_completed, float course_sp)
{
    _timer = _node.create_wall_timer(20ms, [this, on_completed, course_sp]() {
        if (runTick(course_sp)) {
            RCLCPP_INFO(_node.get_logger(), "VTOL transition complete");
            _timer->cancel();
            _timer.reset();
            on_completed();
        }
    });
}

void VTOLTransitionAction::parseAcceleration(const px4_ros2::ActionArguments & arguments)
{
    if (arguments.contains("acceleration")) {
        _acceleration = arguments.at<float>("acceleration");
        RCLCPP_INFO(_node.get_logger(),
            "VTOL transition acceleration %.2f m/s^2", *_acceleration);
    } else {
        _acceleration.reset();
    }
}

// ---------------------------------------------------------------------------
// Forward transition: Multicopter -> Fixed-wing
// ---------------------------------------------------------------------------
bool VTOLForwardTransitionAction::runTick(float course_sp)
{
    _vtol->toFixedwing();

    switch (_vtol->getCurrentState()) {
        case px4_ros2::VTOL::State::TransitionToFixedWing: {
            // Accelerate forward along the course at the configured rate, or fall
            // back to the library default (~zero) if none was given.
            const Eigen::Vector3f acceleration_sp = _acceleration
                ? Eigen::Vector3f{*_acceleration * std::cos(course_sp),
                                  *_acceleration * std::sin(course_sp),
                                  NAN}
                : _vtol->computeAccelerationSetpointDuringTransition();
            sendTransitionSetpoint(course_sp, acceleration_sp);
            return false;
        }
        case px4_ros2::VTOL::State::FixedWing:
            // Done — the fixed-wing trajectory executor takes over from here.
            return true;
        default:
            // Still in multicopter: hold position while the command takes effect.
	    _vtol->toFixedwing();
            return false;
    }
}

void VTOLForwardTransitionAction::run(
    const std::shared_ptr<px4_ros2::ActionHandler> &,
    const px4_ros2::ActionArguments & arguments,
    const std::function<void()> & on_completed)
{
    float course_sp;
    if (arguments.contains("heading")) {
        const float heading_deg = arguments.at<float>("heading");
        course_sp = px4_ros2::degToRad(heading_deg);
        RCLCPP_INFO(_node.get_logger(),
            "Starting forward transition, target heading %.1f deg", heading_deg);
    } else {
        course_sp = currentHeading();
        RCLCPP_INFO(_node.get_logger(),
            "Starting forward transition, holding current heading %.1f deg",
            px4_ros2::radToDeg(course_sp));
    }

    parseAcceleration(arguments);

    runTransition(on_completed, course_sp);
}

// ---------------------------------------------------------------------------
// Back transition: Fixed-wing -> Multicopter
// ---------------------------------------------------------------------------
bool VTOLBackTransitionAction::runTick(float course_sp)
{
    _vtol->toMulticopter();

    switch (_vtol->getCurrentState()) {
        case px4_ros2::VTOL::State::TransitionToMulticopter:
            // The library computes the deceleration setpoint; _acceleration (if
            // set) overrides the back-transition deceleration target.
            sendTransitionSetpoint(course_sp,
                _vtol->computeAccelerationSetpointDuringTransition(_acceleration));
            return false;
        case px4_ros2::VTOL::State::Multicopter:
            // Reached multicopter: hold position, then let the mission continue.
            _trajectory_setpoint->update(Eigen::Vector3f{0.f, 0.f, 0.f}, std::nullopt, course_sp);
            return true;
        default:
            // Still in fixed-wing: keep flying level on course while the command
            // takes effect.
            _fixed_wing_setpoint->updateWithHeightRate(0.f, course_sp);
            return false;
    }
}

void VTOLBackTransitionAction::run(
    const std::shared_ptr<px4_ros2::ActionHandler> &,
    const px4_ros2::ActionArguments & arguments,
    const std::function<void()> & on_completed)
{
    const float course_sp = currentHeading();
    RCLCPP_INFO(_node.get_logger(),
        "Starting back transition, holding heading %.1f deg",
        px4_ros2::radToDeg(course_sp));

    parseAcceleration(arguments);

    runTransition(on_completed, course_sp);
}
