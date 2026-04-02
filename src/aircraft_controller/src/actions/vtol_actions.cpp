#include "actions/vtol_actions.hpp"
using namespace std::chrono_literals;

VTOLTransitionAction::VTOLTransitionAction(px4_ros2::ModeBase & mode)
: _node(mode.node())
, _vtol(std::make_shared<px4_ros2::VTOL>(mode))
, _vehicle_status(std::make_shared<px4_ros2::VehicleStatus>(mode))
{}

void VTOLTransitionAction::deactivate()
{
    if (_timer) {
        _timer->cancel();
        _timer.reset();
    }
}

void VTOLTransitionAction::runTransition(
    const std::shared_ptr<px4_ros2::ActionHandler> & handler,
    const std::function<void()> & on_completed)
{
    handler->runMode(px4_ros2::ModeBase::kModeIDPosctl, [](){});
    _timer = _node.create_wall_timer(500ms, [this, on_completed]() {
        if (_vehicle_status->navState() != px4_ros2::ModeBase::kModeIDPosctl) {
            RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 1000,
                "Waiting for position mode");
            return;
        }
        doTransition();
        if (_vtol->getCurrentState() == targetState()) {
            RCLCPP_INFO(_node.get_logger(), "Transition complete");
            _timer->cancel();
            _timer.reset();
            on_completed();
        }
    });
}

void VTOLForwardTransitionAction::run(
    const std::shared_ptr<px4_ros2::ActionHandler> & handler,
    const px4_ros2::ActionArguments &,
    const std::function<void()> & on_completed)
{
    RCLCPP_INFO(_node.get_logger(), "Starting forward transition");
    runTransition(handler, on_completed);
}

void VTOLBackTransitionAction::run(
    const std::shared_ptr<px4_ros2::ActionHandler> & handler,
    const px4_ros2::ActionArguments &,
    const std::function<void()> & on_completed)
{
    RCLCPP_INFO(_node.get_logger(), "Starting back transition");
    runTransition(handler, on_completed);
}
