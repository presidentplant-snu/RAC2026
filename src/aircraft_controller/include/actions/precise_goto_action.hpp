#pragma once
#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/mission/mission_executor.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/utils/geodesic.hpp>

using namespace px4_ros2::literals;

class PreciseGoToAction : public px4_ros2::ActionInterface
{
public:
  explicit PreciseGoToAction(px4_ros2::ModeBase & mode)
  : _node(mode.node())
  {
    _goto_setpoint = std::make_shared<px4_ros2::MulticopterGotoGlobalSetpointType>(mode);
    _vehicle_global_position = std::make_shared<px4_ros2::OdometryGlobalPosition>(mode);
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(mode);
  }

  std::string name() const override { return "preciseGoTo"; }

  void run(
    const std::shared_ptr<px4_ros2::ActionHandler> & handler,
    const px4_ros2::ActionArguments & arguments,
    const std::function<void()> & on_completed) override
  {
    if (!arguments.contains("lat") || !arguments.contains("lon") ||
        !arguments.contains("alt") || !arguments.contains("heading")) {
      RCLCPP_FATAL(_node.get_logger(), "PreciseGoTo: lat, lon, alt, heading are required!");
      throw std::runtime_error("PreciseGoTo missing required arguments");
    }

    _target = Eigen::Vector3d(
      arguments.at<double>("lat"),
      arguments.at<double>("lon"),
      arguments.at<double>("alt")
    );
    _heading_target = px4_ros2::degToRad(arguments.at<float>("heading"));

    if (arguments.contains("positionPrecision")) {
      _position_error_threshold = arguments.at<float>("positionPrecision");
    }
    if (arguments.contains("headingPrecision")) {
      _heading_error_threshold = px4_ros2::degToRad(arguments.at<float>("headingPrecision"));
    }
    if (arguments.contains("velocityThreshold")) {
      _velocity_threshold = arguments.at<float>("velocityThreshold");
    }

    _on_completed = on_completed;

    RCLCPP_INFO(_node.get_logger(),
      "PreciseGoTo: lat=%.6f lon=%.6f alt=%.1f heading=%.1fdeg posPrecision=%.2f hdgPrecision=%.1fdeg velThresh=%.2f",
      _target.x(), _target.y(), _target.z(),
      px4_ros2::radToDeg(_heading_target),
      _position_error_threshold,
      px4_ros2::radToDeg(_heading_error_threshold),
      _velocity_threshold);

    _timer = _node.create_wall_timer(std::chrono::milliseconds(33), [this]() {
      _goto_setpoint->update(_target, _heading_target);
      if (positionReached() && headingReached() && velocityBelow()) {
        _timer->cancel();
        _on_completed();
      }
    });
  }

private:
  bool positionReached() const
  {
    return px4_ros2::distanceToGlobalPosition(
      _vehicle_global_position->position(), _target) < _position_error_threshold;
  }

  bool headingReached() const
  {
    float err = px4_ros2::wrapPi(_heading_target - _vehicle_local_position->heading());
    return fabsf(err) < _heading_error_threshold;
  }

  bool velocityBelow() const
  {
    return _vehicle_local_position->velocityNed().norm() < _velocity_threshold;
  }

  rclcpp::Node & _node;
  std::shared_ptr<px4_ros2::MulticopterGotoGlobalSetpointType> _goto_setpoint;
  std::shared_ptr<px4_ros2::OdometryGlobalPosition> _vehicle_global_position;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;

  Eigen::Vector3d _target;
  float _heading_target;
  std::function<void()> _on_completed;
  rclcpp::TimerBase::SharedPtr _timer;

  float _position_error_threshold = 1.0f;
  float _heading_error_threshold = 5.0_deg;
  float _velocity_threshold = 1.0f;
};
