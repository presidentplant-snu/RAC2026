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
    if (arguments.contains("dwellTime")) {
      _dwell_time_s = arguments.at<float>("dwellTime");
    }

    _converged_since.reset();

    RCLCPP_INFO(_node.get_logger(),
      "PreciseGoTo: lat=%.6f lon=%.6f alt=%.1f heading=%.1fdeg posPrecision=%.2f "
      "hdgPrecision=%.1fdeg dwellTime=%.1fs",
      _target.x(), _target.y(), _target.z(),
      px4_ros2::radToDeg(_heading_target),
      _position_error_threshold,
      px4_ros2::radToDeg(_heading_error_threshold),
      _dwell_time_s);

    // Step 1: Use runTrajectory to fly to the target
    auto trajectory = std::make_shared<px4_ros2::Mission>(
      std::vector<px4_ros2::MissionItem>{px4_ros2::Waypoint(_target)});

    handler->runTrajectory(trajectory, [this, on_completed]() {
      // Step 2: Coarse arrival — now hold and wait for precision convergence
      RCLCPP_INFO(_node.get_logger(), "PreciseGoTo: coarse arrival, starting precision hold");
      _converged_since.reset();
      _timer = _node.create_wall_timer(std::chrono::milliseconds(33), [this, on_completed]() {
        _goto_setpoint->update(_target, _heading_target);

        if (positionReached() && headingReached()) {
          if (!_converged_since) {
            _converged_since = _node.get_clock()->now();
          }
          auto elapsed = (_node.get_clock()->now() - *_converged_since).seconds();
		  RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 500,
    "PreciseGoTo: holding %.1f/%.1fs", elapsed, _dwell_time_s);
          if (elapsed >= _dwell_time_s) {
            RCLCPP_INFO(_node.get_logger(), "PreciseGoTo: precision target reached (held %.1fs)", _dwell_time_s);
            _timer->cancel();
            on_completed();
          }
        } else {
          if (_converged_since) {
            RCLCPP_DEBUG(_node.get_logger(), "PreciseGoTo: left tolerance, resetting dwell timer");
          }
          _converged_since.reset();
        }
      });
    });
  }

  void deactivate() override
  {
    if (_timer) {
      _timer->cancel();
      _timer.reset();
    }
    _converged_since.reset();
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

  rclcpp::Node & _node;
  std::shared_ptr<px4_ros2::MulticopterGotoGlobalSetpointType> _goto_setpoint;
  std::shared_ptr<px4_ros2::OdometryGlobalPosition> _vehicle_global_position;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;

  Eigen::Vector3d _target;
  float _heading_target{0.f};
  rclcpp::TimerBase::SharedPtr _timer;
  std::optional<rclcpp::Time> _converged_since;

  float _position_error_threshold = 1.0f;
  float _heading_error_threshold = 5.0_deg;
  float _dwell_time_s = 3.0f;
};
