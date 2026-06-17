#pragma once
#include <rclcpp/rclcpp.hpp>

#include <optional>

#include <Eigen/Eigen>

#include <px4_ros2/mission/mission_executor.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/vtol.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/control/setpoint_types/fixedwing/lateral_longitudinal.hpp>
#include <px4_ros2/odometry/local_position.hpp>

// Base class for the VTOL forward/back transition actions.
//
class VTOLTransitionAction : public px4_ros2::ActionInterface
{
public:
    explicit VTOLTransitionAction(px4_ros2::ModeBase & mode);
    void deactivate() override;

protected:
    // Run the transition on a timer until runTick() reports completion.
    // course_sp is the course/heading [rad] held during and after the transition.
    void runTransition(const std::function<void()> & on_completed, float course_sp);

    // Advance the transition by one tick, sending the appropriate setpoint.
    // Returns true once the target VTOL state has been reached.
    virtual bool runTick(float course_sp) = 0;

    // Setpoints sent while the FMU reports it is transitioning (common to both
    // directions): hold altitude, align to course_sp, and apply the given
    // acceleration setpoint along track.
    void sendTransitionSetpoint(float course_sp, const Eigen::Vector3f & acceleration_sp);

    float currentHeading() const { return _vehicle_local_position->heading(); }

    rclcpp::Node & _node;
    std::shared_ptr<px4_ros2::VTOL> _vtol;
    std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
    std::shared_ptr<px4_ros2::FwLateralLongitudinalSetpointType> _fixed_wing_setpoint;
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
    rclcpp::TimerBase::SharedPtr _timer;
};

class VTOLForwardTransitionAction : public VTOLTransitionAction
{
public:
    using VTOLTransitionAction::VTOLTransitionAction;
    std::string name() const override { return "forwardTransition"; }
    void run(const std::shared_ptr<px4_ros2::ActionHandler> & handler,
             const px4_ros2::ActionArguments & arguments,
             const std::function<void()> & on_completed) override;
protected:
    bool runTick(float course_sp) override;
private:
    // Optional forward acceleration [m/s^2] applied along the course during the
    // transition. If unset, the library default is used (which is ~zero).
    std::optional<float> _forward_acceleration;
};

class VTOLBackTransitionAction : public VTOLTransitionAction
{
public:
    using VTOLTransitionAction::VTOLTransitionAction;
    std::string name() const override { return "backTransition"; }
    void run(const std::shared_ptr<px4_ros2::ActionHandler> & handler,
             const px4_ros2::ActionArguments & arguments,
             const std::function<void()> & on_completed) override;
protected:
    bool runTick(float course_sp) override;
};
