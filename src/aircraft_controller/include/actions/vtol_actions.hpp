#pragma once
#include <rclcpp/rclcpp.hpp>

#include <px4_ros2/mission/mission_executor.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/vtol.hpp>
#include <px4_ros2/vehicle_state/vehicle_status.hpp>

class VTOLTransitionAction : public px4_ros2::ActionInterface
{
public:
    explicit VTOLTransitionAction(px4_ros2::ModeBase & mode);
    void deactivate() override;

protected:
    virtual void doTransition() = 0;
    virtual px4_ros2::VTOL::State targetState() const = 0;

    void runTransition(
        const std::shared_ptr<px4_ros2::ActionHandler> & handler,
        const std::function<void()> & on_completed);

    rclcpp::Node & _node;
    std::shared_ptr<px4_ros2::VTOL> _vtol;
    std::shared_ptr<px4_ros2::VehicleStatus> _vehicle_status;
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
    void doTransition() override { _vtol->toFixedwing(); }
    px4_ros2::VTOL::State targetState() const override { return px4_ros2::VTOL::State::FixedWing; }
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
    void doTransition() override { _vtol->toMulticopter(); }
    px4_ros2::VTOL::State targetState() const override { return px4_ros2::VTOL::State::Multicopter; }
};
