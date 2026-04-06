#pragma once

#include <rclcpp/rclcpp.hpp>

#include <px4_ros2/mission/trajectory/trajectory_executor.hpp>
#include <px4_ros2/mission/mission.hpp>
#include <px4_ros2/mission/mission_executor.hpp>
#include <px4_ros2/control/vtol.hpp>
#include <px4_ros2/components/mode.hpp>

#include <aircraft_msgs/msg/aircraft_state.hpp>

#include <px4_ros2/mission/trajectory/multicopter/waypoint_trajectory_executor.hpp>
#include "trajectory/fw_trajectory_executor.hpp"

class VTOLTrajectoryExecutor: public px4_ros2::TrajectoryExecutorInterface
{
	public:
		explicit VTOLTrajectoryExecutor(px4_ros2::ModeBase & mode,
				std::shared_ptr<aircraft_msgs::msg::AircraftState> aircraft_state)
			: _node(mode.node()),
			_aircraft_state(aircraft_state)

	{
		_mc_trajectory_executor = std::make_shared<px4_ros2::multicopter::WaypointTrajectoryExecutor>(mode, 2.0f);
		_fw_trajectory_executor = std::make_shared<FixedWingTrajectoryExecutor>(mode, 4.0f, 4.0f);
		_vtol = std::make_shared<px4_ros2::VTOL>(mode);
	}

		~VTOLTrajectoryExecutor() override = default;

		bool navigationItemTypeSupported(px4_ros2::NavigationItemType type) override
		{
			return type == px4_ros2::NavigationItemType::Waypoint;
		}
		bool frameSupported(px4_ros2::MissionFrame frame) override
		{
			return frame == px4_ros2::MissionFrame::Global;
		}

		void updateSetpoint() override {
			if (auto* exec = activeExecutor()) exec->updateSetpoint();
		}
		void runTrajectory(const px4_ros2::TrajectoryExecutorInterface::TrajectoryConfig & config) override {
			if (auto* exec = activeExecutor()) exec->runTrajectory(config);
		}

	private:
		px4_ros2::TrajectoryExecutorInterface* activeExecutor(){

			switch (_vtol->getCurrentState())
			{
				case px4_ros2::VTOL::State::FixedWing:
					return _fw_trajectory_executor.get();
					break;
				case px4_ros2::VTOL::State::Multicopter:
					return _mc_trajectory_executor.get();
					break;
				default:
					RCLCPP_WARN_THROTTLE(_node.get_logger(), *_node.get_clock(), 
							1000, 
							"VTOL State undefined, not sending any waypoints for now!");
					return nullptr;
					break;
			}
		}

		std::shared_ptr<px4_ros2::VTOL> _vtol;
		rclcpp::Node & _node;

		std::shared_ptr<px4_ros2::TrajectoryExecutorInterface> _mc_trajectory_executor;
		std::shared_ptr<px4_ros2::TrajectoryExecutorInterface> _fw_trajectory_executor;

		std::shared_ptr<aircraft_msgs::msg::AircraftState> _aircraft_state;
};
