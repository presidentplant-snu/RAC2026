#pragma once

#include <rclcpp/rclcpp.hpp>

#include <px4_ros2/mission/trajectory/trajectory_executor.hpp>
#include <px4_ros2/mission/mission.hpp>
#include <px4_ros2/mission/mission_executor.hpp>
#include <px4_ros2/control/vtol.hpp>
#include <px4_ros2/components/mode.hpp>

class VTOLTrajectoryExecutor: public px4_ros2::TrajectoryExecutorInterface
{
	public:
		explicit VTOLTrajectoryExecutor(px4_ros2::ModeBase & mode,
				std::shared_ptr<px4_ros2::TrajectoryExecutorInterface> mc_trajectory_executor,
				std::shared_ptr<px4_ros2::TrajectoryExecutorInterface> fw_trajectory_executor)
			: _node(mode.node()),
			_mc_trajectory_executor(mc_trajectory_executor),
			_fw_trajectory_executor(fw_trajectory_executor)
	{
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
							"VTOL State undefined, cannot continue with waypoint! AHH");
					return nullptr;
					break;
			}
		}

		std::shared_ptr<px4_ros2::VTOL> _vtol;
		rclcpp::Node & _node;

		std::shared_ptr<px4_ros2::TrajectoryExecutorInterface> _mc_trajectory_executor;
		std::shared_ptr<px4_ros2::TrajectoryExecutorInterface> _fw_trajectory_executor;
};
