#pragma once

#include <px4_ros2/mission/mission_executor.hpp>
#include <aircraft_msgs/msg/aircraft_state.hpp>

#include <cstdint>

class UpdateAircraftStateAction : public px4_ros2::ActionInterface {
	public:
		UpdateAircraftStateAction(px4_ros2::ModeBase& mode,
				std::shared_ptr<aircraft_msgs::msg::AircraftState> aircraft_state)
			: _node(mode.node()), _aircraft_state(std::move(aircraft_state)) {}

		std::string name() const override { return "updateAircraftState"; }

		void run(const std::shared_ptr<px4_ros2::ActionHandler>& handler,
				const px4_ros2::ActionArguments& arguments,
				const std::function<void()>& on_completed) override
		{
			if (arguments.contains("trackType")) {
				auto val = static_cast<uint8_t>(arguments.at<unsigned>("trackType"));
				RCLCPP_INFO(_node.get_logger(), "Updating track_type: %u -> %u",
						_aircraft_state->track_type, val);
				_aircraft_state->track_type = val;
			}
			if (arguments.contains("currentPhase")) {
				auto val = static_cast<uint8_t>(arguments.at<unsigned>("currentPhase"));
				RCLCPP_INFO(_node.get_logger(), "Updating current_phase: %u -> %u",
						_aircraft_state->current_phase, val);
				_aircraft_state->current_phase = val;
			}

			if (! arguments.contains("trackType") && !arguments.contains("currentPhase")) {
				RCLCPP_WARN(_node.get_logger(), "updateAircraftState Action called but nothing to update.");
			}

			on_completed();
		}

	private:
		rclcpp::Node& _node;
		std::shared_ptr<aircraft_msgs::msg::AircraftState> _aircraft_state;
};
