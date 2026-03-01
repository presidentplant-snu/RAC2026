#pragma once

#include "rclcpp/rclcpp.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include <aircraft_msgs/msg/camera_tracked_object.hpp>

static const std::string mode_name = "Vision Tracker";

static const std::string _cam_tracked_obj_topic_name = "/camera_tracked_object";

class VisionTrackerMode : public px4_ros2::ModeBase
{
	public:
		explicit VisionTrackerMode(rclcpp::Node & node);

		void onActivate() override;
		void onDeactivate() override;
		void updateSetpoint(float dt_s) override;

		// TODO: Convert all of these to ROS2 parameters.
		float approach_alt{10};
		float descend_alt{2};
		float timeoutLimit = 2;
		bool use_distance_sensor{false};

		float horizontal_approach_vel{1};
		float horizontal_approach_vel_slow{1};
		float descend_vel{1};
		
	private:

		std::shared_ptr<px4_ros2::MulticopterGotoSetpointType> _goto_setpoint;
		std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;

		rclcpp::Node& _node;
		rclcpp::Subscription<aircraft_msgs::msg::CameraTrackedObject>::SharedPtr _cam_tracked_obj_sub;
		rclcpp::Time targetTimestamp{0};
		bool isTargetValid{false};
		Eigen::Vector3f _target_pos_ned;
		float _initial_heading{0};


		void trackedObjectCallback(const aircraft_msgs::msg::CameraTrackedObject::SharedPtr msg );

		enum class State {
			Init,
			Approach,
			Descend,
			Done
		};

		std::string stateName(State state)
		{
			switch (state) {
				case State::Init:
					return "Init";

				case State::Approach:
					return "Approach";

				case State::Descend:
					return "Descend";

				case State::Done:
					return "Done";

				default:
					return "Unknown";
			}
		}

		State _state = State::Init;
		void switchToState(State state);

		// Utility Functions
		bool positionReached(const Eigen::Vector3f & target_position_m) const;
		bool headingReached(float target_heading_rad) const;
		bool checkTargetTimeout();

};
