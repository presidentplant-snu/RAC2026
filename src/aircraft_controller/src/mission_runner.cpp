#include <rclcpp/rclcpp.hpp>

#include <px4_ros2/mission/trajectory/multicopter/waypoint_trajectory_executor.hpp>

#include "mission_runner.hpp"

#include "actions/vtol_actions.hpp"
#include "actions/vision_tracker_mode_action.hpp"

#include "trajectory/vtol_trajectory_executor.hpp"
#include "trajectory/fw_trajectory_executor.hpp"


std::shared_ptr<MissionRunnerNode> MissionRunnerNode::create()
{
	// Can't use make_shared here because constructor is private; use new directly.
	auto node = std::shared_ptr<MissionRunnerNode>(new MissionRunnerNode());
	node->init();
	return node;
}


void MissionRunnerNode::init()
{
	const std::string mission_file =
		this->get_parameter("mission_file").as_string();

	if (mission_file.empty()) {
		RCLCPP_FATAL(get_logger(),
				"Parameter 'mission_file' is required but was not set.");
		throw std::runtime_error("mission_file parameter not set");
	}

	RCLCPP_INFO(get_logger(), "Loading mission from: %s", mission_file.c_str());

	// -----------------------------------------------------------------
	// Build MissionExecutor configuration
	// -----------------------------------------------------------------
	px4_ros2::MissionExecutor::Configuration config;
	
	// TODO: Test VTOL Trajectory Executor
	config.withTrajectoryExecutor<px4_ros2::multicopter::WaypointTrajectoryExecutor>();

	// =================================================================
	// Register custom actions 
	constexpr uint8_t vision_tracker_mode_id = 23;
	config.addCustomAction<VisionTrackerModeAction>(vision_tracker_mode_id);
	config.addCustomAction<VTOLForwardTransitionAction>();
	config.addCustomAction<VTOLBackTransitionAction>();
	// =================================================================

	// TODO: Check if persistencefile works well.
	// config.withPersistenceFile("/app/missions/.state");

	// -----------------------------------------------------------------
	// Create executor
	// -----------------------------------------------------------------
	_executor = std::make_unique<px4_ros2::MissionExecutor>(
			"RAC2026_Mission", config, *this);

	// -----------------------------------------------------------------
	// Callbacks
	// -----------------------------------------------------------------
	_executor->onActivated([this]() {
			RCLCPP_INFO(get_logger(), "Mission activated.");
			});

	_executor->onDeactivated([this]() {
			RCLCPP_INFO(get_logger(), "Mission deactivated.");
			});

	_executor->onProgressUpdate([this](int index) {
			RCLCPP_INFO(get_logger(), "Mission item index: %d", index);
			});

	_executor->onCompleted([this]() {
			RCLCPP_INFO(get_logger(), "Mission completed successfully.");
			});

	_executor->onActivityInfoChange([this](const std::optional<std::string> & info) {
			if (info) {
			RCLCPP_INFO(get_logger(), "Activity: %s", info->c_str());
			}
			});

	// -----------------------------------------------------------------
	// Watch the JSON file — reloads automatically if it changes on disk.
	// -----------------------------------------------------------------
	_monitor = std::make_unique<px4_ros2::MissionFileMonitor>(
			shared_from_this(),
			mission_file,
			[this](std::shared_ptr<px4_ros2::Mission> mission) {
			RCLCPP_INFO(get_logger(), "Mission loaded/updated, setting on executor.");
			_executor->setMission(*mission);
			}
			);

	// -----------------------------------------------------------------
	// Register with PX4 (blocking until FMU responds)
	// -----------------------------------------------------------------
	if (!_executor->doRegister()) {
		throw std::runtime_error("MissionExecutor registration failed");
	}

	RCLCPP_INFO(get_logger(),
			"Mission runner ready. Select this mode in PX4 to start.");
}


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(MissionRunnerNode::create());
	rclcpp::shutdown();
	return 0;
}
