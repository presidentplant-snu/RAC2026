#pragma once
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <aircraft_msgs/msg/camera_tracked_object.hpp>

static const std::string mode_name = "Vision Tracker";
static const std::string _cam_tracked_obj_topic_name = "/camera_tracked_object";

// ------------------------------------------------------------------
// 2D slew rate limiter (XY only)
// Limits velocity and acceleration of a position setpoint, and applies
// a braking constraint so the setpoint decelerates before reaching target.
// ------------------------------------------------------------------
struct SlewRate2D
{
    float dt{0.02f};
    float v_max{1.0f};
    float a_max{1.0f};

    void reset() { _initialized = false; }

    // Returns a slew-limited XY setpoint given current XY position and raw XY target
    Eigen::Vector2f update(const Eigen::Vector2f & current_pos,
                           const Eigen::Vector2f & sp_target)
    {
        if (!_initialized) {
            _sp_last  = current_pos;
            _sp_last2 = current_pos;
            _initialized = true;
        }

        Eigen::Vector2f sp_curr = sp_target;

        // Velocity limit
        Eigen::Vector2f v_sp = (sp_curr - _sp_last) / dt;
        float v_norm = v_sp.norm();
        if (v_norm > v_max) {
            v_sp   = v_sp / v_norm * v_max;
            sp_curr = _sp_last + v_sp * dt;
        }

        // Acceleration limit
        Eigen::Vector2f a_sp = (sp_curr - 2.f * _sp_last + _sp_last2) / (dt * dt);
        float a_norm = a_sp.norm();
        if (a_norm > a_max) {
            a_sp    = a_sp / a_norm * a_max;
            sp_curr = 2.f * _sp_last - _sp_last2 + a_sp * dt * dt;
        }

        // Braking: v_stop from 2as = v^2, with 0.5 safety factor
        float dist_to_target = (current_pos - sp_target).norm();
        float v_stop_max     = 0.5f * sqrtf(a_max * dist_to_target);
        v_sp   = (sp_curr - _sp_last) / dt;
        v_norm = v_sp.norm();
        if (v_norm > v_stop_max) {
            v_sp    = v_sp / v_norm * v_stop_max;
            sp_curr = _sp_last + v_sp * dt;
        }

        _sp_last2 = _sp_last;
        _sp_last  = sp_curr;
        return sp_curr;
    }

private:
    bool           _initialized{false};
    Eigen::Vector2f _sp_last;
    Eigen::Vector2f _sp_last2;
};


class VisionTrackerMode : public px4_ros2::ModeBase
{
public:
    explicit VisionTrackerMode(rclcpp::Node & node);
    void onActivate() override;
    void onDeactivate() override;
    void updateSetpoint(float dt_s) override;

    // TODO: Convert all of these to ROS2 parameters.

    // Altitudes (m above target)
    float approach_alt{13.0f};
    float fine_alt{6.5f};
    float descend_alt{3.5f};

    // Timeouts
    float timeoutLimit{2.0f};
    float fineApproachTimeoutLimit{5.0f};

    bool use_distance_sensor{false};

    // Velocity / acceleration profiles
    float fast_v_max{2.0f};     // Approach + DescendToFine
    float fast_a_max{0.5f};
    float slow_v_max{0.15f};     // FineApproach
    float slow_a_max{0.05f};

    float descend_vel{0.5f};     // Z descent rate (m/s, positive = downward command magnitude)

    // Position thresholds
    float approach_pos_threshold{0.5f};
    float approach_vel_threshold{0.3f};
    float fine_pos_threshold{0.2f};
    float fine_vel_threshold{0.2f};
    float final_alt_threshold{0.15f};

private:
    std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
    std::shared_ptr<px4_ros2::OdometryLocalPosition>  _vehicle_local_position;

    rclcpp::Node& _node;
    rclcpp::Subscription<aircraft_msgs::msg::CameraTrackedObject>::SharedPtr _cam_tracked_obj_sub;

    rclcpp::Time    targetTimestamp{0, 0, RCL_ROS_TIME};
    bool            isTargetValid{false};
    Eigen::Vector3f _target_pos_ned;
    Eigen::Vector3f _descend_target_pos_ned;
    float           _initial_heading{0.f};

    SlewRate2D _slew;

    void trackedObjectCallback(const aircraft_msgs::msg::CameraTrackedObject::SharedPtr msg);

    enum class State {
        Init,
        Approach,
        DescendToFine,
        FineApproach,
        Descend,
        Done
    };

    std::string stateName(State state) const
    {
        switch (state) {
            case State::Init:          return "Init";
            case State::Approach:      return "Approach";
            case State::DescendToFine: return "DescendToFine";
            case State::FineApproach:  return "FineApproach";
            case State::Descend:       return "Descend";
            case State::Done:          return "Done";
            default:                   return "Unknown";
        }
    }

    State _state{State::Init};
    void switchToState(State state);

    // Send a slew-limited XY + fixed Z descent setpoint
    void sendTrackedSetpoint(const Eigen::Vector3f & raw_target, float z_target_ned);

    bool positionReached(const Eigen::Vector3f & target_position_m,
                         float pos_error_threshold = 0.5f,
						 float vel_error_threshold = 0.3f
						 ) const;
    bool headingReached(float target_heading_rad) const;
    bool checkTargetTimeout();
};
