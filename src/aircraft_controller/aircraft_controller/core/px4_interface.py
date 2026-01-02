"""
PX4 Interface - Base class for PX4 communication
Handles all ROS2 and PX4 message subscriptions/publications
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from abc import ABC, abstractmethod
import numpy as np


# PX4 Messages - Subscriptions
from px4_msgs.msg import (
    VehicleStatus,
    VtolVehicleStatus,
    VehicleLocalPosition,
    VehicleAttitude,
    VehicleGlobalPosition,
    SensorGps,
    MissionResult,
    LandingTargetPose
)

# PX4 Messages - Publications
from px4_msgs.msg import (
    VehicleCommand,
    OffboardControlMode,
    TrajectorySetpoint,
    IrlockReport
)

# TODO: helper function for MAV_CMD_DO_SET_ACTUATOR 
# actuator_servos UORB message has multiple publishers unless we offboard servos & motors.

class PX4Interface(Node, ABC):
    """
    Base Interface for communication with PX4 FC
    """

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self._init_vehicle_state()

        self._setup_qos()
        # Subscriber and Publisher 
        self._create_subscribers()
        self._create_publishers()

        self._setup_timers(0.01)

        self.get_logger().info(f"{node_name} initialized")

    # ============================== 
    # Setup Methods
    # ============================== 


    def _init_vehicle_state(self):
        # PX4 Message Holders, ideally should not be accessed outside
        # Use np.nan for numbers or arrays, else use None
        self._vehicle_status: VehicleStatus | None = None
        self._vtol_vehicle_status: VtolVehicleStatus | None = None
        self._vehicle_local_position: VehicleLocalPosition | None = None
        self._vehicle_global_position: VehicleGlobalPosition | None = None
        self._vehicle_attitude: VehicleAttitude | None = None
        self._vehicle_gps: SensorGps | None = None
        self._mission_result: MissionResult | None = None
        self._landing_target_pose: LandingTargetPose | None = None

        # Offboard control mode configuration
        self.offboard_control_mode_params = {
            "position": True,
            "velocity": False,
            "acceleration": False,
            "attitude": False,
            "body_rate": False,
            "thrust_and_torque": False,
            "direct_actuator": False,
        }

        # Vehicle state 
        
        self.vtol_state: int = -1

        # NED Coordinate System (Origin is at EKF2 Start position = usually poweron pos)
        # Units in meters and m/s

        # Reference: https://docs.px4.io/main/en/msg_docs/VehicleLocalPosition

        self.pos: np.ndarray = np.array([np.nan, np.nan, np.nan]) 
        self.vel: np.ndarray = np.array([np.nan, np.nan, np.nan]) 

        # Global LLA System
        # In WGS84 coords
        # Reference: https://docs.px4.io/main/en/msg_docs/VehicleGlobalPosition
        self.pos_gps: np.ndarray = np.array([np.nan, np.nan, np.nan]) 
        self.yaw: float = np.nan # Radians
            
        # Quaternion [w, x, y, z]
        self.attitude_q: np.ndarray = np.array([np.nan, np.nan, np.nan, np.nan]) 

        # Home position (reference frame origin for LLA)
        self.origin_pos: np.ndarray = np.array([np.nan, np.nan, np.nan])

        # Mission state
        self.mission_wp_current: int = -1
        self.mission_wp_reached: int = -1

        # Landing target
        self.landing_target_valid: bool = False
        self.landing_target_pos: np.ndarray = np.array([np.nan, np.nan, np.nan])

        # Heartbeat monitoring
        self.last_heartbeat_time: rclpy.time.Time | None = None

    def _setup_qos(self):
        """Configure QoS profile for publishing and subscribing"""
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

    def _create_subscribers(self):
        """Create all PX4 message subscribers"""

        # TODO: bug with px4, it should be vehicle_status but it's vehicle_status_v1
        self._vehicle_status_subscriber = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v1",
            self._vehicle_status_callback,
            self.qos_profile,
        )

        self._vtol_vehicle_status_subscriber = self.create_subscription(
            VtolVehicleStatus,
            "/fmu/out/vtol_vehicle_status",
            self._vtol_vehicle_status_callback,
            self.qos_profile,
        )

        self._vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._vehicle_local_position_callback,
            self.qos_profile,
        )

        self._vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self._vehicle_attitude_callback,
            self.qos_profile,
        )

        self._vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition,
            "/fmu/out/vehicle_global_position",
            self._vehicle_global_position_callback,
            self.qos_profile,
        )
        
        self._vehicle_gps_subscriber = self.create_subscription(
            SensorGps,
            "/fmu/out/vehicle_gps_position",
            self._vehicle_gps_callback,
            self.qos_profile,
        )

        self._vehicle_mission_subscriber = self.create_subscription(
            MissionResult,
            "/fmu/out/mission_result",
            self._mission_result_callback,
            self.qos_profile,
        )
    
        self._vehicle_mission_subscriber = self.create_subscription(
            LandingTargetPose,
            "/fmu/out/landing_target_pose",
            self._landing_target_pose_callback,
            self.qos_profile,
        )

    def _create_publishers(self):
        """Create all PX4 message publishers"""
        self._vehicle_command_publisher = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", self.qos_profile
        )

        self._offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", self.qos_profile
        )

        self._trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", self.qos_profile
        )
    
        self._irlock_report_publisher = self.create_publisher(
            IrlockReport, "/fmu/in/irlock_report", self.qos_profile
        )

    def _setup_timers(self, timer_period:float = 0.01):
        """Setup ROS2 timers"""

        self._offboard_heartbeat_timer = self.create_timer(
            timer_period, self._offboard_heartbeat_callback
        )

        self.main_timer = self.create_timer(
            timer_period, self._main_timer_callback
        )


    # ============================== 
    # Timer Callbacks
    # ============================== 

    def _offboard_heartbeat_callback(self):
        """Heartbeat callback to maintain offboard mode"""

        now = self.get_clock().now()

        # Per https://docs.px4.io/main/en/flight_modes/offboard ,
        # we need to send a > 2Hz heartbeat for offboard control. 

        # We can send this constantly for simplicity.
        # Offboard control doesn't (shouldn't) happen unless we are in offboard mode and send setpoints.
        
        if self.last_heartbeat_time is not None:
            delta = (now - self.last_heartbeat_time).nanoseconds 
            if delta >= 500_000_000: # 0.5 s in nanoseconds
                # CODE SHOULD NEVER GET TO HERE
                self.get_logger().warn(f"Heartbeat delay detected: {float(delta)/1e9:.3f}s")

        self.last_heartbeat_time = now

        # Publish offboard control mode as well (unless using direct control, don't need to touch
        self.publish_offboard_control_mode(
            position=self.offboard_control_mode_params["position"],
            velocity=self.offboard_control_mode_params["velocity"],
            acceleration=self.offboard_control_mode_params["acceleration"],
            attitude=self.offboard_control_mode_params["attitude"],
            body_rate=self.offboard_control_mode_params["body_rate"],
            thrust_and_torque=self.offboard_control_mode_params["thrust_and_torque"],
            direct_actuator=self.offboard_control_mode_params["direct_actuator"]
        )
    
    def _main_timer_callback(self):
        """Main timer callback - calls the abstract main_loop method"""

        try:
            self.main_loop()
        except Exception as e:
            self.get_logger().error(f"Error in main loop: {e}")

    @abstractmethod
    def main_loop(self):
        # Override with actual control logic
        pass

    # ============================== 
    # Subscription Callbacks
    # ============================== 

    def _vehicle_status_callback(self, msg: VehicleStatus):
        """Callback for vehicle_status UORB message"""
        self._vehicle_status = msg


    def _vtol_vehicle_status_callback(self, msg: VtolVehicleStatus):
        """Callback for vtol_vehicle_status UORB message"""
        self._vtol_vehicle_status = msg
        self.vtol_state = msg.vehicle_vtol_state


    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        """Callback for vehicle_local_position UORB message"""
        self._vehicle_local_position = msg
        
        # These should not be invalid unless FC restarted (or crashed)
        # I will not add preflight checks to the code. It should be done manually + via CLI (ros2 topic echo etc.)
        # TODO: Maybe add a "preflight comms checker" script to check if there's comms between px4 and obc beforehand??

        pos_valid = (msg.xy_valid and msg.z_valid)
        vel_valid = (msg.v_xy_valid and msg.v_z_valid)

        if not pos_valid:
            self.get_logger().warn("local position invalid")

        if not vel_valid:
            self.get_logger().warn("local velocity invalid")
    
        self.pos = np.array([msg.x, msg.y, msg.z])
        self.vel = np.array([msg.vx, msg.vy, msg.vz])
        self.yaw = msg.heading

        # Use for Lidar / Optical Flow. Technically shouldn't be needed when using RTK GPS.
        # For SITL, just ignore because error's spammed.
        #if not msg.dist_bottom_valid:
            #self.get_logger().warn("dist bottom invalid")

        self.dist_bottom = msg.dist_bottom

        if not (msg.xy_global and msg.z_global):
            self.get_logger().warn("Reference position invalid")
        else:
            self.origin_pos = np.array([msg.ref_lat, msg.ref_lon, msg.ref_alt])

    def _vehicle_global_position_callback(self, msg: VehicleGlobalPosition):
        """Callback for vehicle_global_position UORB message"""
        self._vehicle_global_position = msg
        self.pos_gps = np.array([msg.lat, msg.lon, msg.alt])
        
    def _vehicle_attitude_callback(self, msg: VehicleAttitude):
        """Callback for vehicle_attitude UORB message"""
        self._vehicle_attitude = msg
        self.attitude_q = msg.q

    def _vehicle_gps_callback(self, msg: SensorGps):
        """Callback for vehicle_gps UORB message"""
        if msg.fix_type >= 3:
            # Check if GPS has at least FIX_TYPE_3D
            self._vehicle_gps = msg
        else:
            self.get_logger().warn("GPS fix type is less than 3, no valid GPS data")          
    def _mission_result_callback(self, msg: MissionResult):
        """Callback for mission_result UORB message"""
        # This is called only when the waypoint changes. Really annoying behavior from PX4.
        self._mission_result = msg
        self.mission_wp_current = msg.seq_current
        self.mission_wp_reached = msg.seq_reached
    
    def _landing_target_pose_callback(self, msg: LandingTargetPose):
        """Callback for landing_target_pose UORB message"""

        # TODO: Check how this works out with regards to landing_target_valid etc.
        # This should be published everytime irlock report is sent out.
        self._landing_target_pose = msg

        self.landing_target_valid = bool(msg.abs_pos_valid)
        self.landing_target_pos = np.array([msg.x_abs, msg.y_abs, msg.z_abs]) # NED 

    # ============================== 
    # Publication Callbacks
    # ============================== 
   
    # TODO change this to service for receiving vehicle_command_ack messages
    def publish_vehicle_command(
            self, 
            command: int,
            param1: float = float('nan'),
            param2: float = float('nan'),
            param3: float = float('nan'),
            param4: float = float('nan'),
            param5: float = float('nan'),
            param6: float = float('nan'),
            param7: float = float('nan'),
            target_system: int = 1,
            target_component: int = 1
    ) -> None:
        """
        Publish MAVLink vehicle command.
        UORB VehicleCommand message

        Args:
            command: MAVLink command ID (e.g., VehicleCommand.VEHICLE_CMD_*)
            param1-7: Command parameters (meaning depends on command)
            target_system: Target system ID (default: 1)
            target_component: Target component ID (default: 1)
        """
        msg = VehicleCommand()
        msg.command = int(command)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)

        msg.target_system = target_system
        msg.target_component = target_component
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._vehicle_command_publisher.publish(msg)

    def publish_offboard_control_mode(
            self,
            position: bool = False,
            velocity: bool = False,
            acceleration: bool = False,
            attitude: bool = False,
            body_rate: bool = False,
            thrust_and_torque: bool = False,
            direct_actuator: bool = False
    ) -> None:
        """
        Publish offboard control mode configuration.
        See https://docs.px4.io/main/en/flight_modes/offboard for reference
        
        Args:
            position: Enable position control
            velocity: Enable velocity control
            acceleration: Enable acceleration control
            attitude: Enable attitude control
            body_rate: Enable body rate control
            thrust_and_torque: Enable thrust and torque control
            direct_actuator: Enable direct actuator control
        """
        msg = OffboardControlMode()
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = acceleration
        msg.attitude = attitude
        msg.body_rate = body_rate
        msg.thrust_and_torque = thrust_and_torque
        msg.direct_actuator = direct_actuator
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(
        self,
        pos_sp: np.ndarray | None = None,
        vel_sp: np.ndarray | None = None,
        yaw_sp: float | None = None
    ):
        """
        Publish trajectory setpoint for offboard control.
        
        Args:
            pos_sp: Position setpoint in NED frame relative to EKF2 Origin (m)
            vel_sp: Velocity setpoint in NED frame (m/s)
            yaw_sp: Yaw setpoint (radians)
        """

        if self._vehicle_status is None:
            self.get_logger().warn("Vehicle Status Not Received, Blocking.")
            return

        if not self._vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # Even if mode isn't offboard, we can still control the aircraft control loops.
            # finding that out was *not* fun.
            self.get_logger().warn("Not in offboard mode, blocking setpoint")
            return
        
        msg = TrajectorySetpoint()

        if pos_sp is not None:
            msg.position = list(map(float,pos_sp))
        else:
            msg.position = [float('nan')] * 3
        
        # Velocity
        if vel_sp is not None:
            msg.velocity = list(map(float,vel_sp))
        else:
            msg.velocity = [float('nan')] * 3
        
        # Yaw
        msg.yaw = float(yaw_sp) if yaw_sp is not None else float('nan')
        
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._trajectory_setpoint_publisher.publish(msg)       
    
        def publish_irlock_report(self, pos_x: float, pos_y: float):
            msg = IrlockReport()

            msg.pos_x = pos_x
            msg.pos_y = pos_y

            self._irlock_report_publisher.publish(msg)

    # =======================================
    # Vehicle State Helper Functions (can be added on a per-use basis)
    # =======================================
    
    # Mode Definitions are listed in https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/px4_custom_mode.h 

    def is_armed(self) -> bool:
        if self._vehicle_status is None:
            raise RuntimeError("Vehicle status not available")
        return self._vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def is_disarmed(self) -> bool:
        if self._vehicle_status is None:
            raise RuntimeError("Vehicle status not available")
        return self._vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED

    def is_offboard_mode(self) -> bool:
        if self._vehicle_status is None:
            raise RuntimeError("Vehicle status not available")
        return self._vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def is_mission_mode(self) -> bool:
        if self._vehicle_status is None:
            raise RuntimeError("Vehicle status not available")
        return self._vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_MISSION

    def is_takeoff_mode(self):
        """Check if vehicle is in auto takeoff mode"""
        if self._vehicle_status is None:
            raise RuntimeError("Vehicle status not available")
        return (
            self._vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF
        )

    def is_hold_mode(self):
        """Check if vehicle is in hold ( = loiter for FW ) mode """
        if self._vehicle_status is None:
            raise RuntimeError("Vehicle status not available")
        return (
            self._vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
        )
    def is_landing_mode(self):
        """Check if vehicle is in auto takeoff mode"""
        if self._vehicle_status is None:
            raise RuntimeError("Vehicle status not available")
        return (
            self._vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND
        )

    # =======================================
    # Vehicle Command Helper Functions
    # =======================================

    def arm(self):
        """Arm the vehicle. Returns true when armed"""
        if self.is_armed():
            return True

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )
        self.get_logger().info("Arm command sent")

        return False

    def disarm(self) -> bool:
        """Disarm the vehicle. Returns true when disarmed"""
        if self.is_disarmed():
            self.get_logger().info("Vehicle already disarmed")
            return True

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
        )
        self.get_logger().info("Disarm command sent")

        return False

    def set_offboard_mode(self):
        """Switch to offboard mode. Returns true when complete"""
        if not self.is_offboard_mode():
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
                param1=1.0,  # Custom mode enabled
                param2=6.0   # Offboard mode
            )
            self.get_logger().info("Offboard set mode cmt sent")

        return self.is_offboard_mode()

    def set_mission_mode(self) -> bool:
        """Switch to mission mode. Returns true when complete"""
        if not self.is_mission_mode():
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,  # Custom mode enabled
                param2=4.0,  # Auto mode
                param3=4.0   # Mission submode
            )
            self.get_logger().info("Mission set mode cmd sent")

        return self.is_mission_mode()

    def takeoff(self, altitude: float | None = None) -> bool:
        """Command takeoff to specified altitude, returns True when done"""

        # If altitude is nan, it is set to MIS_TAKEOFF_ALT parameter in PX4
        if self.is_takeoff_mode():
            return False

        if altitude is not None:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param7=float(altitude)
            )
            self.get_logger().info(f"Takeoff command sent (alt={altitude})")
        else:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF)
            self.get_logger().info(f"Takeoff command sent (no altitude given)")

        return self.is_hold_mode()


    def land(self):
        """Command landing"""
        if not self.is_landing_mode():
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_NAV_LAND
            )
            self.get_logger().info("Land command sent")
