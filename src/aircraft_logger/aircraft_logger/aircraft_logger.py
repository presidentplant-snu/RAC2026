import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSPresetProfiles
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import SensorGps, VehicleLocalPosition, VehicleAttitude

import numpy as np

import os, time, csv

from dataclasses import dataclass, field, fields, astuple

@dataclass
class LogEntry:
    autoflag: bool      = field(metadata={"header": "Auto Flag"})
    wpt_num: int        = field(metadata={"header": "WPT Number"})
    gps_time: float     = field(metadata={"header": "GPS Time (s)", "precision":6})
    latitude: float     = field(metadata={"header": "Latitude (deg)", "precision":6})
    longitude: float    = field(metadata={"header": "Longitude (deg)", "precision":6})
    altitude: float     = field(metadata={"header": "Altitude (m)", "precision":1})
    ax: float           = field(metadata={"header": "Ax (m/s^2)", "precision":3})
    ay: float           = field(metadata={"header": "Ay (m/s^2)", "precision":3})
    az: float           = field(metadata={"header": "Az (m/s^2)", "precision":3})
    roll: float         = field(metadata={"header": "Roll (deg)", "precision":4})
    pitch: float        = field(metadata={"header": "Pitch (deg)", "precision":4})
    yaw: float          = field(metadata={"header": "Yaw (deg)", "precision":4})

    def format(self):

        def _format_number(num: float, precision:int = 6):
            return np.format_float_positional(num, precision=precision, trim='k', unique=False)

        return [
            _format_number(v, precision=f.metadata.get("precision",6)) if isinstance(v, float) else v
            for f, v in zip(fields(self), astuple(self))
        ]


class AircraftLogger(Node):

    def __init__(self, log_path: str="/app/flight_logs/") -> None:
        super().__init__("aircraft_logger")
        
        self.do_logging = False 

        self.vehicle_local_pos = VehicleLocalPosition()
        self.vehicle_attitude = VehicleAttitude()
        self.vehicle_gps = SensorGps()

        self._setup_subscribers()
        
        os.makedirs(log_path, exist_ok=True)

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        log_file_path = os.path.join(log_path, f"vehicle_log_{timestamp}.csv")
        self.log_file = open(log_file_path, "w", newline="")
        self.log_writer = csv.writer(self.log_file)

        self.get_logger().info(f"Started logging on {log_file_path}")

        headers = [f.metadata["header"] for f in fields(LogEntry)]
        self.log_writer.writerow(headers)
        self.log_file.flush()

        self.log_timer = self.create_timer(0.1, self.log_flight)
    
    # TODO: Add vehicle state subscription
    def _setup_subscribers(self):
        def _get_message_name_version(msg_class):
            if msg_class.MESSAGE_VERSION == 0:
                return ""
            return f"_v{msg_class.MESSAGE_VERSION}"

        qos_profile = QoSPresetProfiles.SENSOR_DATA.value

        self.vehicle_local_pos_subscriber = self.create_subscription(
            VehicleLocalPosition,
            f"/fmu/out/vehicle_local_position{_get_message_name_version(VehicleLocalPosition)}",
            self._vehicle_local_position_callback,
            qos_profile,
        )

        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude,
            f"/fmu/out/vehicle_attitude{_get_message_name_version(VehicleAttitude)}",
            self._vehicle_attitude_callback,
            qos_profile,
        )

        self.vehicle_gps_subscriber = self.create_subscription(
            SensorGps,
            f"/fmu/out/vehicle_gps_position",
            self._vehicle_gps_callback,
            qos_profile,
        )

    """
    Format flight data and write row
    """
    def log_flight(self):
        if not self.do_logging:
            return
        
        def _quat_to_rpy(q):
            q = np.array(q, dtype=float)
            norm = np.linalg.norm(q)
            if norm == 0:
                return (0.0, 0.0, 0.0)
            w, x, y, z = q / norm

            roll  = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
            pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
            yaw   = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

            return roll, pitch, yaw

        roll, pitch, yaw = np.rad2deg(_quat_to_rpy(self.vehicle_attitude.q))

        log_entry = LogEntry(
                autoflag=False,
                wpt_num=0,
                gps_time=float(self.vehicle_gps.time_utc_usec)/100000,
                latitude = self.vehicle_gps.latitude_deg,
                longitude = self.vehicle_gps.longitude_deg,
                altitude = self.vehicle_gps.altitude_ellipsoid_m,
                ax = self.vehicle_local_pos.ax,
                ay = self.vehicle_local_pos.ay,
                az = self.vehicle_local_pos.az,
                roll = roll,
                pitch = pitch,
                yaw = yaw
            )

        print(log_entry)

        self.log_writer.writerow(log_entry.format())
        self.log_file.flush()
    
    def _vehicle_local_position_callback(self,msg):
        self.vehicle_local_pos = msg

    def _vehicle_attitude_callback(self,msg):
        if np.linalg.norm(msg.q) == 0:
            self.get_logger().warn("Quaternion invalid! (norm is 0).")
        self.vehicle_attitude = msg

    def _vehicle_gps_callback(self,msg):
        if msg.fix_type <= 2:
            self.get_logger().warn("GPS Fixtype <= 2")
        if msg.timestamp == 0:
            self.get_logger().warn("GPS Time == 0")

        self.vehicle_gps = msg

def main(args=None):
    rclpy.init(args=args)
    
    aircraft_logger = AircraftLogger(log_path="/app/flight_logs/")
    
    aircraft_logger.do_logging = True
    rclpy.spin(aircraft_logger)


    aircraft_logger.destroy_node()
    rclpy.shutdown()
