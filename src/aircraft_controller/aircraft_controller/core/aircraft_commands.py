from .px4_interface import PX4Interface

import numpy as np
import pymap3d as p3d

class AircraftCommands(PX4Interface):
    """
    Adds flight commands (e.g. move , hold etc.) to PX4Interface
    """

    def __init__(
            self, node_name: str, 
            acceptance_rad_h: float = 2.0, acceptance_rad_z: float = 1.0,
            acceptance_angle_yaw: float = 0.15,
            max_vel_h: float = 5.0, max_vel_z: float = 2.0
         ) -> None:

        super().__init__(node_name)

        # Parameters for hold/move/slewrates. h: horizontal, z: vertical
        # Yaw is in radians
        # TODO: Add this to initialization parameters later on 
        self.acceptance_rad_h = acceptance_rad_h
        self.acceptance_rad_z = acceptance_rad_z

        self.acceptance_angle_yaw = acceptance_angle_yaw 

        self.max_vel_h = max_vel_h
        self.max_vel_z = max_vel_z

        # State vars used for holding position
        # For functions using this, MAKE SURE TO CONTINUE CALLING UNTIL TRACKING COMPLETES,
        # or call clear_saved_state() if exiting early.

        self._saved_state = False
        self._saved_pos = np.ndarray([0,0,0])
        self._saved_pos_gps = np.ndarray([0,0,0])
        self._saved_yaw = 0.0

    def goto_position(
        self, 
        target_pos: np.ndarray, 
        target_yaw: float | None = None
    ) -> bool:
        """
            Sends a trajectory setpoint to position.
            returns True if aircraft is at said position.
        """

        self.publish_trajectory_setpoint(pos_sp=target_pos, yaw_sp=target_yaw)
        return self.is_at_position(target_pos,target_yaw)

    def goto_position_rel(
        self, 
        target_pos: np.ndarray, 
        target_yaw: float | None = None
    ) -> bool:
        """
            Sends a trajectory setpoint to position relative to position when first called.
            returns True if aircraft is at said position.
        """

        if not self._saved_state:
            self.save_state()

        target_pos = self.saved_pos + target_pos
        
        if self.goto_position(target_pos, target_yaw):
            self.clear_saved_state()
            return True
        else:
            return False

    def goto_position_global(
        self,
        target_pos_global: np.ndarray,
        target_yaw: float | None = None
    ) -> bool:
        """
            Sends a trajectory setpoint to position (LLA, WGS84 coords)
            returns True if aircraft is at said position.
        """
        target_pos = self._convert_gps_to_local(target_pos_global)
        self.publish_trajectory_setpoint(pos_sp=target_pos, yaw_sp=target_yaw)
        return self.is_at_position(target_pos,target_yaw)

    def goto_altitude(
        self, 
        target_alt: float,
        target_yaw: float | None = None
    ) -> bool:
        """
            Goes to set altitude (AMSL) and returns if aircraft is at said altitude.
            Holds lateral position and yaw with the saved state
        """
        
        # Note: since we always populate self.saved_* before, there *shouldn't* be any logical errors.
        if not self._saved_state:
            self.save_state()

        target_pos = self.saved_pos

        alt_delta = target_alt - self.pos_gps[:2]
        
        # Subtract since LLA up is +ve and NED up is -ve 
        # i.e. Increase in altitude (alt_delta > 0) means decrease in D coordinate
        target_pos[2] -= alt_delta 
        
        if self.goto_position(target_pos, target_yaw):
            self.clear_saved_state()
            return True
        else:
            return False

    def goto_terrain_dist(
        self, 
        target_dist: float,
        target_yaw: float | None = None
    ) -> bool:
        """
            Goes to set distance from ground (meters) and returns if aircraft is at said distance.
            Needs distance sensor. https://docs.px4.io/main/en/sensor/rangefinders
            Holds lateral position and yaw with the saved state
        """
        
        # Note: since we always populate self.saved_* before, there *shouldn't* be any logical errors.
        if not self._saved_state:
            self.save_state()

        target_pos = self.saved_pos
        
        alt_delta = target_dist - self.dist_bottom
        
        # Subtract since increase in distance is +ve and NED up is -ve 
        # i.e. Increase in distance = Increase in altitude means decrease in D coordinate

        target_pos[2] -= alt_delta 
        
        if self.goto_position(target_pos, target_yaw):
            self.clear_saved_state()
            return True
        else:
            return False
    
    def hold_position(self) -> bool:
        """
        Holds position at the first time this is called after clearing state.
        Returns whether or not position is held at current moment.
        """

        if not self._saved_state:
            self.save_state()

        if self.goto_position(self._saved_pos, self._saved_yaw):
            self.clear_saved_state()
            return True
        else:
            return False

    def is_at_position(
        self,
        target_pos: np.ndarray,
        target_yaw: float | None = None
    ) -> bool:
        is_in_xy = np.linalg.norm(self.pos[:2] - target_pos[:2]) < self.acceptance_rad_h
        is_in_z = np.linalg.norm(self.pos[2] - target_pos[2]) < self.acceptance_rad_z
        
        is_in_yaw = False

        if target_yaw is None:
            is_in_yaw = True
        else:
            is_in_yaw = np.linalg.norm(self.yaw - target_yaw) < self.acceptance_angle_yaw 
        
        # Type Checking shows an error becuase the variables are np.bool not bool.
        return bool(is_in_xy and is_in_z and is_in_yaw)
   
    def save_state(self) -> None:
        if self._saved_state:
            # Should not happen
            self.get_logger().warn("aircraft_commands.py: Overwriting saved state")

        self._saved_state = True
        self._saved_pos = self.pos
        self._saved_pos_gps = self.pos_gps
        self._saved_yaw = self.yaw

        return

    def clear_saved_state(self) -> None:
        self._saved_state = False

        # Set these to None so it throws an error if saved data is used before actually saving state.
        # This can cause warnings for typecheckers. (There might be a better way, leave that as TODO for now.)
        self._saved_pos = None
        self._saved_pos_gps = None
        self._saved_yaw = None

        return

    def _convert_gps_to_local(self, global_pos: np.ndarray):
        """ Convert global LLA coordinates to NED coordinates """
        x,y,z = p3d.geodetic2ned(
                global_pos[0], global_pos[1], global_pos[2],
                self.origin_pos[0], self.origin_pos[1], self.origin_pos[2],
                )

        return np.array([x,y,z])
