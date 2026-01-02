"""
Slew Rate Tracking Mixin
Adds slew rate limited tracking commands to AircraftCommands
"""

import numpy as np

from typing import TYPE_CHECKING

from ..core.aircraft_commands import AircraftCommands

class SlewRateMixin(AircraftCommands):

    def init_slewrate(
        self, 
        dt: float, 
        v_h_max: float, a_h_max: float,
        v_z_max: float, a_z_max: float
    ):
        self.slew_h = SlewRate(dt, v_h_max, a_h_max)
        self.slew_z = SlewRate(dt, v_z_max, a_z_max)

    def goto_slew(
        self,
        target_pos: np.ndarray,
        yaw: float | None = None
    ) -> bool:

        """
            Sends a vel/accel limited trajectory setpoint to given target_pos
            returns True if aircraft is at said position.
        """
        slewed_h = self.slew_h.slew_rate(self.pos[:2], target_pos[:2])
        slewed_z = self.slew_z.slew_rate(self.pos[2], target_pos[2])
        
        slewed_target_pos = np.array([slewed_h[0], slewed_h[1], slewed_z])
        
        self.publish_trajectory_setpoint(slewed_target_pos, yaw_sp=yaw)

        return self.is_at_position(target_pos, yaw)

    def reset_slew(self) -> None:
        self.slew_h.reset()
        self.slew_z.reset()



class SlewRate():
    def __init__(self, dt: float, v_max: float, a_max: float):

        self.dt = dt
        self.v_max = v_max
        self.a_max = a_max

        self.sp_last: np.ndarray | None = None
        self.sp_last2: np.ndarray | None = None

    def slew_rate(self, current_pos, sp_target) -> np.ndarray:
        if self.sp_last is None:
            self.sp_last = current_pos.copy()
            self.sp_last2 = current_pos.copy()

        sp_curr = sp_target.copy()
        
        v_sp = (sp_curr - self.sp_last)/self.dt
        v_norm = np.linalg.norm(v_sp)

        if v_norm > self.v_max:
            v_sp = (v_sp / v_norm) * self.v_max
            sp_curr = self.sp_last + v_sp*self.dt

        a_sp = (sp_curr - 2 * self.sp_last + self.sp_last2) / (self.dt**2)
        a_norm = np.linalg.norm(a_sp)

        if a_norm > self.a_max:
            a_sp = (a_sp / a_norm) * self.a_max
            sp_curr = 2 * self.sp_last - self.sp_last2 + a_sp * self.dt**2


        dist_to_target = np.linalg.norm(current_pos - sp_target)
        # Supposed to be from 2as = v^2 but reduce it by a factor of 2 sqrt 2 just to be on the safe side.
        v_stop_max = 0.5 * np.sqrt(self.a_max * dist_to_target)

        v_sp = (sp_curr - self.sp_last) / self.dt
        v_norm = np.linalg.norm(v_sp)

        if v_norm > v_stop_max:
            v_sp = (v_sp / v_norm) * v_stop_max
            sp_curr = self.sp_last + v_sp*self.dt

        self.sp_last2 = self.sp_last
        self.sp_last = sp_curr.copy()

        return sp_curr
    
    def reset(self):
        self.sp_last = None
        self.sp_last2 = None
