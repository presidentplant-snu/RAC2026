from enum import Enum 
from abc import abstractmethod

from .aircraft_commands import AircraftCommands

class MissionController(AircraftCommands):

    def __init__(self, node_name: str, 
        acceptance_rad_h: float = 2, acceptance_rad_z: float = 1, 
        acceptance_angle_yaw: float = 0.15, 
        max_vel_h: float = 5, max_vel_z: float = 2
    ) -> None:

        super().__init__(node_name, 
                         acceptance_rad_h, acceptance_rad_z, 
                         acceptance_angle_yaw, 
                         max_vel_h, max_vel_z)
        
        self._init_state_machine()
        self.initialize_mission()

    def _init_state_machine(self) -> None :
        self.states = self.define_states()
        self.state_handlers = self.define_state_handlers()
        self.current_state = None
        self.state_start_time: float | None = None
        self.mission_start_time: float | None = None

    @abstractmethod
    def initialize_mission(self) -> None:
        """ Initialize other variables that are needed (e.g. waypoint coord etc.) """ 
        pass

    @abstractmethod
    def define_states(self) -> type[Enum]:
        """ Should return Enum instance containing all the states """
        pass

    @abstractmethod
    def define_state_handlers(self) -> dict:
        """ Should return dictionary containing all callback/handler functions for each state """
        pass

    def main_loop(self) -> None:
        """
        Main control loop executed at nominally 100 Hz by rclpy.
        """

        if self.mission_start_time is None:
            self.mission_start_time = self.get_clock().now().nanoseconds / 1e9

        if self.current_state is None:
            self.current_state = list(self.states)[0]
            self.state_start_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(f"Mission starting in state: {self.current_state.value}")

        # Execute current state handler
        handler = self.state_handlers.get(self.current_state)
        if handler:
            try:
                handler()
            except Exception as e:
                self.get_logger().error(f"Error in state {self.current_state.value}: {e}")
        else:
            self.get_logger().error(f"No handler defined for state: {self.current_state.value}")
    

    def transition_to(self, new_state: Enum) -> None:
        if new_state == self.current_state:
            # There are some instances where "resetting" a state might be needed,
            # However, it's uncommon enough to warrant a warning.
            self.get_logger().warn("Switching to same state")

        old_state = self.current_state
        self.current_state = new_state
        self.state_start_time = self.get_clock().now().nanoseconds / 1e9
        
        self.get_logger().info(f"State transition: {old_state.value} -> {new_state.value}")

    def wait_for_duration(self, duration: float) -> bool:
        """
        Check if duration has elapsed in current state.
        Useful for timed holds, delays, etc.
        """
        return self.get_state_time() >= duration


    def is_initialized(self) -> bool:
        """
        Default initialization handler.
        Returns True only if vehicle is good
        """
        if not self._vehicle_status:
            print("status")
            return False
        
        if not self._vehicle_global_position:
            print("global pos")
            return False

        return True
