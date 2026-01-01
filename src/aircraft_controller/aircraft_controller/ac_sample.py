import rclpy
import numpy as np
from enum import Enum, auto
from .core.mission_controller import MissionController


class SurveyState(Enum):
    INIT = auto()
    ARM = auto()
    OFFBOARD = auto()
    TAKEOFF = auto()
    GO_NORTH = auto()
    LAND = auto()
    COMPLETE = auto()


class SimpleSurvey(MissionController):
    """Simple survey pattern mission"""
    
    def define_states(self) -> type[Enum]:
        return SurveyState
    
    def define_state_handlers(self) -> dict:
        return {
            SurveyState.INIT: self.handle_init,
            SurveyState.ARM: self.handle_arm,
            SurveyState.OFFBOARD: self.handle_goto_offboard,
            SurveyState.TAKEOFF: self.handle_takeoff,
            SurveyState.GO_NORTH: self.handle_go_north,
            SurveyState.LAND: self.handle_land,
            SurveyState.COMPLETE: self.handle_complete,
        }
    
    def initialize_mission(self) -> None:
        # Nothing to do for this example.
        return

    def handle_init(self) -> None:
        """Initialize - use default then create waypoints"""
        if self.is_initialized():
            self.transition_to(SurveyState.ARM)
        
    def handle_arm(self) -> None:
        """Check systems"""

        if not self.is_armed():
            self.arm()
            return
        else:
            self.transition_to(SurveyState.OFFBOARD)

    def handle_goto_offboard(self) -> None:
        if not self.is_offboard_mode():
            self.set_offboard_mode()
            return
        else:
            self.transition_to(SurveyState.TAKEOFF)
    
    def handle_takeoff(self) -> None:
        """Ascend to altitude"""

        if self.goto_altitude(10, self.yaw):
            self.transition_to(SurveyState.LAND)

    def handle_go_north(self) -> None:
        """Go North"""
        if self.goto_position_rel(np.array([5,0,0]), self.yaw):
            self.transition_to(SurveyState.LAND)
    
    def handle_land(self) -> None:
        """Land"""
        self.land()
        if self.is_disarmed():
            self.transition_to(SurveyState.COMPLETE)
    
    def handle_complete(self) -> None:
        """Complete"""
        pass



def main(args=None):
    rclpy.init(args=args)
    mission = SimpleSurvey("simple_survey")
    
    try:
        rclpy.spin(mission)
    except KeyboardInterrupt:
        mission.get_logger().info("Interrupted")
    finally:
        mission.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
