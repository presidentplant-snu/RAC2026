"""
Slew Rate Tracking Mixin
Adds slew rate limited tracking commands to AircraftCommands
"""

import numpy as np

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..core.aircraft_commands import AircraftCommands

class SlewRateMixin:

    if TYPE_CHECKING:
        self: "AircraftCommands"

    # TODO: add slewrate logic
