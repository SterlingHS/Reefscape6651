# Command that drops coral on reef
#
# Asume coral is on dropper and robot is centered in front of reef
# 1. Go to the correct level (L2, L3 or L4)
# 2. Drop Corral until corral is not on Dropper
# 3. Wait for the coral to be dropped
# 4. Go back down to level 1

# Use FRC sequential commands 

from commands2 import SequentialCommandGroup, waitcommand
from commands import ElevatorFloor, CoralDrop

from subsystems.Elevator import Elevator
from subsystems.Dropper import Dropper

class CoralDropAuto(SequentialCommandGroup):
    def __init__(self, elevator:Elevator, dropper:Dropper, floor:int):
        super().__init__(
                ElevatorFloor(elevator, floor), # Sends elevator to the correct floor
                CoralDrop(dropper),            # Drops coral until it is not on the dropper
                waitcommand.WaitCommand(1),     # Waits for coral to be dropped
                ElevatorFloor(elevator, 1)      # Sends elevator back to level 1
        )
    