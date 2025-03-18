# Command that sends elevator to a specific floor

from commands2 import Command
from subsystems.Elevator import Elevator

class ElevatorFloor(Command):
    def __init__(self, elevator:Elevator, floor:int):
        Command.__init__(self)
        self.elevator = elevator
        self.floor = floor

    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        # self.elevator.setElevatorFloor(self.floor)
        self.elevator.setPeriodicFloor(self.floor)

    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return self.elevator.isReachedFloor(self.floor)


    