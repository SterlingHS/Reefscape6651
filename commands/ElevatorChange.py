# Changes the floor of the elevator by 1 or -1
# 

from commands2 import Command
from subsystems.Elevator import Elevator

class ElevatorChange(Command):
    def __init__(self, elevator:Elevator, changeInFloor:int):
        Command.__init__(self)
        self.elevator = elevator
        self.changeInFloor = changeInFloor

    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        # self.elevator.setElevatorFloor(self.floor)
        self.elevator.changeFloor(self.changeInFloor)

    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return True


    