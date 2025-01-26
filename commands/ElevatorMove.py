from commands2 import Command
from subsystems.Elevator import Elevator
from constants import ElevatorConstants


class ElevatorMove(Command):
    def __init__(self, elevator:Elevator, direction:int):
        Command.__init__(self)
        self.elevator = elevator
        self.direction = direction # 1 for up, -1 for down

    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        position = self.elevator.readEncoder()
        if position <= ElevatorConstants.Min and self.direction == -1:
            self.elevator.stopMotor()
        elif position >= ElevatorConstants.Max and self.direction == 1:
            self.elevator.stopMotor()
        else:
            self.elevator.setMotor(self.direction*0.2)

    def end(self, interrupted: bool) -> None:
        self.elevator.stopMotor()
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return super().isFinished()


    