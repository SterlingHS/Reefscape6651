from commands2 import Command
from subsystems.Dropper import Dropper
from subsystems.Elevator import Elevator

from constants import DropperConstants


class CoralIntake(Command):
    def __init__(self, dropper:Dropper, elevator:Elevator):
        Command.__init__(self)
        self.dropper = dropper
        self.elevator = elevator
        self.counter = 0

    def initialize(self) -> None:
        self.elevator.setPeriodicFloor(1)
        self.counter = 0
        return super().initialize()
    
    def execute(self) -> None:
        if self.dropper.is_no_Coral_dropper():
            self.dropper.setDropperVelocity(DropperConstants.DropIntakeSpeed)
        elif self.dropper.is_coral_top():
            if self.counter < 10:
                self.counter += 1
                self.dropper.setDropperVelocity(DropperConstants.DropIntakeSpeed)
            else:
                self.dropper.setDropperVelocity(DropperConstants.DropTransitSpeed)
        else:
            self.dropper.stopMotor()

    def end(self, interrupted: bool) -> None:
        self.dropper.stopMotor()
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return self.dropper.is_Coral_ready()


    