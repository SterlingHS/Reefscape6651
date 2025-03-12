from commands2 import Command
from subsystems.Dropper import Dropper
from constants import DropperConstants


class CoralDrop(Command):
    def __init__(self, dropper:Dropper):
        Command.__init__(self)
        self.dropper = dropper

    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        self.dropper.setDropperVelocity(DropperConstants.DropSpeed)

    def end(self, interrupted: bool) -> None:
        self.dropper.stopMotor()
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return self.dropper.is_no_Coral_dropper()


    