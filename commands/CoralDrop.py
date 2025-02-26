from commands2 import Command
from subsystems.Dropper import Dropper


class CoralDrop(Command):
    def __init__(self, dropper:Dropper):
        Command.__init__(self)
        self.dropper = dropper

    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        self.dropper.setMotor(0.5)

    def end(self, interrupted: bool) -> None:
        self.dropper.stopMotor()
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return self.dropper.is_Coral_dropped()


    