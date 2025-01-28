from commands2 import Command
from subsystems.Dropper import Dropper


class DropCoral(Command):
    def __init__(self, dropper:Dropper):
        Command.__init__(self)
        self.dropper = dropper

    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        self.dropper.setMotor(0.25)

    def end(self, interrupted: bool) -> None:
        self.dropper.stopMotor()
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return super().isFinished()


    