from commands2 import Command
from subsystems.Dropper import Dropper


class DropCoralSpeed(Command):
    def __init__(self, dropper:Dropper, speed:float):
        Command.__init__(self)
        self.dropper = dropper
        self.speed = speed

    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        self.dropper.setDropperVelocity(self.speed)

    def end(self, interrupted: bool) -> None:
        self.dropper.setDropperVelocity(0)
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return super().isFinished()


    