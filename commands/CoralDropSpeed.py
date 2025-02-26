from commands2 import Command
from subsystems.Dropper import Dropper

class CoralDropSpeed(Command):
    def __init__(self, dropper:Dropper, speed:float):
        Command.__init__(self)
        self.dropper = dropper
        self.speed = speed

    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        self.dropper.setDropperVelocityMax(self.speed)

    def end(self, interrupted: bool) -> None:
        self.dropper.setDropperVelocityMax(0)
        # self.dropper.stopMotor()
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return super().isFinished()