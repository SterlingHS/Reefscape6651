# Reset Gyro

from commands2 import Command
from subsystems.SwerveSubsystem import SwerveSubsystem

class GyroReset(Command):
    def __init__(self, swerveSub:SwerveSubsystem):
        Command.__init__(self)
        self.swerveSub = swerveSub
        self.addRequirements(swerveSub)

    def initialize(self) -> None:
        self.swerveSub.zeroHeading()
        return super().initialize()
    
    def execute(self) -> None:
        return super().execute()

    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return True