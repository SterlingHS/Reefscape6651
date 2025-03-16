# Changes the position of the Algae Remover arm

from commands2 import Command
from subsystems.SwerveSubsystem import SwerveSubsystem
from constants import DrivingModes

class DriveChangeMode(Command):
    def __init__(self, swerveSubsystem:SwerveSubsystem, driveMode):
        Command.__init__(self)
        self.swerveS = swerveSubsystem
        self.driveMode = driveMode

    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        self.swerveS.setDrivingMode(self.driveMode)

    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return True


    