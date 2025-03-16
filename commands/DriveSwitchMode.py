# Changes the position of the Algae Remover arm

from commands2 import Command
from subsystems.SwerveSubsystem import SwerveSubsystem
from constants import DrivingModes

class DriveSwitchMode(Command):
    def __init__(self, swerveSubsystem:SwerveSubsystem):
        Command.__init__(self)
        self.swerveS = swerveSubsystem

    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        if self.swerveS.getDrivingMode() == DrivingModes.FieldOriented:
            self.swerveS.setDrivingMode(DrivingModes.ReefAprilTageOriented)
        else:
            self.swerveS.setDrivingMode(DrivingModes.FieldOriented)

    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return True


    