# Command that sends elevator to a specific floor

from commands2 import Command
from subsystems.SwerveSubsystem import SwerveSubsystem

class SwerveToggleTurbo(Command):
    def __init__(self, swerveSubsystem:SwerveSubsystem):
        Command.__init__(self)
        self.swerve = swerveSubsystem

    def initialize(self) -> None:
        # Toggle the driving mode between normal and turbo
        self.swerve.toggleTurboMode()
        return super().initialize()
    
    def execute(self) -> None:
        pass

    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return True


    