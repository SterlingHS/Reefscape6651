####################################################################################################
# SwerveJoystickCmd.py
# This file contains the SwerveJoystickCmd class, which is a
# command that allows the driver to control the swerve drive
# using a joystick.
####################################################################################################

from subsystems.SwerveSubsystem import SwerveSubsystem
from commands2 import Command
from constants import OIConstants
import math
import wpilib


class TunePIDTurning(Command):
    def __init__(self, swerveSub:SwerveSubsystem, xSpeedFunc, ySpeedFunc, turningSpeedFunc):
        Command.__init__(self)

        self.swerveSubsystem = swerveSub
        self.xSpeedFunction = xSpeedFunc
        self.ySpeedFunction = ySpeedFunc
        self.turningSpeedFunction = turningSpeedFunc

        self.addRequirements(swerveSub)

    def initialize(self) -> None:
        return super().initialize()

    def execute(self) -> None:
        # Get the x, y, and rotation values from the joystick
        x = self.xSpeedFunction()
        y = self.ySpeedFunction()
        rot = self.turningSpeedFunction() # Not used

        # Apply a deadband to the joystick
        self.xSpeed = x if abs(x) > OIConstants.kDeadband else 0.0
        self.ySpeed = y if abs(y) > OIConstants.kDeadband else 0.0

        angle = math.atan2(self.ySpeed, self.xSpeed)
        # display angle to shuffleboard
        wpilib.SmartDashboard.putNumber("Angle Joystick", angle)

        self.swerveSubsystem.turningPIDtuneUP(angle)

        return super().execute()

    def end(self, interrupted: bool) -> None:
        self.swerveSubsystem.stopModules()
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return super().isFinished()

    
