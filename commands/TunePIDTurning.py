####################################################################################################
# SwerveJoystickCmd.py
# This file contains the SwerveJoystickCmd class, which is a
# command that allows the driver to control the swerve drive
# using a joystick.
####################################################################################################

from commands2 import Command
from wpimath.filter import SlewRateLimiter
from constants import DriveConstants,OIConstants
from wpimath.kinematics import ChassisSpeeds
from subsystems import SwerveSubsystem
import math
import unit_conversions as conv

signum = lambda x : (x>0)-(x<0) # Function to get the sign of a number (Used?)

class TunePIDTurning(Command):
    def __init__(self, swerveSub, xSpeedFunc, ySpeedFunc, turningSpeedFunc):
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
        self.swerveSubsystem.tuneturningPIDtuneUP(angle)

        return super().execute()

    def end(self, interrupted: bool) -> None:
        self.swerveSubsystem.stopModules()
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return super().isFinished()

    
