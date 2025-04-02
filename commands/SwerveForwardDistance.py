####################################################################################################
# SwerveAutoCmd.py
# This file contains the SwerveAutoCmd class, which is a
# command that to move the robot to a specific pose.
####################################################################################################

from commands2 import Command

import wpilib

from wpimath.controller import PIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d, Pose2d

from subsystems.SwerveSubsystem import SwerveSubsystem

from constants import DriveConstants

from math import pi

class SwerveForwardDistance(Command):
    def __init__(self, swerveSub:SwerveSubsystem, GoalDistance):
        Command.__init__(self)

        self.swerveSub = swerveSub
        self.GoalDistance = GoalDistance # This can be used to pass in a Pose2d object instead of separate values

        self.addRequirements(swerveSub)

    def initialize(self) -> None:
        self.encoderStart = self.swerveSub.readForwardEncoders()
        return super().initialize()

    def execute(self) -> None:        
        self.encoderPosition = self.swerveSub.readForwardEncoders()
        self.swerveSub.moveStraightZero(.3)
        return super().execute()

    def end(self, interrupted: bool) -> None:
        self.swerveSub.stopModules()
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        # check if we have arrived to the Goal
        self.encoderPosition = self.swerveSub.readForwardEncoders()
        averageStart = self.encoderStart[0]
        averagePosition = self.encoderPosition[0]
        condition = abs(averagePosition-averageStart)>self.GoalDistance
        wpilib.SmartDashboard.putNumber("AveragePosition", averagePosition)
        wpilib.SmartDashboard.putNumber("AverageStart", averageStart)

        return condition
    
