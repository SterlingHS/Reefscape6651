####################################################################################################
# SwerveAutoCmd.py
# This file contains the SwerveAutoCmd class, which is a
# command that to move the robot to a specific pose.
####################################################################################################

from commands2 import Command

from wpimath.controller import PIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d, Pose2d

from subsystems.SwerveSubsystem import SwerveSubsystem

from constants import DriveConstants

from math import pi

class SwerveAutoCmd(Command):
    def __init__(self, swerveSub:SwerveSubsystem, GoalPose:Pose2d):
        Command.__init__(self)

        self.swerveSub = swerveSub
        self.GoalPose = GoalPose # This can be used to pass in a Pose2d object instead of separate values
        self.addRequirements(swerveSub)

        # PID to control turning speed of the robot
        self.turningPID = PIDController(25, 0, 0) # P, I, D to be checked
        self.turningPID.enableContinuousInput(0, 360) 
        self.turningPID.setTolerance(1) # around 0.5 degrees

        # PID to control the x and y position of the robot
        self.xPID = PIDController(25, 0, 0) # P, I, D to be checked
        self.xPID.setTolerance(0.01) # 1 cm.
        self.yPID = PIDController(25, 0, 0) # P, I, D to be checked
        self.yPID.setTolerance(0.01) # 1 cm.

        # Trapezoid profile to control x, y and stearing
        self.turningTrap = TrapezoidProfile(TrapezoidProfile.Constraints(DriveConstants.kMaxTurnRateDegPerS, DriveConstants.kMaxTurnAccelerationDegPerSSquared)) # Max velocity and acceleration to be checked
        self.xTrap = TrapezoidProfile(TrapezoidProfile.Constraints(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond, DriveConstants.kTeleDriveMaxAccelerationUnitsPerSeconds)) # Max velocity and acceleration to be checked
        self.yTrap = TrapezoidProfile(TrapezoidProfile.Constraints(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond, DriveConstants.kTeleDriveMaxAccelerationUnitsPerSeconds)) # Max velocity and acceleration to be checked

        self.chassisSpeeds = ChassisSpeeds()

        # Reads the position of the robot
        pose = self.swerveSub.getPoseEstimator()
        self.x = pose.X()
        self.y = pose.Y()
        self.theta = pose.rotation().degrees() % 360 # Normalize the angle to be between 0 and 360

    def initialize(self) -> None:
         # Get the current setpoint for the robot
        self.turningGoal = TrapezoidProfile.State(position=self.GoalPose.rotation().degrees() , velocity=0)
        self.turningSetpoint = TrapezoidProfile.State(position=self.theta, velocity=0)
        self.xGoal = TrapezoidProfile.State(position=self.GoalPose.X(), velocity=0)
        self.xSetpoint = TrapezoidProfile.State(position=self.x, velocity=self.swerveSub.getRealVelocityX())   
        self.yGoal = TrapezoidProfile.State(position=self.GoalPose.Y(), velocity=0)
        self.ySetpoint = TrapezoidProfile.State(position=self.y, velocity=self.swerveSub.getRealVelocityY())
        return super().initialize()

    def execute(self) -> None:
        
        # Reads the position of the robot
        pose = self.swerveSub.getPoseEstimator()
        self.x = pose.X()
        self.y = pose.Y()
        self.theta = pose.rotation().degrees() % 360 # Normalize the angle to be between 0 and 360

        # PID to control the stearing the robot should be facing
        # self.turningSetpoint = self.turningTrap.calculate(.02, self.turningSetpoint, self.turningGoal)
        # self.turningPID.setSetpoint(self.turningSetpoint.position)
        # turningSpeed = self.turningPID.calculate(self.theta)
        
        # PID to control the x position of the robot
        self.xSetpoint = self.xTrap.calculate(.02, self.xSetpoint, self.xGoal)
        self.xPID.setSetpoint(self.xSetpoint.position)
        xSpeed = self.xPID.calculate(self.x)
        print(f"Position = {self.xSetpoint.position} -- xSpeed = {xSpeed}")

        # PID to control the y position of the robot
        # self.ySetpoint = self.yTrap.calculate(.02, self.ySetpoint, self.yGoal)
        # self.yPID.setSetpoint(self.ySetpoint.position)
        # ySpeed = self.yPID.calculate(self.y)

        self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, 
                0, #ySpeed, 
                0, #turningSpeed, 
                Rotation2d(self.theta*pi/180))
    
        # Updates the swerve drive modules
        moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(self.chassisSpeeds)
        self.swerveSub.setModuleStates(moduleStates)

        return super().execute()

    def end(self, interrupted: bool) -> None:
        self.swerveSub.stopModules()
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        # check if we have arrived to the Goal
        errorx = abs(self.GoalPose.X() - self.x)
        errory = 0 #abs(self.GoalPose.Y() - self.y)
        errorTheta = 0 #abs(self.GoalPose.rotation().degrees() - self.theta)
        condition = errorx < .1 and errory < .1 and errorTheta < .1 
        # print(f"condition - {condition}")
        return condition
    
