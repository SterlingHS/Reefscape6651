####################################################################################################
# SwerveAutoCmd.py
# This file contains the SwerveAutoCmd class, which is a
# command that to move the robot to a specific pose.
####################################################################################################

from commands2 import Command

from wpimath.controller import PIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.kinematics import ChassisSpeeds

from subsystems.SwerveSubsystem import SwerveSubsystem

from constants import DriveConstants

class SwerveJoystickCmd(Command):
    def __init__(self, swerveSub:SwerveSubsystem, GoalX, GoalY, GoalTheta):
        Command.__init__(self)

        self.swerveSub = swerveSub
        self.GoalX = GoalX
        self.GoalY = GoalY
        self.GoalTheta = GoalTheta
        self.addRequirements(swerveSub)

        # PID to control turning speed of the robot
        self.turningPID = PIDController(0.5, 0, 0) # P, I, D to be checked
        self.turningPID.enableContinuousInput(0, 360) 
        self.turningPID.setTolerance(0.01) # around 0.5 degrees

        # PID to control the x and y position of the robot
        self.xPID = PIDController(0.5, 0, 0) # P, I, D to be checked
        self.xPID.setTolerance(0.01) # 1 cm.
        self.yPID = PIDController(0.5, 0, 0) # P, I, D to be checked
        self.yPID.setTolerance(0.01) # 1 cm.

        # Trapezoid profile to control x, y and stearing
        self.turningTrap = TrapezoidProfile(TrapezoidProfile.Constraints(DriveConstants.kMaxTurnRateDegPerS, DriveConstants.kMaxTurnAccelerationDegPerSSquared)) # Max velocity and acceleration to be checked
        self.turningGoal = TrapezoidProfile.State(position=self.GoalTheta, velocity=0)
        self.turningSetpoint = TrapezoidProfile.State(position=self.swerveSub.getRotation2d().degrees(), velocity=0)

        self.xTrap = TrapezoidProfile(TrapezoidProfile.Constraints(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond, DriveConstants.kTeleDriveMaxAccelerationUnitsPerSeconds)) # Max velocity and acceleration to be checked
        self.xGoal = TrapezoidProfile.State(position=0, velocity=0)
        self.xSetpoint = TrapezoidProfile.State(position=self.swerveSub.getPose().X(), velocity=self.swerveSub.getXVelocity())   

        self.yTrap = TrapezoidProfile(TrapezoidProfile.Constraints(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond, DriveConstants.kTeleDriveMaxAccelerationUnitsPerSeconds)) # Max velocity and acceleration to be checked
        self.yGoal = TrapezoidProfile.State(position=0, velocity=0)
        self.ySetpoint = TrapezoidProfile.State(position=self.swerveSub.getPose().Y(), velocity=0)

        self.chassisSpeeds = ChassisSpeeds()

    def initialize(self) -> None:
        return super().initialize()

    def execute(self) -> None:

        # PID to control the stearing the robot should be facing
        self.turningSetpoint = self.turningTrap.calculate(.02, self.turningSetpoint, self.turningGoal)
        self.turningPID.setSetpoint(self.turningSetpoint.position)
        turningSpeed = self.turningPID.calculate(self.swerveSub.getRotation2d().degrees())*DriveConstants.kTeleDriveMaxAngularRadiansPerSecond

        # PID to control the x position of the robot
        self.xSetpoint = self.xTrap.calculate(.02, self.xSetpoint, self.xGoal)
        self.xPID.setSetpoint(self.xSetpoint.position)
        xSpeed = self.xPID.calculate(self.swerveSub.getPose().translation().x)*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond

        # PID to control the y position of the robot
        self.ySetpoint = self.yTrap.calculate(.02, self.ySetpoint, self.yGoal)
        self.yPID.setSetpoint(self.ySetpoint.position)
        ySpeed = self.yPID.calculate(self.swerveSub.getPose().translation().y)*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond
        
        self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, self.swerveSub.getRotation2d().degrees())
    
        # Updates the swerve drive modules
        moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(self.chassisSpeeds)
        self.swerveSub.setModuleStates(moduleStates)

        return super().execute()

    def end(self, interrupted: bool) -> None:
        self.swerveSub.stopModules()
        return super().end(interrupted)

    def isFinished(self) -> bool:
        # check if we have arrived to the Goal
        if self.xPID.atSetpoint() and self.yPID.atSetpoint() and self.turningPID.atSetpoint():
            return True
        else:
            return False

    
