####################################################################################################
# SwerveJoystickCmd.py
# This file contains the SwerveJoystickCmd class, which is a
# command that allows the driver to control the swerve drive
# using a joystick.
####################################################################################################

from commands2 import Command

from wpimath.filter import SlewRateLimiter
from wpimath.controller import PIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d

from constants import DriveConstants,OIConstants, DrivingModes
from subsystems.SwerveSubsystem import SwerveSubsystem

from math import pi, atan2

import wpilib
from wpilib import DriverStation


signum = lambda x : (x>0)-(x<0) # Function to get the sign of a number

class SwerveJoystickCmd(Command):
    def __init__(self, swerveSub:SwerveSubsystem, xSpeedFunc, ySpeedFunc, turningSpeedFunc):
        Command.__init__(self)

        self.swerveSub = swerveSub
        self.xSpeedFunction = xSpeedFunc
        self.ySpeedFunction = ySpeedFunc
        self.turningSpeedFunction = turningSpeedFunc

        # the limiters are giving us lag. They don't allow us to slow down quick enough. When we dampen the modules this lag slows down. We need to allow 0 to bypass all of this and stop the modules
        self.XLimiter = SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSeconds)
        self.YLimiter = SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSeconds)

        self.x_direction_states = [0,0,0]
        self.y_direction_states = [0,0,0]
        self.addRequirements(swerveSub)

        # PID to control turning speed of the robot
        self.turningPID = PIDController(0.5, 0, 0) # P, I, D to be checked
        self.turningPID.enableContinuousInput(-180, 180) 
        self.turningPID.setTolerance(1) # around 0.5 degrees

        # Trapezoid profile to control stearing
        self.turningTrap = TrapezoidProfile(TrapezoidProfile.Constraints(100, 100)) # Max velocity and acceleration to be checked
        self.turningGoal = TrapezoidProfile.State(position=0, velocity=0)
        self.turningSetpoint = TrapezoidProfile.State(position=0, velocity=0)

        self.lastAprilTag = 0

    def angle_n180_180(self, angle: float) -> float:
        # This function is used to convert an angle to a value between -pi and pi
        while angle < -180:
            angle += 360
        while angle > pi:
            angle -= 180
        return angle

    def initialize(self) -> None:
        return super().initialize()
    
    def joystick_attenuator(self, x: float, y: float, rot: float):
        # This function is used to scale the joystick inputs to make the robot easier to control
        # This done by cubing the joystick inputs
        x = pow(x, 3.)
        y = pow(y, 3.)
        rot = pow(rot, 3.)
        return x, y, rot

    def execute(self) -> None:
        if DriveConstants.DriveEnabled == False:
            return 0
        ################################################################################
        # Read joystick and filter and attenuate the values
        # Get the x, y, and rotation values from the joystick
        if DriveConstants.DriveEnabled == True: # Disable the drive for testing purposes (to avoid accidents)
            x = self.xSpeedFunction()
            y = self.ySpeedFunction()
            rot = -self.turningSpeedFunction()
        else:
            x = 0 
            y = 0 
            rot = 0

        # Apply a deadband to the joystick
        self.xSpeed = x if abs(x) > OIConstants.kDeadband else 0.0
        self.ySpeed = y if abs(y) > OIConstants.kDeadband else 0.0
        self.turningSpeed = rot if abs(rot) > OIConstants.kDeadband else 0.0

        self.xSpeed, self.ySpeed, self.turningSpeed = self.joystick_attenuator(self.xSpeed, self.ySpeed, self.turningSpeed)

        joystickAngle = atan2(-self.xSpeed,self.ySpeed)*180/pi - 90
        joystickAngle = self.angle_n180_180(joystickAngle)
        wpilib.SmartDashboard.putNumber("Angle Joystick",joystickAngle)

        wpilib.SmartDashboard.putNumber("Joystick x",self.xSpeed)
        wpilib.SmartDashboard.putNumber("Joystick y",self.ySpeed)
        wpilib.SmartDashboard.putNumber("Joystick rot",self.turningSpeed)

        self.xSpeed = self.XLimiter.calculate(self.xSpeed*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond)
        self.ySpeed = self.YLimiter.calculate(self.ySpeed*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond)
        self.turningSpeed *= DriveConstants.kTeleDriveMaxAngularRadiansPerSecond

        #################################################################################
        # Select correct driving mode and apply the correct scaling

        self.chassisSpeeds = ChassisSpeeds()

        # Mode 0: Field Oriented
        if self.swerveSub.getDrivingMode() == DrivingModes.FieldOriented:

            self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    self.xSpeed, self.ySpeed, self.turningSpeed, Rotation2d(self.swerveSub.getHeading()*pi/180))
            
        # Mode 1: Reef Oriented
        elif self.swerveSub.getDrivingMode() == DrivingModes.ReefOriented:
            angle = self.swerveSub.angleToReef()*180/pi
            while angle < 0:
                angle += 360
            while angle > 360:
                angle -= 360
            self.turningPID.setSetpoint(angle)

            # Gets the current angle of the robot
            robotDirection = self.swerveSub.getRotation2d().degrees()
            # Converts it  to an angle between pi and -pi
            while robotDirection > 360:
                robotDirection -= 360
            while robotDirection < 0:
                robotDirection += 360
            self.turningGoal = TrapezoidProfile.State(position=robotDirection, velocity=0)
            self.turningSetpoint = self.turningTrap.calculate(.02, self.turningSetpoint, self.turningGoal)
            self.turningPID.setSetpoint(self.turningSetpoint.position)*DriveConstants.kTeleDriveMaxAngularRadiansPerSecond
            self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    self.xSpeed, self.ySpeed, self.turningSpeed, self.swerveSub.getRotation2d())
            
        # Mode 2: Processor Oriented
        elif self.swerveSub.getDrivingMode() == DrivingModes.ProcessorOriented:
            if DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
                angle_to_processor = 90 # CHECK THIS ANGLE!!!
            else:
                angle_to_processor = 270 # CHECK THIS ANGLE!!!
            self.turningGoal = TrapezoidProfile.State(position=angle_to_processor, velocity=0)
            self.turningSetpoint = self.turningTrap.calculate(.02, self.turningSetpoint, self.turningGoal)
            self.turningPID.setSetpoint(self.turningSetpoint.position)
            self.turningSpeed = self.turningPID.calculate(self.swerveSub.getRotation2d().degrees())*DriveConstants.kTeleDriveMaxAngularRadiansPerSecond
            self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    self.xSpeed, self.ySpeed, self.turningSpeed, self.swerveSub.getRotation2d())
        
        # Mode 3: Coral Station Oriented
        elif self.swerveSub.getDrivingMode() == DrivingModes.CoralStationOriented:
            if DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
                # Checks which coral is the closest
                if self.swerveSub.getPose().Y() < 4.020: # 4.020m is the y coordinate of the middle of the field - CHECK THIS VALUE!!
                    angle_to_coral_station = 126         # CHECK THIS ANGLE!!! Could need to add/subtract 360 or 2pi
                else:
                    angle_to_coral_station = 234         # CHECK THIS ANGLE!!!
            else:
                # Checks which coral is the closest
                if self.swerveSub.getPose().Y() < 4.020:
                    angle_to_coral_station = 54
                else:
                    angle_to_coral_station = 306
            
            self.turningGoal = TrapezoidProfile.State(position=angle_to_coral_station, velocity=0)
            self.turningSetpoint = self.turningTrap.calculate(.02, self.turningSetpoint, self.turningGoal)
            self.turningPID.setSetpoint(self.turningSetpoint.position)
            self.turningSpeed = self.turningPID.calculate(self.swerveSub.getRotation2d().degrees())*DriveConstants.kTeleDriveMaxAngularRadiansPerSecond
            self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    self.xSpeed, self.ySpeed, self.turningSpeed, self.swerveSub.getRotation2d())
            
        # Mode 4: Reef AprilTag Oriented
        elif self.swerveSub.getDrivingMode() == DrivingModes.ReefAprilTageOriented:
            # Read closest april tag number
            aprilTagNumber = self.swerveSub.getClosestAprilTag()
        
            if aprilTagNumber == 0:
                #self.turningPID.setSetpoint(self.swerveSub.angleToReef())
                aprilTagNumber = self.lastAprilTag
            else:
                self.lastAprilTag = aprilTagNumber
            angle = self.swerveSub.angleToAprilTag(aprilTagNumber)
            self.turningGoal = TrapezoidProfile.State(position=angle, velocity=0)
            self.turningSetpoint = self.turningTrap.calculate(.02, self.turningSetpoint, self.turningGoal)
            self.turningPID.setSetpoint(self.turningSetpoint.position)
            print(f"Angle Goal - {angle}   ---   Angle Measurement - {self.swerveSub.getRotation2d().degrees()}")
            self.turningSpeed = self.turningPID.calculate(self.swerveSub.getRotation2d().degrees())*DriveConstants.kTeleDriveMaxAngularRadiansPerSecond
            self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    self.xSpeed, self.ySpeed, self.turningSpeed, self.swerveSub.getRotation2d())
        
        # Updates the swerve drive modules
        moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(self.chassisSpeeds)
        self.swerveSub.setModuleStates(moduleStates)

        return super().execute()

    def end(self, interrupted: bool) -> None:
        self.swerveSub.stopModules()
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return super().isFinished()

    
