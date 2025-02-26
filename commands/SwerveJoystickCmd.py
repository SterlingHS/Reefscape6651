####################################################################################################
# SwerveJoystickCmd.py
# This file contains the SwerveJoystickCmd class, which is a
# command that allows the driver to control the swerve drive
# using a joystick.
####################################################################################################

from commands2 import Command

from wpimath.filter import SlewRateLimiter
from wpimath.controller import PIDController
from wpimath.kinematics import ChassisSpeeds

from constants import DriveConstants,OIConstants, DrivingModes
from subsystems.SwerveSubsystem import SwerveSubsystem

from math import pi

import wpilib
from wpilib import DriverStation


signum = lambda x : (x>0)-(x<0) # Function to get the sign of a number (Used?)

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
        self.turningPID = PIDController(0.1, 0, 0) # P, I, D to be checked
        self.turningPID.enableContinuousInput(-pi, pi) 
        self.turningPID.setTolerance(0.01) # around 0.5 degrees

    def initialize(self) -> None:
        return super().initialize()
    
    def joystick_attenuator(self, x: float, y: float, rot: float):
        # This function is used to scale the joystick inputs to make the robot easier to control
        # This done by cubing the joystick inputs
        x = .6*(x ** 3)
        y = .6*(y ** 3)
        rot = .6*(rot ** 3)
        return x, y, rot

    def execute(self) -> None:

        ################################################################################
        # Read joystick and filter and attenuate the values
        # Get the x, y, and rotation values from the joystick
        if DriveConstants.DriveEnabled == True: # Disable the drive for testing purposes (to avoid accidents)
            x = self.xSpeedFunction()
            y = self.ySpeedFunction()
            rot = self.turningSpeedFunction()
        else:
            x = 0 
            y = 0 
            rot = 0

        # Apply a deadband to the joystick
        self.xSpeed = x if abs(x) > OIConstants.kDeadband else 0.0
        self.ySpeed = y if abs(y) > OIConstants.kDeadband else 0.0
        self.turningSpeed = rot if abs(rot) > OIConstants.kDeadband else 0.0

        self.xSpeed, self.ySpeed, self.turningSpeed = self.joystick_attenuator(self.xSpeed, self.ySpeed, self.turningSpeed)

        # Saves the last 3 values of the joystick to determine if the joystick is at 0
        if len(self.x_direction_states)<3:
            self.x_direction_states.append(self.xSpeed)
        else:
            self.x_direction_states.pop(0)
            self.x_direction_states.append(self.xSpeed)

        if len(self.y_direction_states)<3:
            self.y_direction_states.append(self.ySpeed)
        else:
            self.y_direction_states.pop(0)
            self.y_direction_states.append(self.ySpeed)

        # If the joystick is at 0, reset the limiter and set the speed to 0
        if self.x_direction_states[0]==0 and self.x_direction_states[1]==0 and self.x_direction_states[2]==0:
            self.XLimiter.reset(0)
            self.xSpeed=0
        else: # TO BE CHECKED!!
            self.xSpeed = self.XLimiter.calculate(self.xSpeed)*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond

        if self.y_direction_states[0]==0 and self.y_direction_states[1]==0 and self.y_direction_states[2]==0:
            self.YLimiter.reset(0)
            self.ySpeed=0
        else: # TO BE CHECKED!!
            self.ySpeed = self.YLimiter.calculate(self.ySpeed)*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond

        # TO BE CHECKED!!
        self.turningSpeed *= DriveConstants.kTeleDriveMaxAngularRadiansPerSecond

        #################################################################################
        # Select correct driving mode and apply the correct scaling

        self.chassisSpeeds = ChassisSpeeds()

        # Mode 0: Field Oriented
        if self.swerveSub.getDrivingMode() == DrivingModes.FieldOriented:

            self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    self.xSpeed, self.ySpeed, self.turningSpeed, self.swerveSub.getRotation2d())
            
        # Mode 1: Reef Oriented
        elif self.swerveSub.getDrivingMode() == DrivingModes.ReefOriented:
            self.turningPID.setSetpoint(self.swerveSub.angleToReef())

            # Gets the current angle of the robot
            robotDirection = self.swerveSub.getRotation2d().radians()
            # Converts it  to an angle between pi and -pi
            while robotDirection > pi:
                robotDirection -= 2*pi
            while robotDirection < -pi:
                robotDirection += 2*pi
            self.turningSpeed = self.turningPID.calculate(robotDirection)
            # filter turningspeed so it is between 1 and -1
            self.turningSpeed = max(min(self.turningSpeed, 1), -1)
            self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    self.xSpeed, self.ySpeed, self.turningSpeed, self.swerveSub.getRotation2d())
            
        # Mode 2: Processor Oriented
        elif self.swerveSub.getDrivingMode() == DrivingModes.ProcessorOriented:
            if DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
                angle_to_processor = 90*pi/180 # CHECK THIS ANGLE!!!
            else:
                angle_to_processor = 270*pi/180 # CHECK THIS ANGLE!!!
            self.turningPID.setSetpoint(angle_to_processor)
            self.turningSpeed = self.turningPID.calculate(self.swerveSub.getRotation2d().radians())
            # filter turningspeed so it is between 1 and -1
            self.turningSpeed = max(min(self.turningSpeed, 1), -1)
            self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    self.xSpeed, self.ySpeed, self.turningSpeed, self.swerveSub.getRotation2d())
        
        # Mode 3: Coral Station Oriented
        elif self.swerveSub.getDrivingMode() == DrivingModes.CoralStationOriented:
            if DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
                # Checks which coral is the closest
                if self.swerveSub.getPose().Y() < 4.020: # 4.020m is the y coordinate of the middle of the field - CHECK THIS VALUE!!
                    angle_to_coral_station = 126*pi/180  # CHECK THIS ANGLE!!! Could need to add/subtract 360 or 2pi
                else:
                    angle_to_coral_station = 234*pi/180  # CHECK THIS ANGLE!!!
            else:
                # Checks which coral is the closest
                if self.swerveSub.getPose().Y() < 4.020:
                    angle_to_coral_station = 54*pi/180
                else:
                    angle_to_coral_station = 306*pi/180
            
            self.turningPID.setSetpoint(angle_to_coral_station)
            self.turningSpeed = self.turningPID.calculate(self.swerveSub.getRotation2d().radians())
            # filter turningspeed so it is between 1 and -1
            self.turningSpeed = max(min(self.turningSpeed, 1), -1)
            self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    self.xSpeed, self.ySpeed, self.turningSpeed, self.swerveSub.getRotation2d())

        moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(self.chassisSpeeds)
        self.swerveSub.setModuleStates(moduleStates)

        return super().execute()

    def end(self, interrupted: bool) -> None:
        self.swerveSub.stopModules()
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return super().isFinished()

    
