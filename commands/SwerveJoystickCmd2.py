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

class SwerveJoystickCmd2(Command):
    def __init__(self, swerveSub:SwerveSubsystem, xSpeedFunc, ySpeedFunc, turningSpeedFuncx, turningSpeedFuncy):
        Command.__init__(self)

        self.swerveSub = swerveSub
        self.xSpeedFunction = xSpeedFunc
        self.ySpeedFunction = ySpeedFunc
        self.turningSpeedFunctionx = turningSpeedFuncx
        self.turningSpeedFunctiony = turningSpeedFuncy

        self.addRequirements(swerveSub)

        # PID to control turning speed of the robot
        
        self.turningPID = PIDController(0.01, 0, 0) # P, I, D to be checked
        self.turningPID.enableContinuousInput(0, 360) 
        self.turningPID.setTolerance(1) # around 0.5 degrees

        # Trapezoid profile to control stearing
        self.turningTrap = TrapezoidProfile(TrapezoidProfile.Constraints(DriveConstants.kMaxTurnRateDegPerS, DriveConstants.kMaxTurnAccelerationDegPerSSquared)) # Max velocity and acceleration to be checked
        self.turningGoal = TrapezoidProfile.State(position=0, velocity=0)
        self.turningSetpoint = TrapezoidProfile.State(position=0, velocity=0)

        self.lastAprilTag = 0
        self.lastAngle = 0

        wpilib.SmartDashboard.putNumber("P Steering", 0.01)

        # Offset depending on Alliance color
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            self.angleOffset = 180
        else:
            self.angleOffset = 0

    def initialize(self) -> None:
        return super().initialize()
    
    def joystick_attenuator(self, x: float, y: float, rotx: float, roty: float) -> tuple:
        # This function is used to scale the joystick inputs to make the robot easier to control
        # This done by cubing the joystick inputs
        x = pow(x, 3.)
        y = pow(y, 3.)
        rotx = pow(rotx, 3.)
        roty = pow(roty, 3.)
        return x, y, rotx, roty
    
    def angle_0_360(self, angle: float) -> float:
        # This function is used to convert an angle to a value between -360 and 360
        while angle < 0:
            angle += 360
        while angle > 360:
            angle -= 360
        return angle

    def execute(self) -> None:
        ################################################################################
        # Read joystick and filter and attenuate the values
        # Get the x, y, and rotation values from the joystick
        if DriveConstants.DriveEnabled == True: # Disable the drive for testing purposes (to avoid accidents)
            x = self.xSpeedFunction()
            y = self.ySpeedFunction()
            rotx = -self.turningSpeedFunctionx()
            roty = -self.turningSpeedFunctiony()
        else:
            x = 0 
            y = 0 
            rotx = 0
            roty = 0

        # Apply a deadband to the joystick
        self.xSpeed = x if abs(x) > OIConstants.kDeadband else 0.0
        self.ySpeed = y if abs(y) > OIConstants.kDeadband else 0.0
        self.turningSpeedx = rotx if abs(rotx) > OIConstants.kDeadbandRot else 0.0
        self.turningSpeedy = roty if abs(roty) > OIConstants.kDeadbandRot else 0.0

        # Calculates the andgle of the robot depending on the joystick
        self.xSpeed, self.ySpeed, self.turningSpeedx, self.turningSpeedy = self.joystick_attenuator(self.xSpeed, self.ySpeed, self.turningSpeedx, self.turningSpeedy)
        if self.turningSpeedx != 0 or self.turningSpeedy != 0:
            angleDirectionChassis =  -atan2(-self.turningSpeedx, self.turningSpeedy)*180/pi + self.angleOffset
            angleDirectionChassis = self.angle_0_360(angleDirectionChassis)
            self.lastAngle = angleDirectionChassis
        else:
            angleDirectionChassis = self.lastAngle
        
        # Calculates the andgle of the robot depending on the joystick
        if self.xSpeed==self.ySpeed==0:
            joystickAngle = 0
        else:
            joystickAngle = atan2(-self.xSpeed,self.ySpeed)*180/pi - 90
            joystickAngle = self.angle_0_360(joystickAngle)

        wpilib.SmartDashboard.putNumber("Angle Joystick(x,y)",joystickAngle)
        wpilib.SmartDashboard.putNumber("Angle Chassis(x,y)",angleDirectionChassis)

        wpilib.SmartDashboard.putNumber("Joystick x",self.xSpeed)
        wpilib.SmartDashboard.putNumber("Joystick y",self.ySpeed)
        wpilib.SmartDashboard.putNumber("Joystick rotx",self.turningSpeedx)
        wpilib.SmartDashboard.putNumber("Joystick roty",self.turningSpeedy)

        #################################################################################
        # Select correct driving mode and apply the correct scaling

        self.chassisSpeeds = ChassisSpeeds()

        # Mode 0: Field Oriented
        if self.swerveSub.getDrivingMode() == DrivingModes.FieldOriented:
            
            P = wpilib.SmartDashboard.getNumber("P Steering", 0.01)
            self.turningPID.setP(P)

            # self.turningGoal = TrapezoidProfile.State(position=angleDirectionChassis, velocity=0)
            # self.turningSetpoint = self.turningTrap.calculate(.02, self.turningSetpoint, self.turningGoal)
            # self.turningPID.setSetpoint(self.turningSetpoint.position)

            error = abs(angleDirectionChassis-self.swerveSub.getHeading())
            wpilib.SmartDashboard.putNumber("Error", error)
            self.turningPID.setSetpoint(angleDirectionChassis)
            speedTheta = self.turningPID.calculate(self.swerveSub.getHeading())

            wpilib.SmartDashboard.putNumber("Theta Speed", speedTheta)
            wpilib.SmartDashboard.putNumber("Trap Setpoint", self.turningSetpoint.position)
            self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    self.xSpeed, self.ySpeed, speedTheta, Rotation2d(self.swerveSub.getHeading()*pi/180))
            
        # Mode 1: Reef Oriented
        elif self.swerveSub.getDrivingMode() == DrivingModes.ReefOriented:
            angle = self.swerveSub.angleToReef()
            angle = self.angle_0_360(angle)
            self.turningPID.setSetpoint(angle)

            # Gets the current angle of the robot
            robotDirection = self.swerveSub.getRotation2d().degrees()
            # Converts it  to an angle between -pi and pi
            robotDirection = self.angle_0_360(robotDirection)

            self.turningGoal = TrapezoidProfile.State(position=angle, velocity=0)
            self.turningSetpoint = self.turningTrap.calculate(.02, self.turningSetpoint, self.turningGoal)
            self.turningPID.setSetpoint(self.turningSetpoint.position)*DriveConstants.kTeleDriveMaxAngularRadiansPerSecond
            self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    self.xSpeed, self.ySpeed, self.turningSpeed, self.swerveSub.getRotation2d())
            
        # Mode 2: Processor Oriented
        elif self.swerveSub.getDrivingMode() == DrivingModes.ProcessorOriented:
            if DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
                angle_to_processor = 90*pi/180 # CHECK THIS ANGLE!!!
            else:
                angle_to_processor = 270*pi/180 # CHECK THIS ANGLE!!!
            self.turningGoal = TrapezoidProfile.State(position=angle_to_processor, velocity=0)
            self.turningSetpoint = self.turningTrap.calculate(.02, self.turningSetpoint, self.turningGoal)
            self.turningPID.setSetpoint(self.turningSetpoint.position)
            self.turningSpeed = self.turningPID.calculate(self.swerveSub.getRotation2d().radians())*DriveConstants.kTeleDriveMaxAngularRadiansPerSecond
            self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    self.xSpeed, self.ySpeed, self.turningSpeed, self.swerveSub.getRotation2d())
        
        # Mode 3: Coral Station Oriented
        elif self.swerveSub.getDrivingMode() == DrivingModes.CoralStationOriented:
            if DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
                # Checks which coral is the closest
                if self.swerveSub.getPose().Y() < 4.020: # 4.020m is the y coordinate of the middle of the field - CHECK THIS VALUE!!
                    angle_to_coral_station = 126*pi/180         # CHECK THIS ANGLE!!! Could need to add/subtract 360 or 2pi
                else:
                    angle_to_coral_station = 234*pi/180         # CHECK THIS ANGLE!!!
            else:
                # Checks which coral is the closest
                if self.swerveSub.getPose().Y() < 4.020:
                    angle_to_coral_station = 54*pi/180
                else:
                    angle_to_coral_station = 306*pi/180
            
            self.turningGoal = TrapezoidProfile.State(position=angle_to_coral_station, velocity=0)
            self.turningSetpoint = self.turningTrap.calculate(.02, self.turningSetpoint, self.turningGoal)
            self.turningPID.setSetpoint(self.turningSetpoint.position)
            self.turningSpeed = self.turningPID.calculate(self.swerveSub.getRotation2d().radians())*DriveConstants.kTeleDriveMaxAngularRadiansPerSecond
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
            print(f"Angle Goal - {angle}   ---   Angle Measurement - {self.swerveSub.getRotation2d().radians()}")
            self.turningSpeed = self.turningPID.calculate(self.swerveSub.getRotation2d().degrees())*DriveConstants.kTeleDriveMaxAngularRadiansPerSecond
            self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    self.xSpeed, self.ySpeed, self.turningSpeed, self.swerveSub.getRotation2d())
        
        # Updates the swerve drive modules
        wpilib.SmartDashboard.putNumber("Vx",self.xSpeed)
        wpilib.SmartDashboard.putNumber("Vy",self.ySpeed)
        wpilib.SmartDashboard.putNumber("VTheta",speedTheta)

        moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(self.chassisSpeeds)
        self.swerveSub.setModuleStates(moduleStates)

        return super().execute()

    def end(self, interrupted: bool) -> None:
        self.swerveSub.stopModules()
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return super().isFinished()

    
