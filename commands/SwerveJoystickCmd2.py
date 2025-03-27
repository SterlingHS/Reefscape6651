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
    def __init__(self, swerveSub:SwerveSubsystem, xSpeedFunc, ySpeedFunc, turningSpeedFuncx, turningSpeedFuncy, moveRightFunc, moveLeftFunc):
        Command.__init__(self)

        self.swerveSub = swerveSub
        self.xSpeedFunction = xSpeedFunc
        self.ySpeedFunction = ySpeedFunc
        self.turningSpeedFunctionx = turningSpeedFuncx
        self.turningSpeedFunctiony = turningSpeedFuncy
        self.moveRightFunc = moveRightFunc
        self.moveLeftFunc = moveLeftFunc

        self.addRequirements(swerveSub)

        # PID to control turning speed of the robot
        
        self.turningPID = PIDController(0.01, 0, 0) # P, I, D to be checked
        self.turningPID.enableContinuousInput(0, 360) 
        self.turningPID.setTolerance(1) # 1 degree

        # Trapezoid profile to control stearing
        self.turningTrap = TrapezoidProfile(TrapezoidProfile.Constraints(DriveConstants.kMaxTurnRateDegPerS, DriveConstants.kMaxTurnAccelerationDegPerSSquared)) # Max velocity and acceleration to be checked
        self.turningGoal = TrapezoidProfile.State(position=0, velocity=0)
        self.turningSetpoint = TrapezoidProfile.State(position=0, velocity=0)

        self.lastAprilTag = 0
        self.memTag = []
        self.lastAngle = 0
        self.savingAngle = False
        self.cyclesAngles = 0
        self.lastMode = DrivingModes.FieldOriented

        # Offset depending on Alliance color
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            self.angleOffset = 180
        else:
            self.angleOffset = 0

        # Init slew rate limiters for theta speed
        self.thetaSpeedLimiter = SlewRateLimiter(DriveConstants.kMaxTurnRateDegPerS) # Max velocity to be checked

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
        # Check driving mode
        if self.lastMode != self.swerveSub.getDrivingMode():
            self.lastAngle = self.swerveSub.getHeading()

        ################################################################################
        # Read joystick and filter and attenuate the values
        # Get the x, y, and rotation values from the joystick
        if DriveConstants.DriveEnabled == True: # Disable the drive for testing purposes (to avoid accidents)
            x = self.xSpeedFunction()
            y = self.ySpeedFunction()
            rotx = -self.turningSpeedFunctionx()
            roty = -self.turningSpeedFunctiony()
            self.moveRight = self.moveRightFunc()
            self.moveLeft = self.moveLeftFunc()

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

        # Check if the robot needs to shift sideways
        if self.moveRight and self.xSpeed == 0 and self.ySpeed == 0:
            self.swerveSub.moveRight(DriveConstants.kModeSidewaysSpeedMetersPerSecond)
            return 0
        if self.moveLeft and self.xSpeed == 0 and self.ySpeed == 0:
            self.swerveSub.moveLeft(DriveConstants.kModeSidewaysSpeedMetersPerSecond)
            return 0

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

        #################################################################################
        # Select correct driving mode and apply the correct scaling

        self.chassisSpeeds = ChassisSpeeds()

        # Mode 0: Field Oriented
        if self.swerveSub.getDrivingMode() == DrivingModes.FieldOriented:
            if self.turningSpeedx == 0:
                if self.savingAngle == True:
                    if self.cyclesAngles < 25:
                        self.cyclesAngles += 1
                    else:
                        self.cyclesAngles = 0
                        self.savingAngle = False
                        self.lastAngle = self.swerveSub.getHeading()
                    self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        self.xSpeed, self.ySpeed, self.turningSpeedx, Rotation2d(self.swerveSub.getHeading()*pi/180)) # angle in RADIANS!
                    self.lastAngle = self.swerveSub.getHeading()
                else:
                    # Sets and calculates the setpoint for the turning PID
                    self.turningPID.setSetpoint(self.lastAngle)
                    speedTheta = self.turningPID.calculate(self.swerveSub.getHeading())
                    # Apply slew rate limiter to theta speed
                    speedTheta = self.thetaSpeedLimiter.calculate(speedTheta)
                    # Calculate the chassis speeds using field relative speeds
                    self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                            self.xSpeed, self.ySpeed, speedTheta, Rotation2d(self.swerveSub.getHeading()*pi/180)) # angle in RADIANS!
            else:
                self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        self.xSpeed, self.ySpeed, self.turningSpeedx, Rotation2d(self.swerveSub.getHeading()*pi/180)) # angle in RADIANS!
                self.lastAngle = self.swerveSub.getHeading()
                self.savingAngle = True

        # Mode 1: Reef AprilTag Oriented
        elif self.swerveSub.getDrivingMode() == DrivingModes.ReefAprilTageOriented:
            # Read closest april tag number
            aprilTagNumber = self.swerveSub.getClosestAprilTag()

            # record the last 50 april tag numbers to avoid oscillation
            self.memTag.append(aprilTagNumber)
            if len(self.memTag) > 50:
                self.memTag.pop(0)

            # if the last 50 april tag numbers are the same, use that number
            if len(set(self.memTag)) == 1:
                aprilTagNumber = self.memTag[0]
        
            if aprilTagNumber == 0:
                aprilTagNumber = self.lastAprilTag
            else:
                self.lastAprilTag = aprilTagNumber

            angle = self.swerveSub.angleToAprilTag(aprilTagNumber)
            wpilib.SmartDashboard.putNumber("LastTag", angle)
            wpilib.SmartDashboard.putNumber("AngleAprilTag", self.lastAprilTag )
            self.turningPID.setSetpoint(angle)
            self.turningSpeed = self.turningPID.calculate(self.swerveSub.getHeading())
            self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    self.xSpeed, self.ySpeed, self.turningSpeed, Rotation2d(self.swerveSub.getHeading()*pi/180))

        # Mode 2: Processor Oriented
        elif self.swerveSub.getDrivingMode() == DrivingModes.ProcessorOriented:
            if DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
                angle_to_processor = 90 # CHECK THIS ANGLE!!!
            else:
                angle_to_processor = 270 # CHECK THIS ANGLE!!!
            self.turningPID.setSetpoint(angle_to_processor)
            self.turningSpeed = self.turningPID.calculate(self.swerveSub.getHeading())
            self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    self.xSpeed, self.ySpeed, self.turningSpeed, Rotation2d(self.swerveSub.getHeading()*pi/180))
        
        # Mode 3: Coral Station Oriented
        elif self.swerveSub.getDrivingMode() == DrivingModes.CoralStationOriented:
            if DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
                # Checks which coral is the closest
                if self.swerveSub.getPoseEstimator().Y() < 4.020: # 4.020m is the y coordinate of the middle of the field - CHECK THIS VALUE!!
                    angle_to_coral_station = 126+180        # CHECK THIS ANGLE!!! Could need to add/subtract 360 or 2pi
                else:
                    angle_to_coral_station = 234-180         # CHECK THIS ANGLE!!!
            else:
                # Checks which coral is the closest
                if self.swerveSub.getPoseEstimator().Y() < 4.020:
                    angle_to_coral_station = 54+180
                else:
                    angle_to_coral_station = 306-180

            wpilib.SmartDashboard.putNumber("Angle Station", angle_to_coral_station)
            
            self.turningPID.setSetpoint(angle_to_coral_station)
            self.turningSpeed = self.turningPID.calculate(self.swerveSub.getHeading())
            self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    self.xSpeed, self.ySpeed, self.turningSpeed, Rotation2d(self.swerveSub.getHeading()*pi/180))
                    
        # Updates the swerve drive modules
        wpilib.SmartDashboard.putNumber("Vx",self.xSpeed)
        wpilib.SmartDashboard.putNumber("Vy",self.ySpeed)
        # wpilib.SmartDashboard.putNumber("VTheta",speedTheta)

        moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(self.chassisSpeeds)
        self.swerveSub.setModuleStates(moduleStates)

        return super().execute()

    def end(self, interrupted: bool) -> None:
        self.swerveSub.stopModules()

        return super().end(interrupted)

    def isFinished(self) -> bool:
        return super().isFinished()

    
