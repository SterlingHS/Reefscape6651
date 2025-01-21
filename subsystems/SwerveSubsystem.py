from subsystems.SwerveModule import SwerveModule

from math import cos, sin

import wpimath
from wpimath.geometry import Rotation2d
import wpimath.kinematics
from wpimath.kinematics import SwerveModuleState, ChassisSpeeds

import wpilib
from commands2 import Subsystem
from constants import DriveConstants, AutoConstants
#from pathplannerlib.config import HolonomicPathFollowerConfig, PIDConstants, ReplanningConfig
# from pathplannerlib import HolonomicPathFollower

import navx
import phoenix6

# Autonomous Pathplanner
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
from wpilib import DriverStation


# For NavX to calibrate
from time import sleep

class SwerveSubsystem(Subsystem):
    def __init__(self):
        Subsystem.__init__(self)

        self.frontLeft = SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveMotorReversed,
            DriveConstants.kFrontLeftTurningMotorReversed,
            DriveConstants.kFrontLeftAbsoluteEncoderPort,
            DriveConstants.kFrontLeftAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftAbsoluteEncoderReversed,
            DriveConstants.kFrontLeftForwardPIDk
        )
        self.frontRight = SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveMotorReversed,
            DriveConstants.kFrontRightTurningMotorReversed,
            DriveConstants.kFrontRightAbsoluteEncoderPort,
            DriveConstants.kFrontRightAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightAbsoluteEncoderReversed,
            DriveConstants.kFrontRightForwardPIDk
        )
        self.backLeft = SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveMotorReversed,
            DriveConstants.kBackLeftTurningMotorReversed,
            DriveConstants.kBackLeftAbsoluteEncoderPort,
            DriveConstants.kBackLeftAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftAbsoluteEncoderReversed,
            DriveConstants.kBackLeftForwardPIDk
        )
        self.backRight = SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveMotorReversed,
            DriveConstants.kBackRightTurningMotorReversed,
            DriveConstants.kBackRightAbsoluteEncoderPort,
            DriveConstants.kBackRightAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightAbsoluteEncoderReversed,
            DriveConstants.kBackRightForwardPIDk
        )
        
        self.xvelocity = 0
        self.yvelocity = 0
        self.turningvelocity = 0

        swerveModulePositions = (wpimath.kinematics.SwerveModulePosition(self.frontLeft.getDrivePosition(), Rotation2d(self.frontLeft.getTurningPosition())),
                                wpimath.kinematics.SwerveModulePosition(self.frontRight.getDrivePosition(), Rotation2d(self.frontRight.getTurningPosition())),
                                wpimath.kinematics.SwerveModulePosition(self.backLeft.getDrivePosition(), Rotation2d(self.backLeft.getTurningPosition())),
                                wpimath.kinematics.SwerveModulePosition(self.backRight.getDrivePosition(), Rotation2d(self.backRight.getTurningPosition())),
                                )
        # self.gyro = navx.AHRS(navx.SPI.Port.kMXP)
        self.gyro = navx.AHRS.create_spi()
        self.zeroHeading()
        sleep(1) # Wait for gyro to calibrate... NOT MORE THAN 2 SEC!! or get error.
        self.odometer = wpimath.kinematics.SwerveDrive4Odometry(DriveConstants.kDriveKinematics,self.getRotation2d(), swerveModulePositions)       

        # Pathplanner Configuration
        config = RobotConfig.fromGUISettings() # RobotConfig, this should likely live in your Constants class
        print("Pathplanner Configuration")
        print(config)
        # For Autonomous Pathplanner
        AutoBuilder.configure(
            self.getPose, # Robot pose supplier
            self.resetOdometer, # Method to reset odometry (will be called if your auto has a starting pose)
            self.getChassisSpeed, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            lambda speeds, feedforwards: self.setChassisSpeeds(speeds), # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
            PPHolonomicDriveController( # PPHolonomicController is the built in path following controller for holonomic drive trains
                PIDConstants(5.0, 0.0, 0.0), # Translation PID constants
                PIDConstants(5.0, 0.0, 0.0) # Rotation PID constants
            ),
            config, # The robot configuration
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )

    def zeroHeading(self):
        ''' Zero the gyro heading '''
        self.gyro.reset()

    def getCompass(self)->float: 
        ''' Return heading in degrees for values between 0 and 360 (from Gyro)'''
        return self.gyro.getFusedHeading()

    def getHeading(self):
        ''' Return heading in degrees for values between 0 and 360 (from Gyro)'''
        return self.gyro.getAngle() % 360

    def getContinuousHeading(self):
        ''' Return heading in degrees (from Gyro)'''
        return self.gyro.getAngle()

    def getRotation2d(self):
        ''' Returns heading in Rotation2d format '''
        return Rotation2d.fromDegrees(self.getHeading())

    def getPose(self):
        ''' Returns the pose of the robot '''
        return self.odometer.getPose()
    
    def resetOdometer(self, pose):
        ''' Resets the odometer to a specific pose '''
        self.odometer.resetPosition(self.getRotation2d(), (self.frontLeft.getSwerveModulePosition(), self.frontRight.getSwerveModulePosition(), self.backLeft.getSwerveModulePosition(), self.backRight.getSwerveModulePosition()), pose)
        
    def resetOdometerBackup(self, pose):
        ''' Resets the odometer to a specific pose '''
        # self.odometer.resetPosition(self.getRotation2d(), (self.frontLeft.getSwerveModulePosition(), self.frontRight.getSwerveModulePosition(), self.backLeft.getSwerveModulePosition(), self.backRight.getSwerveModulePosition()), pose)
        self.odometer.resetPosition(self.getRotation2d(), (self.frontLeft.getDrivePosition(), self.frontRight.getDrivePosition(), self.backLeft.getDrivePosition(), self.backRight.getDrivePosition()), pose)

    def resetEncoder(self):
        ''' Resets all the encoders - 
        Forward encoders to 0, rotation encoder to output of Absolute Encoder. '''
        self.frontLeft.resetEncoders()
        self.frontRight.resetEncoders()
        self.backLeft.resetEncoders()
        self.backRight.resetEncoders()

    def readAbsEncoders(self):
        ''' Reads all absolute encoders - Returns list of values [FL, FR, BL, BR] '''
        FL = self.frontLeft.getAbsoluteEncoderRad()
        FR = self.frontRight.getAbsoluteEncoderRad()
        BL = self.backLeft.getAbsoluteEncoderRad()
        BR = self.backRight.getAbsoluteEncoderRad()
        return [FL,FR,BL,BR]
    
    def readTurnEncoders(self):
        ''' Reads all turning encoders - Returns list of values [FL, FR, BL, BR] '''
        FL = self.frontLeft.getTurningPositionClose()
        FR = self.frontRight.getTurningPositionClose()
        BL = self.backLeft.getTurningPositionClose()
        BR = self.backRight.getTurningPositionClose()
        return [FL,FR,BL,BR]
    
    def readForwardEncoders(self):
        ''' Reads all Forward encoders - Returns list of values [FL, FR, BL, BR] '''
        FL = self.frontLeft.getDrivePosition()
        FR = self.frontRight.getDrivePosition()
        BL = self.backLeft.getDrivePosition()
        BR = self.backRight.getDrivePosition()
        return [FL,FR,BL,BR]

    def stopModules(self):
        ''' Stops all motors '''
        self.frontLeft.stop()
        self.frontRight.stop()
        self.backLeft.stop()
        self.backRight.stop()

    def moveStraight(self, speed):
        ''' Moves all motors straight forward at a certain speed '''
        straightState = SwerveModuleState(speed, Rotation2d(0))
        self.frontLeft.setDesiredState(straightState)
        self.frontRight.setDesiredState(straightState)
        self.backLeft.setDesiredState(straightState)
        self.backRight.setDesiredState(straightState)

    def moveStraightField(self, speed):
        ''' Moves all motors straight forward at a certain speed (In field oriented mode) '''
        self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                speed, 0, 0, self.getRotation2d())
        moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(self.chassisSpeeds)
        self.setModuleStates(moduleStates)

    def setModuleStates(self,desiredStates):
        ''' Sets swerve system to go in a specific direction and speed '''
        desiredStates = wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond)
        self.frontLeft.setDesiredState(desiredStates[0])
        self.frontRight.setDesiredState(desiredStates[1])
        self.backLeft.setDesiredState(desiredStates[2])
        self.backRight.setDesiredState(desiredStates[3])

    def setChassisSpeeds(self, chassisSpeeds:ChassisSpeeds):
        ''' Sets the chassis speeds of the robot in Autonomous mode (PathPlanner)'''
        self.xvelocity = chassisSpeeds.vx
        self.yvelocity = chassisSpeeds.vy
        self.turningvelocity = chassisSpeeds.omega
        moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds)
        self.setModuleStates(moduleStates)

    def getChassisSpeed(self):
        ''' Returns the chassis speeds of the robot in Autonomous mode (PathPlanner)'''
        return ChassisSpeeds(self.getXVelocity(), self.getYVelocity(), self.getAngularVelocity())
    
    def shouldFlipPath(self):
        ''' Used for Autonomoud mode (PathPlanner) - Boolean supplier that controls when the path will be mirrored for the red alliance.
        This will flip the path being followed to the red side of the field.
        THE ORIGIN WILL REMAIN ON THE BLUE SIDE '''
        return DriverStation.getAlliance() == DriverStation.Alliance.kBlue

    def getXVelocity(self):
        ''' Returns the X velocity of the robot '''
        return self.xvelocity

    def getYVelocity(self):
        ''' Returns the Y velocity of the robot '''
        return self.yvelocity
    
    def getDesiredVelocity(self):
        ''' Returns the velocity of the robot '''
        return (self.getXVelocity()**2 + self.getYVelocity()**2)**0.5
    
    def getVelocity(self):
        ''' Returns the desired velocity of the robot '''
        # Reads the velocity of each module and averages them
        # for the x axis and the y axis
        # then finds the hypothenuse of the two
        # to get the velocity of the robot
        FLx = self.frontLeft.getDriveVelocity()*cos(self.frontLeft.getTurningPosition())
        FRx = self.frontRight.getDriveVelocity()*cos(self.frontRight.getTurningPosition())
        BLx = self.backLeft.getDriveVelocity()*cos(self.backLeft.getTurningPosition())
        BRx = self.backRight.getDriveVelocity()*cos(self.backRight.getTurningPosition())
        xvelocity = (FLx+FRx+BLx+BRx)/4
        FLy = self.frontLeft.getDriveVelocity()*sin(self.frontLeft.getTurningPosition())
        FRy = self.frontRight.getDriveVelocity()*sin(self.frontRight.getTurningPosition())
        BLy = self.backLeft.getDriveVelocity()*sin(self.backLeft.getTurningPosition())
        BRy = self.backRight.getDriveVelocity()*sin(self.backRight.getTurningPosition())
        yvelocity = (FLy+FRy+BLy+BRy)/4
        return (xvelocity**2 + yvelocity**2)**0.5
        
    def getAngularVelocity(self):
        ''' Returns the angular velocity of the robot '''
        return self.turningvelocity

    # Periodic is called every cycle (20ms)
    def periodic(self):
        ''' The code that runs periodically '''
        # Reads Odometer (location of robot (x,y))
        self.odometer.update(self.getRotation2d(), (self.frontLeft.getSwerveModulePosition(), self.frontRight.getSwerveModulePosition(), self.backLeft.getSwerveModulePosition(), self.backRight.getSwerveModulePosition()))
        
        # Reads Absolute Encoders and sends them to Dashboard
        # absoluteEncoder = self.readAbsEncoders()
        # wpilib.SmartDashboard.putNumber("AbsEnc FL", absoluteEncoder[0])
        # wpilib.SmartDashboard.putNumber("AbsEnc FR", absoluteEncoder[1])
        # wpilib.SmartDashboard.putNumber("AbsEnc BL", absoluteEncoder[2])
        # wpilib.SmartDashboard.putNumber("AbsEnc BR", absoluteEncoder[3])
        
        # # Reads Encoders and sends them to Dashboard
        # turnings = self.readTurnEncoders()
        # wpilib.SmartDashboard.putNumber("Turning FL", turnings[0])
        # wpilib.SmartDashboard.putNumber("Turning FR", turnings[1])
        # wpilib.SmartDashboard.putNumber("Turning BL", turnings[2])
        # wpilib.SmartDashboard.putNumber("Turning BR", turnings[3])

        # # Reads Distance Travelled and sends them to Dashboard
        # forwards = self.readForwardEncoders()
        # wpilib.SmartDashboard.putNumber("Forward FL", forwards[0])
        # wpilib.SmartDashboard.putNumber("Forward FR", forwards[1])
        # wpilib.SmartDashboard.putNumber("Forward BL", forwards[2])
        # wpilib.SmartDashboard.putNumber("Forward BR", forwards[3])

        # wpilib.SmartDashboard.putNumber("Heading", self.getHeading())
        # wpilib.SmartDashboard.putNumber("Continuous Heading", self.getContinuousHeading())
        
    # TEST MOTORS INDEPENDENTLY
    def moveFLTMotor(self):
        self.frontLeft.turningMotor.set(0.6)

    def moveFLDMotor(self):
        self.frontLeft.driveMotor.set_control(phoenix6.controls.DutyCycleOut(0.5))

    def moveFRTMotor(self):
        self.frontRight.turningMotor.set(0.6)

    def moveFRDMotor(self):
        self.frontRight.driveMotor.set_control(phoenix6.controls.DutyCycleOut(0.5))

    def moveBLTMotor(self):
        self.backLeft.turningMotor.set(0.6)

    def moveBLDMotor(self):
        self.backLeft.driveMotor.set_control(phoenix6.controls.DutyCycleOut(0.5))

    def moveBRTMotor(self):
        self.backRight.turningMotor.set(0.6)

    def moveBRDMotor(self):
        self.backRight.driveMotor.set_control(phoenix6.controls.DutyCycleOut(0.5))