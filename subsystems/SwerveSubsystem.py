from subsystems.SwerveModule import SwerveModule

import wpimath
from wpimath.geometry import Rotation2d
import wpimath.kinematics
from wpimath.kinematics import SwerveModuleState, ChassisSpeeds

import wpilib
from commands2 import Subsystem
from constants import DriveConstants, AutoConstants
from pathplannerlib.config import HolonomicPathFollowerConfig, PIDConstants, ReplanningConfig

import navx
import phoenix6

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

        swerveModulePositions = (wpimath.kinematics.SwerveModulePosition(self.frontLeft.getDrivePosition(), Rotation2d(self.frontLeft.getTurningPosition())),
                                wpimath.kinematics.SwerveModulePosition(self.frontRight.getDrivePosition(), Rotation2d(self.frontRight.getTurningPosition())),
                                wpimath.kinematics.SwerveModulePosition(self.backLeft.getDrivePosition(), Rotation2d(self.backLeft.getTurningPosition())),
                                wpimath.kinematics.SwerveModulePosition(self.backRight.getDrivePosition(), Rotation2d(self.backRight.getTurningPosition())),
                                )
        
        self.gyro = navx.AHRS.create_spi()
        self.zeroHeading()
        sleep(1) # Wait for gyro to calibrate... NOT MORE THAN 2 SEC!! or get error.
        self.odometer = wpimath.kinematics.SwerveDrive4Odometry(DriveConstants.kDriveKinematics,self.getRotation2d(), swerveModulePositions)       

        wpilib.SmartDashboard.putNumber("P", 0)
        wpilib.SmartDashboard.putNumber("I", 0)
        wpilib.SmartDashboard.putNumber("D", 0)

    def zeroHeading(self):
        self.gyro.reset()

    def getCompass(self)->float: 
        return self.gyro.getFusedHeading()

    def getHeading(self):
        ''' Return heading in degrees for values between 0 and 360 '''
        return -1*self.gyro.getAngle() % 360

    def getContinuousHeading(self):
        ''' Return heading in degrees '''
        return -1*self.gyro.getAngle()

    def getRotation2d(self):
        ''' Returns heading in Rotation2d format '''
        return Rotation2d.fromDegrees(self.getHeading())

    def getPose(self):
        ''' Returns the pose of the robot '''
        return self.odometer.getPose()

    def getModuleStates(self):
        ''' Returns the states of the swerve modules '''
        states = [self.frontLeft.getState(), self.frontRight.getState(),self.backLeft.getState(),self.backRight.getState()]
        return states
    
    def getSwerveModulePosition(self):
        ''' Returns the position of the swerve module '''
        return wpimath.kinematics.SwerveModulePosition(self.getDrivePosition(), Rotation2d(self.getTurningPosition()))

    def resetOdometer(self, pose):
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


        ''' Used for Autonomoud mode (PathPlanner) - Boolean supplier that controls when the path will be mirrored for the red alliance.
        This will flip the path being followed to the red side of the field.
        THE ORIGIN WILL REMAIN ON THE BLUE SIDE '''
        return DriverStation.getAlliance() == DriverStation.Alliance.kBlue

    # Periodic is called every cycle (20ms)
    def periodic(self):
        # Reads Odometer (location of robot (x,y))
        self.odometer.update(self.getRotation2d(), (self.frontLeft.getSwerveModulePosition(), self.frontRight.getSwerveModulePosition(), self.backLeft.getSwerveModulePosition(), self.backRight.getSwerveModulePosition()))
        
        # Reads Absolute Encoders and sends them to Dashboard
        absoluteEncoder = self.readAbsEncoders()
        wpilib.SmartDashboard.putNumber("AbsEnc FL", absoluteEncoder[0])
        wpilib.SmartDashboard.putNumber("AbsEnc FR", absoluteEncoder[1])
        wpilib.SmartDashboard.putNumber("AbsEnc BL", absoluteEncoder[2])
        wpilib.SmartDashboard.putNumber("AbsEnc BR", absoluteEncoder[3])
        
        # Reads Encoders and sends them to Dashboard
        turnings = self.readTurnEncoders()
        wpilib.SmartDashboard.putNumber("Turning FL", turnings[0])
        wpilib.SmartDashboard.putNumber("Turning FR", turnings[1])
        wpilib.SmartDashboard.putNumber("Turning BL", turnings[2])
        wpilib.SmartDashboard.putNumber("Turning BR", turnings[3])

        # Reads Distance Travelled and sends them to Dashboard
        forwards = self.readForwardEncoders()
        wpilib.SmartDashboard.putNumber("Forward FL", forwards[0])
        wpilib.SmartDashboard.putNumber("Forward FR", forwards[1])
        wpilib.SmartDashboard.putNumber("Forward BL", forwards[2])
        wpilib.SmartDashboard.putNumber("Forward BR", forwards[3])

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

    ################################################################################
    # PID Control for steering robot
    # Used to tune up the PID Turn Constants
    # P=0.6    I=0.1   D=0.02

    def setTurningPID(self, P, I, D):
        # Change PID Constants for Turning modules
        self.frontLeft.setTurningPID(P,I,D)
        self.frontRight.setTurningPID(P,I,D)
        self.backLeft.setTurningPID(P,I,D)
        self.backRight.setTurningPID(P,I,D)
    
    def getTurningPID(self):
        # Returns PID Constants for Turning modules
        return (self.frontLeft.getTurningPID(), self.frontRight.getTurningPID(), self.backLeft.getTurningPID(), self.backRight.getTurningPID())
    
    def setSetTurningPoint(self, angle):
        ''' Sets the the setPoint of the turning PID to a specific angle - Used to tune up PID '''
        self.frontLeft.setSetTurningPosition(angle)
        self.frontRight.setSetTurningPosition(angle)
        self.backLeft.setSetTurningPosition(angle)
        self.backRight.setSetTurningPosition(angle)

    def turningPIDerror(self, angle):
        ''' Returns the error from PID turning - Used to tune up PID'''
        encoder_value_fl = self.frontLeft.getTurningPositionClose()
        encoder_value_fr = self.frontRight.getTurningPositionClose()
        encoder_value_bl = self.backLeft.getTurningPositionClose()
        encoder_value_br = self.backRight.getTurningPositionClose() 
        return (encoder_value_fl-angle, encoder_value_fr-angle, encoder_value_bl-angle, encoder_value_br-angle)

    def turningPIDtuneUP(self, angle):
        ''' Tune up PID for turning '''
        # Reads P, I, D from Shuffleboard
        P = wpilib.SmartDashboard.getNumber("P", 0.6)
        I = wpilib.SmartDashboard.getNumber("I", 0.1)
        D = wpilib.SmartDashboard.getNumber("D", 0.02)

        # Prints the error
        errors = self.turningPIDerror(angle)
        wpilib.SmartDashboard.putNumber("Error FL", errors[0])
        wpilib.SmartDashboard.putNumber("Error FR", errors[1])
        wpilib.SmartDashboard.putNumber("Error BL", errors[2])
        wpilib.SmartDashboard.putNumber("Error BR", errors[3])

        # get PID Constants for Turning modules
        (Pc, Ic, Dc) = self.getTurningPID()[0]

        # Send P, I, D to controllers if values have changed
        if Pc != P or Ic != I or Dc != D:
            self.setTurningPID(P,I,D)

        # Send angle to controllers
        self.setSetTurningPoint(angle)