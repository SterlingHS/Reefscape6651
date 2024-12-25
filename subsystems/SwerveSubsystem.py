from subsystems.SwerveModule import SwerveModule
from wpimath.geometry import Rotation2d
import wpimath.kinematics
import wpimath
import navx
from commands2 import Subsystem
from constants import DriveConstants
import wpilib
from wpimath.kinematics import SwerveModuleState, ChassisSpeeds

import phoenix6

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

        #self.gyro = navx.AHRS(navx.SPI.Port.kMXP)
        swerveModulePositions = (wpimath.kinematics.SwerveModulePosition(self.frontLeft.getDrivePosition(), Rotation2d(self.frontLeft.getTurningPosition())),
                                wpimath.kinematics.SwerveModulePosition(self.frontRight.getDrivePosition(), Rotation2d(self.frontRight.getTurningPosition())),
                                wpimath.kinematics.SwerveModulePosition(self.backLeft.getDrivePosition(), Rotation2d(self.backLeft.getTurningPosition())),
                                wpimath.kinematics.SwerveModulePosition(self.backRight.getDrivePosition(), Rotation2d(self.backRight.getTurningPosition())),
                                )
        self.gyro = navx.AHRS.create_spi()
        self.odometer = wpimath.kinematics.SwerveDrive4Odometry(DriveConstants.kDriveKinematics,self.getRotation2d(), swerveModulePositions)
        #Code example uses a more sophisticated method to control
        #the timing of the zeroHeading() 
        self.zeroHeading()

    def zeroHeading(self):
        self.gyro.reset()

    def getCompass(self)->float:
        return self.gyro.getFusedHeading()

    def getHeading(self):
        return -1*self.gyro.getAngle() % 360

    def getContinuousHeading(self):
        return -1*self.gyro.getAngle()

    def getRotation2d(self):
        return Rotation2d.fromDegrees(self.getHeading())

    def getPose(self):
        return self.odometer.getPose()

    def getModuleStates(self):
        states = [self.frontLeft.getState(), self.frontRight.getState(),self.backLeft.getState(),self.backRight.getState()]
        return states

    def resetOdometer(self, pose):
        self.odometer.resetPosition(self.getRotation2d(), (self.frontLeft.getSwerveModulePosition(), self.frontRight.getSwerveModulePosition(), self.backLeft.getSwerveModulePosition(), self.backRight.getSwerveModulePosition()), pose)

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
        FL = self.frontLeft.getTurningPosition()
        FR = self.frontRight.getTurningPosition()
        BL = self.backLeft.getTurningPosition()
        BR = self.backRight.getTurningPosition()
        return [FL,FR,BL,BR]
    
    def readForwardEncoders(self):
        ''' Reads all Forward encoders - Returns list of values [FL, FR, BL, BR] '''
        FL = self.frontLeft.getDrivePosition()
        FR = self.frontRight.getDrivePosition()
        BL = self.backLeft.getDrivePosition()
        BR = self.backRight.getDrivePosition()
        return [FL,FR,BL,BR]

    def periodic(self):
        self.odometer.update(self.getRotation2d(), (self.frontLeft.getSwerveModulePosition(), self.frontRight.getSwerveModulePosition(), self.backLeft.getSwerveModulePosition(), self.backRight.getSwerveModulePosition()))
        absoluteEncoder = self.readAbsEncoders()
        wpilib.SmartDashboard.putNumber("AbsEnc FL", absoluteEncoder[0])
        wpilib.SmartDashboard.putNumber("AbsEnc FR", absoluteEncoder[1])
        wpilib.SmartDashboard.putNumber("AbsEnc BL", absoluteEncoder[2])
        wpilib.SmartDashboard.putNumber("AbsEnc BR", absoluteEncoder[3])
        turnings = self.readTurnEncoders()
        wpilib.SmartDashboard.putNumber("Turning FL", turnings[0])
        wpilib.SmartDashboard.putNumber("Turning FR", turnings[1])
        wpilib.SmartDashboard.putNumber("Turning BL", turnings[2])
        wpilib.SmartDashboard.putNumber("Turning BR", turnings[3])
        forwards = self.readForwardEncoders()
        wpilib.SmartDashboard.putNumber("Forward FL", forwards[0])
        wpilib.SmartDashboard.putNumber("Forward FR", forwards[1])
        wpilib.SmartDashboard.putNumber("Forward BL", forwards[2])
        wpilib.SmartDashboard.putNumber("Forward BR", forwards[3])
        

    def stopModules(self):
        self.frontLeft.stop()
        self.frontRight.stop()
        self.backLeft.stop()
        self.backRight.stop()

    def moveStraight(self, speed):
        straightState = SwerveModuleState(speed, Rotation2d(0))
        self.frontLeft.setDesiredState(straightState)
        self.frontRight.setDesiredState(straightState)
        self.backLeft.setDesiredState(straightState)
        self.backRight.setDesiredState(straightState)

    def moveStraightField(self, speed):
        self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                speed, 0, 0, self.getRotation2d())
        moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(self.chassisSpeeds)
        self.setModuleStates(moduleStates)

    def setModuleStates(self,desiredStates):
        desiredStates = wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond)
        self.frontLeft.setDesiredState(desiredStates[0])
        self.frontRight.setDesiredState(desiredStates[1])
        self.backLeft.setDesiredState(desiredStates[2])
        self.backRight.setDesiredState(desiredStates[3])


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