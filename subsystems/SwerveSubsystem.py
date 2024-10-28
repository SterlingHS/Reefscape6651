from subsystems.SwerveModule import SwerveModule
from wpimath.geometry import Rotation2d
import wpimath.kinematics
import wpimath
from wpimath.units import volts
import navx
from commands2 import Subsystem
from constants import DriveConstants
import wpilib
from wpilib import RobotController
from wpimath.kinematics import SwerveModuleState, ChassisSpeeds
from wpilib.sysid import SysIdRoutineLog
from commands2.sysid import SysIdRoutine

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

        #self.gyro = AHRS(navx.SPI.Port.kMXP)s
        swerveModulePositions = (wpimath.kinematics.SwerveModulePosition(self.frontLeft.getDrivePosition(), Rotation2d(self.frontLeft.getTurningPosition())),
                                wpimath.kinematics.SwerveModulePosition(self.frontRight.getDrivePosition(), Rotation2d(self.frontRight.getTurningPosition())),
                                wpimath.kinematics.SwerveModulePosition(self.backLeft.getDrivePosition(), Rotation2d(self.backLeft.getTurningPosition())),
                                wpimath.kinematics.SwerveModulePosition(self.backRight.getDrivePosition(), Rotation2d(self.backRight.getTurningPosition())),
                                )
        self.gyro = navx.AHRS.create_spi()
        self.odometer = wpimath.kinematics.SwerveDrive4Odometry(DriveConstants.kDriveKinematics,self.getRotation2d(), swerveModulePositions)
        self.distanceSensor = wpilib.AnalogInput(DriveConstants.kDistanceSensorID)
        #Code example uses a more sophisticated method to control
        #the timing of the zeroHeading() 
        self.zeroHeading()

        # Tell SysId how to plumb the driving voltage to the motors.
        def sysidDrive(voltage: volts) -> None:
            m = -.03
            self.frontLeft.driveMotor.set_control(phoenix6.controls.DutyCycleOut(-m*voltage))
            self.frontRight.driveMotor.set_control(phoenix6.controls.DutyCycleOut(m*voltage))
            self.backLeft.driveMotor.set_control(phoenix6.controls.DutyCycleOut(-m*voltage))
            self.backRight.driveMotor.set_control(phoenix6.controls.DutyCycleOut(m*voltage))

        # Tell SysId to make generated commands require this subsystem, suffix test state in
        # WPILog with this subsystem's name ("drive")
        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(sysidDrive, self.log, self, "Swerve Drive"),)

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

    def getDistanceSensorPos(self):
        #return self.distanceSensor.getPosition()
        return self.distanceSensor.getValue()

    def resetOdometer(self, pose):
        self.odometer.resetPosition(self.getRotation2d(), (self.frontLeft.getSwerveModulePosition(), self.frontRight.getSwerveModulePosition(), self.backLeft.getSwerveModulePosition(), self.backRight.getSwerveModulePosition()), pose)

    def resetEncoder(self):
        self.frontLeft.resetEncoders()
        self.frontRight.resetEncoders()
        self.backLeft.resetEncoders()
        self.backRight.resetEncoders()

    def periodic(self):
        self.odometer.update(self.getRotation2d(), (self.frontLeft.getSwerveModulePosition(), self.frontRight.getSwerveModulePosition(), self.backLeft.getSwerveModulePosition(), self.backRight.getSwerveModulePosition()))

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

    # Tell SysId how to record a frame of data for each motor on the mechanism being
    # characterized.
    def log(self, sys_id_routine: SysIdRoutineLog) -> None:
        # Record a frame for the left motors.  Since these share an encoder, we consider
        # the entire group to be one motor.
        sys_id_routine.motor("drive-front-left").voltage(
            self.frontLeft.driveMotor.get_motor_voltage().value_as_double * RobotController.getBatteryVoltage()
        ).position(self.frontLeft.getDrivePosition()).velocity(
            self.frontLeft.getDriveVelocity()
        )
        
        sys_id_routine.motor("drive-front-right").voltage(
            self.frontRight.driveMotor.get_motor_voltage().value_as_double * RobotController.getBatteryVoltage()
        ).position(self.frontRight.getDrivePosition()).velocity(
            self.frontRight.getDriveVelocity()
        )

        sys_id_routine.motor("drive-back-left").voltage(
            self.backLeft.driveMotor.get_motor_voltage().value_as_double * RobotController.getBatteryVoltage()
        ).position(self.backLeft.getDrivePosition()).velocity(
            self.backLeft.getDriveVelocity()
        )

        sys_id_routine.motor("drive-back-right").voltage(
            self.backRight.driveMotor.get_motor_voltage().value_as_double * RobotController.getBatteryVoltage()
        ).position(self.backRight.getDrivePosition()).velocity(
            self.backRight.getDriveVelocity()
        )


    def sysIdQuasistatic(self, direction: SysIdRoutine.Direction): 
        return self.sys_id_routine.quasistatic(direction)

    def sysIdDynamic(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine.dynamic(direction)