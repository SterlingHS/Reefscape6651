import wpilib.shuffleboard
from subsystems.SwerveModule import SwerveModule

import wpilib
from commands2 import Subsystem
from constants import DriveConstants
import phoenix6

# SysID Code
from wpilib import RobotController
from wpilib.sysid import SysIdRoutineLog
from commands2.sysid import SysIdRoutine
from wpimath.units import volts

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

        def sysidDrive(voltage: volts) -> None:
            ''' Drive to tune up drive system with SysId '''
            self.frontLeft.driveMotor.set_control(phoenix6.controls.VoltageOut(voltage))
            self.frontRight.driveMotor.set_control(phoenix6.controls.VoltageOut(voltage))
            self.backLeft.driveMotor.set_control(phoenix6.controls.VoltageOut(voltage))
            self.backRight.driveMotor.set_control(phoenix6.controls.VoltageOut(voltage))

        phoenix6.SignalLogger.set_path("sysid")
        SysConfig = SysIdRoutine.Config(
            # This is the function that will be called to set the mechanism to a given state
            # recordState = lambda state: phoenix6.SignalLogger.write_string("state", SysIdRoutineLog.stateEnumToString(state)),
        )

        SysMechanism = SysIdRoutine.Mechanism(
            sysidDrive, 
            self.log, 
            self
        )

        self.sys_id_routine = SysIdRoutine(
            SysConfig,
            SysMechanism
        )

    # Periodic is called every cycle (20ms)
    def periodic(self):
        
        # Drivee straight at 0 degrees
        self.setSetTurningPoint(0)

        wpilib.SmartDashboard.putNumber("Turning FL", self.frontLeft.getTurningPosition())
        wpilib.SmartDashboard.putNumber("Turning FR", self.frontRight.getTurningPosition())
        wpilib.SmartDashboard.putNumber("Turning BL", self.backLeft.getTurningPosition())
        wpilib.SmartDashboard.putNumber("Turning BR", self.backRight.getTurningPosition())
        pass

    def setSetTurningPoint(self, angle):
        ''' Sets the the setPoint of the turning PID to a specific angle - Used to tune up PID '''
        self.frontLeft.setSetTurningPosition(angle)
        self.frontRight.setSetTurningPosition(angle)
        self.backLeft.setSetTurningPosition(angle)
        self.backRight.setSetTurningPosition(angle)

################################################################################
# SYSID CODE

    # Tell SysId how to record a frame of data for each motor on the mechanism being
    # characterized.

    def log(self, sys_id_routine: SysIdRoutineLog) -> None:
        # Record a frame for the left motors.  Since these share an encoder, we consider
        # the entire group to be one motor.

        sys_id_routine.motor("drive-front-left"
            ).voltage(self.frontLeft.driveMotor.get_motor_voltage().value_as_double# * RobotController.getBatteryVoltage()
            ).position(self.frontLeft.getDrivePosition()
            ).velocity(self.frontLeft.getDriveVelocity())
        
        sys_id_routine.motor("drive-front-right"
            ).voltage(self.frontRight.driveMotor.get_motor_voltage().value_as_double# * RobotController.getBatteryVoltage()
            ).position(self.frontRight.getDrivePosition()
            ).velocity(self.frontRight.getDriveVelocity())

        sys_id_routine.motor("drive-back-left"
            ).voltage(self.backLeft.driveMotor.get_motor_voltage().value_as_double# * RobotController.getBatteryVoltage()
            ).position(self.backLeft.getDrivePosition()
            ).velocity(self.backLeft.getDriveVelocity())

        sys_id_routine.motor("drive-back-right"
            ).voltage(self.backRight.driveMotor.get_motor_voltage().value_as_double# * RobotController.getBatteryVoltage()
            ).position(self.backRight.getDrivePosition()
            ).velocity(self.backRight.getDriveVelocity())
        pass

    def sysIdQuasistatic(self, direction: SysIdRoutine.Direction):
        ''' Run the SysId routine in quasistatic mode ''' 
        return self.sys_id_routine.quasistatic(direction)

    def sysIdDynamic(self, direction: SysIdRoutine.Direction):
        ''' Run the SysId routine in dynamic mode '''
        return self.sys_id_routine.dynamic(direction)