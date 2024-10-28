from wpimath.geometry import Rotation2d
import wpilib
import rev

import math
from constants import ModuleConstants, DriveConstants
from wpimath.kinematics import SwerveModuleState
from wpimath.controller import PIDController
from wpimath.controller import SimpleMotorFeedforwardMeters
import phoenix6.hardware
import phoenix6

import wpimath.kinematics
class SwerveModule:

    def __init__(self, driveMotorID, turningMotorID, driveMotorReversed, turningMotorReversed,
        absoluteEncoderID, absoluteEncoderOffset, absoluteEncoderReversed, drivePIDk):
            self.absoluteEncoderOffsetRad = absoluteEncoderOffset
            self.absoluteEncoderReversed = absoluteEncoderReversed
            self.absoluteEncoder = wpilib.AnalogInput(absoluteEncoderID)

            self.driveMotor = phoenix6.hardware.TalonFX(driveMotorID)
            self.turningMotor = rev.CANSparkMax(turningMotorID, rev.CANSparkLowLevel.MotorType.kBrushless)

            self.turningMotor.setInverted(turningMotorReversed)

            self.turningEncoder = self.turningMotor.getEncoder()
            self.turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
            self.turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec)
            
            self.turningPIDController = PIDController(ModuleConstants.kPTurning, ModuleConstants.kDTurning, 0)
            self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

            self.drivePIDController = PIDController(drivePIDk[0], drivePIDk[1], drivePIDk[2])
            self.driveFeedbackForward = SimpleMotorFeedforwardMeters(drivePIDk[3], drivePIDk[4], drivePIDk[5])

            self.resetEncoders()

    def getDrivePosition(self):
        return ModuleConstants.kDriveEncoderRot2Meter*self.driveMotor.get_position().value
    def getTurningPosition(self):
        return self.turningEncoder.getPosition()

    def getDriveVelocity(self):
        return ModuleConstants.kDriveEncoderRPM2MeterPerSec*self.driveMotor.get_velocity().value
    

    def getTurningVelocity(self):
        return self.turningEncoder.getVelocity()

    def getAbsoluteEncoderRad(self):
        angle = self.absoluteEncoder.getVoltage()/5.0 #/RobotController.getVoltage5V()
        angle*=2*math.pi
        angle-=self.absoluteEncoderOffsetRad
        direction = -1.0 if self.absoluteEncoderReversed else 1.0
        return angle*direction

    def getAbsoluteEncoderData(self):
        return self.absoluteEncoder.getValue()

    def resetEncoders(self):
        self.driveMotor.set_position(0)
        self.turningEncoder.setPosition(self.getAbsoluteEncoderRad())
        #self.turningEncoder.setPosition(0)

    def getState(self):
        return SwerveModuleState(self.getDriveVelocity(), Rotation2d(self.getTurningPosition()))

    def getSwerveModulePosition(self):
        return wpimath.kinematics.SwerveModulePosition(self.getDrivePosition(), Rotation2d(self.getTurningPosition()))

    def stop(self):
        self.driveMotor.set_control(phoenix6.controls.DutyCycleOut(0))
        self.turningMotor.set(0)

    def setDesiredState(self, state):
        if (abs(state.speed)<0.001):
            self.stop()
        
        
        state = SwerveModuleState.optimize(state, self.getState().angle)
        wpilib.SmartDashboard.putNumber("Module Speed", state.speed)
        
        driveOutput = self.drivePIDController.calculate(self.getDriveVelocity(),state.speed)
        driveFeedForward = self.driveFeedbackForward.calculate(state.speed)
        #self.driveMotor.set_control(phoenix6.controls.DutyCycleOut(state.speed/DriveConstants.kPhysicalMaxSpeedMetersPerSecond))
        self.driveMotor.set_control(phoenix6.controls.DutyCycleOut(driveOutput+driveFeedForward))
        
        self.turningPIDController.setP(ModuleConstants.kPTurning)
        self.turningPIDController.setD(ModuleConstants.kDTurning)
        self.turningMotor.set(0)#self.turningPIDController.calculate(self.getTurningPosition(), state.angle.radians()))
        