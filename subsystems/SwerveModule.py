########################################################################
# Class: SwerveModule
# Purpose: This class is used to control a single swerve module
########################################################################

# Imports
import wpilib
import math

from constants import ModuleConstants, DriveConstants

from wpimath.kinematics import SwerveModuleState
from wpimath.controller import PIDController
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d
import wpimath.kinematics

import phoenix6.hardware
import phoenix6
import rev

# Class: SwerveModule
class SwerveModule:

    def __init__(self, driveMotorID, turningMotorID, driveMotorReversed, turningMotorReversed,
        absoluteEncoderID, absoluteEncoderOffset, absoluteEncoderReversed, drivePIDk):
            self.absoluteEncoderOffsetRad = absoluteEncoderOffset
            self.absoluteEncoderReversed = absoluteEncoderReversed
            
            # Init of Absolute Encoder (CANCoder)
            self.absoluteEncoder = phoenix6.hardware.CANcoder(absoluteEncoderID, "AbsoluteEncoder"+str(absoluteEncoderID))

            # Init of Drive Motor (TalonFX) for Kraken X.60
            self.driveMotor = phoenix6.hardware.TalonFX(driveMotorID)
            # INVERTER NEEDS TO BE DONE MANUALLY USING TUNER

            # Init of Turning Motor (SparkMax) for NEO v1.1
            self.turningMotor = rev.CANSparkMax(turningMotorID, rev.CANSparkLowLevel.MotorType.kBrushless)
            self.turningMotor.setInverted(turningMotorReversed)

            # Init of Encoder to rotate wheel (on the NEO)
            self.turningEncoder = self.turningMotor.getEncoder()
            
            self.turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
            self.turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec)
            
            # Encoder Kraken X.60
            # Encoder is included in TalonFX so no need to initialize it
            # set position conversion factor for driver motor encoder???
            

            # PID Controllers for turning and driving
            self.turningPIDController = PIDController(ModuleConstants.kPTurning, ModuleConstants.kDTurning, 0) # Why is D instead of I?
            self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

            self.drivePIDController = PIDController(drivePIDk[0], drivePIDk[1], drivePIDk[2]) # Why do we have 2 PID controllers?
            self.driveFeedbackForward = SimpleMotorFeedforwardMeters(drivePIDk[3], drivePIDk[4], drivePIDk[5])

            self.resetEncoders()

    # Returns the position of the drive motor in meters
    def getDrivePosition(self):
        return ModuleConstants.kDriveEncoderRot2Meter * self.driveMotor.get_position().value
    
    # Returns the position of the turning motor in radians
    def getTurningPosition(self):
        return self.turningEncoder.getPosition()
    
    # Returns the velocity of the drive motor in meters per second
    def getDriveVelocity(self):
        return ModuleConstants.kDriveEncoderRPM2MeterPerSec * self.driveMotor.get_velocity().value
    
    # Returns the velocity of the turning motor in radians per second
    def getTurningVelocity(self):
        return self.turningEncoder.getVelocity()

    # Returns the absolute position of the turning motor in radians (0-2pi)
    def getAbsoluteEncoderRad(self):
        ##########################################################
        #### CHECK VALUE FROM ENCODER
        #### CHECK WHAT SHOULD BE RETURNED

        # angle = self.absoluteEncoder.getVoltage()/5.0 #/RobotController.getVoltage5V()
        angle = self.absoluteEncoder.get_absolute_position().value
        angle = angle * 2*math.pi
        angle-=self.absoluteEncoderOffsetRad
        direction = -1.0 if self.absoluteEncoderReversed else 1.0
        return angle*direction

    # Returns the absolute position of the turning motor in degrees (0-360)
    def getAbsoluteEncoderData(self):
        ##########################################################
        #### CHECK VALUE FROM ENCODER
        return self.absoluteEncoder.get_absolute_position()

    # Resets all encoders when init
    def resetEncoders(self):
        self.driveMotor.set_position(0)
        self.turningEncoder.setPosition(self.getAbsoluteEncoderRad())
        #self.turningEncoder.setPosition(0)

    # Return the STATE of the Swerve Module
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
        