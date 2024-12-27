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
            self.absoluteEncoder = phoenix6.hardware.CANcoder(absoluteEncoderID)

            # Init of Drive Motor (TalonFX) for Kraken X.60
            self.driveMotor = phoenix6.hardware.TalonFX(driveMotorID)

            # Init of Turning Motor (SparkMax) for NEO v1.1
            self.turningMotor = rev.CANSparkMax(turningMotorID, rev.CANSparkLowLevel.MotorType.kBrushless)
            self.turningMotor.setInverted(turningMotorReversed)

            # Init of Encoder to rotate wheel (on the NEO)
            self.turningEncoder = self.turningMotor.getEncoder()
            
            self.turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
            self.turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec)
            
            ##############################################################################################################
            # Encoder Kraken X.60
            # Encoder is included in TalonFX so no need to initialize it
            # set position conversion factor for driver motor encoder
            # Configure drive motor

            drive_config = self.driveMotor.configurator
            driveMotorConfig = phoenix6.configs.MotorOutputConfigs()
            driveMotorConfig.neutral_mode = phoenix6.signals.NeutralModeValue.BRAKE
            driveMotorConfig.inverted = (
                phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE 
                if driveMotorReversed 
                else phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            )
            drive_gear_ratio_config = phoenix6.configs.FeedbackConfigs().with_sensor_to_mechanism_ratio(1/ModuleConstants.kDriveEncoderRot2Meter)

            # Reduce CAN status frame rates before configuring
            info = self.driveMotor.get_fault_field().get_applied_update_frequency()
            print("Before change of frame rate - Drive Motor CAN status frame rates: ", info)                  # It does not print!!! Why?
            self.driveMotor.get_fault_field().set_update_frequency(frequency_hz=4, timeout_seconds=0.01)
            info2 = self.driveMotor.get_fault_field().get_applied_update_frequency()
            print("After change of frame rate - Drive Motor CAN status frame rates: ", info2)

            # PID for drive forward using TalonFX instead of Roborio     
            self.drive_pid = phoenix6.configs.Slot0Configs().with_k_p(drivePIDk[0]).with_k_i(drivePIDk[1]).with_k_d(drivePIDk[2])
            self.drive_ff = SimpleMotorFeedforwardMeters(kS=drivePIDk[3], kV=drivePIDk[4], kA=drivePIDk[5])

            # Apply the configurations to the drive motor controllers
            drive_config.apply(driveMotorConfig)
            drive_config.apply(self.drive_pid, 0.01)
            drive_config.apply(drive_gear_ratio_config)

            self.driveMotor.get_fault_field().set_update_frequency(info)
            info = self.driveMotor.get_fault_field().get_applied_update_frequency()
            print("After update - Drive Motor CAN status frame rates: ", info)

            ##############################################################################################################

            # PID Controllers for turning and driving on Roborio
            self.turningPIDController = PIDController(ModuleConstants.kPTurning, ModuleConstants.kDTurning, 0) # Why is D instead of I?
            self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

            # Same PID but executed by SparkMax instead of Roborio
            # Let's try both and see which one works better
            # self.RevController = self.turningMotor.getPIDController()
            # self.RevController.setP(ModuleConstants.kPTurning)
            # self.RevController.setI(0)
            # self.RevController.setD(ModuleConstants.kDTurning)   
            # self.RevController.setPositionPIDWrappingMinInput(-math.pi) # Wrapping is the same as Continuous Input
            # self.RevController.setPositionPIDWrappingMaxInput(math.pi)
            # self.RevController.setPositionPIDWrappingEnabled(True)


            # PID Controller for driving controlled by Roborio
            self.drivePIDController = PIDController(drivePIDk[0], drivePIDk[1], drivePIDk[2]) # Why do we have 2 PID controllers?
            self.driveFeedbackForward = SimpleMotorFeedforwardMeters(drivePIDk[3], drivePIDk[4], drivePIDk[5])

            self.resetEncoders()

    # Returns the position of the drive motor in meters
    def getDrivePosition(self):
        return self.driveMotor.get_position().value
    
    # Returns the position of the turning motor in radians
    def getTurningPosition(self):
        return self.turningEncoder.getPosition()
    
    # Returns the velocity of the drive motor in meters per second
    def getDriveVelocity(self): # CHECK IF IT RETURNS THE RIGHT VALUE
        return self.driveMotor.get_velocity().value # * ModuleConstants.kDriveEncoderRPM2MeterPerSec
    
    # Returns the velocity of the turning motor in radians per second
    def getTurningVelocity(self):
        return self.turningEncoder.getVelocity()

    # Returns the absolute position of the turning motor in radians (0-2pi)
    def getAbsoluteEncoderRad(self):
        angle = self.absoluteEncoder.get_absolute_position().value # Output is from -0.5 to 0.5
        angle = angle * 2*math.pi # Output is from -pi to pi
        angle-=self.absoluteEncoderOffsetRad # Offset
        if angle > math.pi: # Makes the output be between -pi and pi
            angle -= 2*math.pi
        if angle < -math.pi:
            angle += 2*math.pi  
        direction = -1.0 if self.absoluteEncoderReversed else 1.0 # Reverses the output if needed
        return angle*direction

    # Returns the absolute position of the turning motor in degrees (0-360) - TO BE REMOVED (DEPRECIATED)
    # def getAbsoluteEncoderData(self):
    #     ##########################################################
    #     #### CHECK VALUE FROM ENCODER
    #     return self.absoluteEncoder.get_absolute_position()

    # Resets all encoders when init
    def resetEncoders(self):
        self.driveMotor.set_position(0)
        self.turningEncoder.setPosition(self.getAbsoluteEncoderRad())

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
        