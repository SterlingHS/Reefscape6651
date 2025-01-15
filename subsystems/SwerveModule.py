########################################################################
# Class: SwerveModule
# Purpose: This class is used to control a single swerve module
########################################################################

# Imports
import wpilib
from math import pi

from constants import ModuleConstants, DriveConstants

from wpimath.kinematics import SwerveModuleState
from wpimath.controller import PIDController
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d
import wpimath.kinematics

import phoenix6.hardware
import phoenix6
from rev import SparkMaxConfig, SparkMax, SparkLowLevel

# Class: SwerveModule
class SwerveModule:

    def __init__(self, driveMotorID, turningMotorID, driveMotorReversed, turningMotorReversed,
        absoluteEncoderID, absoluteEncoderOffset, absoluteEncoderReversed, drivePIDk):
            ''' Constructor for SwerveModule '''

            self.absoluteEncoderOffsetRad = absoluteEncoderOffset
            self.absoluteEncoderReversed = absoluteEncoderReversed
            self.driveMotorID = driveMotorID
            
            # Init of Absolute Encoder (CANCoder)
            self.absoluteEncoder = phoenix6.hardware.CANcoder(absoluteEncoderID)

            # Init of Drive Motor (TalonFX) for Kraken X.60
            self.driveMotor = phoenix6.hardware.TalonFX(driveMotorID)

            # Init of Turning Motor (SparkMax) for NEO v1.1
            self.turningMotor = SparkMax(turningMotorID, SparkLowLevel.MotorType.kBrushless)
            # self.turningMotor.setInverted(turningMotorReversed)

            # Init of Encoder to rotate wheel (on the NEO)
            self.turningEncoder = self.turningMotor.getEncoder()
            # configTurningEncoder = SparkMaxConfig()

            # configTurningEncoder.encoder.positionConversionFactor = ModuleConstants.kTurningEncoderRot2Rad
            # configTurningEncoder.encoder.velocityConversionFactor = ModuleConstants.kTurningEncoderRPM2RadPerSec
            
            # self.turningMotor.configure(config=configTurningEncoder)

            
            ##############################################################################################################
            # Encoder Kraken X.60
            # Encoder is included in TalonFX so no need to initialize it
            # set position conversion factor for driver motor encoder
            # Configure drive motor

            # drive_config = self.driveMotor.configurator
            # driveMotorConfig = phoenix6.configs.MotorOutputConfigs()
            # driveMotorConfig.neutral_mode = phoenix6.signals.NeutralModeValue.BRAKE
            # driveMotorConfig.inverted = (
            #     phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE 
            #     if driveMotorReversed 
            #     else phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            # )
            # drive_gear_ratio_config = phoenix6.configs.FeedbackConfigs().with_sensor_to_mechanism_ratio(1/ModuleConstants.kDriveEncoderRot2Meter)

            # Reduce CAN status frame rates before configuring
            # info = self.driveMotor.get_fault_field().get_applied_update_frequency()
            # print("Before change of frame rate - Drive Motor CAN status frame rates: ", info)                  # It does not print!!! Why?
            # self.driveMotor.get_fault_field().set_update_frequency(frequency_hz=4, timeout_seconds=0.01)
            # info2 = self.driveMotor.get_fault_field().get_applied_update_frequency()
            # print("After change of frame rate - Drive Motor CAN status frame rates: ", info2)

            # PID for drive forward using TalonFX instead of Roborio     
            # self.drive_pid = phoenix6.configs.Slot0Configs().with_k_p(drivePIDk[0]).with_k_i(drivePIDk[1]).with_k_d(drivePIDk[2])
            # self.drive_ff = SimpleMotorFeedforwardMeters(kS=drivePIDk[3], kV=drivePIDk[4], kA=drivePIDk[5])
            
            # create a velocity closed-loop request, voltage output, slot 0 configs for the drive motor
            self.driveMotorRequest = phoenix6.controls.VelocityVoltage(0).with_slot(0)

            # Apply the configurations to the drive motor controllers
            # drive_config.apply(driveMotorConfig)
            # drive_config.apply(self.drive_pid, 0.01)
            # drive_config.apply(drive_gear_ratio_config)

            # self.driveMotor.get_fault_field().set_update_frequency(info)
            # info = self.driveMotor.get_fault_field().get_applied_update_frequency()
            # print("After update - Drive Motor CAN status frame rates: ", info)

            ##############################################################################################################

            # PID Controllers for turning and driving on Roborio
            # self.turningPIDController = PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning)
            # self.turningPIDController.enableContinuousInput(-pi, pi)
            # self.turningPIDController.setTolerance(0.0001, 0.01)

         
            # Same PID but executed by SparkMax instead of Roborio
            # Let's try both and see which one works better
            self.RevController = self.turningMotor.getClosedLoopController()
            # self.RevController.setP(ModuleConstants.kPTurning)
            # self.RevController.setI(0)
            # self.RevController.setD(ModuleConstants.kDTurning)   
            # self.RevController.setPositionPIDWrappingMinInput(-pi) # Wrapping is the same as Continuous Input
            # self.RevController.setPositionPIDWrappingMaxInput(pi)
            # self.RevController.setPositionPIDWrappingEnabled(True)


            # PID Controller for driving controlled by Roborio
            # self.drivePIDController = PIDController(drivePIDk[0], drivePIDk[1], drivePIDk[2])

            # The API documentation for Python feedforward components indicate which unit is being used as wpimath.units.NAME. 
            # Users must take care to use correct units, as Python does not have a type-safe unit system.
            # self.driveFeedbackForward = SimpleMotorFeedforwardMeters(drivePIDk[3], drivePIDk[4], drivePIDk[5])

            # Set the drive motor to 0 and steer direction encoder to absolute encoder
            self.resetEncoders()

    def getDrivePosition(self):
        ''' Returns the position of the drive motor in meters '''
        return self.driveMotor.get_position().value
    
    def getTurningPosition(self):
        ''' Returns the position of the turning motor in radians '''
        return self.turningEncoder.getPosition()
    
    def getTurningPositionClose(self):
        ''' Returns the position of the turning motor in radians between -pi and pi '''
        angle = self.getTurningPosition()
        angle = angle % (2*pi)
        if angle > pi:
            angle = angle - 2*pi
        return angle
    
    def getDriveVelocity(self): # CHECK IF IT RETURNS THE RIGHT VALUE
        ''' Returns the velocity of the drive motor in meters per second '''
        return self.driveMotor.get_velocity().value # * ModuleConstants.kDriveEncoderRPM2MeterPerSec
    
    def getTurningVelocity(self):
        ''' Returns the velocity of the turning motor in radians per second '''
        return self.turningEncoder.getVelocity()

    def getAbsoluteEncoderRad(self):
        ''' Returns the absolute position of the turning motor in radians (0-2pi) '''
        angle = self.absoluteEncoder.get_absolute_position().value # Output is from -0.5 to 0.5
        angle = angle * 2*pi # Output is from -pi to pi
        angle-=self.absoluteEncoderOffsetRad # Offset
        if angle > pi: # Makes the output be between -pi and pi
            angle -= 2*pi
        if angle < -pi:
            angle += 2*pi  
        direction = -1.0 if self.absoluteEncoderReversed else 1.0 # Reverses the output if needed
        return angle*direction

    def resetEncoders(self):
        ''' Resets all encoders '''
        self.driveMotor.set_position(0)
        self.turningEncoder.setPosition(self.getAbsoluteEncoderRad())

    def getState(self):
        ''' Returns the state of the swerve module '''
        return SwerveModuleState(self.getDriveVelocity(), Rotation2d(self.getTurningPosition()))
    
    def getSwerveModulePosition(self):
        ''' Returns the position of the swerve module '''
        return wpimath.kinematics.SwerveModulePosition(self.getDrivePosition(), Rotation2d(self.getTurningPosition()))

    def stop(self):
        ''' Stops the swerve module '''
        self.driveMotor.set_control(phoenix6.controls.DutyCycleOut(0))
        self.turningMotor.set(0)

    def setDesiredState(self, state: SwerveModuleState):
        ''' Sets the desired state of the swerve module '''
        # If the speed is less than 0.001, stop the module
        if (abs(state.speed)<0.001):
            self.stop()
            return 0 # Exit the function
        
        # Optimize the state to turn at the most 90 degrees
        # SwerveModuleState.optimize(state, self.getState().angle)
        state.optimize(Rotation2d(self.getTurningPosition()))
        
        # Calculate the drive output using the PID controller and the feedforward
        # driveOutput = self.drivePIDController.calculate(self.getDriveVelocity(),state.speed)
        # driveFeedForward = self.driveFeedbackForward.calculate(state.speed)
        # #self.driveMotor.set_control(phoenix6.controls.DutyCycleOut(state.speed/DriveConstants.kPhysicalMaxSpeedMetersPerSecond))
        # self.driveMotor.set_control(phoenix6.controls.DutyCycleOut(driveOutput+driveFeedForward))
        rotationPerSecond = state.speed/(pi*ModuleConstants.kWheelDiameterMeters)
        #print(f"Drive Motor {self.driveMotorID} Speed (m/s): {state.speed} Rotation per second: {rotationPerSecond}")
        self.driveMotor.set_control(self.driveMotorRequest.with_velocity(rotationPerSecond))
        # print(f"Drive Motor {self.driveMotorID} Request: {self.driveMotorRequest.with_velocity(state.speed)}")

        # Calculate the turning output using the PID controller
        # outputTurn = self.turningPIDController.calculate(self.getTurningPosition(), state.angle.radians())
        # self.turningMotor.set(outputTurn)
        self.RevController.setReference(state.angle.radians(), SparkLowLevel.ControlType.kPosition)
        
    # def setTurningPID(self, P, I, D):
    #     ''' Sets the PID values for the turning motor - for tuning purposes'''
    #     self.turningPIDController.setP(P)
    #     self.turningPIDController.setI(I)
    #     self.turningPIDController.setD(D)

    # def getTurningPID(self):
    #     ''' Returns the PID values for the turning motor - for tuning purposes '''
    #     return (self.turningPIDController.getP(), self.turningPIDController.getI(), self.turningPIDController.getD())
    
    # def setSetTurningPosition(self, angle):
    #     ''' Sets the desired turning position to angle in radians - Used to tuned up PID'''
        # outputTurn = self.turningPIDController.calculate(self.getTurningPosition(), angle)
        # self.turningMotor.set(outputTurn)
        # self.RevController.setReference(angle, rev.CANSparkLowLevel.ControlType.kPosition)