########################################################################
# Class: SwerveModule
# Purpose: This class is used to control a single swerve module
########################################################################

# Imports
import wpilib
from math import pi

from constants import ModuleConstants, DriveConstants

from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d
import wpimath.kinematics

import phoenix6.hardware
import phoenix6
from phoenix6 import configs
from rev import SparkMaxConfig, SparkMax, SparkLowLevel, SparkBaseConfig
import rev

# Class: SwerveModule
class SwerveModule:

    def __init__(self, driveMotorID, turningMotorID, driveMotorReversed, turningMotorReversed,
        absoluteEncoderID, absoluteEncoderOffset, absoluteEncoderReversed, drivePIDk):
            ''' Constructor for SwerveModule '''

            self.absoluteEncoderOffsetRad = absoluteEncoderOffset
            self.absoluteEncoderReversed = absoluteEncoderReversed
            self.driveMotorID = driveMotorID
            
            # Init of Absolute Encoder
            self.absoluteEncoderOffsetRad = absoluteEncoderOffset
            self.absoluteEncoderReversed = absoluteEncoderReversed
            self.absoluteEncoder = wpilib.AnalogInput(absoluteEncoderID)

            # Init of Drive Motor (TalonFX) for Kraken X.60
            self.driveMotor = phoenix6.hardware.TalonFX(driveMotorID)

            # Init of Turning Motor (SparkMax) for NEO v1.1
            self.turningMotor = SparkMax(turningMotorID, SparkLowLevel.MotorType.kBrushless)

            ##############################################################################################################
            # Config SparkMax for turning motor 
            configRevMotor = SparkMaxConfig()  # Creates a new SparkMaxConfig object
            resetMode = rev.SparkBase.ResetMode(0) # Reset mode is set to Not Reset before Config
            persistMode = rev.SparkBase.PersistMode(1) # Persist mode is set to Save In Lasting Memory
            configRevMotor.inverted(turningMotorReversed) # Inverts the motor if needed
            configRevMotor.setIdleMode(SparkBaseConfig.IdleMode.kBrake) # Sets the idle mode to brake
            # Encoder configuration for position and velocity
            configRevMotor.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
            configRevMotor.encoder.velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec)
            # PID configuration for position control
            configRevMotor.closedLoop.pid(0.6, 0, 0.02, slot=rev.ClosedLoopSlot.kSlot0) # P, I, D
            configRevMotor.closedLoop.positionWrappingEnabled(True) # Wraps the position input
            configRevMotor.closedLoop.positionWrappingInputRange(-pi, pi) # Sets the input range for the position
            # Sends the configuration to the motor
            self.turningMotor.configure(configRevMotor,resetMode,persistMode)

            # PID Controller for turning controlled by SparkMax
            self.RevController = self.turningMotor.getClosedLoopController()

            # Init of Encoder to rotate wheel (on the NEO)
            self.turningEncoder = self.turningMotor.getEncoder()

            ##############################################################################################################
            # Encoder Kraken X.60
            # Encoder is included in TalonFX so no need to initialize it
            # Configure drive motor
            # slot0_configs = configs.Slot0Configs()

            # # PID for drive forward using TalonFX instead of Roborio
            # slot0_configs.k_s = drivePIDk[3] # Add 0.1 V output to overcome static friction
            # slot0_configs.k_v = drivePIDk[4] # A velocity target of 1 rps results in 0.12 V output
            # slot0_configs.k_a = drivePIDk[5] # A velocity target of 1 rps results in 0.12 V output
            # slot0_configs.k_p = drivePIDk[0] # An error of 1 rps results in 0.11 V output
            # slot0_configs.k_i = 0 # no output for integrated error
            # slot0_configs.k_d = 0 # no output for error derivative
            # self.driveMotor.configurator.apply(slot0_configs)
           
            # create a velocity closed-loop request, voltage output, slot 0 configs for the drive motor
            # self.driveMotorRequest = phoenix6.controls.VelocityVoltage(0).with_slot(0)

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

    def getAbsoluteEncoder(self):
        ''' Returns the absolute position of the turning motor in volts '''
        return self.absoluteEncoder.getValue() *2*pi/4095

    def getAbsoluteEncoderRad(self):
        ''' Returns the absolute position of the turning motor in radians (0-2pi) '''
        angleVolts = self.absoluteEncoder.getValue() # Output is 12 bits integer representing voltage 0-5v
        angle = 2*pi*angleVolts/4095 - pi# Converting the voltage to radians
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

    def getSwerveModulePosition(self):
        ''' Returns the position of the swerve module '''
        return wpimath.kinematics.SwerveModulePosition(self.getDrivePosition(), Rotation2d(self.getTurningPosition()))


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
        state.optimize(Rotation2d(self.getTurningPosition()))
        
        # Calculate the rotation per second
        rotationPerSecond = state.speed/(pi*ModuleConstants.kWheelDiameterMeters)

        # Set the drive motor to the desired speed
        #self.driveMotor.set_control(self.driveMotorRequest.with_velocity(rotationPerSecond))
        
        # Calculate the turning output using the PID controller
        #self.RevController.setReference(state.angle.radians(), SparkLowLevel.ControlType.kPosition, slot=rev.ClosedLoopSlot.kSlot0)

    def setSetTurningPosition(self, angle):
        ''' Sets the desired turning position to angle in radians - Used to tuned up PID'''
        # outputTurn = self.turningPIDController.calculate(self.getTurningPosition(), angle)
        # self.turningMotor.set(outputTurn)
        self.RevController.setReference(angle, SparkLowLevel.ControlType.kPosition)