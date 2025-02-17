from commands2 import Subsystem

from constants import ElevatorConstants
from rev import SparkMax, SparkLowLevel, SparkMaxConfig, SparkBaseConfig
import rev


class Elevator(Subsystem):
    def __init__(self):
        Subsystem.__init__(self)

        # Init of Turning Motor (SparkMax) for NEO v1.1
        self.elevatorMotor1 = SparkMax(ElevatorConstants.ElevatorMotorID1, SparkLowLevel.MotorType.kBrushless)
        self.elevatorMotor2 = SparkMax(ElevatorConstants.ElevatorMotorID2, SparkLowLevel.MotorType.kBrushless)

         ##############################################################################################################
        # Config SparkMax for turning motor
        # 
        # Motor1 
        configRevMotor = SparkMaxConfig()  # Creates a new SparkMaxConfig object
        resetMode = rev.SparkBase.ResetMode(0) # Reset mode is set to Not Reset before Config
        persistMode = rev.SparkBase.PersistMode(1) # Persist mode is set to Save In Lasting Memory
        configRevMotor.inverted(ElevatorConstants.ElevatorReversed1) # Inverts the motor if needed
        configRevMotor.setIdleMode(SparkBaseConfig.IdleMode.kBrake) # Sets the idle mode to brake
        # PID configuration for position control
        configRevMotor.closedLoop.pid(
            ElevatorConstants.P1,
            ElevatorConstants.I1,
            ElevatorConstants.D1,
            slot=rev.ClosedLoopSlot.kSlot0)
        
        # For maxMotion
        # configRevMotor.closedLoop.maxMotion.maxVelocity(ElevatorConstants.MaxRPM,slot=rev.ClosedLoopSlot.kSlot0)
        # configRevMotor.closedLoop.maxMotion.maxAcceleration(ElevatorConstants.MaxAcceleration,slot=rev.ClosedLoopSlot.kSlot0)
        
        configRevMotor.closedLoop.outputRange(ElevatorConstants.MaxVelocityDown,ElevatorConstants.MaxVelocityUp,slot=rev.ClosedLoopSlot.kSlot0)
        
        # Soft Limits
        configRevMotor.softLimit.forwardSoftLimit(ElevatorConstants.Max)
        configRevMotor.softLimit.reverseSoftLimit(ElevatorConstants.Min)
        configRevMotor.softLimit.forwardSoftLimitEnabled(True)
        configRevMotor.softLimit.reverseSoftLimitEnabled(True)

        # Encoder configuration for position and velocity
        configRevMotor.encoder.positionConversionFactor(ElevatorConstants.kElevatorEncoderRot2Meter)
        configRevMotor.encoder.velocityConversionFactor(ElevatorConstants.kElevatorEncoderRPM2MeterPerSec)

        # Sends the configuration to the motor
        self.elevatorMotor1.configure(configRevMotor,resetMode,persistMode)

        # Motor2
        configRevMotor = SparkMaxConfig()  # Creates a new SparkMaxConfig object
        resetMode = rev.SparkBase.ResetMode(0) # Reset mode is set to Not Reset before Config
        persistMode = rev.SparkBase.PersistMode(1) # Persist mode is set to Save In Lasting Memory
        configRevMotor.inverted(ElevatorConstants.ElevatorReversed1) # Inverts the motor if needed
        configRevMotor.setIdleMode(SparkBaseConfig.IdleMode.kBrake) # Sets the idle mode to brake
        configRevMotor.follow(self.elevatorMotor1, ElevatorConstants.ElevatorReversed2)

        # Encoder configuration for position and velocity
        configRevMotor.encoder.positionConversionFactor(ElevatorConstants.kElevatorEncoderRot2Meter)
        configRevMotor.encoder.velocityConversionFactor(ElevatorConstants.kElevatorEncoderRPM2MeterPerSec)

        # Sends the configuration to the motor
        self.elevatorMotor2.configure(configRevMotor,resetMode,persistMode)

        # Init of Encoder to rotate wheel (on the NEO)
        self.elevatorEncoder = self.elevatorMotor1.getEncoder()

        # PID Controller for turning controlled by SparkMax
        self.RevController1 = self.elevatorMotor1.getClosedLoopController()

        # Reset encoder
        self.resetEncoder()

        # Init floor
        self.floor = 1

    def readFloor(self):
        return self.floor

    def resetEncoder(self):
        ''' Resets the encoder position '''
        self.elevatorEncoder.setPosition(0)

    def readEncoder(self):
        ''' Reads the encoder position '''
        return self.elevatorEncoder.getPosition()
    
    def readEncoderVelocity(self):
        ''' Reads the encoder velocity '''
        return self.elevatorEncoder.getVelocity()
    
    def setMotor(self, speed):
        ''' Sets the motor speed '''
        if speed > 1:
            speed = 1
        elif speed < -1:
            speed = -1
        self.elevatorMotor1.set(speed)

    def stopMotor(self):
        ''' Stops the motor '''
        self.elevatorMotor1.stopMotor()

    def setElevatorPosition(self, position):
        ''' Sets the motor speed using PID controller'''
        if position < 0:
            position = 0
        # Calculate the turning output using the PID controller
        self.RevController1.setReference(position, SparkLowLevel.ControlType.kPosition, slot=rev.ClosedLoopSlot.kSlot0)

    def setElevatorVelocity(self, speed):
        ''' Sets the motor speed using PID controller'''
        # Calculate the turning output using the PID controller
        self.RevController1.setReference(speed, SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0)

    def setElevatorFloor(self, floor:int):
        ''' Sets the elevator to a specific floor 
        1: L1 - Bottom
        2: L2
        3: L3
        4: L4 - Top
        '''
        if floor == 1:
            self.setElevatorPosition(ElevatorConstants.L1)
        elif floor == 2:
            self.setElevatorPosition(ElevatorConstants.L2)
        elif floor == 3:
            self.setElevatorPosition(ElevatorConstants.L3)
        elif floor == 4:
            self.setElevatorPosition(ElevatorConstants.L4)

    def changeFloor(self,changeInFloor:int):
        if self.floor >= 2 and changeInFloor == -1:
            self.floor = self.floor - 1
        
        if self.floor <= 3 and changeInFloor == 1:
            self.floor = self.floor + 1

    def setPeriodicFloor(self, floor:int):
        self.floor = floor

    def periodic(self):
        if self.floor in [1,2,3,4]:
            self.setElevatorFloor(self.floor)
        return super().periodic()