from commands2 import Subsystem

#from libgrapplefrc import LaserCAN
from constants import DropperConstants
from rev import SparkMax, SparkLowLevel, SparkMaxConfig, SparkBaseConfig
import rev


class Dropper(Subsystem):
    def __init__(self):
        Subsystem.__init__(self)

        # Init of Turning Motor (SparkMax) for NEO v1.1
        self.dropperMotor = SparkMax(DropperConstants.DropperMotorID, SparkLowLevel.MotorType.kBrushless)

         ##############################################################################################################
        # Config SparkMax for turning motor 
        configRevMotor = SparkMaxConfig()  # Creates a new SparkMaxConfig object
        resetMode = rev.SparkBase.ResetMode(0) # Reset mode is set to Not Reset before Config
        persistMode = rev.SparkBase.PersistMode(1) # Persist mode is set to Save In Lasting Memory
        configRevMotor.inverted(DropperConstants.DropperReversed) # Inverts the motor if needed
        configRevMotor.setIdleMode(SparkBaseConfig.IdleMode.kBrake) # Sets the idle mode to brake
        # PID configuration for position control
        configRevMotor.closedLoop.pid(
            DropperConstants.P, 
            DropperConstants.I, 
            DropperConstants.D, 
            slot=rev.ClosedLoopSlot.kSlot0)
        # Sends the configuration to the motor
        self.dropperMotor.configure(configRevMotor,resetMode,persistMode)

        # Init of Encoder to rotate wheel (on the NEO)
        self.dropperEncoder = self.dropperMotor.getEncoder()

        # PID Controller for turning controlled by SparkMax
        self.RevController = self.dropperMotor.getClosedLoopController()

        # Reset encoder
        self.resetEncoder()

    def resetEncoder(self):
        ''' Resets the encoder position '''
        self.dropperEncoder.setPosition(0)

    def readEncoder(self):
        ''' Reads the encoder position '''
        return self.dropperEncoder.getPosition()
    
    def readEncoderVelocity(self):
        ''' Reads the encoder velocity '''
        return self.dropperEncoder.getVelocity()
    
    def setMotor(self, speed):
        ''' Sets the motor speed '''
        if speed > 1:
            speed = 1
        elif speed < -1:
            speed = -1
        self.dropperMotor.set(speed)

    def stopMotor(self):
        ''' Stops the motor '''
        self.dropperMotor.stopMotor()

    def setDropperVelocity(self, speed):
        ''' Sets the motor speed using PID controller'''
        # Calculate the turning output using the PID controller
        self.RevController.setReference(speed, SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0)


