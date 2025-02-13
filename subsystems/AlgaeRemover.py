from commands2 import Subsystem

from constants import AlgaeRemoverConstants
from rev import SparkMax, SparkLowLevel, SparkMaxConfig, SparkBaseConfig
import rev

class AlgaeRemover(Subsystem):
    def __init__(self):
        Subsystem.__init__(self)

        # Init of Motors for subsystem (SparkMax) for NEO v1.1
        self.ARStarMotor = SparkMax(AlgaeRemoverConstants.ARStarMotorID, SparkLowLevel.MotorType.kBrushless)
        self.ARArmMotor = SparkMax(AlgaeRemoverConstants.ARArmMotorID, SparkLowLevel.MotorType.kBrushless)

        ##############################################################################################################
        # Config SparkMax for ARStarMotor
        configRevMotor = SparkMaxConfig()  # Creates a new SparkMaxConfig object
        resetMode = rev.SparkBase.ResetMode(0) # Reset mode is set to Not Reset before Config
        persistMode = rev.SparkBase.PersistMode(1) # Persist mode is set to Save In Lasting Memory
        configRevMotor.inverted(AlgaeRemoverConstants.ARStarReversed) # Inverts the motor if needed
        configRevMotor.setIdleMode(SparkBaseConfig.IdleMode.kBrake) # Sets the idle mode to brake

        # Sends the configuration to the motor
        self.ARStarMotor.configure(configRevMotor,resetMode,persistMode)

        ##############################################################################################################
        # Config SparkMax for ARArmMotor
        configRevMotor = SparkMaxConfig()  # Creates a new SparkMaxConfig object
        resetMode = rev.SparkBase.ResetMode(0) # Reset mode is set to Not Reset before Config
        persistMode = rev.SparkBase.PersistMode(1) # Persist mode is set to Save In Lasting Memory
        configRevMotor.inverted(AlgaeRemoverConstants.ARArmReversed) # Inverts the motor if needed
        configRevMotor.setIdleMode(SparkBaseConfig.IdleMode.kBrake) # Sets the idle mode to brake

        # PID configuration for position control
        configRevMotor.closedLoop.pid(
            AlgaeRemoverConstants.P, 
            AlgaeRemoverConstants.I, 
            AlgaeRemoverConstants.D, 
            slot=rev.ClosedLoopSlot.kSlot0)

        # Sends the configuration to the motor
        self.ARStarMotor.configure(configRevMotor,resetMode,persistMode)

        # Init of Encoder to rotate wheel (on the NEO)
        self.ArmEncoder = self.ARArmMotor.getEncoder()

        # PID Controller for turning controlled by SparkMax
        self.ArmPIDController = self.ARArmMotor.getClosedLoopController()

        # Reset encoder
        self.resetArmEncoder()

        # Init Height of the Arm
        self.height = 0

        # Init Speed of the Star
        self.starSpeed = 0
        
    def resetArmEncoder(self):
        ''' Resets the encoder position '''
        self.ArmEncoder.setPosition(0)

    def readArmEncoder(self):
        ''' Reads the encoder position '''
        return self.ArmEncoder.getPosition()
    
    def readArmEncoderVelocity(self):
        ''' Reads the encoder velocity '''
        return self.ArmEncoder.getVelocity()
    
    def setArmMotor(self, speed):
        ''' Sets the motor speed for the Arm'''
        if speed > 1:
            speed = 1
        elif speed < -1:
            speed = -1
        self.ARArmMotor.set(speed)

    def setStarMotor(self, speed):
        ''' Sets the motor speed for the Star'''
        if speed > 1:
            speed = 1
        elif speed < -1:
            speed = -1
        self.ARStarMotor.set(speed)

    def stopStarMotor(self):
        ''' Stops the motor '''
        self.ARStarMotor.stopMotor()

    def stopArmMotor(self):
        ''' Stops the motor '''
        self.ARArmMotor.stopMotor()

    def setArmHeight(self, height):
        ''' Sets the height of the Arm'''
        if height < 0:
            height = 0
        elif height > AlgaeRemoverConstants.Max:
            height = AlgaeRemoverConstants.Max
        else:
            self.height = height

    def setArmPosition(self, position):
        ''' Sets the motor speed using PID controller'''
        # Calculate the turning output using the PID controller
        self.ArmPIDController.setReference(position, SparkLowLevel.ControlType.kPosition, slot=rev.ClosedLoopSlot.kSlot0)

    def toggleStar(self):
        ''' Toggles the Star'''
        if self.starSpeed == 0:
            self.starSpeed = AlgaeRemoverConstants.starSpeed
        else:
            self.starSpeed = 0

    def periodic(self):
        ''' Runs every loop '''
        self.setArmPosition(self.height)
        self.setStarMotor(self.starSpeed)
        return super().periodic()
