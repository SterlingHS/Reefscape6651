from commands2 import Subsystem

from constants import AlgaeCollectorConstants
from rev import SparkMax, SparkLowLevel, SparkMaxConfig, SparkBaseConfig
import rev


class AlgaeCollector(Subsystem):
    def __init__(self):
        Subsystem.__init__(self)

        # Init of Motors for subsystem (SparkMax) for NEO v1.1
        self.ACStarMotor = SparkMax(AlgaeCollectorConstants.ACStarMotorID, SparkLowLevel.MotorType.kBrushless)
        self.ACArmMotor = SparkMax(AlgaeCollectorConstants.ACArmMotorID, SparkLowLevel.MotorType.kBrushless)

        ##############################################################################################################
        # Config SparkMax for ACStarMotor
        configRevMotor = SparkMaxConfig()  # Creates a new SparkMaxConfig object
        resetMode = rev.SparkBase.ResetMode(0) # Reset mode is set to Not Reset before Config
        persistMode = rev.SparkBase.PersistMode(1) # Persist mode is set to Save In Lasting Memory
        configRevMotor.inverted(AlgaeCollectorConstants.ACStarReversed) # Inverts the motor if needed
        configRevMotor.setIdleMode(SparkBaseConfig.IdleMode.kBrake) # Sets the idle mode to brake

        # Sends the configuration to the motor
        self.ACStarMotor.configure(configRevMotor,resetMode,persistMode)

        ##############################################################################################################
        # Config SparkMax for ACArmMotor
        configRevMotor = SparkMaxConfig()  # Creates a new SparkMaxConfig object
        resetMode = rev.SparkBase.ResetMode(0) # Reset mode is set to Not Reset before Config
        persistMode = rev.SparkBase.PersistMode(1) # Persist mode is set to Save In Lasting Memory
        configRevMotor.inverted(AlgaeCollectorConstants.ACArmReversed) # Inverts the motor if needed
        configRevMotor.setIdleMode(SparkBaseConfig.IdleMode.kBrake) # Sets the idle mode to brake

        # Soft Limits
        configRevMotor.softLimit.forwardSoftLimit(AlgaeCollectorConstants.Max)
        configRevMotor.softLimit.reverseSoftLimit(AlgaeCollectorConstants.Min)
        configRevMotor.softLimit.forwardSoftLimitEnabled(True)
        configRevMotor.softLimit.reverseSoftLimitEnabled(True)
        configRevMotor.limitSwitch.reverseLimitSwitchEnabled(True)
        configRevMotor.limitSwitch.reverseLimitSwitchType(rev.LimitSwitchConfig.Type.kNormallyClosed)
        configRevMotor.limitSwitch.forwardLimitSwitchEnabled(True)
        configRevMotor.limitSwitch.forwardLimitSwitchType(rev.LimitSwitchConfig.Type.kNormallyClosed)


        # PID configuration for position control
        configRevMotor.closedLoop.pid(
            AlgaeCollectorConstants.P, 
            AlgaeCollectorConstants.I, 
            AlgaeCollectorConstants.D, 
            slot=rev.ClosedLoopSlot.kSlot0)

        # Sends the configuration to the motor
        self.ACStarMotor.configure(configRevMotor,resetMode,persistMode)

        # Init of Encoder to rotate wheel (on the NEO)
        self.ArmEncoder = self.ACArmMotor.getEncoder()

        # PID Controller for turning controlled by SparkMax
        self.ArmPIDController = self.ACArmMotor.getClosedLoopController()

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
        self.ACArmMotor.set(speed)

    def setStarMotor(self, speed):
        ''' Sets the motor speed for the Star'''
        if speed > 1:
            speed = 1
        elif speed < -1:
            speed = -1
        self.ACStarMotor.set(speed)

    def stopStarMotor(self):
        ''' Stops the motor '''
        self.ACStarMotor.stopMotor()

    def stopArmMotor(self):
        ''' Stops the motor '''
        self.ACArmMotor.stopMotor()

    def setArmHeight(self, height):
        ''' Sets the height of the Arm'''
        self.height = height

    def setArmPosition(self, position):
        ''' Sets the motor speed using PID controller'''
        # Calculate the turning output using the PID controller
        self.ArmPIDController.setReference(position, SparkLowLevel.ControlType.kPosition, slot=rev.ClosedLoopSlot.kSlot0)

    def toggleStar(self):
        ''' Toggles the Star'''
        if self.starSpeed == 0:
            self.starSpeed = AlgaeCollectorConstants.starSpeed
        else:
            self.starSpeed = 0

    def setStarSpeed(self, speed):
        ''' Sets the speed of the Star'''
        self.starSpeed = speed

    def setMode(self, mode):
        ''' Sets the mode of the Arm'''
        self.mode = mode

    def getMode(self):
        ''' Gets the mode of the Arm'''
        return self.mode

    def isArmUp(self):
        ''' Returns True if the Arm is up'''
        return self.ACArmMotor.getForwardLimitSwitch()

    def periodic(self):
        ''' Runs every loop '''
        if self.isArmUp():
            self.resetArmEncoder()
            if self.height == 1:
                self.height = 0
        self.setArmPosition(self.height)
        self.setStarMotor(self.starSpeed)
        return super().periodic()
