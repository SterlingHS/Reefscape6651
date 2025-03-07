from commands2 import Subsystem
import robotpy 

from rev import SparkMax, SparkLowLevel, SparkMaxConfig, SparkBaseConfig
import rev
from libgrapplefrc import LaserCAN

from constants import DropperConstants

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
        configRevMotor.closedLoop.pidf(
            DropperConstants.P, 
            DropperConstants.I, 
            DropperConstants.D,
            DropperConstants.F,
            slot=rev.ClosedLoopSlot.kSlot0)
        #configRevMotor.closedLoop.maxMotion.maxVelocity(maxVelocity=4000, slot=rev.ClosedLoopSlot.kSlot0)
        #configRevMotor.closedLoop.maxMotion.maxAcceleration(maxAcceleration=8000, slot=rev.ClosedLoopSlot.kSlot0)
        # Sends the configuration to the motor
        self.dropperMotor.configure(configRevMotor,resetMode,persistMode)

        # Init of Encoder to rotate wheel (on the NEO)
        self.dropperEncoder = self.dropperMotor.getEncoder()

        # PID Controller for turning controlled by SparkMax
        self.RevController = self.dropperMotor.getClosedLoopController()

        # Reset encoder
        self.resetEncoder()

        # Init lasercan 
        self.lasercanTop = LaserCAN(DropperConstants.LaserTopCanID)
        self.lasercanBottom = LaserCAN(DropperConstants.LaserBottomCanID)

    def getLaserTop(self):
        ''' Returns the distance from the laser on the top '''
        return self.lasercanTop.get_measurement()
    
    def getLaserBottom(self):
        ''' Returns the distance from the laser on the bottom '''
        return self.lasercanBottom.get_measurement()
    
    def is_coral_top(self):
        ''' Returns true when coral is detect on the top of the dropper. '''
        return self.getLaserTop() < 100
    
    def is_coral_bottom(self):
        ''' Returns true when coral is detect on the bottom of the dropper. '''
        return self.getLaserBottom() < 100    
    
    def is_Coral_ready(self):
        ''' Returns true if coral is ready to be dropped '''
        return not self.is_coral_top and self.is_coral_bottom
     
    def is_no_Coral_dropper(self):
        ''' Returns true if no coral in dropper '''
        return not self.is_coral_top and not self.is_coral_bottom

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
        # filter to provide speed between 1 and -1
        speed = max(-1, min(1, speed))
        self.dropperMotor.set(speed)

    def stopMotor(self):
        ''' Stops the motor '''
        self.dropperMotor.stopMotor()

    def setDropperVelocity(self, speed):
        ''' Sets the motor speed using PID controller'''
        # Calculate the turning output using the PID controller
        self.RevController.setReference(value=speed, ctrl=SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0)

    def setDropperVelocityMax(self, speed):
        ''' Sets the motor speed using PID controller using MAXMotion'''
        # Calculate the turning output using the PID controller
        self.RevController.setReference(value=speed, ctrl=SparkLowLevel.ControlType.kMAXMotionVelocityControl, slot=rev.ClosedLoopSlot.kSlot0)



