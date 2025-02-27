from commands2 import Subsystem

from constants import ElevatorConstants
from rev import SparkMax, SparkLowLevel, SparkMaxConfig, SparkBaseConfig
import rev

from wpimath.units import volts
from wpilib.sysid import SysIdRoutineLog
from commands2.sysid import SysIdRoutine

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

        # Encoder configuration for position and velocity
        configRevMotor.encoder.positionConversionFactor(ElevatorConstants.kElevatorEncoderRot2Meter)
        configRevMotor.encoder.velocityConversionFactor(ElevatorConstants.kElevatorEncoderRPM2MeterPerSec)

        # Sends the configuration to the motor
        self.elevatorMotor1.configure(configRevMotor,resetMode,persistMode)

        # Motor2
        configRevMotor = SparkMaxConfig()  # Creates a new SparkMaxConfig object
        resetMode = rev.SparkBase.ResetMode(0) # Reset mode is set to Not Reset before Config
        persistMode = rev.SparkBase.PersistMode(1) # Persist mode is set to Save In Lasting Memory
        configRevMotor.inverted(ElevatorConstants.ElevatorReversed2) # Inverts the motor if needed
        configRevMotor.setIdleMode(SparkBaseConfig.IdleMode.kBrake) # Sets the idle mode to brake
        configRevMotor.follow(self.elevatorMotor1, ElevatorConstants.FollowerReversed)

        # Encoder configuration for position and velocity
        configRevMotor.encoder.positionConversionFactor(ElevatorConstants.kElevatorEncoderRot2Meter)
        configRevMotor.encoder.velocityConversionFactor(ElevatorConstants.kElevatorEncoderRPM2MeterPerSec)

        # Sends the configuration to the motor
        self.elevatorMotor2.configure(configRevMotor,resetMode,persistMode)

        # Init of Encoder to rotate wheel (on the NEO)
        self.elevatorEncoder = self.elevatorMotor1.getEncoder()

        # Reset encoder
        self.resetEncoder()

        # SYSID CODE
        def sysidElevator(voltage: volts) -> None:
            ''' Drive to tune up drive system with SysId '''
            self.elevatorMotor1.setVoltage(voltage)

        SysConfig = SysIdRoutine.Config(
            # This is the function that will be called to set the mechanism to a given state
            rampRate=volts(.5),
            stepVoltage=volts(4.0),
            timeout=10.0,
        )

        SysMechanism = SysIdRoutine.Mechanism(
            sysidElevator, 
            self.log, 
            self
        )

        self.sys_id_routine = SysIdRoutine(
            SysConfig,
            SysMechanism
        )


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

    def sysIdQuasistatic(self, direction: SysIdRoutine.Direction):
        ''' Run the SysId routine in quasistatic mode ''' 
        return self.sys_id_routine.quasistatic(direction)

    def sysIdDynamic(self, direction: SysIdRoutine.Direction):
        ''' Run the SysId routine in dynamic mode '''
        return self.sys_id_routine.dynamic(direction)
    
    def log(self, sys_id_routine: SysIdRoutineLog) -> None:
        # Record a frame for the left motors.  Since these share an encoder, we consider
        # the entire group to be one motor.
                sys_id_routine.motor("Elevator1"
            ).voltage(self.elevatorMotor1.getBusVoltage()*self.elevatorMotor1.getAppliedOutput()
            ).position(self.elevatorMotor1.getEncoder().getPosition()
            ).velocity(self.elevatorMotor1.getEncoder().getVelocity())
                print(f"Voltage: {self.elevatorMotor1.getBusVoltage()*self.elevatorMotor1.getAppliedOutput()} - position: {self.elevatorMotor1.getEncoder().getPosition()} - velocity: {self.elevatorMotor1.getEncoder().getVelocity()}")
