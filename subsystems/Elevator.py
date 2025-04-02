from commands2 import Subsystem

from wpimath.trajectory import TrapezoidProfile
from wpimath.controller import SimpleMotorFeedforwardMeters
from constants import ElevatorConstants
from rev import SparkMax, SparkLowLevel, SparkMaxConfig, SparkBaseConfig
import rev
import wpilib


class Elevator(Subsystem):
    def __init__(self):
        Subsystem.__init__(self)

        # Init of Turning Motor (SparkMax) for NEO v1.1
        self.elevatorMotor1 = SparkMax(ElevatorConstants.ElevatorMotorID1, SparkLowLevel.MotorType.kBrushless)
        self.elevatorMotor2 = SparkMax(ElevatorConstants.ElevatorMotorID2, SparkLowLevel.MotorType.kBrushless)

         ##############################################################################################################
        # Config SparkMax for elevator motor
        # 
        # Motor1 
        configRevMotor = SparkMaxConfig()  # Creates a new SparkMaxConfig object
        resetMode = rev.SparkBase.ResetMode(0) # Reset mode is set to Not Reset before Config
        persistMode = rev.SparkBase.PersistMode(1) # Persist mode is set to Save In Lasting Memory
        configRevMotor.inverted(ElevatorConstants.ElevatorReversed1) # Inverts the motor if needed
        configRevMotor.setIdleMode(SparkBaseConfig.IdleMode.kBrake) # Sets the idle mode to brake
        # PID configuration for position control
        configRevMotor.closedLoop.pidf(
            ElevatorConstants.P0,
            ElevatorConstants.I0,
            ElevatorConstants.D0,
            ElevatorConstants.kF0,
            slot=rev.ClosedLoopSlot.kSlot0
            )
        configRevMotor.closedLoop.pidf(
            ElevatorConstants.P1,
            ElevatorConstants.I1,
            ElevatorConstants.D1,
            ElevatorConstants.kF1,
            slot=rev.ClosedLoopSlot.kSlot1
            )
                 
        # Soft Limits
        configRevMotor.softLimit.forwardSoftLimit(ElevatorConstants.Max)
        configRevMotor.softLimit.reverseSoftLimit(ElevatorConstants.Min)
        configRevMotor.softLimit.forwardSoftLimitEnabled(True)
        configRevMotor.softLimit.reverseSoftLimitEnabled(True)
        configRevMotor.limitSwitch.reverseLimitSwitchEnabled(True)
        configRevMotor.limitSwitch.reverseLimitSwitchType(rev.LimitSwitchConfig.Type.kNormallyClosed)

        # Encoder configuration for position and velocity
        configRevMotor.encoder.positionConversionFactor(ElevatorConstants.kElevatorEncoderRot2Meter)
        configRevMotor.encoder.velocityConversionFactor(ElevatorConstants.kElevatorEncoderRPM2MeterPerSec)

        # Sends the configuration to the motor
        self.elevatorMotor1.configure(configRevMotor,resetMode,persistMode)

        # Motor2
        configRevMotor = SparkMaxConfig()  # Creates a new SparkMaxConfig object
        resetMode = rev.SparkBase.ResetMode(0) # Reset mode is set to Not Reset before Config
        persistMode = rev.SparkBase.PersistMode(1) # Persist mode is set to Save In Lasting Memory
        configRevMotor.inverted(False) # Inverts the motor if needed
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
        self.stopMotorFlag = True

        ##############################################################################
        # Trapezoidal PID to control the elevator
        self.kDt = 0.02 # cycle time: 20ms = .020s
        self.elevatorTrap = TrapezoidProfile(TrapezoidProfile.Constraints(ElevatorConstants.MaxVelocity, 
                                                                          ElevatorConstants.MaxAcceleration))
        self.elevatorFF = SimpleMotorFeedforwardMeters(kS=ElevatorConstants.kS, 
                                                       kV=ElevatorConstants.kV, 
                                                       kA=ElevatorConstants.kA, 
                                                       dt=self.kDt)
        self.elevatorGoal = TrapezoidProfile.State(position=0,velocity=0)
        self.elevatorSetpoint = TrapezoidProfile.State(position=0,velocity=0)

    def lowerSwitchOn(self):
        return self.elevatorMotor1.getReverseLimitSwitch().get()

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

    def setElevatorPosition(self, goal):
        ''' Sets the motor speed using PID controller. Receives position in Inches [0,54]'''
        # Read the state of the elevator
        currentPosition = self.readEncoder()
        # Checks if the elevator is almost at L1 (3 inches above L1)
        if currentPosition < ElevatorConstants.L1+3 and goal == 0 and self.lowerSwitchOn() == False:
            if self.stopMotorFlag == True: # Stop PID by stopping motors
                self.stopMotor()
                self.stopMotorFlag = False
            else:
                self.setMotor(-0.2) # Keep going down until reach limit switch
        elif goal == 0 and self.lowerSwitchOn(): # If the limit switch is pressed, stop the motor
            self.stopMotorFlag = True
            self.stopMotor()
        else:
            # Sets and calculates the elevator position using trapezoidal profile
            self.elevatorGoal=TrapezoidProfile.State(goal,0)
            self.elevatorSetpoint = self.elevatorTrap.calculate(self.kDt, self.elevatorSetpoint, self.elevatorGoal)

            # send info to the PID controller
            self.RevController1.setReference(value=self.elevatorSetpoint.position, 
                                            ctrl=SparkLowLevel.ControlType.kPosition, 
                                            slot=rev.ClosedLoopSlot.kSlot0)

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

    def isReachedFloor(self, floor:int):
        ''' Checks if the elevator is at the desired floor '''
        height = self.readEncoder()
        if floor == 1:
            return height < ElevatorConstants.L1+3
        elif floor == 2:
            return height > ElevatorConstants.L2-2
        elif floor == 3:
            return height > ElevatorConstants.L3-2
        elif floor == 4:
            return height > ElevatorConstants.L4-2

    def periodic(self):
        # Resets the encoder if the lower limit switch is pressed
        if self.lowerSwitchOn():
            self.resetEncoder()
        # Filters the floor to be between 1 and 4
        if self.floor not in [1,2,3,4]:
            self.floor = 1
        # Sets the elevator to the current floor
        self.setElevatorFloor(self.floor)

        return super().periodic()