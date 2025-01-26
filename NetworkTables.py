import subsystems.SwerveSubsystem
from subsystems.SwerveSubsystem import SwerveSubsystem
import robotcontainer
import wpilib
from constants import ModuleConstants, OIConstants, AutoConstants, DriveConstants
from robotcontainer import RobotContainer
import ntcore
import robotpy_apriltag
from cscore import CameraServer
import cv2
import numpy as np

class NetworkTables:
    def __init__(self, robotContainer: RobotContainer):
        self.container = robotContainer
        # wpilib.SmartDashboard.putNumber("Turning P", 0.2)
        
    

    def updateShuffleboard(self):
        """Updates shuffleboard values"""
        ##### Examples of how to use the Smart Dashboard
        # wpilib.SmartDashboard.putNumber("DPosSensor", self.container.swerveSubsystem.getDistanceSensorPos())
        # wpilib.SmartDashboard.putBoolean("Intake Limit Switch", self.container.intake.intakeLimitSwitch.get())
        # wpilib.SmartDashboard.putData("Auto Choices", self.container.sendableChooser)
        # ModuleConstants.kPTurning = wpilib.SmartDashboard.getNumber("Turning P", 0.2)
        # Σ = "sigma grindset".capitalize()

        # Reads all absolute encoders and push values to Smart Dashboard
        # absoluteEncoder = self.container.swerveSubsystem.readAbsEncoders()
        # wpilib.SmartDashboard.putNumber("AbsEnc FL", absoluteEncoder[0])
        # wpilib.SmartDashboard.putNumber("AbsEnc FR", absoluteEncoder[1])
        # wpilib.SmartDashboard.putNumber("AbsEnc BL", absoluteEncoder[2])
        # wpilib.SmartDashboard.putNumber("AbsEnc BR", absoluteEncoder[3])
        # wpilib.SmartDashboard.putNumber("X axis", self.container.driverController.getRawAxis(OIConstants.kDriverXAxis))
        # wpilib.SmartDashboard.putNumber("Y axis", self.container.driverController.getRawAxis(OIConstants.kDriverYAxis))
        # wpilib.SmartDashboard.putNumber("Rot axis", self.container.driverController.getRawAxis(OIConstants.kDriverRotAxis))
        
        # Reads Absolute Encoders and sends them to Dashboard
        # absoluteEncoder = self.container.swerveSubsystem.readAbsEncoders()
        # wpilib.SmartDashboard.putNumber("AbsEnc FL", absoluteEncoder[0])
        # wpilib.SmartDashboard.putNumber("AbsEnc FR", absoluteEncoder[1])
        # wpilib.SmartDashboard.putNumber("AbsEnc BL", absoluteEncoder[2])
        # wpilib.SmartDashboard.putNumber("AbsEnc BR", absoluteEncoder[3])
        
        # # Reads Encoders and sends them to Dashboard
        # turnings = self.container.swerveSubsystem.readTurnEncoders()
        # wpilib.SmartDashboard.putNumber("Turning FL", turnings[0])
        # wpilib.SmartDashboard.putNumber("Turning FR", turnings[1])
        # wpilib.SmartDashboard.putNumber("Turning BL", turnings[2])
        # wpilib.SmartDashboard.putNumber("Turning BR", turnings[3])

        # # Reads Distance Travelled and sends them to Dashboard
        # forwards = self.container.swerveSubsystem.readForwardEncoders()
        # wpilib.SmartDashboard.putNumber("Forward FL", forwards[0])
        # wpilib.SmartDashboard.putNumber("Forward FR", forwards[1])
        # wpilib.SmartDashboard.putNumber("Forward BL", forwards[2])
        # wpilib.SmartDashboard.putNumber("Forward BR", forwards[3])

        # Reads the velocity of the robot and the dedesired velocity
        # velocity = self.container.swerveSubsystem.getVelocity()
        # desiredVelocity = self.container.swerveSubsystem.getDesiredVelocity()
        # errorVelocity = desiredVelocity - velocity
        # wpilib.SmartDashboard.putNumber("Velocity", velocity)
        # wpilib.SmartDashboard.putNumber("Desired Velocity", desiredVelocity)
        # wpilib.SmartDashboard.putNumber("Velocity Error", errorVelocity)

        wpilib.SmartDashboard.putNumber("Angle Gyro", self.container.swerveSubsystem.getHeading())
        wpilib.SmartDashboard.putNumber("Elevator Encoder", self.container.elevator.readEncoder())
        pass

