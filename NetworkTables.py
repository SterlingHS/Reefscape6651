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
        pass

