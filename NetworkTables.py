import subsystems.SwerveSubsystem
from subsystems.SwerveSubsystem import SwerveSubsystem
import robotcontainer
import wpilib
from wpilib import Field2d
from constants import ModuleConstants, OIConstants, AutoConstants, DriveConstants
from robotcontainer import RobotContainer
import ntcore
import robotpy_apriltag
from cscore import CameraServer
import cv2
import numpy as np
from math import pi

class NetworkTables:
    def __init__(self, robotContainer: RobotContainer):
        self.container = robotContainer
        
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
        
        # Reads Distance Travelled and sends them to Dashboard
        forwards = self.container.swerveSubsystem.readForwardEncoders()
        wpilib.SmartDashboard.putNumber("Forward FL", forwards[0])
        wpilib.SmartDashboard.putNumber("Forward FR", forwards[1])
        wpilib.SmartDashboard.putNumber("Forward BL", forwards[2])
        wpilib.SmartDashboard.putNumber("Forward BR", forwards[3])

        # Reads the velocity of the robot and the dedesired velocity
        # velocity = self.container.swerveSubsystem.getVelocity()
        # desiredVelocity = self.container.swerveSubsystem.getDesiredVelocity()
        # errorVelocity = desiredVelocity - velocity
        # wpilib.SmartDashboard.putNumber("Velocity", velocity)
        # wpilib.SmartDashboard.putNumber("Desired Velocity", desiredVelocity)
        # wpilib.SmartDashboard.putNumber("Velocity Error", errorVelocity)

        wpilib.SmartDashboard.putNumber("Angle Gyro", self.container.swerveSubsystem.getHeading())
        # wpilib.SmartDashboard.putNumber("Angle Gyro (radians)", self.container.swerveSubsystem.getHeading()*pi/180)

        # Elevator
        wpilib.SmartDashboard.putNumber("Elevator Encoder", self.container.elevator.readEncoder())
        wpilib.SmartDashboard.putNumber("Elevator floor", self.container.elevator.readFloor())
        wpilib.SmartDashboard.putBoolean("Elevator Limit Switch", self.container.elevator.lowerSwitchOn())

        # Algae Collector
        wpilib.SmartDashboard.putNumber("Arm Height", self.container.algaeC.readArmEncoder())
        wpilib.SmartDashboard.putBoolean("Arm Up", self.container.algaeC.isArmUp())

        # Dropper
        wpilib.SmartDashboard.putNumber("Top sensor", self.container.dropper.getLaserTop())
        wpilib.SmartDashboard.putNumber("Bottom sensor", self.container.dropper.getLaserBottom())
        wpilib.SmartDashboard.putBoolean("Coral Top", self.container.dropper.is_coral_top())
        wpilib.SmartDashboard.putBoolean("Coral Bottom", self.container.dropper.is_coral_bottom())

        # Drive system
        # Reads Encoders and sends them to Dashboard
        # turnings = self.container.swerveSubsystem.readTurnEncoders()
        # wpilib.SmartDashboard.putNumber("Turning FL", turnings[0]*180/pi)
        # wpilib.SmartDashboard.putNumber("Turning FR", turnings[1]*180/pi)
        # wpilib.SmartDashboard.putNumber("Turning BL", turnings[2]*180/pi)
        # wpilib.SmartDashboard.putNumber("Turning BR", turnings[3]*180/pi)

        # DesiredStates = self.container.swerveSubsystem.getDesiredModuleStates()
        # wpilib.SmartDashboard.putNumber("Desired FL", DesiredStates[0].angle.degrees())
        # wpilib.SmartDashboard.putNumber("Desired FR", DesiredStates[1].angle.degrees())
        # wpilib.SmartDashboard.putNumber("Desired BL", DesiredStates[2].angle.degrees())
        # wpilib.SmartDashboard.putNumber("Desired BR", DesiredStates[3].angle.degrees())

        # Pose SmartDashBoard
        field = Field2d()
        poseEstimator = self.container.swerveSubsystem.getPoseEstimator()
        field.setRobotPose(poseEstimator)
        wpilib.SmartDashboard.putData("poseEstimator", field)
        wpilib.SmartDashboard.putNumber("Estimator X", poseEstimator.X())
        wpilib.SmartDashboard.putNumber("Estimator Y", poseEstimator.Y())
        angle = poseEstimator.rotation().degrees()
        wpilib.SmartDashboard.putNumber("Est Theta", angle)

        odometer = self.container.swerveSubsystem.getPose()
        # field.setRobotPose(odometer)
        # wpilib.SmartDashboard.putNumberArray("odometer", odometer)
        wpilib.SmartDashboard.putNumber("Odometer X", odometer.X())
        wpilib.SmartDashboard.putNumber("Odometer Y", odometer.Y())
        angle = odometer.rotation().degrees()
        wpilib.SmartDashboard.putNumber("Odo Theta", angle)



