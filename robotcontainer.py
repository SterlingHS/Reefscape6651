# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib

import commands2
import commands2.button
from commands2 import waitcommand

from commands.SwerveAutoCmd import SwerveAutoCmd
from commands.SwerveJoystickCmd2 import SwerveJoystickCmd2
from commands.DriveSwitchMode import DriveSwitchMode
from commands.ElevatorChange import ElevatorChange
from commands.ACBallInSetUp import ACBallInSetUp
from commands.ACBallOutSetUp import ACBallOutSetUp
from commands.ACBallCancel import ACBallCancel
from commands.CoralIntake import CoralIntake
from commands.CoralDrop import CoralDrop
from commands.CoralDropAuto import CoralDropAuto
from commands.SwerveToggleTurbo import SwerveToggleTurbo
from commands.GyroReset import GyroReset
from commands.ElevatorFloor import ElevatorFloor

from subsystems.SwerveSubsystem import SwerveSubsystem
from subsystems.Dropper import Dropper
from subsystems.Elevator import Elevator
from subsystems.AlgaeCollector import AlgaeCollector

from constants import DrivingModes, OIConstants
from constants import DropperConstants

from pathplannerlib.auto import PathPlannerAuto, NamedCommands, AutoBuilder

from wpilib import SendableChooser
from wpimath.geometry import Pose2d, Rotation2d
from math import pi

class RobotContainer:
    '''
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    '''

    def __init__(self):
        '''The container for the robot. Contains subsystems, OI devices, and commands.'''
        self.swerveSubsystem = SwerveSubsystem()
        self.dropper = Dropper()
        self.elevator = Elevator()
        self.algaeC = AlgaeCollector()

        # The driver's controller
        self.driverController = wpilib.PS5Controller(OIConstants.kDriverControllerPort)
        # self.codriverController = wpilib.XboxController(OIConstants.kCodriverControllerPort)
  
        self.swerveSubsystem.setDefaultCommand(
           SwerveJoystickCmd2(
               self.swerveSubsystem,
               lambda : self.driverController.getRawAxis(OIConstants.kDriverYAxis),
               lambda : self.driverController.getRawAxis(OIConstants.kDriverXAxis),
               lambda : self.driverController.getRawAxis(OIConstants.kDriverXRotAxis),
               lambda : self.driverController.getRawAxis(OIConstants.kDriverYRotAxis),
               lambda : self.driverController.getSquareButton(),
               lambda : self.driverController.getCircleButton()
           )
        )

        # Build an auto chooser. This will use Commands.none() as the default option.
        # self.autoChooser = AutoBuilder.buildAutoChooser()
        # wpilib.SmartDashboard.putData("Auto Chooser", self.autoChooser)

        autocommand0: commands2.cmd.Command = None

        #Poses for Autonomous
        Pose22L = Pose2d(7.159, 2.809, Rotation2d.fromDegrees(120)) # Pose for Left 22
        Pose17L = Pose2d(3.693, 2.972, Rotation2d.fromDegrees(240)) # Pose for Right 17
        Pose17R = Pose2d(3.975, 2.811, Rotation2d.fromDegrees(240)) # Pose for Left 17
        PoseCoralRight = Pose2d(1.629, 0.683, Rotation2d.fromDegrees(234)) # Pose for Right Coral

        DropAuto = ElevatorFloor(self.elevator, 4).andThen(
            CoralDrop(self.dropper).andThen(
            waitcommand.WaitCommand(.25).andThen(
            ElevatorFloor(self.elevator, 1))))

        IntakeAuto =  CoralIntake(self.dropper, self.elevator)
        WaitSimpleMid = waitcommand.WaitCommand(10).andThen(PathPlannerAuto("SimpleMid"))
        WaitHighMid2019 = waitcommand.WaitCommand(10).andThen(PathPlannerAuto("HighMid-20-19"))
        WaitLowStart2217 = waitcommand.WaitCommand(10).andThen(PathPlannerAuto("LowStart-22-17"))
        SwerveAutoTest = SwerveAutoCmd(self.swerveSubsystem, 0, 5.6, 0)
        # autocommands4 = commands2.WaitCommand(8).andThen(autocommands3)
        
        # Register Named Commands
        NamedCommands.registerCommand('DropAuto', DropAuto)
        NamedCommands.registerCommand('IntakeAuto', IntakeAuto)

        # Autonomous with SwerveAutoCmd
        Right22h17 =    SwerveAutoCmd(Pose22L).andThen(
                        DropAuto.andThen(
                        SwerveAutoCmd(PoseCoralRight).andThen(
                        IntakeAuto.andThen(
                        SwerveAutoCmd(Pose17L).andThen(
                        DropAuto.andThen(
                        SwerveAutoCmd(PoseCoralRight).andThen(
                        IntakeAuto.andThen(
                        SwerveAutoCmd(Pose17R).andThen(
                        DropAuto
                        ))))))))) # PathPlannerAuto("Right-22-17")

        self.sendableChooser = SendableChooser()
        # #self.sendableChooser.addOption("Blue/Red Mid", autocommand1)
        self.sendableChooser.setDefaultOption("Nothing", autocommand0)
        try:
            self.sendableChooser.addOption("LowStart-22", PathPlannerAuto("LowStart-22-17"))
            self.sendableChooser.addOption("TopStart-18", PathPlannerAuto("TopStart-18"))
            self.sendableChooser.addOption("SimpleMid", PathPlannerAuto("SimpleMid"))
            self.sendableChooser.addOption("BottomStart-18", PathPlannerAuto("BottomStart-18"))
            self.sendableChooser.addOption("Middle-21-19", PathPlannerAuto("Middle-21-19"))
            self.sendableChooser.addOption("HighMid-20-19", PathPlannerAuto("HighMid-20-19"))
            self.sendableChooser.addOption("Mid-21-17", PathPlannerAuto("Middle-21-17"))
            self.sendableChooser.addOption("Wait-SimpleMid", WaitSimpleMid)
            self.sendableChooser.addOption("Wait-HighMid2019", WaitHighMid2019)
            self.sendableChooser.addOption("Wait-LowStart2217", WaitLowStart2217)
            self.sendableChooser.addOption("TestAutoCMD", SwerveAutoTest)
            self.sendableChooser.addOption("AutoSwv-Right-22-17", Right22h17)
        except:
            print("##########################################################################")
            print("AutoTest not found")

        # Configure the button bindings
        self.configureButtonBindings()
        
    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created via the button
        factories on commands2.button.CommandGenericHID or one of its
        subclasses (commands2.button.CommandJoystick or command2.button.CommandXboxController).
        """
        # commands2.button.JoystickButton(
        #     self.driverController, wpilib.XboxController.Button.kA).whileTrue(commands.PickUpSimple2.PickUpSimple2(self.intake, self.blinkinSubsystem).andThen(
        #     commands.DropNote.DropNote(self.intake)))
        
        # commands2.button.POVButton(
        #     self.driverController, 270).whileTrue(commands.LowerRightHook.LowerRightHook(self.climber)) 

        # CORAL DROPPER
        commands2.button.JoystickButton(
            self.driverController, wpilib.PS5Controller.Button.kR1).whileTrue(CoralDrop(self.dropper))
        commands2.button.JoystickButton(
            self.driverController, wpilib.PS5Controller.Button.kL1).whileTrue(CoralIntake(self.dropper, self.elevator))

        # ALGAE COLLECTOR
        commands2.button.JoystickButton(
            self.driverController, wpilib.PS5Controller.Button.kOptions).onTrue(ACBallCancel(self.algaeC))
        
        commands2.button.JoystickButton(
            self.driverController, wpilib.PS5Controller.Button.kTriangle).whileTrue(ACBallInSetUp(self.algaeC))
        
        commands2.button.JoystickButton(
            self.driverController, wpilib.PS5Controller.Button.kCross).whileTrue(ACBallOutSetUp(self.algaeC))

        # ELEVATOR
        commands2.button.POVButton(
            self.driverController, 0).onTrue(ElevatorChange(self.elevator,1))

        commands2.button.POVButton(
            self.driverController, 180).onTrue(ElevatorChange(self.elevator,-1))
        
        # Swerve
        commands2.button.JoystickButton(
            self.driverController, wpilib.PS5Controller.Button.kOptions).onTrue(GyroReset(self.swerveSubsystem)
        )
        
        # Driving
        commands2.button.POVButton(
             self.driverController, 270).onTrue(DriveSwitchMode(self.swerveSubsystem))
        commands2.button.JoystickButton(
            self.driverController, wpilib.PS5Controller.Button.kL2).onTrue(SwerveToggleTurbo(self.swerveSubsystem))
        
        # Test SwerveAutoCMD
        commands2.button.JoystickButton(
            self.driverController, wpilib.PS5Controller.Button.kR2).whileTrue(SwerveAutoCmd(self.swerveSubsystem, 5.6, 0, 0))
        

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main :class:`.Robot` class.
        there's more at the door
        :returns: the command to run in autonomous
        """
        return self.sendableChooser.getSelected()