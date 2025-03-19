# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib

import commands2
import commands2.button
from commands2 import waitcommand

from commands.SwerveJoystickCmd2 import SwerveJoystickCmd2
from commands.DriveSwitchMode import DriveSwitchMode
from commands.ElevatorChange import ElevatorChange
from commands.ACBallInSetUp import ACBallInSetUp
from commands.ACBallOutSetUp import ACBallOutSetUp
from commands.ACBallCancel import ACBallCancel
from commands.CoralIntake import CoralIntake
from commands.CoralDrop import CoralDrop
from commands.CoralDropAuto import CoralDropAuto
from commands.GyroReset import GyroReset
from commands.ElevatorFloor import ElevatorFloor

from subsystems.SwerveSubsystem import SwerveSubsystem
from subsystems.Dropper import Dropper
from subsystems.Elevator import Elevator
from subsystems.AlgaeCollector import AlgaeCollector

from constants import DrivingModes, OIConstants
from constants import DropperConstants

from pathplannerlib.auto import PathPlannerAuto, NamedCommands

from wpilib import SendableChooser

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
               lambda : self.driverController.getRawAxis(OIConstants.kDriverYRotAxis)
           )
        )

        # Build an auto chooser. This will use Commands.none() as the default option.
        # self.autoChooser = AutoBuilder.buildAutoChooser()
        # wpilib.SmartDashboard.putData("Auto Chooser", self.autoChooser)

        autocommand0: commands2.cmd.Command = None

        DropAuto = ElevatorFloor(self.elevator, 4).andThen(
            CoralDrop(self.dropper).andThen(
            waitcommand.WaitCommand(.25).andThen(
            ElevatorFloor(self.elevator, 1))))
        
        IntakeAuto =  CoralIntake(self.dropper, self.elevator)
        
        # autocommands4 = commands2.WaitCommand(8).andThen(autocommands3)
        
        # Register Named Commands
        NamedCommands.registerCommand('DropAuto', DropAuto)
        NamedCommands.registerCommand('IntakeAuto', IntakeAuto)

        self.sendableChooser = SendableChooser()
        # #self.sendableChooser.addOption("Blue/Red Mid", autocommand1)
        self.sendableChooser.setDefaultOption("Nothing", autocommand0)
        try:
            self.sendableChooser.addOption("LowStart-20", PathPlannerAuto("LowStart-20-17"))
            self.sendableChooser.addOption("Mid-21-17", PathPlannerAuto("Mid-21-17"))
            self.sendableChooser.addOption("SimpleMid", PathPlannerAuto("SimpleMid"))
        except:
            print("##########################################################################")
            print("AutoTest not found")
        wpilib.SmartDashboard.putData("Auto Chooser", self.sendableChooser)

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

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main :class:`.Robot` class.
        there's more at the door
        :returns: the command to run in autonomous
        """
        return self.sendableChooser.getSelected()