# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib

import commands2
import commands2.button

from commands.SwerveJoystickCmd import SwerveJoystickCmd
from commands.SwerveJoystickCmd2 import SwerveJoystickCmd2
from commands.CoralDrop import CoralDrop
from commands.CoralDropSpeed import CoralDropSpeed
from commands.ElevatorFloor import ElevatorFloor
from commands.ElevatorMove import ElevatorMove
from commands.ElevatorChange import ElevatorChange
from commands.ARArmChange import ARArmChange
from commands.ACBallInSetUp import ACBallInSetUp
from commands.ACBallOutSetUp import ACBallOutSetUp
from commands.ACArmChange import ACArmChange
from commands.ACBallCancel import ACBallCancel
from commands.CoralIntake import CoralIntake
from commands.CoralDrop import CoralDrop
from commands.GyroReset import GyroReset

from subsystems.SwerveSubsystem import SwerveSubsystem
from subsystems.Dropper import Dropper
from subsystems.Elevator import Elevator
from subsystems.AlgaeRemover import AlgaeRemover
from subsystems.AlgaeCollector import AlgaeCollector

from constants import OIConstants
from constants import DropperConstants

from pathplannerlib.auto import PathPlannerAuto, AutoBuilder

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
        #self.algaeR = AlgaeRemover()
        self.algaeC = AlgaeCollector()

        # The driver's controller
        self.driverController = wpilib.PS5Controller(OIConstants.kDriverControllerPort)
        self.codriverController = wpilib.XboxController(OIConstants.kCodriverControllerPort)

        # self.codriverController = wpilib.XboxController(OIConstants.kCodriverControllerPort)
        
        # Set the default commands for the subsystems
        # self.swerveSubsystem.setDefaultCommand(
        #    SwerveJoystickCmd(
        #        self.swerveSubsystem,
        #        lambda : self.driverController.getRawAxis(OIConstants.kDriverYAxis),
        #        lambda : self.driverController.getRawAxis(OIConstants.kDriverXAxis),
        #        lambda : self.driverController.getRawAxis(OIConstants.kDriverXRotAxis)
        #    )
        # )
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

        # autocommand1 = commands.MoveSpeedAndTime.MoveSpeedAndTime(self.swerveSubsystem, -3, 0.5).andThen(
        #     commands.uptospeed.UpToSpeed(self.shooter).andThen(
        #     commands.intakeIntoShooter.intakeIntoShooter(self.shooter, self.intake,self.blinkinSubsystem, constants.ShootConstants.far))).andThen(
        #     commands.MoveSpeedTimeField.MoveSpeedTimeField(self.swerveSubsystem, -4.2, 1))
        
        # autocommands4 = commands2.WaitCommand(8).andThen(autocommands3)
               
        self.sendableChooser = SendableChooser()
        # #self.sendableChooser.addOption("Blue/Red Mid", autocommand1)
        self.sendableChooser.setDefaultOption("Nothing", autocommand0)
        try:
            self.sendableChooser.addOption("Test", PathPlannerAuto("TestDrive"))
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


        # ALGAE REMOVER
        # commands2.button.POVButton(
        #     self.driverController, 90).onTrue(ARArmChange(self.algaeR,0))

        # commands2.button.POVButton(
        #     self.driverController, 270).onTrue(ARArmChange(self.algaeR,5))
        

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main :class:`.Robot` class.
        there's more at the door
        :returns: the command to run in autonomous
        """
        return PathPlannerAuto('TestDrive')
        return self.sendableChooser.getSelected()