# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib

import commands2
import commands2.button
from commands.TunePIDTurning import TunePIDTurning
from commands.SwerveJoystickCmd import SwerveJoystickCmd

from subsystems.SwerveSubsystem import SwerveSubsystem
from constants import OIConstants

from pathplannerlib.auto import PathPlannerAuto

# Import the sysid routine
from commands2.sysid import SysIdRoutine
import phoenix6

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

        # The driver's controller
        self.driverController = wpilib.XboxController(OIConstants.kDriverControllerPort)
        # self.codriverController = wpilib.XboxController(OIConstants.kCodriverControllerPort)
        
        # Set the default commands for the subsystems
        self.swerveSubsystem.setDefaultCommand(
           SwerveJoystickCmd(
               self.swerveSubsystem,
               lambda : self.driverController.getRawAxis(OIConstants.kDriverYAxis),
               lambda : self.driverController.getRawAxis(OIConstants.kDriverXAxis),
               lambda : self.driverController.getRawAxis(OIConstants.kDriverRotAxis)
           )
        )

        # Tune UP PID for turning
        # self.swerveSubsystem.setDefaultCommand(
        #    TunePIDTurning(
        #        self.swerveSubsystem,
        #        lambda : self.driverController.getRawAxis(OIConstants.kDriverYAxis),
        #        lambda : self.driverController.getRawAxis(OIConstants.kDriverXAxis),
        #        lambda : self.driverController.getRawAxis(OIConstants.kDriverRotAxis)
        #    )
        # )
        autocommand0: commands2.cmd.Command = None

        # autocommand1 = commands.MoveSpeedAndTime.MoveSpeedAndTime(self.swerveSubsystem, -3, 0.5).andThen(
        #     commands.uptospeed.UpToSpeed(self.shooter).andThen(
        #     commands.intakeIntoShooter.intakeIntoShooter(self.shooter, self.intake,self.blinkinSubsystem, constants.ShootConstants.far))).andThen(
        #     commands.MoveSpeedTimeField.MoveSpeedTimeField(self.swerveSubsystem, -4.2, 1))
        
        # autocommands4 = commands2.WaitCommand(8).andThen(autocommands3)
               
        self.sendableChooser = wpilib.SendableChooser()
        #self.sendableChooser.addOption("Blue/Red Mid", autocommand1)
        self.sendableChooser.setDefaultOption("Nothing", autocommand0)
        #self.sendableChooser.setDefaultOption("Test", PathPlannerAuto("AutoTest"))

        # Configure the button bindings
        self.configureButtonBindings()
        
    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created via the button
        factories on commands2.button.CommandGenericHID or one of its
        subclasses (commands2.button.CommandJoystick or command2.button.CommandXboxController).
        """
        '''commands2.button.JoystickButton(
            self.driverController, wpilib.XboxController.Button.kA).whileTrue(commands.PickUpSimple2.PickUpSimple2(self.intake, self.blinkinSubsystem).andThen(
            commands.DropNote.DropNote(self.intake)))'''
        
        ''' commands2.button.POVButton(
            self.driverController, 270).whileTrue(commands.LowerRightHook.LowerRightHook(self.climber)) '''

        commands2.button.JoystickButton(
                self.driverController, wpilib.XboxController.Button.kLeftBumper(commands2.runcommand(phoenix6.SignalLogger.start)))
        commands2.button.JoystickButton(
                self.driverController, wpilib.XboxController.Button.kLeftBumper(commands2.runcommand(phoenix6.SignalLogger.stop)))
        # commands2.button.JoystickButton(
        #     self.driverController, wpilib.XboxController.Button.kA).whileTrue(SwerveSubsystem.sysIdQuasistatic(self.swerveSubsystem, SysIdRoutine.Direction.kForward))
        # commands2.button.JoystickButton(
        #     self.driverController, wpilib.XboxController.Button.kB).whileTrue(SwerveSubsystem.sysIdQuasistatic(self.swerveSubsystem, SysIdRoutine.Direction.kReverse))
        # commands2.button.JoystickButton(
        #     self.driverController, wpilib.XboxController.Button.kX).whileTrue(SwerveSubsystem.sysIdDynamic(self.swerveSubsystem, SysIdRoutine.Direction.kForward))
        # commands2.button.JoystickButton(
        #     self.driverController, wpilib.XboxController.Button.kY).whileTrue(SwerveSubsystem.sysIdDynamic(self.swerveSubsystem, SysIdRoutine.Direction.kReverse))

        pass

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main :class:`.Robot` class.
        there's more at the door
        :returns: the command to run in autonomous
        """
       
        #autocommand1 = AutoTrajectory.AutoTrajectory(self.swerveSubsystem, "/home/lvuser/py/autoTrajectories/Blue22.wpilib.json")
        return self.sendableChooser.getSelected()