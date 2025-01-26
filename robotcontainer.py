# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib

import commands2
import commands2.button

from subsystems.SwerveSubsystem import SwerveSubsystem
from subsystems.Elevator import Elevator
from constants import OIConstants

# Import for the sysid routine
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
        self.elevator = Elevator()

        # The driver's controller
        self.driverController = wpilib.XboxController(OIConstants.kDriverControllerPort)
        
        autocommand0: commands2.cmd.Command = None

        # Configure the button bindings
        self.configureButtonBindings()
        
    def configureButtonBindings(self):
        '''Create some button bindings and link them to commands.'''
        
        # commands2.button.JoystickButton(
        #         self.driverController, wpilib.XboxController.Button.kLeftBumper).onTrue(commands2.cmd.runOnce(phoenix6.SignalLogger.start))
        # commands2.button.JoystickButton(
        #         self.driverController, wpilib.XboxController.Button.kRightBumper).onTrue(commands2.cmd.runOnce(phoenix6.SignalLogger.stop))
        
        # Sysid routine for the swerve drive
        # commands2.button.JoystickButton(
        #     self.driverController, wpilib.XboxController.Button.kA).whileTrue(SwerveSubsystem.sysIdQuasistatic(self.swerveSubsystem, SysIdRoutine.Direction.kForward))
        # commands2.button.JoystickButton(
        #     self.driverController, wpilib.XboxController.Button.kB).whileTrue(SwerveSubsystem.sysIdQuasistatic(self.swerveSubsystem, SysIdRoutine.Direction.kReverse))
        # commands2.button.JoystickButton(
        #     self.driverController, wpilib.XboxController.Button.kX).whileTrue(SwerveSubsystem.sysIdDynamic(self.swerveSubsystem, SysIdRoutine.Direction.kForward))
        # commands2.button.JoystickButton(
        #     self.driverController, wpilib.XboxController.Button.kY).whileTrue(SwerveSubsystem.sysIdDynamic(self.swerveSubsystem, SysIdRoutine.Direction.kReverse))

        # Sysid routine for the elevator
        commands2.button.JoystickButton(
            self.driverController, wpilib.XboxController.Button.kA).whileTrue(Elevator.sysIdQuasistatic(self.elevator, SysIdRoutine.Direction.kForward))
        commands2.button.JoystickButton(
            self.driverController, wpilib.XboxController.Button.kB).whileTrue(Elevator.sysIdQuasistatic(self.elevator, SysIdRoutine.Direction.kReverse))
        commands2.button.JoystickButton(
            self.driverController, wpilib.XboxController.Button.kX).whileTrue(Elevator.sysIdDynamic(self.elevator, SysIdRoutine.Direction.kForward))
        commands2.button.JoystickButton(
            self.driverController, wpilib.XboxController.Button.kY).whileTrue(Elevator.sysIdDynamic(self.elevator, SysIdRoutine.Direction.kReverse))


        pass