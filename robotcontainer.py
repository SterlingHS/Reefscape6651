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
        # Tune UP PID for turning
        self.swerveSubsystem.setDefaultCommand(
           TunePIDTurning(
               self.swerveSubsystem,
               lambda : self.driverController.getRawAxis(OIConstants.kDriverYAxis),
               lambda : self.driverController.getRawAxis(OIConstants.kDriverXAxis),
               lambda : self.driverController.getRawAxis(OIConstants.kDriverRotAxis)
           )
        )
        autocommand0: commands2.cmd.Command = None

        self.sendableChooser = wpilib.SendableChooser()

        # Configure the button bindings
        self.configureButtonBindings()
        
    def configureButtonBindings(self):
        pass

    def getAutonomousCommand(self) -> commands2.Command:
        return self.sendableChooser.getSelected()