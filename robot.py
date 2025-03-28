#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import typing

import wpilib
from wpilib import DataLogManager
import commands2
import commands2.cmd
import robotcontainer
from cscore import CameraServer as CS
from cscore import VideoMode
import cv2
from NetworkTables import NetworkTables
from libgrapplefrc import can_bridge_tcp

"""
The VM is configured to automatically run this class, and to call the functions corresponding to
each mode, as described in the TimedRobot documentation. If you change the name of this class or
the package after creating this project, you must also update the build.gradle file in the
project.
"""


class MyRobot(commands2.TimedCommandRobot):
    """
    Our default robot class, pass it to wpilib.run
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        self.autonomousCommand: typing.Optional[commands2.Command] = None
        # Initialize camera
        CS.enableLogging()
        camera = CS.startAutomaticCapture("Top Camera", 0)
        camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 160, 120, 60)
        
        # Start Log File
        DataLogManager.start()
        
        # For LaserCAN
        can_bridge_tcp()

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = robotcontainer.RobotContainer()
        self.shuffleboardinfo = NetworkTables(self.container)

    def robotPeriodic(self) -> None:
        self.shuffleboardinfo.updateShuffleboard()
        return super().robotPeriodic()

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        # self.container.swerveSubsystem.resetEncoder()
        self.autonomousCommand = self.container.getAutonomousCommand()

        # schedule the autonomous command (example)
        if self.autonomousCommand is not None:
            self.autonomousCommand.schedule()
        else:
            print("no auto command?")

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
        self.shuffleboardinfo.updateShuffleboard()

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomousCommand is not None:
            self.autonomousCommand.cancel()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        self.shuffleboardinfo.updateShuffleboard()

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self) -> None:
        self.shuffleboardinfo.updateShuffleboard()
        return super().testPeriodic()


if __name__ == "__main__":
    wpilib.run(MyRobot)