#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import typing

import wpilib
import commands2
import commands2.cmd
import robotcontainer


class MyRobot(commands2.TimedCommandRobot):

    def robotInit(self) -> None:

        self.autonomousCommand: typing.Optional[commands2.Command] = None

        self.container = robotcontainer.RobotContainer()

    def robotPeriodic(self) -> None:
        return super().robotPeriodic()

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""

        # schedule the autonomous command (example)
        if self.autonomousCommand is not None:
            self.autonomousCommand.schedule()
        else:
            print("no auto command?")

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomousCommand is not None:
            self.autonomousCommand.cancel()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self) -> None:
        return super().testPeriodic()


if __name__ == "__main__":
    wpilib.run(MyRobot)