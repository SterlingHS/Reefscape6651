####################################################################################################
# SwerveAutoCmd.py
# This file contains the SwerveAutoCmd class, which is a
# command that to move the robot to a specific pose.
####################################################################################################

from commands2 import Command

import wpilib

from wpimath.controller import PIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d, Pose2d

from subsystems.SwerveSubsystem import SwerveSubsystem

from constants import DriveConstants

from math import pi

class SwerveAutoCmd(Command):
    def __init__(self, swerveSub:SwerveSubsystem, GoalPose:Pose2d):
        Command.__init__(self)

        self.swerveSub = swerveSub
        self.GoalPose = GoalPose # This can be used to pass in a Pose2d object instead of separate values
        self.addRequirements(swerveSub)

        # PID to control turning speed of the robot
        self.turningPID = PIDController(.01, 0, 0) # P, I, D to be checked
        self.turningPID.enableContinuousInput(0, 360) 
        self.turningPID.setTolerance(1) 

        # PID to control the x and y position of the robot
        self.xPID = PIDController(.25, 0, 0) # P, I, D to be checked
        self.xPID.setTolerance(0.01) # 1 cm/second
        self.yPID = PIDController(.25, 0, 0) # P, I, D to be checked
        self.yPID.setTolerance(0.01) # 1 cm/second

        self.chassisSpeeds = ChassisSpeeds()

        self.transAcc = .75  # 0.75 m/s^2
        self.maxVel = .5      # .5 m/s
        self.rotAcc = 10     # 30 deg/s^2
        self.maxRotRate = 10 # 30 degrees/s

        self.MaxError = 0.01
        self.MaxRotError = 1
        self.slowMoveLimit = 0.2
        self.slowMoveVel = 0.08
        self.VerySlowMoveVel = 0.07
        self.colorDirection = 1

        self.addRequirements(swerveSub)

    def initialize(self) -> None:

        # Reads the position of the robot
        pose = self.swerveSub.getPose()
        self.x = pose.X()
        self.y = pose.Y()
        self.theta = pose.rotation().degrees() % 360 # Normalize the angle to be between 0 and 360

        self.xSetVelocity = 0
        self.ySetVelocity = 0
        self.headingSetVelocity = 0    

        # Flag Goal Reached
        self.goalReachedX = False
        self.goalLaser = False
        self.goalReachedY = False
        self.goalReachedHeading = False

        self.colorDirection = -1 if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed else 1
        return super().initialize()

    def execute(self) -> None:
        wpilib.SmartDashboard.putBoolean("LaserForward", self.goalLaser)
        wpilib.SmartDashboard.putBoolean("GoalReachedX", self.goalReachedX)
        # Reads the position of the robot
        pose = self.swerveSub.getPose()
        self.x = pose.X()
        self.y = pose.Y()
        self.theta = pose.rotation().degrees() % 360 # Normalize the angle to be between 0 and 360

        # Read Instant Velocity
        instantVelocitX = self.swerveSub.getRealVelocityX()
        instantVelocitY = self.swerveSub.getRealVelocityY()
        instantVelocitTheta = self.swerveSub.getGyroRotationVelocity()

        wpilib.SmartDashboard.putNumber("Instant Velocity X",instantVelocitTheta)
        
        # PID to control the x position of the robot
        if self.goalLaser == False:      
            self.ProfiledTrapCalcX()
        else:
            self.AdvanceWithLaser()
        xSpeed = self.xPID.calculate(instantVelocitX, self.xSetVelocity)
        xSpeed = self.xSetVelocity

        # PID to control the y position of the robot
        self.ProfiledTrapCalcY()
        ySpeed = self.yPID.calculate(instantVelocitY, self.ySetVelocity)

        # PID to control the theta position of the robot
        #  self.ProfiledTrapCalcHeading()

        headingSpeed = self.turningPID.calculate(self.theta, self.goalReachedHeading)
        wpilib.SmartDashboard.putNumber("Headin Speed", headingSpeed)

        self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                -xSpeed, 
                -ySpeed, 
                0, #headingSpeed, 
                Rotation2d(self.theta*pi/180))
    
        # Updates the swerve drive modules
        moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(self.chassisSpeeds)
        self.swerveSub.setModuleStates(moduleStates)

        return super().execute()

    def end(self, interrupted: bool) -> None:
        self.swerveSub.stopModules()
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        # check if we have arrived to the Goal
        errorx = abs(self.GoalPose.X() - self.x)
        errory = abs(self.GoalPose.Y() - self.y)
        errorTheta = abs(self.GoalPose.rotation().degrees() - self.theta)
        condition = self.swerveSub.getLaserForward() < 160 and errory < self.MaxError #and errorTheta < .1 

        return condition
    
####################################################################################################

    def AdvanceWithLaser(self):
        ''' Moves robot forward using laserCAN'''
        direction = 1 if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed else -1
        direction = direction * self.colorDirection
        if self.swerveSub.getLaserForward() > 160:
            self.xSetVelocity = direction * self.slowMoveVel
            print(f"Laser Forward - {self.xSetVelocity}")
        else:
            self.xSetVelocity = 0
            print("Laser Stop")

    def ProfiledTrapCalcX(self):
        ''' Calculates the next position and velocity of the robot in the x axis '''
        XMax = self.maxVel**2/self.transAcc # Position to reach max velocity
        direction = -1 if self.x > self.GoalPose.X() else 1
        direction = direction * self.colorDirection

        # Slow motion if error is small
        if self.MaxError < abs(self.x-self.GoalPose.X()) < self.slowMoveLimit or self.goalReachedX == True:
            self.xSetVelocity = direction * self.VerySlowMoveVel
            self.goalReachedX = True
            self.goalLaser = True
            print("Almost There - X")
        
        if abs(self.x-self.GoalPose.X()) < self.MaxError:
            self.goalLaser = True
            print("There - X")

        if self.goalReachedX == False:
            # calculates the next position and velocity on a 20ms cycle
            # Fase 0: Deceleration
            if abs(self.x-self.GoalPose.X()) < XMax:
                self.xSetVelocity -= self.transAcc * 0.02 * direction
                print(f"Deceleration - x: {self.x} Goal: {self.GoalPose.X()} Vel: {self.xSetVelocity}")
                if direction == 1 and self.xSetVelocity < self.slowMoveVel:
                    self.xSetVelocity = self.slowMoveVel
                if direction == -1 and self.xSetVelocity > -self.slowMoveVel:
                    self.xSetVelocity = -self.slowMoveVel
            # Fase 1: Acceleration
            elif direction == 1 and self.xSetVelocity < self.maxVel:
                self.xSetVelocity += self.transAcc * 0.02
                print(f"Accelerating {self.xSetVelocity} {self.maxVel}")
            elif direction == -1 and self.xSetVelocity > -self.maxVel:
                self.xSetVelocity -= self.transAcc * 0.02
                print(f"Accelerating {self.xSetVelocity} {self.maxVel}")
            # Fase 2: Keep Max Velocity
            else:
                self.xSetVelocity = self.maxVel * direction
        wpilib.SmartDashboard.putNumber("x", self.x)
        wpilib.SmartDashboard.putNumber("Goal x", self.GoalPose.X())
        wpilib.SmartDashboard.putNumber("XMax", XMax)
        wpilib.SmartDashboard.putNumber("Direction", direction)
        wpilib.SmartDashboard.putNumber("xSetVel", self.xSetVelocity)
        wpilib.SmartDashboard.putBoolean("GoalReachedX", self.goalReachedX)

    def ProfiledTrapCalcY(self):
        ''' Calculates the next position and velocity of the robot in the y axis '''
        YMax = self.maxVel**2/self.transAcc
        direction = -1 if self.y > self.GoalPose.Y() else 1
        direction = direction * self.colorDirection

         # Slow motion if error is small
        if self.MaxError < abs(self.y-self.GoalPose.Y()) < self.slowMoveLimit or self.goalReachedY == True:
            self.ySetVelocity = direction * self.VerySlowMoveVel
            self.goalReachedY = True
            print("Almost there - Y")
        
        if abs(self.y-self.GoalPose.Y()) < self.MaxError:
            self.ySetVelocity = 0
            print("There - Y")

        if self.goalReachedY == False:
            # calculates the next position and velocity on a 20ms cycle
            # Fase 0: Deceleration
            if abs(self.y-self.GoalPose.Y()) < YMax:
                self.ySetVelocity -= self.transAcc * 0.02 * direction
                if direction == 1 and self.ySetVelocity < self.slowMoveVel:
                    self.ySetVelocity = self.slowMoveVel
                if direction == -1 and self.ySetVelocity > -self.slowMoveVel:
                    self.ySetVelocity = -self.slowMoveVel
            # Fase 1: Acceleration
            elif direction == 1 and self.ySetVelocity < self.maxVel:
                self.ySetVelocity += self.transAcc * 0.02
            elif direction == -1 and self.ySetVelocity > -self.maxVel:
                self.ySetVelocity -= self.transAcc * 0.02
            # Fase 2: Keep Max Velocity
            else:
                self.ySetVelocity = self.maxVel * direction
        wpilib.SmartDashboard.putNumber("y", self.y)
        wpilib.SmartDashboard.putNumber("Goal y", self.GoalPose.Y())
        wpilib.SmartDashboard.putNumber("YMax", YMax)
        wpilib.SmartDashboard.putNumber("Directiony", direction)
        wpilib.SmartDashboard.putNumber("YSetVel", self.xSetVelocity)

    def ProfiledTrapCalcHeading(self):
        ''' Calculates the next position and velocity of the robot in the y axis '''
        RotMax = self.maxRotRate**2/self.rotAcc
        goalAngle = self.GoalPose.rotation().degrees() % 360
        # Check quickest way to turn to the goal
        if abs(self.theta - goalAngle) < 180:
            direction = -1 if self.theta > goalAngle else 1
        else:
            direction = 1 if self.theta > goalAngle else -1

         # Slow motion if error is small
        if abs(self.theta-goalAngle) < 5 or self.goalReachedHeading == True:
            self.headingSetVelocity = direction * 5
            self.goalReachedHeading = True
            print("Almost there - Theta")
        
        if abs(self.theta-goalAngle) < 2:
            self.SetVelocity = 0
            print("There - Theta")

        if self.goalReachedHeading == False:
            # calculates the next position and velocity on a 20ms cycle
            # Fase 0: Deceleration
            if abs(self.theta-goalAngle) < RotMax:
                self.headingSetVelocity -= self.rotAcc * 0.02 * direction
                if direction == 1 and self.headingSetVelocity < 0.10:
                    self.headingSetVelocity = .1
                if direction == -1 and self.headingSetVelocity > -.1:
                    self.headingSetVelocity = -.1
            # Fase 1: Acceleration
            elif direction == 1 and self.headingSetVelocity < self.maxRotRate:
                self.headingSetVelocity += self.rotAcc * 0.02
            elif direction == -1 and self.headingSetVelocity > -self.maxRotRate:
                self.headingSetVelocity -= self.rotAcc * 0.02
            # Fase 2: Keep Max Velocity
            else:
                self.headingSetVelocity = self.maxRotRate * direction
        print(f"Theta = {self.theta} - Goal = {goalAngle} - velocity = {self.headingSetVelocity} - Direction {direction}")
        wpilib.SmartDashboard.putNumber("angle", self.theta)
        wpilib.SmartDashboard.putNumber("Goal", goalAngle)
        wpilib.SmartDashboard.putNumber("velocity", self.headingSetVelocity)
        wpilib.SmartDashboard.putNumber("Direction", direction)




