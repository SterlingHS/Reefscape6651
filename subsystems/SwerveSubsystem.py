from subsystems.SwerveModule import SwerveModule

from math import cos, sin, pi

import wpimath
from wpimath.geometry import Rotation2d, Pose2d
import wpimath.kinematics
from wpimath.kinematics import SwerveModuleState, ChassisSpeeds
from wpimath.estimator import SwerveDrive4PoseEstimator
from ntcore import NetworkTableInstance

from commands2 import Subsystem
from constants import DriveConstants, FieldOrientedConstants, DrivingModes, ReefPositions

import wpilib
import navx

# Autonomous Pathplanner
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
from wpimath.geometry import Pose2d, Rotation2d

from wpilib import DriverStation, Timer

# For NavX to calibrate
from time import sleep

class SwerveSubsystem(Subsystem):
    def __init__(self):
        Subsystem.__init__(self)

        # Init Swerve Modules
        self.frontLeft = SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveMotorReversed,
            DriveConstants.kFrontLeftTurningMotorReversed,
            DriveConstants.kFrontLeftAbsoluteEncoderPort,
            DriveConstants.kFrontLeftAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftAbsoluteEncoderReversed,
            DriveConstants.kFrontLeftForwardPIDk,
            DriveConstants.kFrontLeftTurningPIDk
        )
        self.frontRight = SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveMotorReversed,
            DriveConstants.kFrontRightTurningMotorReversed,
            DriveConstants.kFrontRightAbsoluteEncoderPort,
            DriveConstants.kFrontRightAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightAbsoluteEncoderReversed,
            DriveConstants.kFrontRightForwardPIDk,
            DriveConstants.kFrontRightTurningPIDk
        )
        self.backLeft = SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveMotorReversed,
            DriveConstants.kBackLeftTurningMotorReversed,
            DriveConstants.kBackLeftAbsoluteEncoderPort,
            DriveConstants.kBackLeftAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftAbsoluteEncoderReversed,
            DriveConstants.kBackLeftForwardPIDk,
            DriveConstants.kBackLeftTurningPIDk
        )
        self.backRight = SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveMotorReversed,
            DriveConstants.kBackRightTurningMotorReversed,
            DriveConstants.kBackRightAbsoluteEncoderPort,
            DriveConstants.kBackRightAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightAbsoluteEncoderReversed,
            DriveConstants.kBackRightForwardPIDk,
            DriveConstants.kBackRightTurningPIDk
        )
        
        # Init variables for velocity
        self.xvelocity = 0
        self.yvelocity = 0
        self.turningvelocity = 0

        # Init Swerve Position
        self.swerveModulePositions = (wpimath.kinematics.SwerveModulePosition(self.frontLeft.getDrivePosition(), Rotation2d(self.frontLeft.getTurningPosition())),
                                wpimath.kinematics.SwerveModulePosition(self.frontRight.getDrivePosition(), Rotation2d(self.frontRight.getTurningPosition())),
                                wpimath.kinematics.SwerveModulePosition(self.backLeft.getDrivePosition(), Rotation2d(self.backLeft.getTurningPosition())),
                                wpimath.kinematics.SwerveModulePosition(self.backRight.getDrivePosition(), Rotation2d(self.backRight.getTurningPosition())),
                                )
              
        # self.gyro = navx.AHRS(navx.SPI.Port.kMXP)
        self.gyro = navx.AHRS.create_spi()
        self.zeroHeading()
        sleep(1) # Wait for gyro to calibrate... NOT MORE THAN 2 SEC!! or get error.

        # Init SwerveDrive2PoseEstimator
        self.odometer = wpimath.kinematics.SwerveDrive4Odometry(
                                    DriveConstants.kDriveKinematics,
                                    self.getRotation2d(),
                                    self.swerveModulePositions)       

        # Pathplanner Configuration
        config = RobotConfig.fromGUISettings() # RobotConfig, this should likely live in your Constants class
        print("Pathplanner Configuration")
        print(config)
        # For Autonomous Pathplanner
        AutoBuilder.configure(
            self.getPose,#self.getPoseEstimator, # Robot pose supplier
            self.resetOdometer, # Method to reset odometry (will be called if your auto has a starting pose)
            self.getChassisSpeed, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            lambda speeds, feedforwards: self.setChassisSpeeds(speeds), # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
            PPHolonomicDriveController( # PPHolonomicController is the built in path following controller for holonomic drive trains
                PIDConstants(5, 0, 0), # Translation PID constants
                PIDConstants(5, 0, 0) # Rotation PID constants
            ),
            config, # The robot configuration
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )

        # Init reef depending on Allliance
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            self.reefLocationX = FieldOrientedConstants.BlueReefX
            self.reefLocationY = FieldOrientedConstants.BlueReefY
        else:
            self.reefLocationX = FieldOrientedConstants.RedReefX
            self.reefLocationY = FieldOrientedConstants.RedReefY

        # Driving Modes
        self.last_reef_angle = 0
        self.drivingMode = DrivingModes.FieldOriented
        # Mode 0 = Field Oriented
        # Mode 1 = Reef AprilTag Oriented
        # Mode 2 = Processor Oriented
        # Mode 3 = Coral Station Oriented

        # Init Limelight
        self.inst = NetworkTableInstance.getDefault() # Initialize the NetworkTable for Limelight
        self.inst.setServerTeam(6651)
        self.inst.startDSClient() # Start the NetworkTable client

        # Get the Limelight table and set the camera mode to 1 (vision processing)
        tableLL = self.inst.getTable("limelight") # Get the Limelight table
        # Set IMU to 1
        tableLL.getEntry("imumode").setInteger(1) # Set the camera mode to 1 (vision processing)
        tableLL.getEntry("stddevs").setDoubleArray([])
        
        self.listYaw = []
        self.PoseEstimatorInit = False

        # Init DesiredStates of the Swerve Modules
        self.desiredStates = [SwerveModuleState(0, Rotation2d(0)),
                              SwerveModuleState(0, Rotation2d(0)),
                              SwerveModuleState(0, Rotation2d(0)),
                              SwerveModuleState(0, Rotation2d(0))]

        # Init TurboMode
        self.turboMode = False

################################################################################################
############## Gyro Methods

    def zeroHeading(self):
        ''' Zero the gyro heading '''
        self.gyro.reset()

    def offSetGyro(self, angleLimelight):
        ''' Offsets gyro depending on Limelight feedback '''
        offset = angleLimelight
        self.gyro.setAngleAdjustment(offset)

    def getCompass(self)->float: 
        ''' Return heading in degrees for values between 0 and 360 (from Gyro)'''
        return -self.gyro.getFusedHeading()

    def getHeading(self):
        ''' Return heading in degrees for values between 0 and 360 (from Gyro)'''
        return -self.gyro.getAngle() % 360

    def getContinuousHeading(self):
        ''' Return heading in degrees (from Gyro)'''
        return -self.gyro.getAngle()
    
################################################################################################
############## Odometer Methods

    def getRotation2d(self):
        ''' Returns heading in Rotation2d format '''
        return Rotation2d.fromDegrees(self.getHeading())

    def getPose(self):
        ''' Returns the pose of the robot '''
        return self.odometer.getPose()

    def getPoseEstimator(self):
        ''' Returns the pose of the robot using PoseEstimator '''
        try:
            return self.poseEstimator.getEstimatedPosition()
        except:
            return Pose2d(0,0,Rotation2d(0))
    
    def resetOdometer(self, pose):
        ''' Resets the odometer to a specific pose '''
        self.odometer.resetPosition(self.getRotation2d(), 
                                    (self.frontLeft.getSwerveModulePosition(), 
                                     self.frontRight.getSwerveModulePosition(), 
                                     self.backLeft.getSwerveModulePosition(), 
                                     self.backRight.getSwerveModulePosition()), 
                                     pose)
        pass
        
    def resetEncoder(self):
        ''' Resets all the encoders - 
        Forward encoders to 0, rotation encoder to output of Absolute Encoder. '''
        self.frontLeft.resetEncoders()
        self.frontRight.resetEncoders()
        self.backLeft.resetEncoders()
        self.backRight.resetEncoders()

    def readAbsEncoders(self):
        ''' Reads all absolute encoders - Returns list of values [FL, FR, BL, BR] '''
        FL = self.frontLeft.getAbsoluteEncoderRad()
        FR = self.frontRight.getAbsoluteEncoderRad()
        BL = self.backLeft.getAbsoluteEncoderRad()
        BR = self.backRight.getAbsoluteEncoderRad()
        return [FL,FR,BL,BR]
    
    def readTurnEncoders(self):
        ''' Reads all turning encoders - Returns list of values [FL, FR, BL, BR] '''
        FL = self.frontLeft.getTurningPositionClose()
        FR = self.frontRight.getTurningPositionClose()
        BL = self.backLeft.getTurningPositionClose()
        BR = self.backRight.getTurningPositionClose()
        return [FL,FR,BL,BR]
    
    def readForwardEncoders(self):
        ''' Reads all Forward encoders - Returns list of values [FL, FR, BL, BR] '''
        FL = self.frontLeft.getDrivePosition()
        FR = self.frontRight.getDrivePosition()
        BL = self.backLeft.getDrivePosition()
        BR = self.backRight.getDrivePosition()
        return [FL,FR,BL,BR]

################################################################################################
############## Movement Methods

    def stopModules(self):
        ''' Stops all motors '''
        self.frontLeft.stop()
        self.frontRight.stop()
        self.backLeft.stop()
        self.backRight.stop()

    def moveRight(self, speed):
        ''' Moves all motors to the right at a certain speed '''
        angle = self.getHeading()-90
        speedx = speed*cos(angle*pi/180)
        speedy = speed*sin(angle*pi/180)
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedx, speedy, 0,Rotation2d(self.getHeading()*pi/180))
        moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds)
        self.setModuleStates(moduleStates)
    
    def moveLeft(self, speed):
        ''' Moves all motors to the left at a certain speed '''
        angle = self.getHeading()+90
        speedx = speed*cos(angle*pi/180)
        speedy = speed*sin(angle*pi/180)
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedx, speedy, 0 ,Rotation2d(self.getHeading()*pi/180))
        moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds)
        self.setModuleStates(moduleStates)

    def moveStraight(self, speed):
        ''' Moves all motors straight forward at a certain speed '''
        straightState = SwerveModuleState(speed, Rotation2d(0))
        self.frontLeft.setDesiredState(straightState)
        self.frontRight.setDesiredState(straightState)
        self.backLeft.setDesiredState(straightState)
        self.backRight.setDesiredState(straightState)

    def moveStraightField(self, speed):
        ''' Moves all motors straight forward at a certain speed (In field oriented mode) '''
        self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                speed, 0, 0, self.getRotation2d())
        moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(self.chassisSpeeds)
        self.setModuleStates(moduleStates)

    def setModuleStates(self,desiredStates):
        ''' Sets swerve system to go in a specific direction and speed '''
        # Calculate the attainable max speed based on the current mode (turbo or not)
        attainableMaxSpeed = DriveConstants.kPhysicalMaxSpeedMetersPerSecond * (.50 if self.turboMode else 1)
        
        # Desaturate the wheel speeds to ensure they don't exceed the max speed
        desiredStates = wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desiredStates, 
            attainableMaxSpeed)
        
        # Set the desired state for each swerve module
        self.frontLeft.setDesiredState(desiredStates[0])
        self.frontRight.setDesiredState(desiredStates[1])
        self.backLeft.setDesiredState(desiredStates[2])
        self.backRight.setDesiredState(desiredStates[3])

        # Store the desired states for later use (e.g. for PathPlanner)
        self.desiredStates = desiredStates

    def getDesiredModuleStates(self):
        ''' Returns the Desired state of the swerve modules '''
        return self.desiredStates
    
    def setChassisSpeeds(self, chassisSpeeds:ChassisSpeeds):
        ''' Sets the chassis speeds of the robot in Autonomous mode (PathPlanner)'''
        self.xvelocity = chassisSpeeds.vx
        self.yvelocity = chassisSpeeds.vy
        self.turningvelocity = chassisSpeeds.omega

        chassisSpeeds.vx = -chassisSpeeds.vx
        chassisSpeeds.vy = -chassisSpeeds.vy
        moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds)
        self.setModuleStates(moduleStates)

    def getChassisSpeed(self):
        ''' Returns the chassis speeds of the robot in Autonomous mode (PathPlanner)'''
        return ChassisSpeeds(self.getXVelocity(), self.getYVelocity(), self.getAngularVelocity())
    
    def shouldFlipPath(self):
        ''' Used for Autonomoud mode (PathPlanner) - Boolean supplier that controls when the path will be mirrored for the red alliance.
        This will flip the path being followed to the red side of the field.
        THE ORIGIN WILL REMAIN ON THE BLUE SIDE '''
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed
    
################################################################################################
############## Velocity Methods

    def getXVelocity(self):
        ''' Returns the X velocity of the robot '''
        return self.xvelocity

    def getYVelocity(self):
        ''' Returns the Y velocity of the robot '''
        return self.yvelocity
    
    def getDesiredVelocity(self):
        ''' Returns the velocity of the robot '''
        return (self.getXVelocity()**2 + self.getYVelocity()**2)**0.5
    
    def getVelocity(self):
        ''' Returns the desired velocity of the robot '''
        # Reads the velocity of each module and averages them
        # for the x axis and the y axis
        # then finds the hypothenuse of the two
        # to get the velocity of the robot
        FLx = self.frontLeft.getDriveVelocity()*cos(self.frontLeft.getTurningPosition())
        FRx = self.frontRight.getDriveVelocity()*cos(self.frontRight.getTurningPosition())
        BLx = self.backLeft.getDriveVelocity()*cos(self.backLeft.getTurningPosition())
        BRx = self.backRight.getDriveVelocity()*cos(self.backRight.getTurningPosition())
        xvelocity = (FLx+FRx+BLx+BRx)/4
        FLy = self.frontLeft.getDriveVelocity()*sin(self.frontLeft.getTurningPosition())
        FRy = self.frontRight.getDriveVelocity()*sin(self.frontRight.getTurningPosition())
        BLy = self.backLeft.getDriveVelocity()*sin(self.backLeft.getTurningPosition())
        BRy = self.backRight.getDriveVelocity()*sin(self.backRight.getTurningPosition())
        yvelocity = (FLy+FRy+BLy+BRy)/4
        return (xvelocity**2 + yvelocity**2)**0.5
        
    def getAngularVelocity(self):
        ''' Returns the angular velocity of the robot '''
        return self.turningvelocity

    def getTurboMode(self):
        ''' Returns the turbo mode '''
        return self.turboMode
    
    def setTurboMode(self, mode):
        ''' Sets the turbo mode '''
        self.turboMode = mode

    def toggleTurboMode(self):
        ''' Toggle the turbo mode '''
        self.turboMode = not self.turboMode

###############################################################################################
############## Reef Oriented Methods

    def angleToAprilTag(self, aprilTag):
        ''' Returns the angle to the April Tag in degrees (0-360)'''
        # if alliance is blue
        if (DriverStation.getAlliance() == DriverStation.Alliance.kBlue and 17 <= aprilTag <= 22) or (DriverStation.getAlliance() == DriverStation.Alliance.kRed and 6 <= aprilTag <= 11):
            try:
                self.last_reef_angle = ReefPositions.reefAngles[int(aprilTag)-1]
                self.last_reef_angle = self.last_reef_angle % 360
            except:
                print(f"AprilTage error - {aprilTag}")
        return 0
    
    def getClosestAprilTag(self):
        ''' Read limelight and returns the closest April Tag '''
        if self.validity != 0:
            return self.aprilTagNumber
        return 0

    def isPoseInitialized(self):
        ''' Returns if the Pose Estimator has been initialized '''
        return not self.PoseEstimatorInit

    def readLimelight(self):
        ''' Read Limelight and returns the data '''
        ##################################################################
        # Using the NetworkTable to read Limelight data
        table = self.inst.getTable("limelight")
        # Get the latest values from Limelight
        self.botpose = table.getEntry("botpose").getDoubleArray([0,0,0,0,0,0])
        self.botpose_wpiblue = table.getEntry("botpose_orb_wpiblue").getDoubleArray([0,0,0,0,0,0])
        self.capture_latency = table.getEntry("cl").getDouble(0)
        self.timestamp = table.getEntry("ts").getDouble(0)
        self.validity = table.getEntry("tv").getDouble(0)
        self.aprilTagNumber = table.getEntry("tid").getDouble(0)
        self.cl = table.getEntry("cl").getDouble(0)
        self.tl = table.getEntry("tl").getDouble(0)

        # Get the status of the Limelight
        status = table.getEntry("hw").getDoubleArray([0,0,0,0])
        # print(f"Status - {status}")
        wpilib.SmartDashboard.putNumber("Limelight CPU Temp",status[1])
        wpilib.SmartDashboard.putNumber("Limelight Temp",status[3]) 

        ##################################################################
        # Check if the PoseEstimator has been initialized
        if self.PoseEstimatorInit == True:
            # Read yaw from limelight's posewpiblue
            self.yaw_lime = self.botpose_wpiblue[5] if self.botpose_wpiblue else 0
            wpilib.SmartDashboard.putNumber("Yaw", self.yaw_lime)
        else: # Read yaw and set the gyro offset for calibration
            try:
                self.yaw_lime = self.botpose[5]
                wpilib.SmartDashboard.putNumber("Yaw", self.yaw_lime)
                # Save the yaw for calibration of NavX on a list
                if len(self.listYaw) < 10: #and isinstance(self.yaw_lime,float): 
                    if self.yaw_lime != 0:
                        self.listYaw.append(self.yaw_lime)
                    # If we have 10 values, calculate the average and set the gyro offset
                if len(self.listYaw) == 10:
                    yaw = sum(self.listYaw)/len(self.listYaw) # Average the yaw values
                    print(f"Yaw = {yaw}")
                    if self.aprilTagNumber in [6, 7, 8, 9, 10, 11]: # If we see red reef
                        if yaw < 180 :
                            yaw = yaw + 180
                        elif yaw >= 180:
                            yaw = yaw - 180
                    self.offSetGyro(-yaw) # Set the gyro offset

                    # Initialize the pose estimator 
                    # Sets tghe flag to True to indicate that the PoseEstimator has been initialized
                    self.PoseEstimatorInit = True
                    # Initialize the pose estimator with the current pose from the Limelight
                    self.poseEstimator = SwerveDrive4PoseEstimator(
                                DriveConstants.kDriveKinematics, 
                                self.getRotation2d(), 
                                self.swerveModulePositions,
                                Pose2d(
                                    self.botpose_wpiblue[0], 
                                    self.botpose_wpiblue[1], 
                                    Rotation2d.fromDegrees(yaw)))
                    self.odometer.resetPose( Pose2d(
                                    self.botpose_wpiblue[0], 
                                    self.botpose_wpiblue[1], 
                                    Rotation2d.fromDegrees(yaw)))
                    self.poseEstimator.setVisionMeasurementStdDevs((0.1,0.1,0.1))
                    print(f"{self.botpose_wpiblue[0]} - {self.botpose_wpiblue[1]} - {self.botpose_wpiblue[5]}")
                    # push the yaw to the limelight
                    table.getEntry("robot_orientation_set").setDoubleArray([yaw,0,0,0,0,0]) # Set the orientation to the Limelight    
                    # Set IMU to 2
                    table.getEntry("imumode").setInteger(2) # Set the camera mode to 2 (robot orientation)
            except:
                pass 

    def updateOdometry(self):
        ''' Update the odometry using the Limelight and the botpose '''
        
        # Update the pose estimator with the latest state
        self.poseEstimator.update(
                                    self.getRotation2d(), 
                                    (
                                        self.frontLeft.getSwerveModulePosition(), 
                                        self.frontRight.getSwerveModulePosition(), 
                                        self.backLeft.getSwerveModulePosition(), 
                                        self.backRight.getSwerveModulePosition()
                                    )
                                )
        
        wpilib.SmartDashboard.putNumber("before X", self.poseEstimator.getEstimatedPosition().X())
        wpilib.SmartDashboard.putNumber("before Y", self.poseEstimator.getEstimatedPosition().Y())
        wpilib.SmartDashboard.putNumber("before Theta", self.poseEstimator.getEstimatedPosition().rotation().degrees())

        if self.validity == 1: # add condition to check if distance is less than 2 meters
            if self.gyro.getRate() < 720: # if the robot is spinning too fast, ignore the vision measurement
                # Add the vision measurement to the PoseEstimator
                self.poseEstimator.addVisionMeasurement(
                                        Pose2d(
                                            self.botpose_wpiblue[0], 
                                            self.botpose_wpiblue[1], 
                                            Rotation2d.fromDegrees(self.botpose_wpiblue[5])
                                            ), 
                                        Timer.getFPGATimestamp() - (self.cl + self.tl), # Time since the vision measurement was taken
                                        )
            
        wpilib.SmartDashboard.putNumber("Validity",self.validity)
        
        ##############################################################################################
############## Driving Modes

    def nextDrivingMode(self):
        ''' Changes to the next driving mode '''
        self.drivingMode = (self.drivingMode + 1) % 4

    def getDrivingMode(self):
        ''' Returns the current driving mode '''
        return self.drivingMode
    
    def setDrivingMode(self, mode):
        ''' Sets the driving mode '''
        self.drivingMode = mode

##############################################################################################
############## Periodic Methods

    # Periodic is called every cycle (20ms)
    def periodic(self):
        ''' The code that runs periodically '''
        # Read Limelight
        self.readLimelight()
    
        # Update Odometry
        if self.PoseEstimatorInit == True:
            self.updateOdometry()
        self.odometer.update(
                            self.getRotation2d(),
                            (
                                self.frontLeft.getSwerveModulePosition(), 
                                self.frontRight.getSwerveModulePosition(), 
                                self.backLeft.getSwerveModulePosition(), 
                                self.backRight.getSwerveModulePosition()
                            )
                        )
            
        # Sends data to dashboard
        # wpilib.SmartDashboard.putNumber("Pose X", self.getPose().X())
        # wpilib.SmartDashboard.putNumber("Pose Y", self.getPose().Y())
        # wpilib.SmartDashboard.putNumber("Pose Heading", self.getPose().rotation().degrees())
        wpilib.SmartDashboard.putNumber("PoseBlueX", self.botpose_wpiblue[0])
        wpilib.SmartDashboard.putNumber("PoseBlueY", self.botpose_wpiblue[1])
        # wpilib.SmartDashboard.putNumber("Validity LL", self.validity)
        # wpilib.SmartDashboard.putNumber("AprilTag Number", self.aprilTagNumber)
        wpilib.SmartDashboard.putNumber("PoseBlueHeading", self.botpose_wpiblue[5])
        wpilib.SmartDashboard.putNumber("Gen Vel", self.getDesiredVelocity())
        
        # Reads Absolute Encoders and sends them to Dashboard
        # absoluteEncoder = self.readAbsEncoders()
        # wpilib.SmartDashboard.putNumber("AbsEnc FL", absoluteEncoder[0])
        # wpilib.SmartDashboard.putNumber("AbsEnc FR", absoluteEncoder[1])
        # wpilib.SmartDashboard.putNumber("AbsEnc BL", absoluteEncoder[2])
        # wpilib.SmartDashboard.putNumber("AbsEnc BR", absoluteEncoder[3])
        
        # # Reads Encoders and sends them to Dashboard
        # turnings = self.readTurnEncoders()
        # wpilib.SmartDashboard.putNumber("Turning FL", turnings[0])
        # wpilib.SmartDashboard.putNumber("Turning FR", turnings[1])
        # wpilib.SmartDashboard.putNumber("Turning BL", turnings[2])
        # wpilib.SmartDashboard.putNumber("Turning BR", turnings[3])

        # # Reads Distance Travelled and sends them to Dashboard
        # forwards = self.readForwardEncoders()
        # wpilib.SmartDashboard.putNumber("Forward FL", forwards[0])
        # wpilib.SmartDashboard.putNumber("Forward FR", forwards[1])
        # wpilib.SmartDashboard.putNumber("Forward BL", forwards[2])
        # wpilib.SmartDashboard.putNumber("Forward BR", forwards[3])

        # wpilib.SmartDashboard.putNumber("Heading", self.getHeading())
        # wpilib.SmartDashboard.putNumber("Continuous Heading", self.getContinuousHeading())

        # wpilib.SmartDashboard.putNumber("OdoX", self.odometer.getPose().X())
        # wpilib.SmartDashboard.putNumber("OdoY", self.odometer.getPose().Y())
        # wpilib.SmartDashboard.putNumber("OdoT", self.odometer.getPose().rotation().degrees())
        wpilib.SmartDashboard.putNumber("VX",self.xvelocity)
        wpilib.SmartDashboard.putNumber("VY",self.yvelocity)
        wpilib.SmartDashboard.putNumber("VTheta",self.turningvelocity)
        