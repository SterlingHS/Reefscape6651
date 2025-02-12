# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

"""
A place for the constant values in the code that may be used in more than one place. 
This offers a convenient resources to teams who need to make both quick and universal
changes.
"""

import math
import unit_conversions as conv
import wpilib
import wpimath.trajectory
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d
import wpimath

class DriveConstants:
    # absoulteEncoderCountsPerRev = 4096

    kFrontLeftDriveMotorPort = 12 # neos
    kFrontLeftTurningMotorPort = 11 # sparks
    kFrontLeftDriveMotorReversed = False
    kFrontLeftTurningMotorReversed = True
    kFrontLeftAbsoluteEncoderPort = 1
    kFrontLeftAbsoluteEncoderOffsetRad = 1.94479545
    kFrontLeftAbsoluteEncoderReversed = True
    kFrontLeftForwardPIDk = [0.16095, 0, 0, 0.072731, 2.1947, 0.033795] # [P, I, D, kS, KV, kA]

    kFrontRightDriveMotorPort = 14 # neos
    kFrontRightTurningMotorPort = 13 # sparks
    kFrontRightDriveMotorReversed = True
    kFrontRightTurningMotorReversed = True
    kFrontRightAbsoluteEncoderPort = 3
    kFrontRightAbsoluteEncoderOffsetRad = .24
    kFrontRightAbsoluteEncoderReversed = True
    kFrontRightForwardPIDk = [0.65477, 0, 0, 0.038577, 2.2129, 0.098999] # [P, I, D, kS, KV, kA]

    kBackLeftDriveMotorPort = 22 # neos
    kBackLeftTurningMotorPort = 21 # sparks
    kBackLeftDriveMotorReversed = False
    kBackLeftTurningMotorReversed = True
    kBackLeftAbsoluteEncoderPort = 0
    kBackLeftAbsoluteEncoderOffsetRad = -3.14
    kBackLeftAbsoluteEncoderReversed = True
    kBackLeftForwardPIDk = [0.2359, 0, 0, 0.06964, 2.2443, 0.059663] # [P, I, D, kS, KV, kA]

    kBackRightDriveMotorPort = 24 # neos
    kBackRightTurningMotorPort = 23 # sparks
    kBackRightDriveMotorReversed = True
    kBackRightTurningMotorReversed = True
    kBackRightAbsoluteEncoderPort = 2
    kBackRightAbsoluteEncoderOffsetRad = 2.004 
    kBackRightAbsoluteEncoderReversed = True
    kBackRightForwardPIDk = [0.065547, 0, 0, 0.074308, 2.1954, 0.037429] # [P, I, D, kS, KV, kA]

    #THIS IS IN METERS PER SECOND. This means at 100% speed how fast is the robot going. I suggest we run tests to figure this out. We can use the navx to display the speed in meters per second and give the robot max power without the limiters.
    kPhysicalMaxSpeedMetersPerSecond = 5 #5 MPS is about 11 miles per hour 

    kMaxTurnRateDegPerS = 300 
    kMaxTurnAccelerationDegPerSSquared = 100 

    kMaxTurnRateRadPerS = math.radians(kMaxTurnRateDegPerS)
    kMaxTurnAccelerationRadPerSSquared = math.radians(kMaxTurnAccelerationDegPerSSquared)

    kTrackWidth = conv.inches_meters(22+5/8)  # 22 5/8 inches into meters
    kWheelBase = kTrackWidth # Square robot

    kDriveKinematics = SwerveDrive4Kinematics(
        Translation2d(kWheelBase/2, kTrackWidth/2),  #LF
        Translation2d(kWheelBase/2, -kTrackWidth/2), #RF
        Translation2d(-kWheelBase/2, kTrackWidth/2), #BL
        Translation2d(-kWheelBase/2, -kTrackWidth/2) #BR
    )
    #this ends up being the damp factor. Right now this is 4.5/9 meaning the max output of the motors should be 50%
    kTeleDriveMaxSpeedMetersPerSecond = 5 #17*0.3048 #17 feet per second into meters
    kTeleDriveMaxAngularRadiansPerSecond = 8.5 # 17 ft/sec * (2pi radians / (2pi* 2 ft)) #Transformed 17 feet/sec into radians/sec

    kTeleDriveMaxAccelerationUnitsPerSeconds = kTeleDriveMaxSpeedMetersPerSecond #Taken from MaxSpeedDrive
    kTeleDriveMaxAngularAccelerationUnitsPerSeconds = 0.8 #2*12*0.3048/2 #Transformed from MaxAcceleration

    # For autonomous mode (PathPlanner)
    kDriveBaseRadius = math.sqrt((kTrackWidth/2)**2+(kWheelBase/2)**2) # Diagonal distance from center to wheel

class ModuleConstants:
    kWheelDiameterMeters = 0.1016 # 4 inches into meters
    kDriveMotorGearRatio = 1/6.12 # L3 gearbox
    kTurningMotorGearRatio = 1/12.8 # 12.8:1 gear ratio
    kDriveEncoderRot2Meter = kDriveMotorGearRatio * math.pi * kWheelDiameterMeters
    kTurningEncoderRot2Rad = kTurningMotorGearRatio*2*math.pi
    kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter/60
    kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad/60
    
    # To be checked
    kPTurning = 0.6
    kITurning = 0
    kDTurning = 0.02

class OIConstants:
    kDriverControllerPort = 0
    kCodriverControllerPort = 1
    kDeadband = 0.05
    kDriverYAxis = 1
    kDriverXAxis = 0
    kDriverRotAxis = 4
    kDriverFieldOrientedButtonIdx = 3


class AutoConstants:
    from constants import DriveConstants
    kMaxSpeedMetersPerSecond = 5
    kMaxAccelerationMetersPerSecondSquared = 3
    kPThetaController = 1
    kThetaControllerConstraints = wpimath.trajectory.TrapezoidProfileRadians.Constraints(DriveConstants.kMaxTurnRateRadPerS, DriveConstants.kMaxTurnAccelerationRadPerSSquared)
    autoTranslationP = 5
    autoTranslationI = 0
    autoTranslationD = 0
    autoRotationP = 5
    autoRotationI = 0
    autoTRotationD = 0
    kPXController = 0.1
    kPYController = 0.1

class DropperConstants:
    DropperMotorID = 60
    DropperReversed = False
    P = 0.6 
    I = 0
    D = 0.02

class ElevatorConstants:
    ElevatorMotorID1 = 61
    ElevatorMotorID2 = 62
    ElevatorReversed1 = False
    ElevatorReversed2 = True
    MaxVelocityUp = .8
    MaxVelocityDown = -0.4
    
    # For maxMotion
    MaxRPM = 8000
    MaxAcceleration = 5000

    # Max and Min values for the elevator for Soft Limits
    Max = 53
    Min = 0

    # PID Constants
    P1 = .5 # it may be 0.05
    I1 = 0
    D1 = .4 # it may be 0.02 

    # Encoder Constants Conversion
    kElevatorEncoderRot2Meter = 1.0121457*52.5/92.38
    kElevatorEncoderRPM2MeterPerSec = kElevatorEncoderRot2Meter/60

    # Position of different levels
    L1 = 0
    L2 = 13 # 11 inches
    L3 = 29 # 28 inches
    L4 = 53 # 52 inches
    
