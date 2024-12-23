# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

"""
A place for the constant values in the code that may be used in more than one place. 
This offers a convenient resources to teams who need to make both quick and universal
changes.
"""

import math
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d
import wpimath
import wpilib
import wpimath.trajectory

class DriveConstants:
    absoulteEncoderCountsPerRev = 4096
    kFrontLeftDriveMotorPort = 12
    kFrontLeftTurningMotorPort = 11
    kFrontLeftDriveMotorReversed = False
    kFrontLeftTurningMotorReversed = False
    kFrontLeftAbsoluteEncoderPort = 13
    kFrontLeftAbsoluteEncoderOffsetRad = (2906/absoulteEncoderCountsPerRev)*(2*math.pi)#
    kFrontLeftAbsoluteEncoderReversed = False
    kFrontLeftForwardPIDk = [0.1, 0, 0, 0.1, 0, 0]

    kFrontRightDriveMotorPort = 22
    kFrontRightTurningMotorPort = 21
    kFrontRightDriveMotorReversed = True
    kFrontRightTurningMotorReversed = False
    kFrontRightAbsoluteEncoderPort = 23
    kFrontRightAbsoluteEncoderOffsetRad = (87/absoulteEncoderCountsPerRev)*(2*math.pi)
    kFrontRightAbsoluteEncoderReversed = False
    kFrontRightForwardPIDk = [0.1, 0, 0, 0.1, 0, 0]

    kBackLeftDriveMotorPort = 32
    kBackLeftTurningMotorPort = 31
    kBackLeftDriveMotorReversed = True
    kBackLeftTurningMotorReversed = False
    kBackLeftAbsoluteEncoderPort = 33
    kBackLeftAbsoluteEncoderOffsetRad = (201/absoulteEncoderCountsPerRev)*(2*math.pi)
    kBackLeftAbsoluteEncoderReversed = False
    kBackLeftForwardPIDk = [0.1, 0, 0, 0.1, 0, 0]

    kBackRightDriveMotorPort = 42
    kBackRightTurningMotorPort = 41
    kBackRightDriveMotorReversed = False
    kBackRightTurningMotorReversed = False
    kBackRightAbsoluteEncoderPort = 43
    kBackRightAbsoluteEncoderOffsetRad = (2851/absoulteEncoderCountsPerRev)*(2*math.pi)
    kBackRightAbsoluteEncoderReversed = False
    kBackRightForwardPIDk = [0.1, 0, 0, 0.1, 0, 0]

    kEncoderCPR = 1024
    kWheelDiameterInches = 4

    shuffleMotor = 0

    # Assumes the encoders are directly mounted on the wheel shafts
    kEncoderDistancePerPulse = (kWheelDiameterInches * math.pi) / kEncoderCPR

    #THIS IS IN METERS PER SECOND. This means at 100% speed how fast is the robot going. I suggest we run tests to figure this out. We can use the navx to display the speed in meters per second and give the robot max power without the limiters.
    kPhysicalMaxSpeedMetersPerSecond = 9 #9 MPS is about 20 miles per hour 

    kGyroReversed = False

    kStabilizationP = 1
    kStabilizationI = 0.5
    kStabilizationD = 0

    kTurnP = 1
    kTurnI = 0
    kTurnD = 0

    kMaxTurnRateDegPerS = 300 # was 100
    kMaxTurnAccelerationDegPerSSquared = 100 # was 300

    kMaxTurnRateRadPerS = math.radians(kMaxTurnRateDegPerS)
    kMaxTurnAccelerationRadPerSSquared = math.radians(kMaxTurnAccelerationDegPerSSquared)

    kTurnToleranceDeg = 5
    kTurnRateToleranceDegPerS = 5  # degrees per second, was 10

    kTrackWidth = 0.60  #meters
    kWheelBase = 0.59 #meters
    kDriveKinematics = SwerveDrive4Kinematics(
        Translation2d(kWheelBase/2, kTrackWidth/2),  #LF
        Translation2d(kWheelBase/2, -kTrackWidth/2), #RF
        Translation2d(-kWheelBase/2, kTrackWidth/2), #BL
        Translation2d(-kWheelBase/2, -kTrackWidth/2) #BR
    )
    #this ends up being the damp factor. Right now this is 4.5/9 meaning the max output of the motors should be 50%
    kTeleDriveMaxSpeedMetersPerSecond = 6.8 #14.5*0.3048 #14.5 feet per second into meters
    kTeleDriveMaxAngularRadiansPerSecond = 12 #(14.5*12)*0.3048/2 #Transformed 14.5 feet into radians per sec

    kTeleDriveMaxAccelerationUnitsPerSeconds = kTeleDriveMaxSpeedMetersPerSecond #Taken from MaxSpeedDrive

    kTeleDriveMaxAngularAccelerationUnitsPerSeconds = 0.8 #2*12*0.3048/2 #Transformed from MaxAcceleration

    kTeleDriveMaxDeccelrationUnitsPerSecond = kTeleDriveMaxSpeedMetersPerSecond

    kDistanceSensorID = 7


class ModuleConstants:
    # Checked
    kWheelDiameterMeters = 0.1016 # 4 inches into meters
    kDriveMotorGearRatio = 1/6.12 # L3 gearbox
    kTurningMotorGearRatio = 1/12.8
    kDriveEncoderRot2Meter = kDriveMotorGearRatio*math.pi*kWheelDiameterMeters
    kTurningEncoderRot2Rad = kTurningMotorGearRatio*2*math.pi
    kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter/60
    kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad/60
    
    # Do be checked
    kPTurning = 0.4
    kDTurning = 0.1
    #maxSpeed = 0.6


class OIConstants:
    kDriverControllerPort = 0
    kCodriverControllerPort = 1
    kDeadband = 0.05
    kDriverYAxis = 1
    kDriverXAxis = 0
    kDriverRotAxis = 4
    kDriverFieldOrientedButtonIdx = 3

class BlinkinConstants:
    kBlinkinPort = 0 #PWM Controller

class AutoConstants:
    from constants import DriveConstants
    kMaxSpeedMetersPerSecond = 6
    kMaxAccelerationMetersPerSecondSquared = 3
    kPThetaController = 1
    kThetaControllerConstraints = wpimath.trajectory.TrapezoidProfileRadians.Constraints(DriveConstants.kMaxTurnRateRadPerS, DriveConstants.kMaxTurnAccelerationRadPerSSquared)

    kPXController = 0.1
    kPYController = 0.1
