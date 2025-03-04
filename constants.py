# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

"""
A place for the constant values in the code that may be used in more than one place. 
This offers a convenient resources to teams who need to make both quick and universal
changes.
"""

from enum import Enum
import math
import unit_conversions as conv
import wpilib
import wpimath.trajectory
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d
import wpimath

class DriveConstants:

    kFrontLeftDriveMotorPort = 12
    kFrontLeftTurningMotorPort = 11
    kFrontLeftDriveMotorReversed = False
    kFrontLeftTurningMotorReversed = True
    kFrontLeftAbsoluteEncoderPort = 13
    kFrontLeftAbsoluteEncoderOffsetRad = -1.270136
    kFrontLeftAbsoluteEncoderReversed = False
    kFrontLeftForwardPIDk = [0.16095, 0, 0, 0.072731, 2.1947, 0.033795] # [P, I, D, kS, KV, kA]

    kFrontRightDriveMotorPort = 22
    kFrontRightTurningMotorPort = 21
    kFrontRightDriveMotorReversed = True
    kFrontRightTurningMotorReversed = True
    kFrontRightAbsoluteEncoderPort = 23
    kFrontRightAbsoluteEncoderOffsetRad = 1.67050507
    kFrontRightAbsoluteEncoderReversed = False
    kFrontRightForwardPIDk = [0.65477, 0, 0, 0.038577, 2.2129, 0.098999] # [P, I, D, kS, KV, kA]

    kBackLeftDriveMotorPort = 32
    kBackLeftTurningMotorPort = 31
    kBackLeftDriveMotorReversed = False
    kBackLeftTurningMotorReversed = True
    kBackLeftAbsoluteEncoderPort = 33
    kBackLeftAbsoluteEncoderOffsetRad = -0.3021942
    kBackLeftAbsoluteEncoderReversed = False
    kBackLeftForwardPIDk = [0.2359, 0, 0, 0.06964, 2.2443, 0.059663] # [P, I, D, kS, KV, kA]

    kBackRightDriveMotorPort = 42
    kBackRightTurningMotorPort = 41
    kBackRightDriveMotorReversed = True
    kBackRightTurningMotorReversed = True
    kBackRightAbsoluteEncoderPort = 43
    kBackRightAbsoluteEncoderOffsetRad = 0.409157827
    kBackRightAbsoluteEncoderReversed = False
    kBackRightForwardPIDk = [0.065547, 0, 0, 0.074308, 2.1954, 0.037429] # [P, I, D, kS, KV, kA]

    #THIS IS IN METERS PER SECOND. This means at 100% speed how fast is the robot going. I suggest we run tests to figure this out. We can use the navx to display the speed in meters per second and give the robot max power without the limiters.
    kPhysicalMaxSpeedMetersPerSecond = 3 #5 MPS is about 11 miles per hour 

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
    kTeleDriveMaxSpeedMetersPerSecond = 3 #17*0.3048 #17 feet per second into meters
    kTeleDriveMaxAngularRadiansPerSecond = 8.5 # 17 ft/sec * (2pi radians / (2pi* 2 ft)) #Transformed 17 feet/sec into radians/sec

    kTeleDriveMaxAccelerationUnitsPerSeconds = 3#kTeleDriveMaxSpeedMetersPerSecond #Taken from MaxSpeedDrive
    kTeleDriveMaxAngularAccelerationUnitsPerSeconds = 0.8 #2*12*0.3048/2 #Transformed from MaxAcceleration

    # For autonomous mode (PathPlanner)
    kDriveBaseRadius = math.sqrt((kTrackWidth/2)**2+(kWheelBase/2)**2) # Diagonal distance from center to wheel

    # Drive enabler
    DriveEnabled = False

class ModuleConstants:
    kWheelDiameterMeters = 0.1016 # 4 inches into meters
    kDriveMotorGearRatio = 1/6.12 # L3 gearbox
    kTurningMotorGearRatio = 1/(150/7) # 150:7 gear ratio
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
    LaserTopCanID = 55
    LaserBottomCanID = 56
    DropperMotorID = 60
    DropperReversed = False
    P = 0.0002005 # 0.0001
    I = 0
    D = 0
    kS = 0.52148
    kV = 0.0010255
    kA = 0.00012698
    kG = 0.0022378
    F = 0.0021141649
    DropSpeed = 180
    DropIntakeSpeed = 1
    DropTransitSpeed = 0.5

class ElevatorConstants:

    # PID results - With Springs
    # Ks = 0.12955
    # Kv = 0.21955
    # Ka = 0.022299
    # Kg = 0.16265
    # Kp = 0.021546

    ElevatorMotorID1 = 62 # Sparkmax with limit switches
    ElevatorMotorID2 = 61 # Follower
    ElevatorReversed1 = True
    ElevatorReversed2 = True
    MaxVelocityUp = 1
    MaxVelocityDown = -1
    
    # For maxMotion
    MaxRPM = 8000
    MaxAcceleration = 8000

    # Max and Min values for the elevator for Soft Limits
    Min = 0
    Max = 54

    # PID Constants
    # PID for L1, L2 and L3 - PID Position
    P0 = .025
    I0 = 0
    D0 = .01
    kF0 = 0 # For Elevator WITH Springs
    # kF0 = 0.0021141649 # For Elevator WITHOUT Springs

    # PID for L4 - PID
    P1 = .015
    I1 = 0
    kF0 = 0 # For Elevator WITH Springs
    # kF0 = 0.0021141649 # For Elevator WITHOUT Springs

    # Encoder Constants Conversion
    kElevatorEncoderRot2Meter = 1.0121457*52.5/92.38
    kElevatorEncoderRPM2MeterPerSec = kElevatorEncoderRot2Meter/60

    # Position of different levels
    L1 = 1
    L2 = 14 # 11 inches
    L3 = 30 # 28 inches
    L4 = 54 # 52 inches, Max height 54 inches
    
class AlgaeRemoverConstants:
    ARStarMotorID = 47
    ARArmMotorID = 48
    ARStarReversed = False
    ARArmReversed = False
    Max = 6
    P = 0.03
    I = 0
    D = 0.005
    starSpeed = 0.1
    highPosition = 1
    lowPosition = 4

class AlgaeCollectorConstants:
    ACStarMotorID = 45
    ACArmMotorID = 46
    ACStarReversed = False
    ACArmReversed = False
    Max = 0
    P = 0.03
    I = 0
    D = 0.005
    starSpeed = 0.1
    highPosition = -1
    lowPosition = -12
    algaeArmHeight = -11

class FieldOrientedConstants: 
    # Andymark Field
    # Centers in inches: Blue (176.745, 158.30) and Red (514.13, 158.30)
    # Centers in meter: Blue (4.489, 4.020) and Red (13.059, 4.020)
    # Welded Field
    # Centers: Blue (176.745, 158.50) and Red (514.13, 158.50)
    # Centers in meter: Blue (4.489, 4.026) and Red (13.059, 4.026)
    RedReefX = 4.489 
    RedReefY = 13.059
    BlueReefX = 4.489
    BlueReefY = 4.020

class ReefPositions: 
    # List of reef angles depending on the reef number
    reefAngles = [126,234,270,0,0,300,0,60,120,180,240,54,306,180,180,90,240,180,120,60,0,300]

#     # (x,y,angle) in meters and radians
#     # First tuple is center
#     # Second tuple is left reef (6 inches from center)
#     # Third tuple is right reef (6 inches from center)
#     ### Blue side
#     B17 = ((,,),(,,),(,,))
#     B18 =
#     ...

#     ### Red side
#     B6 = 
#     ...


class DrivingModes(Enum):
    # Mode 0 = Field Oriented
    # Mode 1 = Reef Oriented
    # Mode 2 = Processor Oriented
    # Mode 3 = Coral Station Oriented
    # Coral Station Processor Oriented
    #   Red2   - 234 degrees
    #   Blue12 -  54 degrees
    # Coral Station Far side Oriented
    #   Red1   - 126 degrees
    #   Blue13 - 306 degrees
    # Mode 4 = Reef AprilTag Oriented
    FieldOriented = 0
    ReefOriented = 1
    ProcessorOriented = 2
    CoralStationOriented = 3
    ReefAprilTageOriented = 4