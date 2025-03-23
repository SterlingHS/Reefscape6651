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
    kFrontLeftDriveMotorReversed = True
    kFrontLeftTurningMotorReversed = True
    kFrontLeftAbsoluteEncoderPort = 13
    kFrontLeftAbsoluteEncoderOffsetRad = 1.894466 #-1.270136
    kFrontLeftAbsoluteEncoderReversed = False
    kFrontLeftForwardPIDk = [2.0084, 0, 0, 0.08517, 2.1436, 0.3437] # [P, I, D, kS, KV, kA]
    #kFrontLeftForwardPIDk = [0.16095, 0, 0, 0.072731, 2.1947, 0.033795] # [P, I, D, kS, KV, kA]
    kFrontLeftTurningPIDk = [.5, 0, 0, 0.16923, 0.41276, 0.018337] # [P, I, D, kS, KV, kA]

    kFrontRightDriveMotorPort = 22
    kFrontRightTurningMotorPort = 21
    kFrontRightDriveMotorReversed = False
    kFrontRightTurningMotorReversed = True
    kFrontRightAbsoluteEncoderPort = 23
    kFrontRightAbsoluteEncoderOffsetRad = -1.556990499 #1.67050507
    kFrontRightAbsoluteEncoderReversed = False
    kFrontRightForwardPIDk = [1.2013, 0, 0, 0.11318, 2.1258, 0.3308] # [P, I, D, kS, KV, kA]
    # kFrontRightForwardPIDk = [0.65477, 0, 0, 0.038577, 2.2129, 0.098999] # [P, I, D, kS, KV, kA]
    kFrontRightTurningPIDk = [0.5, 0, 0, 0.15048, 0.41825, 0.025982] # [P, I, D, kS, KV, kA]

    kBackLeftDriveMotorPort = 32
    kBackLeftTurningMotorPort = 31
    kBackLeftDriveMotorReversed = True
    kBackLeftTurningMotorReversed = True
    kBackLeftAbsoluteEncoderPort = 33
    kBackLeftAbsoluteEncoderOffsetRad = 2.80411688 #-0.3021942
    kBackLeftAbsoluteEncoderReversed = False
    kBackLeftForwardPIDk = [0.94239, 0, 0, 0.15683, 2.2217, 0.13994] # [P, I, D, kS, KV, kA]
    # kBackLeftForwardPIDk = [0.2359, 0, 0, 0.06964, 2.2443, 0.059663] # [P, I, D, kS, KV, kA]
    kBackLeftTurningPIDk = [0.5, 0, 0, 0.16312, 0.41332, 0.024786] # [P, I, D, kS, KV, kA]

    kBackRightDriveMotorPort = 42
    kBackRightTurningMotorPort = 41
    kBackRightDriveMotorReversed = False
    kBackRightTurningMotorReversed = True
    kBackRightAbsoluteEncoderPort = 43
    kBackRightAbsoluteEncoderOffsetRad = -2.6675925901 #0.409157827
    kBackRightAbsoluteEncoderReversed = False
    kBackRightForwardPIDk = [0.40995, 0, 0, 0.15053, 2.2362, 0.084626] # [P, I, D, kS, KV, kA]
    # kBackRightForwardPIDk = [0.065547, 0, 0, 0.074308, 2.1954, 0.037429] # [P, I, D, kS, KV, kA]
    kBackRightTurningPIDk = [.5, 0, 0, 0.16853, 0.41734, 0.019524] # [P, I, D, kS, KV, kA]

    #THIS IS IN METERS PER SECOND. This means at 100% speed how fast is the robot going. I suggest we run tests to figure this out. We can use the navx to display the speed in meters per second and give the robot max power without the limiters.
    kPhysicalMaxSpeedMetersPerSecond = 1.5 #5 MPS is about 11 miles per hour 
    kModeSidewaysSpeedMetersPerSecond = 0.3

    kMaxTurnRateDegPerS = 200 
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
    kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond #.45 #17*0.3048 #17 feet per second into meters
    kTeleDriveMaxAngularRadiansPerSecond = 1 # 17 ft/sec * (2pi radians / (2pi* 2 ft)) #Transformed 17 feet/sec into radians/sec

    kTeleDriveMaxAccelerationUnitsPerSeconds = 3 #kTeleDriveMaxSpeedMetersPerSecond #Taken from MaxSpeedDrive
    kTeleDriveMaxAngularAccelerationUnitsPerSeconds = 0.8 #2*12*0.3048/2 #Transformed from MaxAcceleration

    # For autonomous mode (PathPlanner)
    kDriveBaseRadius = math.sqrt((kTrackWidth/2)**2+(kWheelBase/2)**2) # Diagonal distance from center to wheel

    # Drive enabler
    DriveEnabled = True

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
    kDeadbandRot = .5
    kDriverYAxis = 1
    kDriverXAxis = 0
    kDriverXRotAxis = 2
    kDriverYRotAxis = 5
    kDriverMoveRight = 8 # To be checked
    kDriverMoveLeft = 9

class AutoConstants:
    from constants import DriveConstants
    kMaxSpeedMetersPerSecond = 1
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
    P = 2.0297*10**-7 # 0.0002954
    I = 0
    D = 0
    F = 0.00109051254 # 1/917 NEO Kv
    kS = 0.2332
    kV = 0.001061
    kA = 0.00012624

    DropSpeed = 400
    DropIntakeSpeed = 200
    DropTransitSpeed = 200

class ElevatorConstants:

    # PID results - With Springs
    kS = 0.12955
    kV = 0.21955
    kA = 0.022299
    kG = 0.16265
    kP = 0.021546

    ElevatorMotorID1 = 62 # Sparkmax with limit switches
    ElevatorMotorID2 = 61 # Follower
    ElevatorReversed1 = True
    ElevatorReversed2 = True
    
    # For maxMotion
    MaxVelocity = 1200
    MaxAcceleration = 1200

    # Max and Min values for the elevator for Soft Limits
    Min = 0
    Max = 54

    # PID Constants
    # PID for L1, L2 and L3 - PID Position
    P0 = .05
    I0 = 0
    D0 = .01
    kF0 = 0 # For Elevator WITH Springs
    # kF0 = 0.0021141649 # For Elevator WITHOUT Springs

    # PID for L4 - PID
    P1 = .015
    I1 = 0
    D1 = .01
    kF1 = 0 # For Elevator WITH Springs
    # kF1 = 0.0021141649 # For Elevator WITHOUT Springs

    # Encoder Constants Conversion
    kElevatorEncoderRot2Meter = 1.0121457*52.5/92.38
    kElevatorEncoderRPM2MeterPerSec = kElevatorEncoderRot2Meter/60

    # Position of different levels
    L1 = 0
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
    ACArmReversed = True
    Max = 0
    Min = -14
    P = 0.025
    I = 0
    D = 0.005
    starSpeed = 0.4
    highPosition = 0 # It will reseted to 0 once it reaches the top
    lowPosition = -16
    algaeArmCollecting = -16

class FieldOrientedConstants: 
    # Andymark Field
    # Centers in inches: Blue (176.745, 158.30) and Red (514.13, 158.30)
    # Centers in meter: Blue (4.489, 4.020) and Red (13.059, 4.020)
    # Welded Field
    # Centers: Blue (176.745, 158.50) and Red (514.13, 158.50)
    # Centers in meter: Blue (4.489, 4.026) and Red (13.059, 4.026)
    RedReefY = 4.489 
    RedReefX = 13.059
    BlueReefY = 4.489
    BlueReefX = 4.020

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
    # Mode 1 = Reef AprilTag Oriented
    # Mode 2 = Processor Oriented
    # Mode 3 = Coral Station Oriented
    # Coral Station Processor Oriented
    #   Red2   - 234 degrees
    #   Blue12 -  54 degrees
    # Coral Station Far side Oriented
    #   Red1   - 126 degrees
    #   Blue13 - 306 degrees
    FieldOriented = 0
    ReefAprilTageOriented = 1
    ProcessorOriented = 2
    CoralStationOriented = 3