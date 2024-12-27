import wpimath.geometry, wpimath.trajectory, wpimath.controller
from constants import DriveConstants, AutoConstants
import commands2
import wpimath.trajectory
from subsystems.SwerveSubsystem import SwerveSubsystem
from math import pi
import json


#Accepts current swerveSubsytem and file path of the trajectory to be followed
def AutoTrajectory(swerveSubsystem: SwerveSubsystem,file: str):
        swerveSubsystem = swerveSubsystem
        with open(file, 'r') as f:
            data = json.load(f)
            rotation = wpimath.geometry.Rotation2d(data[0]["pose"]["rotation"]["radians"])
            translationx = data[0]["pose"]["translation"]["x"]
            translationy = data[0]["pose"]["translation"]["y"]
            translation = wpimath.geometry.Translation2d(x=translationx, y=translationy)
            
        startPose = wpimath.geometry.Pose2d(rotation=rotation, translation=translation)
        trajectory_config = wpimath.trajectory.TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, 
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        trajectory_config.setKinematics(DriveConstants.kDriveKinematics)
        trajectory = wpimath.trajectory.TrajectoryUtil.fromPathweaverJson(file)

    
        xController = wpimath.controller.PIDController(7.8, 0.8, .8)
        yController = wpimath.controller.PIDController(7.8, 0.8, .8)
       
        ThetaProfiledPIDController = wpimath.controller.ProfiledPIDControllerRadians(0.5, 0, 0, AutoConstants.kThetaControllerConstraints)
        
        ThetaProfiledPIDController.enableContinuousInput(-pi, pi)

        swerveControllerCommand = commands2.SwerveControllerCommand( 
            trajectory=trajectory, 
            pose=lambda : swerveSubsystem.getPose(), 
            kinematics=DriveConstants.kDriveKinematics,
            controller=wpimath.controller.HolonomicDriveController(xController,
                yController,
                ThetaProfiledPIDController),
            outputModuleStates=lambda listModuleStates : swerveSubsystem.setModuleStates(listModuleStates),
            requirements=(swerveSubsystem,None),
            desiredRotation=None
        )

        
        sequentialCommandGroup = commands2.SequentialCommandGroup(
            commands2.InstantCommand(lambda : swerveSubsystem.resetOdometer(startPose), swerveSubsystem),
            swerveControllerCommand,
            commands2.InstantCommand(lambda : swerveSubsystem.stopModules())
        )

        return sequentialCommandGroup
