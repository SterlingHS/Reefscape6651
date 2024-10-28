from commands2 import Command
from wpimath.filter import SlewRateLimiter
from constants import DriveConstants,OIConstants
from wpimath.kinematics import ChassisSpeeds
import wpilib
import ntcore
signum = lambda x : (x>0)-(x<0)
class SwerveJoystickCmd(Command):
    def __init__(self, swerveSubsystem, xSpeedFunction, ySpeedFunction, turningSpeedFunction):
        Command.__init__(self)
        

        self.swerveSubsystem = swerveSubsystem
        self.xSpeedFunction = xSpeedFunction
        self.ySpeedFunction = ySpeedFunction
        self.turningSpeedFunction = turningSpeedFunction
        # the limiters are giving us lag. They don't allow us to slow down quick enough. When we dampen the modules this lag slows down. We need to allow 0 to bypass all of this and stop the modules
        self.XLimiter = SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSeconds)
        self.YLimiter = SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSeconds)

        self.x_direction_states = [0,0,0]
        self.y_direction_states = [0,0,0]
        self.addRequirements(swerveSubsystem)

    def initialize(self) -> None:
        return super().initialize()

    def execute(self) -> None:
        x = self.xSpeedFunction()
        y = self.ySpeedFunction()
        rot = self.turningSpeedFunction()

        self.xSpeed = x if abs(x) > OIConstants.kDeadband else 0.0
        self.ySpeed = y if abs(y) > OIConstants.kDeadband else 0.0
        try:
            tx = ntcore.NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0)

            
            if tx != 0:
                if tx > 1.5:
                    turningSpeed = 0.055
                elif tx < -1.5:
                    turningSpeed = -0.055
                else:
                    turningSpeed = 0
            else:
                turningSpeed = 0
        except:
            turningSpeed = 0
        self.turningSpeed = (rot+turningSpeed) if abs(rot+turningSpeed) > OIConstants.kDeadband else 0.0


        if len(self.x_direction_states)<3:
            self.x_direction_states.append(self.xSpeed)
        else:
            self.x_direction_states.pop(0)
            self.x_direction_states.append(self.xSpeed)

        if len(self.y_direction_states)<3:
            self.y_direction_states.append(self.ySpeed)
        else:
            self.y_direction_states.pop(0)
            self.y_direction_states.append(self.ySpeed)

        

        if self.x_direction_states[0]==0 and self.x_direction_states[1]==0 and self.x_direction_states[2]==0:
            self.XLimiter.reset(0)
            self.xSpeed=0
        else:
            self.xSpeed = self.XLimiter.calculate(self.xSpeed)*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond

        if self.y_direction_states[0]==0 and self.y_direction_states[1]==0 and self.y_direction_states[2]==0:
            self.YLimiter.reset(0)
            self.ySpeed=0
        else:
            self.ySpeed = self.YLimiter.calculate(self.ySpeed)*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond

        self.turningSpeed *= DriveConstants.kTeleDriveMaxAngularRadiansPerSecond


        self.chassisSpeeds = ChassisSpeeds()

        """if (self.fieldOrientedFunction):
            self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                self.xSpeed, self.ySpeed, self.turningSpeed, self.swerveSubsystem.getRotation2d()
            )
        else:
            self.chassisSpeeds = ChassisSpeeds(self.xSpeed, self.ySpeed, self.turningSpeed)"""

        #self.chassisSpeeds = ChassisSpeeds(self.xSpeed, self.ySpeed, self.turningSpeed)

        self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                self.xSpeed, self.ySpeed, self.turningSpeed, self.swerveSubsystem.getRotation2d())


        moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(self.chassisSpeeds)
        self.swerveSubsystem.setModuleStates(moduleStates)
        return super().execute()

    def end(self, interrupted: bool) -> None:
        self.swerveSubsystem.stopModules()
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return super().isFinished()

    
