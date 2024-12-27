from commands2 import Command
from wpimath.filter import SlewRateLimiter
from constants import DriveConstants,OIConstants
from wpimath.kinematics import ChassisSpeeds

signum = lambda x : (x>0)-(x<0) # Function to get the sign of a number (Used?)

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
        # Get the x, y, and rotation values from the joystick
        x = self.xSpeedFunction()
        y = self.ySpeedFunction()
        rot = self.turningSpeedFunction()

        # Apply a deadband to the joystick
        self.xSpeed = x if abs(x) > OIConstants.kDeadband else 0.0
        self.ySpeed = y if abs(y) > OIConstants.kDeadband else 0.0
        self.turningSpeed = rot if abs(rot) > OIConstants.kDeadband else 0.0

        # Saves the last 3 values of the joystick to determine if the joystick is at 0
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

        # If the joystick is at 0, reset the limiter and set the speed to 0
        if self.x_direction_states[0]==0 and self.x_direction_states[1]==0 and self.x_direction_states[2]==0:
            self.XLimiter.reset(0)
            self.xSpeed=0
        else: # TO BE CHECKED!!
            self.xSpeed = self.XLimiter.calculate(self.xSpeed)*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond

        if self.y_direction_states[0]==0 and self.y_direction_states[1]==0 and self.y_direction_states[2]==0:
            self.YLimiter.reset(0)
            self.ySpeed=0
        else: # TO BE CHECKED!!
            self.ySpeed = self.YLimiter.calculate(self.ySpeed)*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond

        # TO BE CHECKED!!
        self.turningSpeed *= DriveConstants.kTeleDriveMaxAngularRadiansPerSecond

        self.chassisSpeeds = ChassisSpeeds()

        # if (self.fieldOrientedFunction):
        #     self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        #         self.xSpeed, self.ySpeed, self.turningSpeed, self.swerveSubsystem.getRotation2d()
        #     )
        # else:
        #     self.chassisSpeeds = ChassisSpeeds(self.xSpeed, self.ySpeed, self.turningSpeed)

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

    
