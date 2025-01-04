from commands2 import Command
import subsystems.Blinkin as blinkin

class UpdateBlinkin(Command):
    def __init__(self, BlinkinSubsystem) -> None:
        self.blinkinSubsystem = BlinkinSubsystem
        super().__init__()

    def execute(self) -> None:
        self.blinkinSubsystem.updatePWM()
        return super().execute()

    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return super().isFinished()