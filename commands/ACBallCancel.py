# Cancel the algae collector ball command
# Stops Star and sets arm to high position

from commands2 import Command
from subsystems.AlgaeCollector import AlgaeCollector
from constants import AlgaeCollectorConstants

class ACBallCancel(Command):
    def __init__(self, algaeC:AlgaeCollector):
        Command.__init__(self)
        self.algaeC = algaeC

    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        self.algaeC.setArmHeight(AlgaeCollectorConstants.highPosition)
        self.algaeC.setStarSpeed(0)

    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return True


    