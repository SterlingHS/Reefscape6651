# 

from commands2 import Command
from subsystems.AlgaeCollector import AlgaeCollector
from constants import AlgaeCollectorConstants

class ACBallIn(Command):
    def __init__(self, algaeC:AlgaeCollector):
        Command.__init__(self)
        self.algaeC = algaeC

    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        self.algaeC.setStarMotor(AlgaeCollectorConstants.starSpeed)

    def end(self, interrupted: bool) -> None:
        self.algaeC.stopStarMotor()
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return super().isFinished()


    