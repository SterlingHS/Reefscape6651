# Sets up the Algae Collector to spit out the ball (Algae)

from commands2 import Command
from subsystems.AlgaeCollector import AlgaeCollector
from constants import AlgaeCollectorConstants


class ACBallOutSetUp(Command):
    def __init__(self, algaeC:AlgaeCollector):
        Command.__init__(self)
        self.algaeC = algaeC

    def initialize(self) -> None:
        self.algaeC.setArmHeight(AlgaeCollectorConstants.algaeArmCollecting)
        self.algaeC.setStarSpeed(-AlgaeCollectorConstants.starSpeed)
        return super().initialize()
    
    def execute(self) -> None:
        pass

    def end(self, interrupted: bool) -> None:
        self.algaeC.setArmHeight(AlgaeCollectorConstants.highPosition)
        self.algaeC.setStarSpeed(0)
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return super().isFinished()


    