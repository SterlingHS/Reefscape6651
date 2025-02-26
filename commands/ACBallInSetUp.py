# Sets up the Algae Collector for the Ball In command

from commands2 import Command
from subsystems.AlgaeCollector import AlgaeCollector
from constants import AlgaeCollectorConstants

class ACBallInSetUp(Command):
    def __init__(self, algaeC:AlgaeCollector):
        Command.__init__(self)
        self.algaeC = algaeC

    def initialize(self) -> None:
        self.algaeC.setArmHeight(AlgaeCollectorConstants.algaeArmHeight)
        self.algaeC.setStarSpeed(AlgaeCollectorConstants.starSpeed)
        print("Ball In Setup")
        return super().initialize()
    
    def execute(self) -> None:
        if self.algaeC.readArmEncoder() <= AlgaeCollectorConstants.algaeArmHeight:
            self.algaeC.stopArmMotor()
            print(f"Ball In Setup - Stop Motor: {self.algaeC.readArmEncoder()} {AlgaeCollectorConstants.algaeArmHeight}")

    def end(self, interrupted: bool) -> None:
        self.algaeC.stopArmMotor()
        self.algaeC.setStarSpeed(0)
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return super().isFinished()


    