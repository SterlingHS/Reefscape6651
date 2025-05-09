# Sets up the Algae Collector for the Ball In command

from commands2 import Command
from subsystems.AlgaeCollector import AlgaeCollector
from constants import AlgaeCollectorConstants

class ACBallInSetUp(Command):
    def __init__(self, algaeC:AlgaeCollector):
        Command.__init__(self)
        self.algaeC = algaeC

    def initialize(self) -> None:
        self.algaeC.setArmHeight(AlgaeCollectorConstants.algaeArmCollecting)
        self.algaeC.setStarSpeed(AlgaeCollectorConstants.starSpeed)
        return super().initialize()
    
    def execute(self) -> None:
        if self.algaeC.readArmEncoder() <= AlgaeCollectorConstants.algaeArmCollecting:
            #self.algaeC.stopArmMotor()
            self.algaeC.setArmMotor(-.02) # Force to hold the arm from bouncing
            self.algaeC.setMode(1)
        pass
            
    def end(self, interrupted: bool) -> None:
        self.algaeC.stopArmMotor()
        self.algaeC.setStarSpeed(0)
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return super().isFinished()


    