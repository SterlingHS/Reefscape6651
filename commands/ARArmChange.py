# Changes the position of the Algae Remover arm

from commands2 import Command
from subsystems.AlgaeRemover import AlgaeRemover

class ARArmChange(Command):
    def __init__(self, algaeR:AlgaeRemover, changeHeight:int):
        Command.__init__(self)
        self.algaeR = algaeR
        self.changeHeight = changeHeight

    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        # self.elevator.setElevatorFloor(self.floor)
        self.algaeR.setArmHeight(self.changeHeight)

    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return True


    