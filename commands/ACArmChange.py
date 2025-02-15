from commands2 import Command
from subsystems.AlgaeCollector import AlgaeCollector


class ACArmChange(Command):
    def __init__(self, algaeC:AlgaeCollector, changeHeight:int):
        Command.__init__(self)
        self.algaeC = algaeC
        self.changeHeight = changeHeight

    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        # self.elevator.setElevatorFloor(self.floor)
        self.algaeC.setArmHeight(self.changeHeight)

    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return True


    