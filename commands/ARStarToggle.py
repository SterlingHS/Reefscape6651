from commands2 import Command
from subsystems.AlgaeRemover import AlgaeRemover


class ARStarToggle(Command):
    def __init__(self, algaeR:AlgaeRemover):
        Command.__init__(self)
        self.algaeR = algaeR

    def initialize(self) -> None:
        return super().initialize()
    
    def execute(self) -> None:
        # self.elevator.setElevatorFloor(self.floor)
        self.algaeR.toggleStar()

    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return True


    