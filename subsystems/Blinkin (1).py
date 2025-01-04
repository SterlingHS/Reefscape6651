from commands2 import Subsystem
import wpilib
import constants
class Blinkin(Subsystem):
    def __init__(self):
        Subsystem.__init__(self)

        self.blinkinController = wpilib.PWMMotorController(name="blinkin",channel=constants.BlinkinConstants.kBlinkinPort)
        self.blinkinDefaultMode = -0.31
        self.blinkinMode = self.blinkinDefaultMode

    def goingForward(self):
        self.blinkinMode = 0.65
    
    def setDefaultMode(self):
        self.blinkinMode=self.blinkinDefaultMode
    
    def setBlinkinMode(self, mode):
        self.blinkinMode = mode

    def updatePWM(self):
        self.blinkinController.set(self.blinkinMode)

    