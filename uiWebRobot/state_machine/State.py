import utility
from uiWebRobot.state_machine.FrontEndObjects import FrontEndObjects, ButtonState
from shared_class.robot_synthesis import RobotSynthesis
from logger import LoggerFactory

class State(object):

    def __init__(self, socketio, file_logger: utility.Logger):
        self.__logger = LoggerFactory.create(self.__class__.__name__)
        self.socketio = socketio
        self.__file_logger = file_logger
        self.robot_synthesis_value: RobotSynthesis = None
        msg = 'Processing current state :', str(self)
        self.__file_logger.write_and_flush(msg)
        self.__logger.info(msg)
        self.statusOfUIObject: FrontEndObjects = FrontEndObjects(   fieldButton=ButtonState.ENABLE,
                                                                    startButton=ButtonState.ENABLE,
                                                                    continueButton=ButtonState.ENABLE,
                                                                    stopButton=ButtonState.NOT_HERE,
                                                                    wheelButton=ButtonState.DISABLE,
                                                                    removeFieldButton=ButtonState.ENABLE,
                                                                    joystick=True,
                                                                    slider=0)

    def on_event(self, event):
        """
        Handle events that are delegated to this State.
        """
        pass

    def on_socket_data(self, data):
        """
        Handle navigation events that are delegated to this State.
        """
        pass 

    def getStatusOfControls(self):
        return self.statusOfUIObject
    
    def getField(self):
        return self.field

    def __repr__(self):
        """
        Leverages the __str__ method to describe the State.
        """
        return self.__str__()

    def __str__(self):
        """
        Returns the name of the State.
        """
        return self.__class__.__name__