import sys
sys.path.append('../')
import utility
from enum import Enum

class Events(Enum):
    ERROR = -1
    LIST_VALIDATION = 0
    CREATE_FIELD = 1
    VALIDATE_FIELD = 2
    START_MAIN = 3
    CONTINUE_MAIN = 4
    START_AUDIT = 5
    CONTINUE_AUDIT = 6
    CONFIG_IS_SET = 7
    WORK_IS_DONE = 8
    STOP = 9
    WHEEL = 10
    AUDIT_ENABLE = 11
    AUDIT_DISABLE = 12
    VALIDATE_FIELD_NAME = 13

class State(object):

    def __init__(self, socketio, logger: utility.Logger):
        self.socketio = socketio
        self.logger = logger
        msg = 'Processing current state :', str(self)
        self.logger.write_and_flush(msg)
        print(msg)
        self.statusOfUIObject = {
            "joystick": True, #True or False
            "fieldButton": True, #True or False or charging or None
            "startButton": True, #True or False or charging or None
            "continueButton": True, #True or False or charging or None
            "stopButton": None, #True or charging or None
            "wheelButton": False, #True or False
            "audit": False #True or False or use or not-use
        }

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