import sys
sys.path.append('../')

import utility

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