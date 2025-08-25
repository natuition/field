import sys
sys.path.append('../')

from flask_socketio import SocketIO

import utility
from uiWebRobot.state_machine import State
from config import config
from shared_class.robot_synthesis import RobotSynthesis

#This state corresponds when the robot has an error.
from uiWebRobot.state_machine.FrontEndObjects import FrontEndObjects, ButtonState

class ErrorState(State.State):

    def __init__(self, socketio: SocketIO, logger: utility.Logger, reason: str = None):
        self.robot_synthesis_value = RobotSynthesis.HS
        self.socketio = socketio
        self.logger = logger
        self.reason = reason
        if config.UI_VERBOSE_LOGGING:
            msg = f"[{self.__class__.__name__}] -> Error"
            self.logger.write_and_flush(msg+"\n")
            print(msg)

        self.statusOfUIObject = FrontEndObjects(fieldButton=ButtonState.DISABLE,
                                                startButton=ButtonState.DISABLE,
                                                continueButton=ButtonState.DISABLE,
                                                stopButton=ButtonState.NOT_HERE,
                                                wheelButton=ButtonState.NOT_HERE,
                                                removeFieldButton=ButtonState.DISABLE,
                                                joystick=False,
                                                slider=config.SLIDER_CREATE_FIELD_DEFAULT_VALUE)

        self.field = None

        self.socketio.emit('reload', {}, namespace='/broadcast', broadcast=True)

        if config.UI_VERBOSE_LOGGING:
            msg = f"[{self.__class__.__name__}] -> Reload web page !"
            self.logger.write_and_flush(msg+"\n")
            print(msg)

    def getStatusOfControls(self):
        return self.statusOfUIObject

    def getField(self):
        return self.field

    def on_socket_data(self, data):
        return self

    def on_event(self, event):
        return self

    def getReason(self):
        return self.reason