from flask_socketio import SocketIO

from config import config
import utility
from uiWebRobot.state_machine import State
from shared_class.robot_synthesis import RobotSynthesis
from uiWebRobot.state_machine.FrontEndObjects import FrontEndObjects, ButtonState
from logger import Logger

class ErrorState(State.State):

    def __init__(self, socketio: SocketIO, file_logger: utility.Logger, reason: str = None):
        self.__logger = Logger.create(self.__class__.__name__)
        self.robot_synthesis_value = RobotSynthesis.HS
        self.socketio = socketio
        self.__file_logger = file_logger
        self.reason = reason
        msg = f"Error"
        self.__file_logger.write_and_flush(msg+"\n")
        self.__logger.error(msg)

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

        msg = f"Reload web page !"
        self.__file_logger.write_and_flush(msg+"\n")
        self.__logger.info(msg)

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