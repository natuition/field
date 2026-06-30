from flask_socketio import SocketIO

from config import config
from uiWebRobot.state_machine import State
from uiWebRobot.state_machine.states import WorkingState
from uiWebRobot.state_machine.states import ErrorState
from uiWebRobot.state_machine.Events import Events
from uiWebRobot.state_machine.FrontEndObjects import FrontEndObjects, ButtonState, AuditButtonState
from uiWebRobot.state_machine import utilsFunction
from shared_class.robot_synthesis import RobotSynthesis
import utility
from logger import Logger

class StartingState(State.State):
    """This state corresponds when the robot configures it to start from zero the work. """

    def __init__(self, socketio: SocketIO, file_logger: utility.Logger, isAudit=False):
        self.__logger = Logger.create(self.__class__.__name__)
        self.robot_synthesis_value = RobotSynthesis.UI_STARTING_STATE
        self.socketio = socketio
        self.__file_logger = file_logger
        self.isAudit = isAudit

        self.socketio.emit('start_main', {"status": "pushed"}, namespace='/button', broadcast=True)
        msg = f"Edit config file (CONTINUE_PREVIOUS_PATH:{False})"
        self.__file_logger.write_and_flush(msg + "\n")
        self.__logger.info(msg)
        utilsFunction.changeConfigValue("CONTINUE_PREVIOUS_PATH", False)

        self.statusOfUIObject = FrontEndObjects(fieldButton=ButtonState.DISABLE,
                                                startButton=ButtonState.CHARGING,
                                                continueButton=ButtonState.DISABLE,
                                                stopButton=ButtonState.NOT_HERE,
                                                wheelButton=ButtonState.NOT_HERE,
                                                removeFieldButton=ButtonState.DISABLE,
                                                joystick=False,
                                                slider=config.SLIDER_CREATE_FIELD_DEFAULT_VALUE)

        if isAudit:
            self.statusOfUIObject.audit = AuditButtonState.IN_USE
        else:
            self.statusOfUIObject.audit = AuditButtonState.NOT_IN_USE

        self.field = None

    def on_event(self, event):
        if event == Events.CONFIG_IS_SET:
            self.statusOfUIObject.startButton = ButtonState.NOT_HERE
            self.statusOfUIObject.stopButton = True
            return WorkingState.WorkingState(self.socketio, self.__file_logger, self.isAudit, False)
        else:
            return ErrorState.ErrorState(self.socketio, self.__file_logger)

    def on_socket_data(self, data):
        if data["type"] == 'getInputVoltage':
            return self
        return ErrorState.ErrorState(self.socketio, self.__file_logger)

    def getStatusOfControls(self):
        return self.statusOfUIObject

    def getField(self):
        return self.field