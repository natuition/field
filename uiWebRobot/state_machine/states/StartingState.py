import sys
sys.path.append('../')

from flask_socketio import SocketIO
from state_machine import State
from state_machine.states import WorkingState
from state_machine.states import ErrorState
from state_machine import Events
from state_machine.FrontEndObjects import FrontEndObjects, ButtonState, AuditButtonState
from state_machine import utilsFunction
from shared_class.robot_synthesis import RobotSynthesis
from config import config
import utility

# This state corresponds when the robot configures it to start from zero the work.
class StartingState(State.State):

    def __init__(self, socketio: SocketIO, logger: utility.Logger, isAudit=False):
        self.robot_synthesis_value = RobotSynthesis.UI_STARTING_STATE
        self.socketio = socketio
        self.logger = logger
        self.isAudit = isAudit

        self.socketio.emit('start', {"status": "pushed"}, namespace='/button', broadcast=True)
        msg = f"[{self.__class__.__name__}] -> Edit fichier config (CONTINUE_PREVIOUS_PATH:{False},AUDIT_MODE:{isAudit})"
        self.logger.write_and_flush(msg + "\n")
        print(msg)
        utilsFunction.changeConfigValue("CONTINUE_PREVIOUS_PATH", False)
        utilsFunction.changeConfigValue("AUDIT_MODE", isAudit)

        self.statusOfUIObject = FrontEndObjects(fieldButton=ButtonState.DISABLE,
                                                startButton=ButtonState.CHARGING,
                                                continueButton=ButtonState.DISABLE,
                                                stopButton=ButtonState.NOT_HERE,
                                                wheelButton=ButtonState.DISABLE,
                                                removeFieldButton=ButtonState.DISABLE,
                                                joystick=False,
                                                slider=config.SLIDER_CREATE_FIELD_DEFAULT_VALUE)

        if isAudit:
            self.statusOfUIObject.audit = AuditButtonState.IN_USE
        else:
            self.statusOfUIObject.audit = AuditButtonState.NOT_IN_USE

        self.field = None

    def on_event(self, event):
        if event == Events.Events.CONFIG_IS_SET:
            self.statusOfUIObject.startButton = ButtonState.NOT_HERE
            self.statusOfUIObject.stopButton = True
            return WorkingState.WorkingState(self.socketio, self.logger, self.isAudit, False)
        else:
            return ErrorState.ErrorState(self.socketio, self.logger)

    def on_socket_data(self, data):
        if data["type"] == 'getInputVoltage':
            return self
        return ErrorState.ErrorState(self.socketio, self.logger)

    def getStatusOfControls(self):
        return self.statusOfUIObject

    def getField(self):
        return self.field