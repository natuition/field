import sys
sys.path.append('../')

from flask_socketio import SocketIO

from state_machine import State
from state_machine.states import WorkingState
from state_machine.states import ErrorState
from state_machine import Events
from state_machine.FrontEndObjects import FrontEndObjects, ButtonState, AuditButtonState, PhysicalBlocageFEO
from state_machine import utilsFunction
from shared_class.robot_synthesis import RobotSynthesis
from config import config
import utility

# This state corresponds when the robot configures it to continue the last job.
class ResumeState(State.State):

    def __init__(self, socketio: SocketIO, logger: utility.Logger, isAudit=False, wasPhysicallyBlocked=False):
        self.robot_synthesis_value = RobotSynthesis.UI_CONTINUE_STATE
        self.socketio = socketio
        self.logger = logger
        self.isAudit = isAudit
        self.__wasPhysicallyBlocked = wasPhysicallyBlocked

        self.statusOfUIObject = FrontEndObjects(fieldButton=ButtonState.DISABLE,
                                                startButton=ButtonState.DISABLE,
                                                continueButton=ButtonState.CHARGING,
                                                stopButton=ButtonState.NOT_HERE,
                                                wheelButton=ButtonState.NOT_HERE,
                                                removeFieldButton=ButtonState.DISABLE,
                                                joystick=False,
                                                slider=config.SLIDER_CREATE_FIELD_DEFAULT_VALUE,
                                                physicalBlocage=PhysicalBlocageFEO.DISABLE
                                                )

        if self.__wasPhysicallyBlocked :
            self.statusOfUIObject.physicalBlocage = PhysicalBlocageFEO.RELOADING
            self.__ui_languages, self.__current_ui_language = utilsFunction.get_ui_language()
            message = self.__ui_languages["Physical_blocage_reloading"][self.__current_ui_language]
            self.socketio.emit('popup_modal', {"message_name":"reloading", "message":message, "type_alert":"alert-success"}, namespace='/broadcast', broadcast=True)
        
        self.socketio.emit('continue_main', {"status": "pushed"}, namespace='/button', broadcast=True)
        if config.UI_VERBOSE_LOGGING:
            msg = f"[{self.__class__.__name__}] -> Edit config file (CONTINUE_PREVIOUS_PATH:{True})"
            self.logger.write_and_flush(msg + "\n")
            print(msg)
        utilsFunction.changeConfigValue("CONTINUE_PREVIOUS_PATH", True)

        if isAudit:
            self.statusOfUIObject.audit = AuditButtonState.IN_USE
        else:
            self.statusOfUIObject.audit = AuditButtonState.NOT_IN_USE

        self.field = None

    def on_event(self, event):
        if event == Events.Events.CONFIG_IS_SET:
            self.statusOfUIObject.continueButton = ButtonState.NOT_HERE
            self.statusOfUIObject.stopButton = ButtonState.ENABLE
            return WorkingState.WorkingState(self.socketio, self.logger, self.isAudit, True, self.__wasPhysicallyBlocked)
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