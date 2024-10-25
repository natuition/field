import sys
sys.path.append('../')

from flask_socketio import SocketIO

from state_machine import State
from shared_class.robot_synthesis import RobotSynthesis

from uiWebRobot.state_machine.FrontEndObjects import ButtonState, FrontEndObjects
from config import config
import utility

# This state corresponds when the robot is physically blocking.
class PhysicalBlocageState(State.State) :
    def __init__(self, socketio: SocketIO, logger: utility.Logger) :
        self.socketio = socketio
        self.logger = logger
        self.robot_synthesis_value = RobotSynthesis.UI_PHYSICAL_BLOCAGE

        msg = f"[{self.__class__.__name__}] -> Physically blocked"
        self.logger.write_and_flush(msg + "\n")
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