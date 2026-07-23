from flask_socketio import SocketIO
import posix_ipc
from urllib.parse import quote, unquote
import logging
import sys

from config import config
from uiWebRobot.state_machine import State
from uiWebRobot.state_machine.states import WaitWorkingState
from uiWebRobot.state_machine.states import ErrorState
from uiWebRobot.state_machine.Events import Events
from uiWebRobot.state_machine.FrontEndObjects import FrontEndObjects, ButtonState
from shared_class.robot_synthesis import RobotSynthesis
import utility
import adapters
from logger import LoggerFactory


class CreateFieldStateWithNavX(State.State):
    """This state corresponds when the robot is generating the work area. """

    def __init__(self,
                 socketio: SocketIO,
                 file_logger: utility.Logger,
                 smoothie: adapters.SmoothieAdapter,
                 vesc_engine: adapters.VescAdapterV4):
        self.__logger = LoggerFactory.create(self.__class__.__name__)
        self.robot_synthesis_value = RobotSynthesis.UI_CREATE_FIELD_STATE
        self.socketio = socketio
        self.__file_logger = file_logger
        self.smoothie = smoothie
        self.vesc_engine = vesc_engine
        
        self.socketio.emit('field', {"status": "pushed"}, namespace='/button', broadcast=True)
        self.statusOfUIObject = FrontEndObjects(fieldButton=ButtonState.CHARGING,
                                                startButton=ButtonState.DISABLE,
                                                continueButton=ButtonState.DISABLE,
                                                stopButton=ButtonState.ENABLE,
                                                wheelButton=ButtonState.NOT_HERE,
                                                removeFieldButton=ButtonState.DISABLE,
                                                joystick=True,
                                                slider=config.SLIDER_CREATE_FIELD_DEFAULT_VALUE)

        self.field = None
        self.manoeuvre = False

        try:
            self.notificationQueue = posix_ipc.MessageQueue(config.QUEUE_NAME_UI_NOTIFICATION)
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except:
            self.notificationQueue = None
    
        # self.__ui_languages, self.__current_ui_language = utilsFunction.get_ui_language()


    def on_event(self, event):
        if event == Events.STOP:
            self.socketio.emit('stop', {"status": "pushed"}, namespace='/button', broadcast=True)
            self.statusOfUIObject.fieldButton = ButtonState.NOT_HERE
            self.statusOfUIObject.stopButton = ButtonState.CHARGING

            # try:
            #     self.fieldCreator.setSecondPoint()
            # except TimeoutError:
            #     if self.notificationQueue is not None:
            #         self.notificationQueue.send(json.dumps({"message_name": "No_GPS_for_field"}))
            #     return WaitWorkingState.WaitWorkingState(self.socketio, self.__file_logger, False, self.smoothie, self.vesc_engine)

            # self.field = self.fieldCreator.calculateField()
            # if not config.TWO_POINTS_FOR_CREATE_FIELD and not config.FORWARD_BACKWARD_PATH:
            #     self.manoeuvre = True
            #     if config.MAKE_MANEUVER_AFTER_FIELD_CREATE:
            #         self.fieldCreator.manoeuvre()
            #     self.manoeuvre = False

            self.statusOfUIObject.stopButton = ButtonState.NOT_HERE
            self.statusOfUIObject.fieldButton = ButtonState.VALIDATE
            self.socketio.emit('field', {"status": "finish"}, namespace='/button', broadcast=True)
            return self
        elif event == Events.VALIDATE_FIELD:
            return self
        elif event == Events.VALIDATE_FIELD_NAME:
            self.socketio.emit('field', {"status": "validate"}, namespace='/button', broadcast=True)
            return WaitWorkingState.WaitWorkingState(self.socketio, self.__file_logger, True, self.smoothie, self.vesc_engine)
        elif event == Events.WHEEL:
            self.smoothie.freewheels()
            return self
        else:
            try:
                if self.smoothie is not None:
                    self.smoothie.disconnect()
                    self.smoothie = None
                if self.vesc_engine is not None:
                    self.vesc_engine.close()
                    self.vesc_engine = None
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except Exception as e:
                self.__file_logger.write_and_flush(e + "\n")
            return ErrorState.ErrorState(self.socketio, self.__file_logger)

    def on_socket_data(self, data):
        if data["type"] == "create_field":
            msg = f"File value : {data['value']}."
            self.__file_logger.write_and_flush(msg + "\n")
            self.__logger.debug(msg)
            # self.statusOfUIObject.fieldButton = ButtonState.NOT_HERE

        return self

    def getStatusOfControls(self):
        return self.statusOfUIObject

    def getField(self):
        return self.field
