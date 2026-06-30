import threading
import os
from flask_socketio import SocketIO

from config import config
from uiWebRobot.state_machine import utilsFunction
from uiWebRobot.state_machine.Events import Events
from uiWebRobot.state_machine.states import WaitWorkingState
from uiWebRobot.state_machine import State
from shared_class.robot_synthesis import RobotSynthesis
from uiWebRobot.EnvironnementConfig import EnvironnementConfig
import utility
from logger import Logger


# This state were robot is start, this state corresponds when the ui reminds the points to check before launching the robot.
class CheckState(State.State):

    def __init__(self, socketio: SocketIO, file_logger: utility.Logger):
        self.__logger = Logger.create(self.__class__.__name__)
        self.robot_synthesis_value = RobotSynthesis.UI_CHECK_STATE
        self.socketio = socketio
        self.__file_logger = file_logger

        self.statusOfUIObject = {}

        self.field = None

        if config.UI_VERBOSE_LOGGING:
            msg = f"initVesc"
            self.__file_logger.write_and_flush(msg + "\n")
            self.__logger.info(msg)
        self.vesc_engine = utilsFunction.initVesc(self.__file_logger)
        self.vesc_engine.close()
        del self.vesc_engine
        if config.UI_VERBOSE_LOGGING:
            msg = f"Close and recreate vesc"
            self.__file_logger.write_and_flush(msg + "\n")
            self.__logger.info(msg)
        self.vesc_engine = utilsFunction.initVesc(self.__file_logger)

        self.__voltage_thread_alive = True
        self.input_voltage = {"input_voltage": "?"}
        self.__voltage_thread = threading.Thread(target=utilsFunction.voltage_thread_tf,
                                                 args=(lambda: self.__voltage_thread_alive,
                                                       self.vesc_engine, 
                                                       self.socketio,
                                                       self.input_voltage,
                                                       self.__file_logger),
                                                 daemon=True)
        self.__voltage_thread.start()

        if EnvironnementConfig.NATUITION_CHECKLIST():
            self.statusOfUIObject["checkbox"] = True
        else:
            self.statusOfUIObject["checkbox"] = False

    def on_event(self, event):

        if event == Events.LIST_VALIDATION:
            self.socketio.emit('data', {"ACK": "list_validation"}, namespace='/server', broadcast=True)
            EnvironnementConfig.NATUITION_CHECKLIST(True)
            self.__stop_thread()
            if config.NTRIP:
                msg = f"Restarting ntripClient.service..."
                self.__file_logger.write_and_flush(msg + "\n")
                self.__logger.info(msg)
                os.system("sudo systemctl restart ntripClient.service")
            return WaitWorkingState.WaitWorkingState(self.socketio, self.__file_logger, False, vesc_engine=self.vesc_engine)
        
        else:
            self.socketio.emit(
                'reload', {}, namespace='/broadcast', broadcast=True)
            return self

    def on_socket_data(self, data):
        if data["type"] == 'list_validation':
            try:
                with open("./yolo/" + data["strategy"] + ".conf") as file:
                    for line in file:
                        content = line.split("#")[0].strip()
                        if content != "" and "=" in content:
                            key, value = content.split("=")[:2]
                            utilsFunction.changeConfigValue(key.strip(), value.strip())
            except KeyboardInterrupt:
                raise KeyboardInterrupt
        else:
            self.socketio.emit(
                'reload', {}, namespace='/broadcast', broadcast=True)
        return self
        """ elif data["type"] == 'getInputVoltage':
            utilsFunction.sendInputVoltage(
                self.socketio, self.input_voltage["input_voltage"]) """
        

    def getStatusOfControls(self):
        return self.statusOfUIObject

    def getField(self):
        return self.field
    
    def __stop_thread(self):
        self.__voltage_thread_alive = False
        self.__voltage_thread.join()
