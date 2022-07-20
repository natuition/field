import sys
sys.path.append('../')

from flask_socketio import SocketIO
import threading
import signal
import re

from state_machine.states import State
from state_machine.states import ErrorState
from state_machine.states import WaitWorkingState
from state_machine.states import Events
from state_machine.utilsFunction import *
from config import config

# This state were robot is start, this state corresponds when the ui reminds the points to check before launching the robot.
class CheckState(State.State):

    def __init__(self, socketio: SocketIO, logger: utility.Logger):
        self.socketio = socketio
        self.logger = logger
        self.cam = None
        try:
            msg = f"[{self.__class__.__name__}] -> startLiveCam"
            self.logger.write_and_flush(msg + "\n")
            print(msg)
            self.cam = startLiveCam()
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except Exception as e:
            raise e

        self.statusOfUIObject = {}

        self.field = None

        msg = f"[{self.__class__.__name__}] -> initVesc"
        self.logger.write_and_flush(msg + "\n")
        print(msg)
        self.vesc_engine = initVesc(self.logger)

        self.__voltage_thread_alive = True
        self.input_voltage = {"input_voltage": "?"}
        self.__voltage_thread = threading.Thread(target=voltage_thread_tf,
                                                 args=(lambda: self.__voltage_thread_alive,
                                                       self.vesc_engine, self.socketio,
                                                       self.input_voltage),
                                                 daemon=True)
        self.__voltage_thread.start()

    def on_event(self, event):
        if event == Events.Events.LIST_VALIDATION:
            self.__voltage_thread_alive = False
            if self.cam:
                os.killpg(os.getpgid(self.cam.pid), signal.SIGINT)
                self.cam.wait()
                os.system("sudo systemctl restart nvargus-daemon")
            if config.NTRIP:
                os.system("sudo systemctl restart ntripClient.service")
            return WaitWorkingState.WaitWorkingState(self.socketio, self.logger, False, vesc_engine=self.vesc_engine)
        else:
            self.socketio.emit('reload', {}, namespace='/broadcast', broadcast=True)
            """self.__voltage_thread_alive = False
            try:
                if self.cam:
                    os.killpg(os.getpgid(self.cam.pid), signal.SIGINT)
                    self.cam.wait()
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except Exception as e:
                self.logger.write_and_flush(str(e)+"\n")
            return ErrorState.ErrorState(self.socketio, self.logger)"""
            return self

    def on_socket_data(self, data):
        if data["type"] == 'allChecked':
            try:
                with open("../yolo/" + data["strategy"] + ".conf") as file:
                    for line in file.readlines():
                        content = line.split("#")[0]
                        content = re.sub('[^0-9a-zA-Z._="/]+', '', content)
                        if content:
                            changeConfigValue(content.split("=")[0], content.split("=")[1])
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except Exception as e:
                self.logger.write_and_flush(e + "\n")
                return ErrorState.ErrorState(self.socketio, self.logger)
        elif data["type"] == 'getInputVoltage':
            sendInputVoltage(self.socketio, self.input_voltage["input_voltage"])
        else:
            self.socketio.emit('reload', {}, namespace='/broadcast', broadcast=True)
        return self

    def getStatusOfControls(self):
        return self.statusOfUIObject

    def getField(self):
        return self.field
