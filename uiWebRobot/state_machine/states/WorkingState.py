import sys
sys.path.append('../')

from flask_socketio import SocketIO
import signal
import posix_ipc
import threading
from datetime import datetime, timezone
import os
import json
import time

from state_machine import State
from state_machine.states import WaitWorkingState
from state_machine.states import ErrorState
from state_machine import Events
from shared_class.robot_synthesis import RobotSynthesis

from uiWebRobot.state_machine.FrontEndObjects import AuditButtonState, ButtonState, FrontEndObjects
from uiWebRobot.state_machine import utilsFunction
from config import config
import utility

# This state corresponds when the robot is working.
class WorkingState(State.State):

    def __init__(self, socketio: SocketIO, logger: utility.Logger, isAudit: bool, isResume: bool):
        if isResume:
            self.robot_synthesis_value = RobotSynthesis.UI_CONTINUE_STATE
        else:
            self.robot_synthesis_value = RobotSynthesis.UI_STARTING_STATE
        self.socketio = socketio
        self.logger = logger
        self.isAudit = isAudit
        self.allPath = []
        self.isResume = isResume
        self.detected_plants = dict()
        self.extracted_plants = dict()
        self.last_path_all_points = list()
        self.previous_sessions_working_time = None

        self.statusOfUIObject = FrontEndObjects(fieldButton=ButtonState.DISABLE,
                                                startButton=ButtonState.DISABLE,
                                                continueButton=ButtonState.DISABLE,
                                                stopButton=ButtonState.ENABLE,
                                                wheelButton=ButtonState.NOT_HERE,
                                                removeFieldButton=ButtonState.DISABLE,
                                                joystick=False,
                                                slider=config.SLIDER_CREATE_FIELD_DEFAULT_VALUE)

        if isAudit:
            self.statusOfUIObject.audit = AuditButtonState.IN_USE
        else:
            self.statusOfUIObject.audit = AuditButtonState.NOT_IN_USE

        if self.isResume:
            self.statusOfUIObject.continueButton = ButtonState.NOT_HERE
        else:
            self.statusOfUIObject.startButton = ButtonState.NOT_HERE

        self.field = None
        self.lastGpsQuality = "1"
        
        if config.UI_VERBOSE_LOGGING:
            msg = f"[{self.__class__.__name__}] -> Creating the message queue between main and ui."
            self.logger.write_and_flush(msg + "\n")
            print(msg)
        
        try:
            posix_ipc.unlink_message_queue(config.QUEUE_NAME_UI_MAIN)
            if config.UI_VERBOSE_LOGGING:
                msg = f"[{self.__class__.__name__}] -> Message queue exist : unlink..."
                self.logger.write_and_flush(msg + "\n")
                print(msg)
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except posix_ipc.ExistentialError:
            pass
        except Exception as e:
            msg = f"[{self.__class__.__name__}] -> <{e.__class__.__name__}> : {str(e)}."
            self.logger.write_and_flush(msg + "\n")
            print(msg)
        try:
            self.msgQueue = posix_ipc.MessageQueue(config.QUEUE_NAME_UI_MAIN, posix_ipc.O_CREX)
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except Exception as e:
            msg = f"[{self.__class__.__name__}] -> <{e.__class__.__name__}> : {str(e)}."
            self.logger.write_and_flush(msg + "\n")
            print(msg)
        
        if config.UI_VERBOSE_LOGGING:
            msg = f"[{self.__class__.__name__}] -> Creating thread to read messages sent by main."
            self.logger.write_and_flush(msg + "\n")
            print(msg)
        self._main_msg_thread_alive = True
        self._main_msg_thread = threading.Thread(target=self._main_msg_thread_tf, daemon=True)
        self._main_msg_thread.start()

        if config.UI_VERBOSE_LOGGING:
            msg = f"[{self.__class__.__name__}] -> Launching main."
            self.logger.write_and_flush(msg + "\n")
            print(msg)
        self.main = utilsFunction.startMain()
        self.timeStartMain = datetime.now(timezone.utc)
        self.__main_not_received_stop = True

    def on_event(self, event):
        if event == Events.Events.STOP:
            self.socketio.emit('stop', {"status": "pushed"}, namespace='/button', broadcast=True)
            self.statusOfUIObject.stopButton = ButtonState.CHARGING
            
            if config.UI_VERBOSE_LOGGING:
                msg = f"[{self.__class__.__name__}] -> Kill main"
                self.logger.write_and_flush(msg + "\n")
                print(msg)
            
            while self.__main_not_received_stop:
                if config.UI_VERBOSE_LOGGING:
                    msg = f"[{self.__class__.__name__}] -> Send KeyboardInterrupt to main"
                    self.logger.write_and_flush(msg + "\n")
                    print(msg)
                os.killpg(os.getpgid(self.main.pid), signal.SIGINT)
                time.sleep(3)
            
            if config.UI_VERBOSE_LOGGING:
                msg = f"[{self.__class__.__name__}] -> Wait main"
                self.logger.write_and_flush(msg + "\n")
                print(msg)
            
            self.main.wait()
            
            if config.UI_VERBOSE_LOGGING:
                msg = f"[{self.__class__.__name__}] -> Restart camera"
                self.logger.write_and_flush(msg + "\n")
                print(msg)
            
            os.system("sudo systemctl restart nvargus-daemon")
            
            if config.UI_VERBOSE_LOGGING:
                msg = f"[{self.__class__.__name__}] -> Try to stop main thread if alive"
                self.logger.write_and_flush(msg + "\n")
                print(msg)
            self._main_msg_thread_alive = False
            
            if config.UI_VERBOSE_LOGGING:
                msg = f"[{self.__class__.__name__}] -> Wait main thread"
                self.logger.write_and_flush(msg + "\n")
                print(msg)
            
            self._main_msg_thread.join()
            
            if config.UI_VERBOSE_LOGGING:
                msg = f"[{self.__class__.__name__}] -> Send validate stop"
                self.logger.write_and_flush(msg + "\n")
                print(msg)
            
            self.socketio.emit('stop', {"status": "finish"}, namespace='/button', broadcast=True)
            if self.isResume:
                self.statusOfUIObject.continueButton = ButtonState.ENABLE
            else:
                self.statusOfUIObject.startButton = ButtonState.ENABLE
            self.statusOfUIObject.stopButton = ButtonState.NOT_HERE
            return WaitWorkingState.WaitWorkingState(self.socketio, self.logger, False)
        else:
            self._main_msg_thread_alive = False
            self._main_msg_thread.join()
            self.msgQueue.close()
            return ErrorState.ErrorState(self.socketio, self.logger)

    def on_socket_data(self, data):
        if data["type"] == "getStats":
            self.sendLastStatistics()
            return self
        elif data["type"] == 'getInputVoltage':
            return self
        elif data["type"] == 'getLastPath':
            if len(self.last_path_all_points) > 0:
                self.socketio.emit('updateLastPath', 
                                   json.dumps(self.last_path_all_points), 
                                   namespace='/map')
            return self
        else:
            self._main_msg_thread_alive = False
            self._main_msg_thread.join()

            return ErrorState.ErrorState(self.socketio, self.logger)

    def getStatusOfControls(self):
        return self.statusOfUIObject

    def getField(self):
        return self.field

    def sendLastStatistics(self):
        data = dict()
        data["weeds"] = self.extracted_plants
        data["time"] = self.timeStartMain.isoformat()
        data["previous_sessions_working_time"] = self.previous_sessions_working_time
        self.socketio.emit('statistics', data, namespace='/server', broadcast=True)

    def _main_msg_thread_tf(self):
        while self._main_msg_thread_alive:
            try:
                msg = self.msgQueue.receive(timeout=2)
                data = json.loads(msg[0])
                if "stopping" in data:
                    if data["stopping"]:
                        self._main_msg_thread_alive = False
                        self.__main_not_received_stop = False
                        msg = f"[{self.__class__.__name__}] -> Receved main stopping !"
                        self.logger.write_and_flush(msg + "\n")
                        print(msg)
                        continue
                elif "start" in data:
                    if data["start"]:
                        if config.UI_VERBOSE_LOGGING:
                            msg = f"[{self.__class__.__name__}] -> Main started !"
                            self.logger.write_and_flush(msg + "\n")
                            print(msg)
                        self.socketio.emit('startMain', {"status": "finish", "audit": self.isAudit,
                                                        "first_point_no_extractions": config.FIRST_POINT_NO_EXTRACTIONS},
                                        namespace='/button', broadcast=True)
                elif "datacollector" in data:
                    self.detected_plants = data["datacollector"][0]
                    self.extracted_plants = data["datacollector"][1]
                    self.previous_sessions_working_time = data["datacollector"][2]
                    self.sendLastStatistics()
                elif "last_gps" in data:
                    data = data["last_gps"]
                    self.allPath.append([data[1], data[0]])
                    if self.lastGpsQuality != data[2]:
                        self.lastGpsQuality = data[2]
                    self.socketio.emit('updatePath', json.dumps([self.allPath, self.lastGpsQuality]), namespace='/map', broadcast=True)
                    self.socketio.emit('updateGPSQuality', self.lastGpsQuality, namespace='/gps', broadcast=True)
                    
                elif "last_gps_list_file" in data:
                    last_gps_list_file = data["last_gps_list_file"]
                    with open("../" + last_gps_list_file, "r") as gps_his_file:
                        self.last_path_all_points.append(list())
                        for line in gps_his_file.readlines():
                            if line.startswith("[") and line.endswith("]\n"):
                                parsed_point = line[1:-1].split(", ")
                                self.last_path_all_points[-1].append([float(parsed_point[1]), float(parsed_point[0])])
                            else:
                                self.last_path_all_points.append(list())
                    self.socketio.emit('updateLastPath', 
                                    json.dumps(self.last_path_all_points), 
                                    namespace='/map',
                                    broadcast=True)
                elif "display_instruction_path" in data:
                    data = data["display_instruction_path"]
                    self.socketio.emit('updateDisplayInstructionPath', json.dumps([elem[::-1] for elem in data]),
                                    namespace='/map', broadcast=True)
                elif "clear_path" in data:
                    self.allPath.clear()
                elif "input_voltage" in data:
                    if data["input_voltage"] == "Bumper":
                        utilsFunction.sendBumperInfo(self.socketio, "Bumper")
                    elif data["input_voltage"] == "Reseting":
                        utilsFunction.sendBumperInfo(self.socketio, "Reseting")
                    else:
                        utilsFunction.sendInputVoltage(self.socketio, data["input_voltage"])
                    
            except posix_ipc.BusyError:
                continue
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except Exception as e:
                print(f"[{self.__class__.__name__}] -> Error during queue receive : {e}.")
                continue
        if config.UI_VERBOSE_LOGGING:        
            msg = f"[{self.__class__.__name__}] -> Close msgQueue..."
            self.logger.write_and_flush(msg + "\n")
            print(msg)
        self.msgQueue.close()
        if config.UI_VERBOSE_LOGGING:
            msg = f"[{self.__class__.__name__}] -> Unlink msgQueue..."
            self.logger.write_and_flush(msg + "\n")
            print(msg)
        try:
            self.msgQueue.unlink()
        except posix_ipc.ExistentialError:
            pass