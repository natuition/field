from flask_socketio import SocketIO
import signal
import posix_ipc
import threading
from datetime import datetime, timezone
import os
import json
import time

from config import config
from uiWebRobot.state_machine import State
from uiWebRobot.state_machine.states import WaitWorkingState
from uiWebRobot.state_machine.states import PhysicalBlocageState
from uiWebRobot.state_machine.states import ErrorState
from uiWebRobot.state_machine.Events import Events
from uiWebRobot.state_machine.GearboxProtection import GearboxProtection
from uiWebRobot.state_machine.FrontEndObjects import AuditButtonState, ButtonState, FrontEndObjects, PhysicalBlocageFEO
from uiWebRobot.state_machine import GearboxProtection, utilsFunction
from uiWebRobot.state_machine.GearboxProtection import GearboxProtection
from shared_class.robot_synthesis import RobotSynthesis
import utility
from logger import LoggerFactory


class WorkingState(State.State):
    """This state corresponds when the robot is working. """

    def __init__(self, socketio: SocketIO, file_logger: utility.Logger, isAudit: bool, isResume: bool, wasPhysicallyBlocked: bool = False):
        if isResume:
            self.robot_synthesis_value = RobotSynthesis.UI_CONTINUE_STATE
        else:
            self.robot_synthesis_value = RobotSynthesis.UI_STARTING_STATE
        self.__logger = LoggerFactory.create(self.__class__.__name__)
        self.__logger.setLevel(config.LOG_LEVEL_UI)
        self.socketio = socketio
        self.__file_logger = file_logger
        self.isAudit = isAudit
        self.__wasPhysicallyBlocked = wasPhysicallyBlocked
        self.allPath = []
        self.isResume = isResume
        self.allPath = []
        self.detected_plants = dict()
        self.extracted_plants = dict()
        self.last_path_all_points = list()
        self.previous_sessions_working_time = None
        self.__gearbox_protection = GearboxProtection()

        self.statusOfUIObject = FrontEndObjects(fieldButton=ButtonState.DISABLE,
                                                startButton=ButtonState.DISABLE,
                                                continueButton=ButtonState.DISABLE,
                                                stopButton=ButtonState.ENABLE,
                                                wheelButton=ButtonState.NOT_HERE,
                                                removeFieldButton=ButtonState.DISABLE,
                                                joystick=False,
                                                slider=config.SLIDER_CREATE_FIELD_DEFAULT_VALUE)
        
        if self.__wasPhysicallyBlocked :
            self.statusOfUIObject.physicalBlocage = PhysicalBlocageFEO.RELOADING

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
        
        msg = f"Creating the message queue between main and ui."
        self.__file_logger.write_and_flush(msg + "\n")
        self.__logger.info(msg)
        
        try:
            posix_ipc.unlink_message_queue(config.QUEUE_NAME_UI_MAIN)
            msg = f"Message queue exist : unlink..."
            self.__file_logger.write_and_flush(msg + "\n")
            self.__logger.debug(msg)
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except posix_ipc.ExistentialError:
            pass
        except Exception as e:
            msg = f"<{e.__class__.__name__}> : {str(e)}."
            self.__file_logger.write_and_flush(msg + "\n")
            self.__logger.error(msg)
        try:
            self.msgQueue = posix_ipc.MessageQueue(config.QUEUE_NAME_UI_MAIN, posix_ipc.O_CREX)
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except Exception as e:
            msg = f"<{e.__class__.__name__}> : {str(e)}."
            self.__file_logger.write_and_flush(msg + "\n")
            self.__logger.error(msg)
        
        msg = f"Creating thread to read messages sent by main."
        self.__file_logger.write_and_flush(msg + "\n")
        self.__logger.debug(msg)
        self._main_msg_thread_alive = True
        self._main_msg_thread = threading.Thread(target=self._main_msg_thread_tf, daemon=True)
        self._main_msg_thread.start()

        msg = f"Launching main."
        self.__file_logger.write_and_flush(msg + "\n")
        self.__logger.debug(msg)
        self.main = utilsFunction.startMain()
        self.timeStartMain = datetime.now(timezone.utc)
        self.__main_not_received_stop = True
        
    def __kill_main_and_wait(self):
        msg = f"Send KeyboardInterrupt to main"
        self.__file_logger.write_and_flush(msg + "\n")
        self.__logger.info(msg)
        os.killpg(os.getpgid(self.main.pid), signal.SIGINT)
        time.sleep(3)
        
        msg = f"Wait main"
        self.__file_logger.write_and_flush(msg + "\n")
        self.__logger.debug(msg)
        self.main.wait()
        
        msg = f"Try to stop main thread if alive"
        self.__file_logger.write_and_flush(msg + "\n")
        self.__logger.debug(msg)
        self._main_msg_thread_alive = False
        
        msg = f"Wait main thread"
        self.__file_logger.write_and_flush(msg + "\n")
        self.__logger.debug(msg)
        
        self._main_msg_thread.join()
        
        msg = f"Send validate stop"
        self.__file_logger.write_and_flush(msg + "\n")
        self.__logger.debug(msg)

    def on_event(self, event):
        
        if event == Events.STOP:
            self.__logger.info(f"Stop event received, update stop button.")
            self.socketio.emit('stop', {"status": "pushed"}, namespace='/button', broadcast=True)
            self.statusOfUIObject.stopButton = ButtonState.CHARGING
            
            msg = f"Killing main..."
            self.__file_logger.write_and_flush(msg + "\n")
            self.__logger.info(msg)
            
            self.__logger.info(f"Waiting for main to stop...")
            while self.__main_not_received_stop:
                self.__kill_main_and_wait()
                
            self.__logger.info(f"Main stopped, update stop button.")
            self.socketio.emit('stop', {"status": "finish"}, namespace='/button', broadcast=True)
            if self.isResume:
                self.statusOfUIObject.continueButton = ButtonState.ENABLE
            else:
                self.statusOfUIObject.startButton = ButtonState.ENABLE
            self.statusOfUIObject.stopButton = ButtonState.NOT_HERE
            
            self.__logger.info(f"Main stopped, change state to WaitWorkingState.")
            return WaitWorkingState.WaitWorkingState(self.socketio, self.__file_logger, False)
        
        elif event == Events.PHYSICAL_BLOCAGE:
            self.statusOfUIObject = FrontEndObjects(fieldButton=ButtonState.DISABLE,
                                                startButton=ButtonState.DISABLE,
                                                continueButton=ButtonState.DISABLE,
                                                stopButton=ButtonState.CHARGING,
                                                wheelButton=ButtonState.NOT_HERE,
                                                removeFieldButton=ButtonState.DISABLE,
                                                joystick=True,
                                                slider=config.SLIDER_CREATE_FIELD_DEFAULT_VALUE,
                                                physicalBlocage=PhysicalBlocageFEO.DETECTED
                                                )
            self.socketio.emit('stop', {"status": "physical_blocage"}, namespace='/button', broadcast=True)

            msg = f"Kill main"
            self.__file_logger.write_and_flush(msg + "\n")
            self.__logger.debug(msg)

            while self.__main_not_received_stop:
                self.__kill_main_and_wait()

            self.socketio.emit('physical_blocage', namespace='/server', broadcast=True)
            if self.isResume:
                self.statusOfUIObject.continueButton = ButtonState.ENABLE
            else:
                self.statusOfUIObject.startButton = ButtonState.ENABLE
            return PhysicalBlocageState.PhysicalBlocageState(self.socketio, self.__file_logger, False)
        
        elif event == Events.CLOSE_APP:
            msg = f"Kill main abruptly (SIGKILL)"
            self.__file_logger.write_and_flush(msg + "\n")
            self.__logger.debug(msg)
            os.killpg(os.getpgid(self.main.pid), signal.SIGKILL)
        else:
            self._main_msg_thread_alive = False
            self._main_msg_thread.join()
            self.msgQueue.close()
            return ErrorState.ErrorState(self.socketio, self.__file_logger)

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
        # When parameters for trigger are changed by the UI
        elif data["type"] == 'penetrometry_new_params':
            try:
                queue_params = posix_ipc.MessageQueue(config.PENETROMETRY_PARAMS_QUEUE_NAME)
                queue_params.send(json.dumps(data), timeout=0.01)
                queue_params.close()
            except Exception as e:
                self.__logger.error("PENETROMETRY, sending params in queue:", e)
            return self
        else:
            self._main_msg_thread_alive = False
            self._main_msg_thread.join()

            return ErrorState.ErrorState(self.socketio, self.__file_logger)

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

        self.queue_penetrometry_data = None
        if config.PENETROMETRY_ANALYSE_MODE:
            # Waiting for queue creating by adapter
            while(self.queue_penetrometry_data is None):
                try:
                    self.queue_penetrometry_data = posix_ipc.MessageQueue(config.PENETROMETRY_DATA_QUEUE_NAME)
                except posix_ipc.ExistentialError:
                    pass
        
        
        while self._main_msg_thread_alive:

            if self.queue_penetrometry_data is not None:
                msg = None
                try:
                    msg = self.queue_penetrometry_data.receive(0.01)
                except posix_ipc.BusyError:
                    pass # If queue is empty continue loop, it will refill
                if msg is not None:
                    self.__logger.info(f"Envoie des données de l'extraction au client WEB")
                    self.socketio.emit('penetrometry_datas', json.loads(msg[0]), namespace="/server", broadcast=True)


            try:
                msg = self.msgQueue.receive(timeout=2)
                data = json.loads(msg[0])

                if "stopping" in data:
                    if data["stopping"]:
                        self._main_msg_thread_alive = False
                        self.__main_not_received_stop = False
                        if config.UI_VERBOSE_LOGGING: 
                            msg = f"Receved main stopping !"
                            self.__file_logger.write_and_flush(msg + "\n")
                            self.__logger.info(msg)
                        continue

                elif "start" in data:
                    if data["start"]:
                        if config.UI_VERBOSE_LOGGING:
                            msg = f"Main started !"
                            self.__file_logger.write_and_flush(msg + "\n")
                            self.__logger.info(msg)
                        self.socketio.emit('start_main', {"status": "finish", "audit": self.isAudit,
                                                        "first_point_no_extractions": config.FIRST_POINT_NO_EXTRACTIONS},
                                        namespace='/button', broadcast=True)
                        if self.__wasPhysicallyBlocked :
                            self.statusOfUIObject.physicalBlocage = PhysicalBlocageFEO.DISABLE
                            self.socketio.emit('popup_modal_hide', {}, namespace='/broadcast', broadcast=True)
                elif "datacollector" in data:
                    self.detected_plants = data["datacollector"][0]
                    self.extracted_plants = data["datacollector"][1]
                    self.previous_sessions_working_time = data["datacollector"][2]
                    self.sendLastStatistics()
                    self.__gearbox_protection.store_number_of_extracts(data["datacollector"][1])
                    
                elif "last_gps" in data:
                    data = data["last_gps"]
                    self.allPath.append([data[1], data[0]])
                    if self.lastGpsQuality != data[2]:
                        self.lastGpsQuality = data[2]
                    self.socketio.emit('updatePath', json.dumps([self.allPath, self.lastGpsQuality]), namespace='/map', broadcast=True)
                    self.socketio.emit('updateGPSQuality', self.lastGpsQuality, namespace='/gps', broadcast=True)
                    self.__gearbox_protection.store_coord(data[0], data[1], data[2])
                    if(self.__gearbox_protection.is_physically_blocked() and config.CHECK_PHYSICAL_BLOCAGE) :
                        utilsFunction.change_state(Events.PHYSICAL_BLOCAGE)

                elif "last_gps_list_file" in data:
                    last_gps_list_file = data["last_gps_list_file"]
                    with open("./" + last_gps_list_file, "r") as gps_his_file:
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
                    if data["input_voltage"] == "Main":
                        utilsFunction.sendBumperInfo(self.socketio, "Main")
                    elif data["input_voltage"] == "Bumper":
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
                self.__logger.error(f"Error during queue receive : {e}.", stack_info=True)
                continue
        
        msg = f"Close msgQueue..."
        self.__file_logger.write_and_flush(msg + "\n")
        self.__logger.debug(msg)
        self.msgQueue.close()
        
        msg = f"Unlink msgQueue..."
        self.__file_logger.write_and_flush(msg + "\n")
        self.__logger.debug(msg)
        
        try:
            self.msgQueue.unlink()
        except posix_ipc.ExistentialError:
            pass
        
        # Closing file descriptor and removing queue if it exist
        if self.queue_penetrometry_data is not None:
            msg = f"Close queue_penetrometry_data..."
            self.__file_logger.write_and_flush(msg + "\n")
            self.__logger.debug(msg)
            self.queue_penetrometry_data.close()
            msg = f"Unlink queue_penetrometry_data..."
            self.__file_logger.write_and_flush(msg + "\n")
            self.__logger.debug(msg)
            try:
                self.queue_penetrometry_data.unlink()
            except posix_ipc.ExistentialError:
                pass