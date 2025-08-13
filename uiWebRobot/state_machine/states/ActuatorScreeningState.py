import sys
sys.path.append('../')

from flask_socketio import SocketIO

from uiWebRobot.state_machine import State
from uiWebRobot.state_machine.states.ErrorState import ErrorState
from uiWebRobot.state_machine.Events import Events
from uiWebRobot.state_machine import utilsFunction
from shared_class.robot_synthesis import RobotSynthesis
from config import config
import utility
import adapters
import hashlib
import subprocess
import os
import time
import threading
import csv
import math

# This state corresponds when the robot screening his actuator.
class ActuatorScreeningState(State.State):

    DATA_IN_CSV = False

    def __init__(self,
                 socketio: SocketIO,
                 logger: utility.Logger,
                 smoothie: adapters.SmoothieAdapter,
                 vesc_engine: adapters.VescAdapterV4):
        self.robot_synthesis_value = RobotSynthesis.UI_ACTUATOR_SCREENING_STATE
        self.socketio = socketio
        self.logger = logger
        self.smoothie = smoothie
        self.vesc_engine = vesc_engine
        self.__screening_shotting_thread_alive = False
        self.__screening_shotting_thread = None
        self.__z_motor_stats_thread_alive = True
        self.__z_motor_stats_thread = None
        self.__z_motor_can_id = 2

        self.__radar_chart = False
        self.__x_data_cpt = 0
        self.__x_data = list()
        self.__y_data = list()
        self.__y_data.append(list())
        self.__y_data.append(list())

        try:
            if self.smoothie is None:
                if config.UI_VERBOSE_LOGGING:
                    msg = f"[{self.__class__.__name__}] -> initSmoothie"
                    self.logger.write_and_flush(msg + "\n")
                    print(msg)
                self.smoothie = utilsFunction.initSmoothie(self.logger)
            else:
                if config.UI_VERBOSE_LOGGING:
                    msg = f"[{self.__class__.__name__}] -> no need to initSmoothie"
                    self.logger.write_and_flush(msg + "\n")
                    print(msg)
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except Exception as e:
            raise e

        self.statusOfUIObject = {"currentHTML": "ActuatorScreening.html", "hasStarted": False, "count" : 0}

        res = self.smoothie.ext_calibrate_cork()
        if res != self.smoothie.RESPONSE_OK:
            return ErrorState(self.socketio, self.logger, res)

        self.__z_motor_stats_thread =  threading.Thread(target=self.__z_motor_stats_tf, daemon=True)
        self.__z_motor_stats_thread.start()

    def __z_motor_stats_tf(self):
        try:
            fieldnames = ["avg_iq", "rpm"]
            if ActuatorScreeningState.DATA_IN_CSV:
                csvfile = open('force.csv', 'w', newline='')
                csv_writer = csv.DictWriter(csvfile, fieldnames=fieldnames, delimiter='\t')
                csv_writer.writeheader()
            while(self.__z_motor_stats_thread_alive):
                data = self.vesc_engine.get_sensors_data_of_can_id(fieldnames, self.__z_motor_can_id)
                if data is not None and self.__screening_shotting_thread_alive:
                    data_filtred = {k:(v if not math.isnan(v) else 0) for k, v in data.items()}
                    if ActuatorScreeningState.DATA_IN_CSV:
                        csv_writer.writerow({k:(f"{str(v).replace('.',',')}") for k, v in data_filtred.items()})
                    if self.__radar_chart:
                        self.socketio.emit('statistics', {"operation": "add", "y_data": [data_filtred["avg_iq"],data_filtred["rpm"]]}, namespace='/server', broadcast=True)
                    else:
                        self.__x_data.append(self.__x_data_cpt)
                        self.__y_data[0].append(data_filtred["avg_iq"]*(10**42))
                        self.__y_data[1].append(data_filtred["rpm"]/7)
                self.__x_data_cpt+=1
                time.sleep(0.01)
            if ActuatorScreeningState.DATA_IN_CSV:
                csvfile.close()
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except Exception as e:
            print(e)

    def __screening_shotting_tf(self):
        while(self.__screening_shotting_thread_alive):
            time.sleep(0.1)
            res = self.smoothie.custom_move_for(Z_F=config.Z_F_EXTRACTION_DOWN, Z=config.EXTRACTION_Z)
            self.smoothie.wait_for_all_actions_done()
            if res != self.smoothie.RESPONSE_OK:
                return Exception(res)
            time.sleep(0.1)
            res = self.smoothie.ext_cork_up()
            if res != self.smoothie.RESPONSE_OK:
                return Exception(res)
            self.statusOfUIObject["count"] += 1
            self.socketio.emit('screening_status', {"count": self.statusOfUIObject["count"]}, namespace='/server', broadcast=True)
            if self.__radar_chart:
                self.socketio.emit('statistics', {"operation": "clear"}, namespace='/server', broadcast=True)
            else:
                self.socketio.emit('statistics', {"operation": "set", "x_data": self.__x_data, "y_data": self.__y_data}, namespace='/server', broadcast=True)
                self.__x_data = list()
                self.__y_data[0] = list()
                self.__y_data[1] = list()
                self.__x_data_cpt = 0

    def __threads_stop_join(self):
        self.__screening_shotting_thread_alive = False
        if self.__screening_shotting_thread is not None:
            self.__screening_shotting_thread.join()
        self.__z_motor_stats_thread_alive = False
        if self.__z_motor_stats_thread is not None:
            self.__z_motor_stats_thread.join()

    def on_event(self, event):
        if event == Events.ACTUATOR_SCREENING_START:
            self.__screening_shotting_thread_alive = True
            if self.__screening_shotting_thread is None:
                self.__screening_shotting_thread =  threading.Thread(target=self.__screening_shotting_tf,
                                                    daemon=True)
                self.__screening_shotting_thread.start()
            self.socketio.emit('screening_status', "started", namespace='/server', broadcast=True)
            self.statusOfUIObject["hasStarted"]= True
            return self
        elif event == Events.ACTUATOR_SCREENING_PAUSE:
            self.__screening_shotting_thread_alive = False
            if self.__screening_shotting_thread is not None:
                self.__screening_shotting_thread.join()
            self.__screening_shotting_thread = None
            res = self.smoothie.ext_cork_up()
            if res != self.smoothie.RESPONSE_OK:
                return ErrorState(self.socketio, self.logger, res)
            self.socketio.emit('screening_status', "paused", namespace='/server', broadcast=True)
            self.statusOfUIObject["hasStarted"]= False
            return self
        elif event == Events.ACTUATOR_SCREENING_STOP:
            self.__threads_stop_join()
            from state_machine.states.WaitWorkingState import WaitWorkingState
            res = self.smoothie.ext_calibrate_cork()
            if res != self.smoothie.RESPONSE_OK:
                return ErrorState(self.socketio, self.logger, res)
            self.socketio.emit('href_to', {"href": "/", "delay": 1000}, namespace='/server', broadcast=True)
            return WaitWorkingState(self.socketio, self.logger, False, self.smoothie, self.vesc_engine)
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
                self.logger.write_and_flush(e + "\n")
            return ErrorState(self.socketio, self.logger)

    def on_socket_data(self, data):
        return self

    def getStatusOfControls(self):
        return self.statusOfUIObject

    def getField(self):
        return None