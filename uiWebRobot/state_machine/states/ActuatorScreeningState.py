import sys
sys.path.append('../')

from flask_socketio import SocketIO

from state_machine import State
from state_machine.states.ErrorState import ErrorState
from state_machine.Events import Events
from state_machine import utilsFunction
from config import config
import utility
import adapters
import hashlib
import subprocess
import os
import time
import threading

# This state corresponds when the robot screening his actuator.
class ActuatorScreeningState(State.State):

    def __init__(self,
                 socketio: SocketIO,
                 logger: utility.Logger,
                 smoothie: adapters.SmoothieAdapter,
                 vesc_engine: adapters.VescAdapterV4):
        self.socketio = socketio
        self.logger = logger
        self.smoothie = smoothie
        self.vesc_engine = vesc_engine
        self.__screening_shotting_thread_alive = False
        self.__screening_shotting_thread = None
        self.count = 0

        try:
            if self.smoothie is None:
                msg = f"[{self.__class__.__name__}] -> initSmoothie"
                self.logger.write_and_flush(msg + "\n")
                print(msg)
                self.smoothie = utilsFunction.initSmoothie(self.logger)
            else:
                msg = f"[{self.__class__.__name__}] -> no need to initSmoothie"
                self.logger.write_and_flush(msg + "\n")
                print(msg)
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except Exception as e:
            raise e

        self.statusOfUIObject = {"currentHTML": "ActuatorScreening.html"}

        res = self.smoothie.ext_calibrate_cork()
        if res != self.smoothie.RESPONSE_OK:
            return ErrorState(self.socketio, self.logger, res)

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
            self.count += 1
            self.socketio.emit('screening_status', {"counter": self.count}, namespace='/server', broadcast=True)

    def on_event(self, event):
        if event == Events.ACTUATOR_SCREENING_START:
            self.__screening_shotting_thread_alive = True
            if self.__screening_shotting_thread is None:
                self.__screening_shotting_thread =  threading.Thread(target=self.__screening_shotting_tf,
                                                    daemon=True)
                self.__screening_shotting_thread.start()
            self.socketio.emit('screening_status', "started", namespace='/server', broadcast=True)
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
            return self
        elif event == Events.ACTUATOR_SCREENING_STOP:
            self.__screening_shotting_thread_alive = False
            if self.__screening_shotting_thread is not None:
                self.__screening_shotting_thread.join()
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