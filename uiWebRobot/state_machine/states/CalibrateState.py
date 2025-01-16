import sys
sys.path.append('../')

from flask_socketio import SocketIO

from state_machine import State
from state_machine.states.ErrorState import ErrorState
from state_machine.Events import Events
from state_machine import utilsFunction
from config import config
from shared_class.robot_synthesis import RobotSynthesis
from deployement.cameraCalibration import CameraCalibration
import utility
import adapters
import hashlib
import subprocess
import os


# This state corresponds when the robot is calibrate of plant targeting precision.
class CalibrateState(State.State):

    def __init__(self,
                 socketio: SocketIO,
                 logger: utility.Logger,
                 smoothie: adapters.SmoothieAdapter,
                 vesc_engine: adapters.VescAdapterV4):
        self.robot_synthesis_value = RobotSynthesis.UI_CALIBRATE_STATE
        self.socketio = socketio
        self.logger = logger
        self.smoothie = smoothie
        self.vesc_engine = vesc_engine
        self.__cork_to_camera_distance = {"X" : config.CORK_TO_CAMERA_DISTANCE_X, "Y": config.CORK_TO_CAMERA_DISTANCE_Y}
        self.__password = "02bbcaee0bf08f23f17bb1ea2bfd40f94d33a011f23db826ba7eaf5e8b1ec66aad9376f6d8512dadf2bc5ef54262ae02dfad12b56d2de9cb3ad77c4c55f90563"
        
        try:
            if self.smoothie is None:
                if config.UI_VERBOSE_LOGGING:
                    msg = f"[{self.__class__.__name__}] -> initSmoothie"
                    self.logger.write_and_flush(msg + "\n")
                    print(msg)
                self.smoothie = utilsFunction.initSmoothie(self.logger)

            if config.UI_VERBOSE_LOGGING:
                msg = f"[{self.__class__.__name__}] -> initCameraCalibration"
                self.logger.write_and_flush(msg + "\n")
                print(msg)
            self.cameraCalibration = CameraCalibration()
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except Exception as e:
            raise e

        self.statusOfUIObject = {"currentHTML": "CalibratePassword.html"}

        res = self.smoothie.ext_calibrate_cork()
        if res != self.smoothie.RESPONSE_OK:
            return ErrorState(self.socketio, self.logger, res)


    def on_event(self, event):
        if event == Events.CALIBRATION_DETECT:
            restart = True
            while restart:
                res = subprocess.run("python3 cameraCalibration.py", stderr=subprocess.DEVNULL, shell=True, cwd=os.getcwd().split("/uiWebRobot")[0]+"/deployement")
                if res.returncode in [0, 1]:
                    restart = False
            with open('../deployement/target_detection.jpg', 'rb') as f:
                image_data = f.read()
            self.socketio.emit('image', {'image_data': image_data, "label": "ok" if res.returncode==0 else "nok"}, namespace='/server', broadcast=True)
            return self
        elif event == Events.CALIBRATION_MOVE:
            self.cameraCalibration.set_targets("../deployement/")
            self.statusOfUIObject["currentHTML"] = "CalibrateMove.html"
            self.socketio.emit('href_to', {"href": "/calibrate"}, namespace='/server', broadcast=True)
            return self
        elif event == Events.CALIBRATION_VALIDATE:
            utilsFunction.changeConfigValue("CORK_TO_CAMERA_DISTANCE_X",self.__cork_to_camera_distance["X"])
            utilsFunction.changeConfigValue("CORK_TO_CAMERA_DISTANCE_Y",self.__cork_to_camera_distance["Y"])
            res = self.smoothie.ext_calibrate_cork()
            if res != self.smoothie.RESPONSE_OK:
                return ErrorState(self.socketio, self.logger, res)
            self.socketio.emit('save_applied', namespace='/server', broadcast=True)
            return self
        elif event == Events.CALIBRATION_CANCEL:
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
        if data["type"] == "run_move_to_target":
            self.cameraCalibration.offset_calibration_step_move_with_distance(self.smoothie,self.__cork_to_camera_distance["X"],self.__cork_to_camera_distance["Y"])
            self.socketio.emit('move', {'move': True}, namespace='/server', broadcast=True)
            return self
        elif data["type"] == "step_axis_xy":
            self.smoothie.custom_move_for(X_F=config.X_F_MAX, Y_F=config.Y_F_MAX, X=data["value"] if data["axis"].upper()=="X" else 0, Y=data["value"] if data["axis"].upper()=="Y" else 0)
            self.__cork_to_camera_distance[data["axis"].upper()] += data["value"]
            return self
        return self

    def getStatusOfControls(self):
        return self.statusOfUIObject

    def getField(self):
        return None
    
    def checkPassword(self, password: str) -> str:
        return hashlib.sha512(bytes(password, 'utf-8')).hexdigest() == self.__password