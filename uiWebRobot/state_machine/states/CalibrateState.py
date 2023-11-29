import sys
sys.path.append('../')

from flask_socketio import SocketIO

from state_machine import State
from state_machine.states.WaitWorkingState import WaitWorkingState
from state_machine.states.ErrorState import ErrorState
from state_machine.Events import Events
from state_machine import utilsFunction
from config import config
from deployement.cameraCalibration import CameraCalibration
import utility
import adapters


# This state corresponds when the robot is calibrate of plant targeting precision.
class CalibrateState(State.State):

    def __init__(self,
                 socketio: SocketIO,
                 logger: utility.Logger,
                 smoothie: adapters.SmoothieAdapter,
                 vesc_engine: adapters.VescAdapterV4):
        self.socketio = socketio
        self.logger = logger
        self.smoothie = smoothie
        self.vesc_engine = vesc_engine
        self.__cork_to_camera_distance = {"X" : config.CORK_TO_CAMERA_DISTANCE_X, "Y": config.CORK_TO_CAMERA_DISTANCE_Y}
        
        try:
            if self.smoothie is None:
                msg = f"[{self.__class__.__name__}] -> initSmoothie"
                self.logger.write_and_flush(msg + "\n")
                print(msg)
                self.smoothie = utilsFunction.initSmoothie(self.logger)

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
            res = self.cameraCalibration.offset_calibration_step_detect()
            label = "ok"
            if "isn't" in res:
                label = "nok"
            with open('./target_detection.jpg', 'rb') as f:
                image_data = f.read()
            self.socketio.emit('image', {'image_data': image_data, "label": label}, namespace='/server', broadcast=True)
            return self
        elif event == Events.CALIBRATION_MOVE:
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