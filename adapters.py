import connectors
import multiprocessing
import time
import navigation
import utility
import math
import threading
import serial
import pyvesc
import re
import json
from serial import SerialException

from typing import Any, Dict, List, Tuple, Type, Optional, TypeVar, Union, cast
Number = Union[int, float]
OptionalNumber = Optional[Number]
ResultType = TypeVar("ResultType")

from config import config
from detection import DetectedPlantBox
from client import Client
from common import MVICustomResultType, MVIPipelineDescriptor, MVIProperty, MVIState
from message import CallType, ResultMessage
from protos import DetectionResultDTO, ResultDTO, DetectionResult, KeypointDTO
from logger import LoggerFactory

class SmoothieAdapter:
    RESPONSE_OK = "ok\r\n"
    RESPONSE_ALARM_LOCK = "error:Alarm lock\n"
    RESPONSE_HALT = "!!\r\n"
    RESPONSE_IGNORED = "ok - ignored\n"
    RESPONSE_HOMING_FAILED = "ERROR: Homing cycle failed - check the max_travel settings"
    RESPONSE_AFTER_M999 = "WARNING: After HALT you should HOME as position is currently unknown"
    AXIS_LABELS: List[str] = ["X", "Y", "Z", "A", "B", "C"]

    def __init__(self, smoothie_host: str, calibration_at_init: bool=True):
        
        self.__logger = LoggerFactory.create(self.__class__.__name__)
        
        if type(smoothie_host) is not str:
            raise TypeError(f"invalid smoothie_host type: should be str, received " + type(smoothie_host).__name__)

        if config.SMOOTHIE_BACKEND == 1: # type: ignore
            self.__smc = connectors.SmoothieV11TelnetConnector(smoothie_host)
        elif config.SMOOTHIE_BACKEND == 2: # type: ignore
            self.__smc = connectors.SmoothieV11SerialConnector(smoothie_host, config.SMOOTHIE_BAUDRATE)
        else:
            raise ValueError(f"wrong config.SMOOTHIE_BACKEND value: " + str(smoothie_host))

        self.__sync_locker = multiprocessing.RLock()
        self.__x_cur = multiprocessing.Value("d", 0)
        self.__y_cur = multiprocessing.Value("d", 0)
        self.__z_cur = multiprocessing.Value("d", 0)
        self.__a_cur = multiprocessing.Value("d", 0)
        self.__b_cur = multiprocessing.Value("d", 0)
        self.__c_cur = multiprocessing.Value("d", 0)
        self.__axis_cur = {
            "X": self.__x_cur,
            "Y": self.__y_cur,
            "Z": self.__z_cur,
            "A": self.__a_cur,
            "B": self.__b_cur,
            "C": self.__c_cur,
        }

        #< Code de base
        # res = None
        # for i in range(3):
        #     res = self.switch_to_relative()
        #     if res != self.RESPONSE_OK:
        #         msg = f"Attempt {i + 1} of switching smoothie to relative is failed, smoothie response:\n{res}"
        #         print(msg)
        #     else:
        #         break
        # else:
        #     msg = f"All attempts of switching smoothie to relative were failed! Last smoothie's response:\n{res}"
        #     print(msg)
        #     raise Exception(msg)
        #> Code de base

        #< Code patché rapidement pour la démo
        res = None
        for i in range(3):
            res = self.switch_to_relative()
            if SmoothieAdapter.check_res_smoothie(res):
                msg = f"Attempt {i + 1} of switching smoothie to relative is failed, smoothie response:\n{res}"
                self.__logger.info(msg)
            else:
                if(res == self.RESPONSE_OK):
                    self.__logger.info(f"The Smoothie switched to relative mode without detecting any bugs. The response was: {res}")
                else:
                    self.__logger.info(f"A bug was detected during the Smoothie's switch to relative mode, but it was handled by the bug fix. The response was: {res}")
                break
        else:
            msg = f"All attempts of switching smoothie to relative were failed! Last smoothie's response:\n{res}"
            self.__logger.error(msg)
            raise Exception(msg)
        #> Code patché rapidement pour la démo

        if config.SEEDER_QUANTITY > 0:
            self.seeder_close()
            res = self.seeder_close()
            if SmoothieAdapter.check_res_smoothie(res):
                msg = f"Couldn't lock seeder during smoothie adapter initialization! Smoothie response: {res}"
                self.__logger.error(msg)

        if calibration_at_init:
            # TODO: temporary crutch - vesc is moving Z upward before smoothie loads, so we need to lower the cork a bit down
            res = self.custom_move_for(Z_F=config.Z_F_EXTRACTION_DOWN, Z=5)
            self.wait_for_all_actions_done()
            if SmoothieAdapter.check_res_smoothie(res):
                self.__logger.error("Couldn't move cork down for Z5! Calibration errors on Z axis are possible!")

            res = self.ext_calibrate_cork()
            if SmoothieAdapter.check_res_smoothie(res):
                self.__logger.error("Initial cork calibration was failed, smoothie response:\n", res)  # TODO: what if so??
                raise Exception("Initial cork calibration was failed!")

    def __enter__(self):
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any):
        self.__smc.disconnect()

    def disconnect(self):
        self.__smc.disconnect()
        
    @staticmethod
    def check_res_smoothie(res: str) -> bool:
        return (("!" in res) or ("error" in res) or ("ERROR" in res) or ("WARNING" in res) or ("ignored" in res))

    @property
    def is_disconnect(self) -> bool:
        return self.__smc.is_open

    def get_connector(self):
        """Only for debug!"""

        return self.__smc

    def wait_for_all_actions_done(self):
        """Wait for the queue to be empty and the motors to stop before the M400 answers ok

        Sends 'M400' command to smoothie. Returns smoothie answer message."""
        with self.__sync_locker:
            self.__smc.write("M400")
            # "ok\r\n"
            return self.__smc.read_some()

    def halt(self):
        """Halt all operations, turn off heaters, go into Halt state.

        Sends 'M112' command to smoothie. Returns smoothie answer message."""
        with self.__sync_locker:
            self.__smc.write("M112")
            # "ok Emergency Stop Requested - reset or M999 required to exit HALT state\r\n"
            return self.__smc.read_some() + self.__smc.read_some() if self.__smc is connectors.SmoothieV11TelnetConnector else self.__smc.read_some() # type: ignore

    def reset(self):
        with self.__sync_locker:
            self.__smc.write("reset")
            return self.__smc.read_some()

    def freewheels(self):
        """Disable stepper motors.

        Sends 'M18' command to smoothie. Returns smoothie answer message."""
        with self.__sync_locker:
            self.__smc.write("M18")
            return self.__smc.read_some()
        
    def tighten_wheels(self):
        """Disable stepper motors.

        Sends 'M17' command to smoothie. Returns smoothie answer message."""
        with self.__sync_locker:
            self.__smc.write("M17")
            return self.__smc.read_some()

    def reset_halted_state(self):
        """Reset from a halted state caused by limit switch, M112 or kill switch

        Sends 'M999' command to smoothie. Returns smoothie answer message."""
        with self.__sync_locker:
            self.__smc.write("M999")
            return self.__smc.read_some()

    def checkendstop(self, axe: str):
        with self.__sync_locker:
            self.__smc.write("M119")
            response = self.__smc.read_some()
            matches = re.findall(f"(?:(?:{axe}_min)|(?:{axe}_max)):(.)", response)
            if matches:
                return matches[0]
            return 1  # refaire demande

    def switch_to_relative(self):
        """Relative mode (command is modal)

        Sends 'G91' command to smoothie. Returns smoothie answer message."""
        with self.__sync_locker:
            self.__smc.write("G91")
            # "ok\r\n"
            return self.__smc.read_some()

    def set_current_coordinates(self, X: OptionalNumber=None, Y:OptionalNumber=None, Z: OptionalNumber=None, A: OptionalNumber=None, B:OptionalNumber=None, C:OptionalNumber=None)-> str:
        with self.__sync_locker:
            if self.__check_arg_types([type(None)], X, Y, Z, A, B, C):
                raise TypeError(f"at least one axis shouldn't be None")
            if not self.__check_arg_types([float, int, type(None)], X, Y, Z, A, B, C):
                raise TypeError(f"incorrect axis current value(s) type(s)")

            g_code = "G92"

            if X is not None:
                g_code += " X" + str(self.mm_to_smoothie(X, "X"))
            if Y is not None:
                g_code += " Y" + str(self.mm_to_smoothie(Y, "Y"))
            if Z is not None:
                g_code += " Z" + str(self.mm_to_smoothie(Z, "Z"))
            if A is not None:
                g_code += " A" + str(self.mm_to_smoothie(A, "A"))
            if B is not None:
                g_code += " B" + str(self.mm_to_smoothie(B, "B"))
            if C is not None:
                g_code += " C" + str(self.mm_to_smoothie(C, "C"))

            self.__smc.write(g_code)
            response = self.__smc.read_some()

            if response == self.RESPONSE_OK:
                if X is not None:
                    self.__x_cur.value = X
                if Y is not None:
                    self.__y_cur.value = Y
                if Z is not None:
                    self.__z_cur.value = Z
                if A is not None:
                    self.__a_cur.value = A
                if B is not None:
                    self.__b_cur.value = B
                if C is not None:
                    self.__c_cur.value = C
            return response

    def get_adapter_current_coordinates(self):
        with self.__sync_locker:
            return {
                "X": self.__x_cur.value,
                "Y": self.__y_cur.value,
                "Z": self.__z_cur.value,
                "A": self.__a_cur.value,
                "B": self.__b_cur.value
                # "C": self._c_cur.value
            }

    def get_smoothie_current_coordinates(self, convert_to_mms: bool=True)-> Dict[str, float]:
        """

        :param convert_to_mms: 
        :return: 
        """
        """
        Answers:
        M114:   'ok C: X:2.0240 Y:0.0000 Z:0.0000\r\n'
        M114.1  'ok WCS: X:2.0250 Y:0.0000 Z:0.0000\r\n'
        M114.2  'ok MCS: X:2.0250 Y:0.0000 Z:0.0000 A:0.0000 B:0.0000\r\n'
        M114.3  'ok APOS: X:2.0250 Y:0.0000 Z:0.0000 A:0.0000 B:0.0000\r\n'
        M114.4  'ok MP: X:2.0240 Y:0.0000 Z:0.0000 A:0.0000 B:0.0000\r\n'
        """

        with self.__sync_locker:
            self.__smc.write("M114.2")
            response, coordinates = (self.__smc.read_some() + self.__smc.read_some()
                                     if type(self.__smc) is connectors.SmoothieV11TelnetConnector
                                     else self.__smc.read_some())[:-2].split(" ")[2:], {}
            coordinates: Dict[str, float] = {}
            for coord in response:
                coordinates[coord[0]] = float(coord[2:])
                if convert_to_mms:
                    coordinates[coord[0]] = self.smoothie_to_mm(coordinates[coord[0]], coord[0])
            return coordinates

    @classmethod
    def compare_coordinates(cls, coordinates_a: Dict[str, Number], coordinates_b: Dict[str, Number], precision:float=1e-10)-> bool:
        if type(coordinates_a) != dict or type(coordinates_b) != dict:
            raise AttributeError(f"[{cls.__name__}] -> coordinates should be stored in dict")
        if len(coordinates_a) != len(coordinates_b):
            raise AttributeError(f"[{cls.__name__}] -> coordinates dicts should have similar items count")

        for key in coordinates_a:
            if abs(coordinates_a[key] - coordinates_b[key]) > precision:
                return False
        return True

    def custom_move_for(self, *,
                        X_F: OptionalNumber=None,
                        Y_F: OptionalNumber=None,
                        Z_F: OptionalNumber=None,
                        A_F: OptionalNumber=None,
                        B_F: OptionalNumber=None,
                        C_F: OptionalNumber=None,
                        X : OptionalNumber=None,
                        Y : OptionalNumber=None,
                        Z : OptionalNumber=None,
                        A : OptionalNumber=None,
                        B : OptionalNumber=None,
                        C : OptionalNumber=None):
        """Movement by some value(s)

        Minimal force is applied if multiple values are given
        """

        with self.__sync_locker:
            # check given forces
            if self.__check_arg_types([type(None)], X_F, Y_F, Z_F, A_F, B_F, C_F):
                raise TypeError(f"at least one given force value shouldn't be a None")
            if not self.__check_arg_types([float, int, type(None)], X_F, Y_F, Z_F, A_F, B_F, C_F):
                raise TypeError(f"incorrect force value(s) type(s)")

            # check given axes
            if self.__check_arg_types([type(None)], X, Y, Z, A, B, C):
                raise TypeError(f"at least one given axis value shouldn't be a None")
            if not self.__check_arg_types([float, int, type(None)], X, Y, Z, A, B, C):
                raise TypeError(f"incorrect axis value(s) type(s)")

            # apply min of given forces (and pass by Nones)
            min_f_msg = "(min force value applied)"
            min_f = min([item for item in [X_F, Y_F, Z_F, A_F, B_F, C_F] if item is not None])
            g_code = "G0"

            if X is not None:
                # validate force
                err_msg = self.__validate_force(min_f, min_f_msg, config.X_F_MIN, config.X_F_MAX, "X_F_MIN", "X_F_MAX")
                if err_msg:
                    return err_msg
                # validate axis
                err_msg = self.__validate_axis(self.__x_cur.value,
                                               X,
                                               "X",
                                               self.smoothie_to_mm(config.X_MIN, "X"),
                                               self.smoothie_to_mm(config.X_MAX, "X"),
                                               "X_MIN",
                                               "X_MAX")
                if err_msg:
                    return err_msg
                g_code += " X" + str(self.mm_to_smoothie(X, "X"))

            if Y is not None:
                # validate force
                err_msg = self.__validate_force(min_f, min_f_msg, config.Y_F_MIN, config.Y_F_MAX, "Y_F_MIN", "Y_F_MAX")
                if err_msg:
                    return err_msg
                # validate axis
                err_msg = self.__validate_axis(self.__y_cur.value,
                                               Y,
                                               "Y",
                                               self.smoothie_to_mm(config.Y_MIN, "Y"),
                                               self.smoothie_to_mm(config.Y_MAX, "Y"),
                                               "Y_MIN",
                                               "Y_MAX")
                if err_msg:
                    return err_msg
                g_code += " Y" + str(self.mm_to_smoothie(Y, "Y"))

            if Z is not None:
                # validate force
                err_msg = self.__validate_force(min_f, min_f_msg, config.Z_F_MIN, config.Z_F_MAX, "Z_F_MIN", "Z_F_MAX")
                if err_msg:
                    return err_msg
                # validate axis
                err_msg = self.__validate_axis(self.__z_cur.value,
                                               Z,
                                               "Z",
                                               self.smoothie_to_mm(config.Z_MIN, "Z"),
                                               self.smoothie_to_mm(config.Z_MAX, "Z"),
                                               "Z_MIN",
                                               "Z_MAX")
                if err_msg:
                    return err_msg
                g_code += " Z" + str(self.mm_to_smoothie(Z, "Z"))

            if A is not None:
                # validate force
                err_msg = self.__validate_force(min_f, min_f_msg, config.A_F_MIN, config.A_F_MAX, "A_F_MIN", "A_F_MAX")
                if err_msg:
                    return err_msg
                # validate axis
                err_msg = self.__validate_axis(self.__a_cur.value,
                                               A,
                                               "A",
                                               self.smoothie_to_mm(config.A_MIN, "A"),
                                               self.smoothie_to_mm(config.A_MAX, "A"),
                                               "A_MIN",
                                               "A_MAX")
                if err_msg:
                    return err_msg
                g_code += " A" + str(self.mm_to_smoothie(A, "A"))

            if B is not None:
                # validate force
                err_msg = self.__validate_force(min_f, min_f_msg, config.B_F_MIN, config.B_F_MAX, "B_F_MIN", "B_F_MAX")
                if err_msg:
                    return err_msg
                # validate axis
                err_msg = self.__validate_axis(self.__b_cur.value,
                                               B,
                                               "B",
                                               self.smoothie_to_mm(config.B_MIN, "B"),
                                               self.smoothie_to_mm(config.B_MAX, "B"),
                                               "B_MIN",
                                               "B_MAX")
                if err_msg:
                    return err_msg
                g_code += " B" + str(self.mm_to_smoothie(B, "B"))

            if C is not None:
                # validate force
                err_msg = self.__validate_force(min_f, min_f_msg, config.C_F_MIN, config.C_F_MAX, "C_F_MIN", "C_F_MAX")
                if err_msg:
                    return err_msg
                # validate axis
                err_msg = self.__validate_axis(self.__c_cur.value,
                                               C,
                                               "C",
                                               self.smoothie_to_mm(config.C_MIN, "C"),
                                               self.smoothie_to_mm(config.C_MAX, "C"),
                                               "C_MIN",
                                               "C_MAX")
                if err_msg:
                    return err_msg
                g_code += " C" + str(self.mm_to_smoothie(C, "C"))

            g_code += " F" + str(min_f)

            self.__smc.write(g_code)
            response = self.__smc.read_some()

            if response == self.RESPONSE_OK:
                if X is not None:
                    self.__x_cur.value += X
                if Y is not None:
                    self.__y_cur.value += Y
                if Z is not None:
                    self.__z_cur.value += Z
                if A is not None:
                    self.__a_cur.value += A
                if B is not None:
                    self.__b_cur.value += B
                if C is not None:
                    self.__c_cur.value += C
            return response

    def custom_move_to(self, *,
                       X_F: OptionalNumber=None,
                       Y_F: OptionalNumber=None,
                       Z_F: OptionalNumber=None,
                       A_F: OptionalNumber=None,
                       B_F: OptionalNumber=None,
                       C_F: OptionalNumber=None,
                       X: OptionalNumber=None,
                       Y: OptionalNumber=None,
                       Z: OptionalNumber=None,
                       A: OptionalNumber=None,
                       B: OptionalNumber=None,
                       C: OptionalNumber=None)-> str:
        """Movement to the specified position"""

        with self.__sync_locker:
            # check given forces
            if self.__check_arg_types([type(None)], X_F, Y_F, Z_F, A_F, B_F, C_F):
                raise TypeError(f"at least one given force value shouldn't be a None")
            if not self.__check_arg_types([float, int, type(None)], X_F, Y_F, Z_F, A_F, B_F, C_F):
                raise TypeError(f"incorrect force value(s) type(s)")

            # check given axes
            if self.__check_arg_types([type(None)], X, Y, Z, A, B, C):
                raise TypeError(f"at least one given axis value shouldn't be a None")
            if not self.__check_arg_types([float, int, type(None)], X, Y, Z, A, B, C):
                raise TypeError(f"incorrect axis value(s) type(s)")

            # apply min of given forces (and pass by Nones)
            min_f_msg = "(min force value applied)"
            min_f = min([item for item in [X_F, Y_F, Z_F, A_F, B_F, C_F] if item is not None])
            g_code = "G0"
            
            sm_x_mm, sm_y_mm, sm_z_mm, sm_a_mm, sm_b_mm, sm_c_mm = 0.0,  0.0,  0.0,  0.0,  0.0,  0.0

            if X is not None:
                # validate force
                err_msg = self.__validate_force(min_f, min_f_msg, config.X_F_MIN, config.X_F_MAX, "X_F_MIN", "X_F_MAX")
                if err_msg:
                    return err_msg
                # validate axis
                err_msg = self.__validate_axis(0,
                                               X,
                                               "X",
                                               self.smoothie_to_mm(config.X_MIN, "X"),
                                               self.smoothie_to_mm(config.X_MAX, "X"),
                                               "X_MIN",
                                               "X_MAX")
                if err_msg:
                    return err_msg
                sm_x_mm = X - self.__x_cur.value
                g_code += " X" + str(self.mm_to_smoothie(sm_x_mm, "X"))

            if Y is not None:
                # validate force
                err_msg = self.__validate_force(min_f, min_f_msg, config.Y_F_MIN, config.Y_F_MAX, "Y_F_MIN", "Y_F_MAX")
                if err_msg:
                    return err_msg
                # validate axis
                err_msg = self.__validate_axis(0,
                                               Y,
                                               "Y",
                                               self.smoothie_to_mm(config.Y_MIN, "Y"),
                                               self.smoothie_to_mm(config.Y_MAX, "Y"),
                                               "Y_MIN",
                                               "Y_MAX")
                if err_msg:
                    return err_msg
                sm_y_mm = Y - self.__y_cur.value
                g_code += " Y" + str(self.mm_to_smoothie(sm_y_mm, "Y"))

            if Z is not None:
                # validate force
                err_msg = self.__validate_force(min_f, min_f_msg, config.Z_F_MIN, config.Z_F_MAX, "Z_F_MIN", "Z_F_MAX")
                if err_msg:
                    return err_msg
                err_msg = self.__validate_axis(0,
                                               Z,
                                               "Z",
                                               self.smoothie_to_mm(config.Z_MIN, "Z"),
                                               self.smoothie_to_mm(config.Z_MAX, "Z"),
                                               "Z_MIN",
                                               "Z_MAX")
                if err_msg:
                    return err_msg
                sm_z_mm = Z - self.__z_cur.value
                g_code += " Z" + str(self.mm_to_smoothie(sm_z_mm, "Z"))

            if A is not None:
                # validate force
                err_msg = self.__validate_force(min_f, min_f_msg, config.A_F_MIN, config.A_F_MAX, "A_F_MIN", "A_F_MAX")
                if err_msg:
                    return err_msg
                # validate axis
                err_msg = self.__validate_axis(0,
                                               A,
                                               "A",
                                               self.smoothie_to_mm(config.A_MIN, "A"),
                                               self.smoothie_to_mm(config.A_MAX, "A"),
                                               "A_MIN",
                                               "A_MAX")
                if err_msg:
                    return err_msg
                sm_a_mm = A - self.__a_cur.value
                g_code += " A" + str(self.mm_to_smoothie(sm_a_mm, "A"))

            if B is not None:
                # validate force
                err_msg = self.__validate_force(min_f, min_f_msg, config.B_F_MIN, config.B_F_MAX, "B_F_MIN", "B_F_MAX")
                if err_msg:
                    return err_msg
                # validate axis
                err_msg = self.__validate_axis(0,
                                               B,
                                               "B",
                                               self.smoothie_to_mm(config.B_MIN, "B"),
                                               self.smoothie_to_mm(config.B_MAX, "B"),
                                               "B_MIN",
                                               "B_MAX")
                if err_msg:
                    return err_msg
                sm_b_mm = B - self.__b_cur.value
                g_code += " B" + str(self.mm_to_smoothie(sm_b_mm, "B"))

            if C is not None:
                # validate force
                err_msg = self.__validate_force(min_f, min_f_msg, config.C_F_MIN, config.C_F_MAX, "C_F_MIN", "C_F_MAX")
                if err_msg:
                    return err_msg
                err_msg = self.__validate_axis(0,
                                               C,
                                               "C",
                                               self.smoothie_to_mm(config.C_MIN, "C"),
                                               self.smoothie_to_mm(config.C_MAX, "C"),
                                               "C_MIN",
                                               "C_MAX")
                if err_msg:
                    return err_msg
                sm_c_mm = C - self.__c_cur.value
                g_code += " C" + str(self.mm_to_smoothie(sm_c_mm, "C"))

            g_code += " F" + str(min_f)

            self.__smc.write(g_code)
            response = self.__smc.read_some()

            if response == self.RESPONSE_OK:
                if X is not None:
                    self.__x_cur.value += sm_x_mm
                if Y is not None:
                    self.__y_cur.value += sm_y_mm
                if Z is not None:
                    self.__z_cur.value += sm_z_mm
                if A is not None:
                    self.__a_cur.value += sm_a_mm
                if B is not None:
                    self.__b_cur.value += sm_b_mm
                if C is not None:
                    self.__c_cur.value += sm_c_mm
            return response

    def custom_separate_xy_move_for(self, *,
                                    X_F: OptionalNumber=None,
                                    Y_F: OptionalNumber=None,
                                    X: OptionalNumber=None,
                                    Y: OptionalNumber=None)-> str:
        """Temporary wrapper for custom_move_for function, separates X and Y axes movement if X:Y ratio exceeds given
        threshold

        Supports only X and Y axes movement.
        """
        with self.__sync_locker:
            if config.ALLOW_SEPARATE_XY_MOVEMENT and X is not None and Y is not None and X_F is not None \
                    and Y_F is not None:
                rel_x, rel_y = abs(X), abs(Y)
                if (rel_x != 0 and rel_y != 0 and rel_x / rel_y > config.XY_SEP_MOV_MAX_RATIO_THRESHOLD) or \
                        (rel_x != 0 and rel_y == 0):
                    # X movement
                    res = self.custom_move_for(X_F=X_F, X=X)
                    if SmoothieAdapter.check_res_smoothie(res):
                        err_msg = f"Couldn't do separate X movement:\n" + res
                        return err_msg
                    # Y movement
                    res = self.custom_move_for(Y_F=Y_F, Y=Y)
                    if SmoothieAdapter.check_res_smoothie(res):
                        err_msg = f"Couldn't do separate Y movement:\n" + res
                        return err_msg
                    return res
            return self.custom_move_for(X_F=X_F, Y_F=Y_F, X=X, Y=Y)

    def custom_separate_xy_move_to(self, *,
                                   X_F:OptionalNumber=None,
                                   Y_F:OptionalNumber=None,
                                   X: OptionalNumber=None,
                                   Y: OptionalNumber=None)-> str:
        """Temporary wrapper for custom_move_to function, separates X and Y axes movement if X:Y ratio exceeds given
        threshold

        Supports only X and Y axes movement.
        """

        with self.__sync_locker:
            if config.ALLOW_SEPARATE_XY_MOVEMENT and X is not None and Y is not None and X_F is not None \
                    and Y_F is not None:
                rel_x, rel_y = abs(X - self.__x_cur.value), abs(Y - self.__y_cur.value)
                if (rel_x != 0 and rel_y != 0 and rel_x / rel_y > config.XY_SEP_MOV_MAX_RATIO_THRESHOLD) or \
                        (rel_x != 0 and rel_y == 0):
                    # X movement
                    res = self.custom_move_to(X_F=X_F, X=X)
                    if SmoothieAdapter.check_res_smoothie(res):
                        err_msg = f"Couldn't do separate X movement:\n" + res
                        return err_msg
                    # Y movement
                    res = self.custom_move_to(Y_F=Y_F, Y=Y)
                    if SmoothieAdapter.check_res_smoothie(res):
                        err_msg = f"Couldn't do separate Y movement:\n" + res
                        return err_msg
                    return res
            return self.custom_move_to(X_F=X_F, Y_F=Y_F, X=X, Y=Y)

    def seeder_close(self):
        """Close exit of robot's seeder

        Sends 'M280 S2' command to smoothie. Returns smoothie answer message."""

        self.__smc.write(f"M280 S{config.SEEDER_CLOSE_COMMAND}")
        return self.__smc.read_some()

    def seeder_open(self):
        """Open exit of robot's seeder

        Sends 'M280 S5.5' command to smoothie. Returns smoothie answer message."""

        self.__smc.write(f"M280 S{config.SEEDER_OPEN_COMMAND}")
        return self.__smc.read_some()

    def nav_calibrate_wheels(self):
        """Calibrates nav. wheels and sets their current position to adapter and smoothie.
        NOT TESTED YET!
        """

        with self.__sync_locker:
            res = self.custom_move_for(A_F=config.A_F_MAX, A=config.A_MAX)
            self.wait_for_all_actions_done()
            if SmoothieAdapter.check_res_smoothie(res):
                return res

            res = self.custom_move_for(A_F=config.A_F_MAX, A=-(abs(config.A_MIN) + abs(config.A_MAX)))
            self.wait_for_all_actions_done()
            if SmoothieAdapter.check_res_smoothie(res):
                return res

            return self.set_current_coordinates(A=config.A_MIN)

    def ext_calibrate_cork(self) -> str:

        if not set(config.CALIBRATION_ORDER).issubset(set(["X", "Y", "Z", "A", "B", "C"])):
            raise ValueError(f"unsupported axis label or wrong type")

        for axis_label in config.CALIBRATION_ORDER:
                
            if eval("config.USE_"+axis_label+"_AXIS_CALIBRATION"):
                res = self.__calibrate_axis(
                    self.__axis_cur[axis_label],
                    axis_label,
                    eval("config."+axis_label+"_MIN"), 
                    eval("config."+axis_label+"_MAX"), 
                    eval("config."+axis_label+"_AXIS_CALIBRATION_TO_MAX")
                )
                if SmoothieAdapter.check_res_smoothie(res):
                    raise RuntimeError(f"Couldn't calibrate {axis_label} axis, smoothie response:\n" + res)

        return self.RESPONSE_OK

    def ext_cork_up(self) -> str:
        # cork up is done by Z axis calibration
        if config.USE_Z_AXIS_CALIBRATION:
            # TODO: stub (G28 isn't reading F value from smoothie config, it uses last received F)
            response = self.custom_move_for(Z_F=config.Z_F_EXTRACTION_UP, Z=-0.1)
            if response != self.RESPONSE_OK:
                return response

            response = self.__calibrate_axis(self.__z_cur,
                                             "Z",
                                             config.Z_MIN,
                                             config.Z_MAX,
                                             config.Z_AXIS_CALIBRATION_TO_MAX)

            if self.RESPONSE_HOMING_FAILED in response:
                for i in range(config.RETRY_CORK_UP_MIN, config.RETRY_CORK_UP_MAX+config.RETRY_CORK_UP_STEP, config.RETRY_CORK_UP_STEP):
                    response = self.__smc.read_some()
                    msg = f"Homing failed during cork up, retry with Z{i} down before up."
                    self.__logger.error(msg)
                    response = self.reset_halted_state()
                    if self.RESPONSE_AFTER_M999 in response:
                        response = self.__smc.read_some()
                        if not self.RESPONSE_OK[:2] in response:
                            return response
                    elif not self.RESPONSE_OK in response:
                        return response

                    response = self.custom_move_for(Z_F=config.Z_F_EXTRACTION_DOWN, Z=i)
                    response = self.__calibrate_axis(self.__z_cur,
                                         "Z",
                                         config.Z_MIN,
                                         config.Z_MAX,
                                         config.Z_AXIS_CALIBRATION_TO_MAX)
                    if self.RESPONSE_HOMING_FAILED not in response:
                        return response
                return response
            else:
                return response

        else:
            raise RuntimeError(
                f"picking up corkscrew with stoppers usage requires Z axis calibration permission in config"
            )

    @classmethod
    def mm_to_smoothie(cls, mm_axis_val: Number, axis_label: str)-> float:
        """Converts given mms value to smoothie value applying (multiplying) coefficient corresponding to given axis
        label

        Example: config coefficient = 0.5, given mms value = 100, returned smoothie value = 50
        """

        if axis_label not in cls.AXIS_LABELS:
            raise ValueError(f"[{cls.__name__}] -> unsupported axis label or wrong type")
        if not SmoothieAdapter.__check_arg_types([int, float], mm_axis_val):
            raise TypeError(f"[{cls.__name__}] -> axis_value should be float or int")

        coefficient = getattr(
            config,
            "{}_COEFFICIENT_TO_MM".format(axis_label)
        )

        return float(mm_axis_val) * float(coefficient)

    @classmethod
    def smoothie_to_mm(cls, sm_axis_val:Number, axis_label: str)-> float:
        """Converts given smoothie value to mms value applying (dividing) coefficient corresponding to given axis
        label

        Example: coefficient = 0.5, given smoothie value = 50, returned mms value = 100
        """

        if axis_label not in cls.AXIS_LABELS:
            raise ValueError(f"[{cls.__name__}] -> unsupported axis label or wrong type")
        if not SmoothieAdapter.__check_arg_types([int, float], sm_axis_val):
            raise TypeError(f"[{cls.__name__}] -> axis_value should be float or int")
        
        coefficient = getattr(
            config,
            "{}_COEFFICIENT_TO_MM".format(axis_label)
        )

        return float(sm_axis_val) / float(coefficient)

    @classmethod
    def __check_arg_types(cls, types: List[Type[Any]], *args: Any) -> bool:
        """Returns True if all given variables (*args) types are in given types list, False otherwise
        """
        if len(args) < 1:
            raise TypeError(f"[{cls.__name__}] -> item(s) to check is missed")
        if type(types) is not list:
            raise TypeError(f"[{cls.__name__}] -> expected list of types, received " + str(type(types)))
        if len(types) < 1:
            raise ValueError(f"[{cls.__name__}] -> list of types should contain at least one item")

        for arg in args:
            if type(arg) not in types:
                return False
        return True

    @classmethod
    def __validate_axis(cls, 
                        cur_axis_val: Number, 
                        mov_axis_val: Number, 
                        key_label: str, 
                        key_min: Number, 
                        key_max: Number, 
                        key_min_label: str, 
                        key_max_label: str) -> Optional[str]:
        """Checks if given axis movement can be done. Returns None if value is ok, info/error message otherwise.

        Receives smoothie values (may be not in mms).
        """

        if cur_axis_val + mov_axis_val > key_max:
            return f"[{cls.__name__}] -> Value {0} for {1} goes beyond max acceptable range of {3} = {2}, as current value is {4}" \
                .format(mov_axis_val, key_label, key_max, key_max_label, cur_axis_val)
        if cur_axis_val + mov_axis_val < key_min:
            return f"[{cls.__name__}] -> Value {0} for {1} goes beyond min acceptable range of {3} = {2}, as current value is {4}" \
                .format(mov_axis_val, key_label, key_min, key_min_label, cur_axis_val)
        return None

    @classmethod
    def __validate_force(cls, value:Number, key_label:str, key_min:Number, key_max:Number, key_min_label:str, key_max_label:str)-> Optional[str]:
        """Checks if given force can be applied. Returns None if value is ok, info/error message otherwise.
        """

        if value > key_max:
            return f"[{cls.__name__}] -> Value {value} for {key_label} goes beyond max acceptable range of {key_max_label} = {key_max}"
        if value < key_min:
            return f"[{cls.__name__}] -> Value {value} for {key_label} goes beyond min acceptable range of {key_min_label} = {key_min}"
        return None

    def __calibrate_axis(self,
                        axis_cur: Any,
                        axis_label: str,
                        sm_axis_min: Number,
                        sm_axis_max: Number,
                        axis_calibration_to_max: bool):
        # TODO: need to implement outer axix_cur var if removing multiprocessing.Value in future

        with self.__sync_locker:
            # TODO: stub (G28 isn't reading F value from smoothie config, it uses last received F)
            if axis_label == "Z":
                response = self.custom_move_for(Z_F=config.Z_F_EXTRACTION_UP, Z=-0.1)
                if response != self.RESPONSE_OK:
                    return response

            # do calibration
            if axis_calibration_to_max:
                self.__smc.write("G28 {0}{1}".format(axis_label, config.CALIBRATION_DISTANCE))
                response = self.__smc.read_some()
                if response == self.RESPONSE_OK:
                    sm_val, axis_cur.value = sm_axis_max, self.smoothie_to_mm(sm_axis_max, axis_label)
                else:
                    return response
            else:
                self.__smc.write("G28 {0}{1}".format(axis_label, -config.CALIBRATION_DISTANCE))
                response = self.__smc.read_some()
                if response == self.RESPONSE_OK:
                    sm_val, axis_cur.value = sm_axis_min, self.smoothie_to_mm(sm_axis_min, axis_label)
                else:
                    return response

            # set fresh current coordinates on smoothie too
            self.__smc.write("G92 {0}{1}".format(axis_label, sm_val))
            return self.__smc.read_some()
        

class VescAdapterV4:
    """Provides multiple vesc control with smooth RPM controls

    To add more vesc engines:
    1) Add unique vesc ID key for users as class static member below
    2) Add keys to config.py to make it possible to enable-disable your new vesc and set its settings easily
    3) Add new vesc engine initialization code to 'INIT ALL ALLOWED VESCS HERE' section (use code there as example)
    """

    # unique vesc ID keys for users (it's a keys for vesc can IDs, not IDs themselves)
    PROPULSION_KEY = 0
    EXTRACTION_KEY = 1

    def __init__(self, ser_port: str, ser_baudrate: int, alive_freq: float, check_freq: float, stopper_check_freq: float):
        self.__logger = LoggerFactory.create(self.__class__.__name__)
        self.__locker = threading.Lock()
        self.__reconnect_locker = threading.Lock()

        self.__ser_port = ser_port
        self.__ser_baudrate = ser_baudrate

        self.__stopper_check_freq = stopper_check_freq
        self.__alive_freq = alive_freq
        self.__check_freq = check_freq
        self.__next_alive_time = time.time()

        self.__can_ids: Dict[int, Optional[int]] = dict()
        self.__current_rpm: Dict[int, Number] = dict()
        self.__target_rpm: Dict[int, Number] = dict()
        self.__use_smooth_accel: Dict[int, bool] = dict()
        self.__smooth_accel_next_t: Dict[int, float] = dict()
        self.__use_smooth_decel: Dict[int, bool] = dict()
        self.__smooth_decel_next_t: Dict[int, float] = dict()
        self.__time_to_move: Dict[int, float] = dict()
        self.__start_time: Dict[int, float] = dict()
        self.__is_moving: Dict[int, float] = dict()
        self.__stop_request: Dict[int, bool] = dict()
        self.__last_stop_time: Dict[int, float] = dict()
        self.__stopper_signals: Dict[int, int] = dict()

        self.__ser = serial.Serial(port=self.__ser_port, baudrate=ser_baudrate)
        self.__ser.reset_input_buffer()
        self.__ser.reset_output_buffer()
        self.__ser.timeout = config.VESC_TIMEOUT_READ

        # INIT ALL ALLOWED VESCS HERE
        # init PROPULSION vesc (currently it's parent vesc so it has no checkings for ID and has parent's ID=None)
        if config.VESC_ALLOW_PROPULSION:
            if config.VESC_PROPULSION_AUTODETECT_CAN_ID:
                raise NotImplementedError(f"Can id detection is not confirmed to work fine.")
            else:
                prop_can_id = config.VESC_PROPULSION_CAN_ID
            self.__can_ids[self.PROPULSION_KEY] = prop_can_id  # parent vesc has ID=None
            self.__current_rpm[self.PROPULSION_KEY] = 0
            self.__target_rpm[self.PROPULSION_KEY] = 0
            self.__use_smooth_accel[self.PROPULSION_KEY] = False
            self.__smooth_accel_next_t[self.PROPULSION_KEY] = 0.0
            self.__use_smooth_decel[self.PROPULSION_KEY] = False
            self.__smooth_decel_next_t[self.PROPULSION_KEY] = 0.0
            self.__time_to_move[self.PROPULSION_KEY] = 0
            self.__start_time[self.PROPULSION_KEY] = 0.0
            self.__is_moving[self.PROPULSION_KEY] = False
            self.__stop_request[self.PROPULSION_KEY] = False
            self.__last_stop_time[self.PROPULSION_KEY] = 0.0
            self.__stopper_signals[self.PROPULSION_KEY] = config.VESC_PROPULSION_STOP_SIGNAL

        # init EXTRACTION vesc
        if config.VESC_ALLOW_EXTRACTION:
            if config.VESC_EXTRACTION_AUTODETECT_CAN_ID:
                raise NotImplementedError("can id detection is not confirmed to work fine")
                # ext_can_id = self.get_unregistered_can_id()
            else:
                ext_can_id = config.VESC_EXTRACTION_CAN_ID
            if ext_can_id is not None: # type: ignore
                self.__can_ids[self.EXTRACTION_KEY] = ext_can_id
                self.__current_rpm[self.EXTRACTION_KEY] = 0
                self.__target_rpm[self.EXTRACTION_KEY] = 0
                self.__use_smooth_accel[self.EXTRACTION_KEY] = False
                self.__smooth_accel_next_t[self.PROPULSION_KEY] = 0.0
                self.__use_smooth_decel[self.EXTRACTION_KEY] = False
                self.__smooth_decel_next_t[self.PROPULSION_KEY] = 0.0
                self.__time_to_move[self.EXTRACTION_KEY] = 0
                self.__start_time[self.EXTRACTION_KEY] = 0.0
                self.__is_moving[self.EXTRACTION_KEY] = False
                self.__stop_request[self.EXTRACTION_KEY] = False
                self.__last_stop_time[self.EXTRACTION_KEY] = 0.0
                self.__stopper_signals[self.EXTRACTION_KEY] = config.VESC_EXTRACTION_STOP_SIGNAL
            else:
                # TODO what robot should do if initialization was failed?
                self.__logger.error(f"Extraction vesc initialization fail: couldn't determine extraction vesc ID.")
        # init any new vescs (add vesc init code here)
        # ...

        self.__keep_thread_alive = True
        self._movement_ctrl_th = threading.Thread(target=self._movement_ctrl_th_tf, daemon=True)
        self._movement_ctrl_th.start()

        # DO ALL ALLOWED CALIBRATIONS HERE
        # propulsion vasc calibration
        if config.VESC_PROPULSION_CALIBRATE_AT_INIT:
            self.set_target_rpm(config.VESC_PROPULSION_CALIBRATION_RPM, self.PROPULSION_KEY)
            self.set_time_to_move(config.VESC_PROPULSION_CALIBRATION_MAX_TIME, self.PROPULSION_KEY)
            self.start_moving(self.PROPULSION_KEY)
            res = self.wait_for_stopper_hit(self.PROPULSION_KEY, config.VESC_PROPULSION_CALIBRATION_MAX_TIME)
            self.stop_moving(self.PROPULSION_KEY)
            if not res:
                # TODO what robot should do if calibration was failed (there was no stopper hit)?
                self.__logger.error("Stopped vesc PROPULSION engine calibration due timeout (stopper signal wasn't received!)")

        # extraction vesc calibration
        if config.VESC_EXTRACTION_CALIBRATE_AT_INIT:
            # Z-5 fix (move cork little down to stop touching stopper)
            self.set_target_rpm(config.VESC_EXTRACTION_CALIBRATION_Z5_FIX_RPM, self.EXTRACTION_KEY)
            self.set_time_to_move(config.VESC_EXTRACTION_CALIBRATION_Z5_FIX_TIME, self.EXTRACTION_KEY)
            self.start_moving(self.EXTRACTION_KEY)
            self.wait_for_stop(self.EXTRACTION_KEY)

            # calibration
            self.set_target_rpm(config.VESC_EXTRACTION_CALIBRATION_RPM, self.EXTRACTION_KEY)
            self.set_time_to_move(config.VESC_EXTRACTION_CALIBRATION_MAX_TIME, self.EXTRACTION_KEY)
            self.start_moving(self.EXTRACTION_KEY)
            res = self.wait_for_stopper_hit(self.EXTRACTION_KEY, config.VESC_EXTRACTION_CALIBRATION_MAX_TIME)
            self.stop_moving(self.EXTRACTION_KEY)
            if not res:
                # TODO what robot should do if calibration was failed (there was no stopper hit)?
                self.__logger.error(f"Stopped vesc EXTRACTION engine calibration due timeout (stopper signal wasn't received!).")
        # do any new calibrations (add vesc calibration code here)
        # ...
        self.__last_reconnect_time = time.time() - 60

    def __enter__(self):
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any):
        self.close()

    def __del__(self):
        self.close()
        del self.__ser

    def close(self):
        
        if self.__ser.is_open:
            self.__keep_thread_alive = False

            for engine_key in self.__can_ids:
                self.stop_moving(engine_key)

            self._movement_ctrl_th.join(1)
            self.__ser.close()

    def reconnect_vesc(self) :
        with self.__reconnect_locker :
            if time.time() - self.__last_reconnect_time < 60 :
                return

            self.__logger.debug(self)
            self.__last_reconnect_time = time.time()
            self.__ser.close()

            smoothie_vesc_addr = utility.get_smoothie_vesc_addresses()
            while not "vesc" in smoothie_vesc_addr:
                msg = f"Couldn't get vesc's USB address, stopping attempt to unlock with lifeline."
                self.__logger.info(msg)
                time.sleep(1)
                smoothie_vesc_addr = utility.get_smoothie_vesc_addresses()
                
            vesc_address = smoothie_vesc_addr["vesc"]
            msg = f"Finding vesc's USB address at '{vesc_address}'."
            self.__logger.info(msg)
            
            could_open_port = False
            while not could_open_port :
                try : 
                    self.__ser = serial.Serial(port=vesc_address, baudrate=self.__ser_baudrate)
                    could_open_port = True
                    self.__ser.reset_input_buffer()
                    self.__ser.reset_output_buffer()
                    self.__ser.timeout = 5
                    self.__logger.info(f"It is reconnected!")
                except KeyboardInterrupt:
                    raise KeyboardInterrupt
                except Exception as e:
                    self.__logger.error(f"Could not open port ({e}).")
                    time.sleep(1)

    def get_unregistered_can_id(self):
        for can_id in range(0, 253):
            data = self.__get_firmware_version(['version_major', 'version_minor'], can_id) # type: ignore
            if data is not None and can_id not in self.__can_ids.values():
                return can_id
        return None

    def __get_firmware_version(self, report_field_names: List[str], can_id: int) -> Optional[Dict[str, Any]]:
        in_buf = b''
        with self.__locker:
            try :
                encoded_msg = pyvesc.encode_request(pyvesc.GetFirmwareVersion(can_id=can_id))# type: ignore
                if isinstance(encoded_msg, bytes):
                    self.__ser.write(encoded_msg)
                else:  
                    raise ValueError(f"Failed to encode GetFirmwareVersion message for can_id={can_id}.")
                while self.__ser.in_waiting > 0:
                    in_buf += self.__ser.read(self.__ser.in_waiting)
            except SerialException :
                self.reconnect_vesc()


        if len(in_buf) == 0:
            return None
        response, consumed = pyvesc.decode(in_buf) # type: ignore
        if consumed == 0:
            return None

        if isinstance(response, pyvesc.GetFirmwareVersion):
            report_row: Dict[str, Any] = {}
            for field_name in report_field_names:
                report_row[field_name] = getattr(response, field_name)
            return report_row
        return None

    def _movement_ctrl_th_tf(self):
        """Target function of movement control thread (only inner usage).

        Implements keeping multiple vesc engines alive and stopping them by timers if they were set.
        """
        try:
            while self.__keep_thread_alive:
                with self.__locker:
                    now = time.time()

                    # Process each active engine
                    for engine_key in self.__can_ids:
                        if not self.__keep_thread_alive:
                            break

                        if not self.__is_moving[engine_key]:
                            continue

                        can_id = self.__can_ids[engine_key]

                        # Engine movement timeout or stop request
                        timeout_reached = now - self.__start_time[engine_key] >= self.__time_to_move[engine_key]
                        stop_requested = self.__stop_request[engine_key]

                        if timeout_reached or stop_requested:
                            # Immediate engine stop
                            if not self.__use_smooth_decel[engine_key]:
                                try:
                                    encoded_msg = pyvesc.encode(pyvesc.SetRPM(0, can_id=can_id))# type: ignore
                                    if isinstance(encoded_msg, bytes):
                                        self.__ser.write(
                                            encoded_msg
                                        )
                                    else:
                                        raise ValueError(f"Failed to encode SetRPM message for can_id={can_id}.")
                                except SerialException:
                                    self.reconnect_vesc()

                                self.__current_rpm[engine_key] = 0
                                self.__last_stop_time[engine_key] = time.time()
                                self.__is_moving[engine_key] = False
                                self.__stop_request[engine_key] = False

                            # Smooth engine stop
                            elif time.time() >= self.__smooth_decel_next_t[engine_key]:
                                self.__smooth_decel_next_t[engine_key] = (
                                    time.time() + config.VESC_SMOOTH_DECEL_TIME_STEP
                                )

                                # Reduce speed
                                if abs(self.__current_rpm[engine_key]) > config.VESC_SMOOTH_DECEL_RPM_STEP:
                                    self.__current_rpm[engine_key] += (
                                        -config.VESC_SMOOTH_DECEL_RPM_STEP
                                        if self.__current_rpm[engine_key] > 0
                                        else config.VESC_SMOOTH_DECEL_RPM_STEP
                                    )

                                    try:
                                        encoded_msg = pyvesc.encode(pyvesc.SetRPM(self.__current_rpm[engine_key],can_id=can_id)) # type: ignore
                                        if isinstance(encoded_msg, bytes):
                                            self.__ser.write(
                                                encoded_msg
                                            )
                                        else:
                                            raise ValueError(f"Failed to encode SetRPM message for can_id={can_id}.")
                                    except SerialException:
                                        self.reconnect_vesc()

                                # Stop engine
                                else:
                                    try:
                                        encoded_msg = pyvesc.encode(pyvesc.SetRPM(0, can_id=can_id)) # type: ignore
                                        if isinstance(encoded_msg, bytes):
                                            self.__ser.write(
                                                encoded_msg
                                            )
                                        else:
                                            raise ValueError(f"Failed to encode SetRPM message for can_id={can_id}.")
                                    except SerialException:
                                        self.reconnect_vesc()

                                    self.__current_rpm[engine_key] = 0
                                    self.__last_stop_time[engine_key] = time.time()
                                    self.__is_moving[engine_key] = False
                                    self.__stop_request[engine_key] = False

                        # Smooth start engine if needed
                        elif (
                            self.__use_smooth_accel[engine_key]
                            and time.time() >= self.__smooth_accel_next_t[engine_key]
                        ):
                            self.__smooth_accel_next_t[engine_key] = (
                                time.time() + config.VESC_SMOOTH_ACCEL_TIME_STEP
                            )

                            # Set engine to target RPM because difference is <= RPM step
                            if abs(self.__target_rpm[engine_key] - self.__current_rpm[engine_key]) <= config.VESC_SMOOTH_ACCEL_RPM_STEP:
                                if self.__current_rpm[engine_key] != self.__target_rpm[engine_key]:
                                    try:
                                        encoded_msg = pyvesc.encode(pyvesc.SetRPM(self.__target_rpm[engine_key], can_id=can_id)) # type: ignore
                                        if isinstance(encoded_msg, bytes):
                                            self.__ser.write(
                                                encoded_msg
                                            )
                                        else:
                                            raise ValueError(f"Failed to encode SetRPM message for can_id={can_id}.")
                                    except SerialException:
                                        self.reconnect_vesc()

                                    self.__current_rpm[engine_key] = self.__target_rpm[engine_key]

                            # Increase current RPM by RPM step
                            else:
                                self.__current_rpm[engine_key] += (
                                    config.VESC_SMOOTH_ACCEL_RPM_STEP
                                    if self.__target_rpm[engine_key] > self.__current_rpm[engine_key]
                                    else -config.VESC_SMOOTH_ACCEL_RPM_STEP
                                )

                                try:
                                    encoded_msg = pyvesc.encode(pyvesc.SetRPM(self.__current_rpm[engine_key], can_id=can_id)) # type: ignore
                                    if isinstance(encoded_msg, bytes):
                                        self.__ser.write(
                                            encoded_msg
                                        )
                                    else:
                                        raise ValueError(f"Failed to encode SetRPM message for can_id={can_id}.")
                                except SerialException:
                                    self.reconnect_vesc()

                    # Send alive to each active engine
                    if time.time() > self.__next_alive_time:
                        self.__next_alive_time = time.time() + 1 / self.__alive_freq

                        for engine_key in self.__is_moving:
                            can_id = self.__can_ids[engine_key]

                            try:
                                if self.__is_moving[engine_key]:
                                    encoded_msg = pyvesc.encode(pyvesc.SetAlive(can_id=can_id)) # type: ignore
                                    if isinstance(encoded_msg, bytes):
                                        self.__ser.write(
                                            encoded_msg
                                        )
                                    else:
                                        raise ValueError(f"Failed to encode SetAlive message for can_id={can_id}.")
                            except (SerialException, OSError) as e:
                                if getattr(e, "errno", None) == 5 or isinstance(e, SerialException):
                                    self.reconnect_vesc()

                # Wait for next checking tick
                time.sleep(1 / self.__check_freq)

        except serial.SerialException as ex:
            self.__logger.error(f"{ex}")
            self.__logger.error(f"Movement control thread stopped")
        except Exception as ex:
            self.__logger.error(f"Unexpected error in movement control thread: {ex}")
            self.__logger.error(f"Movement control thread stopped")
            

    def start_moving(self, engine_key: int, smooth_acceleration: bool = False, smooth_deceleration: bool = False):
        with self.__locker:
            self.__use_smooth_accel[engine_key] = smooth_acceleration
            self.__use_smooth_decel[engine_key] = smooth_deceleration
            self.__start_time[engine_key] = time.time()

            if smooth_deceleration:
                self.__smooth_decel_next_t[engine_key] = 0
            if smooth_acceleration:
                self.__smooth_accel_next_t[engine_key] = 0
            else:
                try :
                    encoded_msg = pyvesc.encode(pyvesc.SetRPM(self.__target_rpm[engine_key],can_id=self.__can_ids[engine_key])) # type: ignore
                    if isinstance(encoded_msg, bytes):
                        self.__ser.write(encoded_msg)
                    else:
                        raise TypeError(f"pyvesc.encode returned unexpected type: {type(encoded_msg)}") # type: ignore
                except SerialException :
                    self.reconnect_vesc()
                self.__current_rpm[engine_key] = self.__target_rpm[engine_key]
            self.__is_moving[engine_key] = True

    def stop_moving(self, engine_key: int, smooth_deceleration: bool = False):        
        self.__logger.debug(f"Stopping engine {engine_key} with smooth deceleration: {smooth_deceleration}.")
        with self.__locker:
            self.__use_smooth_decel[engine_key] = smooth_deceleration

            if smooth_deceleration:
                self.__smooth_decel_next_t[engine_key] = 0
                self.__stop_request[engine_key] = True
            else:
                try :
                    encoded_msg = pyvesc.encode(pyvesc.SetRPM(0, can_id=self.__can_ids[engine_key])) # type: ignore
                    if isinstance(encoded_msg, bytes):
                        self.__ser.write(encoded_msg)
                    else:
                        raise TypeError(f"pyvesc.encode returned unexpected type: {type(encoded_msg)}") # type: ignore
                except SerialException :
                    self.reconnect_vesc()
                self.__current_rpm[engine_key] = 0
                self.__last_stop_time[engine_key] = time.time()
                self.__is_moving[engine_key] = False

    def wait_for_stop(self, engine_key: int, timeout: Optional[float]=None):
        """Blocks caller thread until specified engine is end his work or timeout time is out (if timeout was set).

        Returns True if engine has ended his work, returns False if timeout waiting time is out.
        """

        end_t = time.time() + timeout if timeout is not None else float("inf")

        while True:
            with self.__locker:
                if not self.__is_moving[engine_key]:
                    return True
            if timeout is not None and time.time() > end_t:
                return False
            time.sleep(1 / self.__check_freq)

    def wait_for_stop_any(self, timeout: Optional[float]=None) -> None:
        raise NotImplementedError(f"This feature is not implemented yet")

    def wait_for_stopper_hit(self,
                             engine_key: int,
                             timeout: Optional[float]=None,
                             stop_engine_if_timeout: bool=True,
                             stop_engine_if_stopper_hit: bool=True):
        """Blocks caller thread until specified engine stopper hit or timeout time is out (if timeout was set).

        Returns True if stopper was hit, returns False if timeout waiting time is out
        or engine was stopped by its own work timer.
        """

        end_t = time.time() + timeout if timeout is not None else float("inf")

        while True:
            with self.__locker:
                if not self.__is_moving[engine_key]:
                    return False
            if time.time() > end_t:
                if stop_engine_if_timeout:
                    self.stop_moving(engine_key)
                return False
            time.sleep(1 / self.__stopper_check_freq)

    def wait_for_stopper_hit_any(self) -> None:
        raise NotImplementedError(f"This feature is not implemented yet")

    def set_current_rpm(self, rpm: Number, engine_key: int):
        """Set as current and apply given RPM on specified by engine_key vesc engine.

        NOTICE: engine will smoothly back to its target RPM if smooth acceleration is enabled for engine_key engine.
        To apply RPM immediately, set RPM you want to apply as target RPM first by using set_target_rpm method before
        calling this.
        """
        with self.__locker:
            try :
                encoded_msg = pyvesc.encode(pyvesc.SetRPM(rpm, can_id=self.__can_ids[engine_key])) # type: ignore
                if isinstance(encoded_msg, bytes):
                    self.__ser.write(encoded_msg)
                else:
                    raise TypeError(f"pyvesc.encode returned unexpected type: {type(encoded_msg)}") # type: ignore
            except SerialException :
                self.reconnect_vesc()
            self.__current_rpm[engine_key] = rpm

    def set_target_rpm(self, rpm: Number, engine_key: int):
        """Set given RPM as target RPM for specified engine_key vesc engine.

        NOTICE: engine will speed up to this RPM smoothly if smooth acceleration is enabled for engine_key vesc engine,
        otherwise this RPM will be applied immediately during engine start. In this case high RPM values may lead to
        strong jerk during the start.
        """
        with self.__locker:
            self.__target_rpm[engine_key] = rpm

    def set_time_to_move(self, time_to_move: Number, engine_key: int):
        if time_to_move < 0:
            msg = f"time_to_move must be >= 0, got {str(time_to_move)} instead"
            raise ValueError(msg)

        with self.__locker:
            self.__time_to_move[engine_key] = time_to_move

    def set_alive_freq(self, alive_freq: Number):
        if alive_freq < 0:
            msg = f"alive_freq must be >= 0, got {str(alive_freq)} instead"
            raise ValueError(msg)

        with self.__locker:
            self.__alive_freq = alive_freq

    def set_check_freq(self, check_freq: Number):
        if check_freq < 0:
            msg = f"check_freq must be >= 0, got {str(check_freq)} instead"
            raise ValueError(msg)

        with self.__locker:
            self.__check_freq = check_freq

    def set_smooth_acceleration(self, smooth_acceleration: bool, engine_key: int):
        with self.__locker:
            self.__use_smooth_accel[engine_key] = smooth_acceleration

    def set_smooth_deceleration(self, smooth_deceleration: bool, engine_key: int):
        with self.__locker:
            self.__use_smooth_decel[engine_key] = smooth_deceleration

    def get_smooth_acceleration(self, engine_key: int):
        with self.__locker:
            return self.__use_smooth_accel[engine_key]

    def get_smooth_deceleration(self, engine_key: int):
        with self.__locker:
            return self.__use_smooth_decel[engine_key]

    def get_last_stop_time(self, engine_key: int):
        with self.__locker:
            return self.__last_stop_time[engine_key]

    def get_last_start_time(self, engine_key: int):
        with self.__locker:
            return self.__start_time[engine_key]

    def get_last_movement_time(self, engine_key: int):
        """Returns last movement time if VESCs are not working at the moment;
        returns current working time if VESCs are working at the moment.
        """

        with self.__locker:
            if math.isclose(self.__start_time[engine_key], 0):
                return 0
            elif self.__is_moving[engine_key] or math.isclose(self.__last_stop_time[engine_key], 0):
                return time.time() - self.__start_time[engine_key]
            else:
                return self.__last_stop_time[engine_key] - self.__start_time[engine_key]

    def get_current_rpm(self, engine_key: int):
        """Returns specified vesc engine current RPM"""

        return self.__current_rpm[engine_key]

    def get_target_rpm(self, engine_key: int):
        """Returns specified vesc engine target RPM"""

        return self.__target_rpm[engine_key]

    def get_sensors_data_of_can_id(self, report_field_names: List[str], can_id: int)-> Optional[Dict[str, Any]]:
        in_buf = b''
        with self.__locker:
            try :
                encoded_msg = pyvesc.encode_request(pyvesc.GetValues(can_id=can_id))# type: ignore
                if isinstance(encoded_msg, bytes):
                    self.__ser.write(encoded_msg)
                else:
                    raise ValueError(f"Failed to encode GetValues message for can_id={can_id}.")
                while self.__ser.in_waiting > 0:
                    in_buf += self.__ser.read(self.__ser.in_waiting)
            except SerialException :
                self.reconnect_vesc()

        if len(in_buf) == 0:
            return None
        response, consumed = pyvesc.decode(in_buf)# type: ignore
        if consumed == 0:
            return None

        if isinstance(response, pyvesc.GetValues):
            report_row: Dict[str, Any] = {}
            for field_name in report_field_names:
                report_row[field_name] = getattr(response, field_name)
            return report_row
        return None

    def get_sensors_data(self, report_field_names: List[str], engine_key: int)-> Optional[Dict[str, Any]]:
        in_buf = b''
        with self.__locker:
            try : 
                encoded_msg = pyvesc.encode_request(pyvesc.GetValues(can_id=self.__can_ids[engine_key]))# type: ignore
                if isinstance(encoded_msg, bytes):
                    self.__ser.write(encoded_msg)
                else:
                    raise ValueError(f"Failed to encode GetValues message for can_id={self.__can_ids[engine_key]}.")
                while self.__ser.in_waiting > 0:
                    in_buf += self.__ser.read(self.__ser.in_waiting)
            except SerialException :
                self.reconnect_vesc()

        if len(in_buf) == 0:
            return None
        response, consumed = pyvesc.decode(in_buf)# type: ignore
        if consumed == 0:
            return None

        if isinstance(response, pyvesc.GetValues):
            report_row: Dict[str, Any] = {}
            for field_name in report_field_names:
                report_row[field_name] = getattr(response, field_name)
            return report_row
        return None

    def is_moving(self, engine_key: int):
        with self.__locker:
            return self.__is_moving[engine_key]



class GPSUbloxAdapter:
    """Provides access to the robot's on-board GPS navigator (UBLOX card)"""

    def __init__(self, ser_port: str, ser_baudrate: int, last_pos_count: int):
        if last_pos_count < 1:
            raise ValueError(f"last_pos_count shouldn't be less than 1, got {last_pos_count} instead")
        
        self.__logger = LoggerFactory.create(self.__class__.__name__)

        self._position_is_fresh = False
        self._last_pos_count = last_pos_count
        self._ser_port = ser_port
        self._ser_baudrate = ser_baudrate
        self._last_pos_container: List[navigation.GPSPoint] = []
        self._sync_locker = multiprocessing.RLock()

        self._serial = self._get_new_connection()

        self._keep_thread_alive = True
        self._reader_thread = threading.Thread(target=self._reader_thread_tf, daemon=True)
        self._reader_thread.start()

    def __enter__(self):
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any):
        self.close()

    def __del__(self):
        self.close()

    def close(self):
        self._keep_thread_alive = False
        self._reader_thread.join()
        if self._serial.is_open:
            self._serial.close()

    def disconnect(self):
        """Obsolete method, stills here for backward compatibility, use close() instead"""

        self.close()

    def reconnect(self):
        self._serial = self._get_new_connection(self._serial)

    def get_fresh_position(self) -> List[Any]:
        """Waits for new fresh position from gps and returns it, blocking until new position received.
        Returns copy of stored position (returned value can be safely changed with no worrying about obj reference
        features)"""

        # self._get_fresh_time = time.time()
        
        # while len(self._last_pos_container) < 1:
            # if time.time() - self._get_fresh_time > config.NO_GPS_TIMEOUT:
                # raise TimeoutError
            # pass
        
        with self._sync_locker:
            self._position_is_fresh = False
            
        while True:
            # if time.time() - self._get_fresh_time > config.NO_GPS_TIMEOUT:
                # raise TimeoutError
            with self._sync_locker:
                if not self._position_is_fresh:
                    continue
            return self.get_last_position()

    def get_fresh_position_v2(self) -> navigation.GPSPoint:
        """Waits for new fresh position from gps and returns it, blocking until new position received.
        Returns copy of stored position (returned value can be safely changed with no worrying about obj reference
        features)"""

        # self._get_fresh_time = time.time()

        # while len(self._last_pos_container) < 1:
        # if time.time() - self._get_fresh_time > config.NO_GPS_TIMEOUT:
        # raise TimeoutError
        # pass

        with self._sync_locker:
            self._position_is_fresh = False

        while True:
            # if time.time() - self._get_fresh_time > config.NO_GPS_TIMEOUT:
            # raise TimeoutError
            with self._sync_locker:
                if not self._position_is_fresh:
                    continue
            return self.get_last_position_v2()

    def get_last_position(self) -> List[Any]:
        """Waits until at least one position is stored, returns last saved position copy at the moment of call
        (reference type safe)

        Returned position is in "old list" format.
        """

        while len(self._last_pos_container) < 1:
            pass
        with self._sync_locker:
            return self._last_pos_container[-1].as_old_list

    def get_last_position_non_blocking(self) -> Optional[List[Number]]:
        """Returns None if no positions are stored, returns last saved position copy at the moment of call
        (reference type safe)

        Returned position is in "old list" format.
        """

        with self._sync_locker:
            return self._last_pos_container[-1].as_old_list if len(self._last_pos_container) > 0 else None

    def get_last_position_v2(self) -> navigation.GPSPoint:
        """Waits until at least one position is stored, returns last saved position copy at the moment of call
        (reference type safe)

        Returned position is an instance of navigation.GPSPoint class.
        """

        while len(self._last_pos_container) < 1:
            pass
        with self._sync_locker:
            # TODO currently it's not a deep copy
            return self._last_pos_container[-1]

    def get_last_position_v2_non_blocking(self) -> Optional[navigation.GPSPoint]:
        """Returns None if no positions are stored, returns last saved position copy at the moment of call
        (reference type safe)

        Returned position is an instance of navigation.GPSPoint class.
        """

        with self._sync_locker:
            # TODO currently it's not a deep copy
            return self._last_pos_container[-1] if len(self._last_pos_container) > 0 else None

    def get_last_positions_list(self) -> List[List[Number]]:
        """Waits until at least one position is stored, returns list of last saved positions copies at the moment of
        call (reference type safe)"""

        get_last_positions_list_time = time.time()
        positions: List[List[Number]] = []

        while len(self._last_pos_container) < 1:
            if time.time() - get_last_positions_list_time > config.NO_GPS_TIMEOUT:
                raise TimeoutError

        with self._sync_locker:
            for point in self._last_pos_container:
                positions.append(point.as_old_list)
            return positions

    def get_stored_pos_count(self):
        return len(self._last_pos_container)

    def _reader_thread_tf(self):
        try:
            while self._keep_thread_alive:
                position = self._read_from_gps()
                with self._sync_locker:
                    if len(self._last_pos_container) == self._last_pos_count:
                        self._last_pos_container.pop(0)
                    self._last_pos_container.append(position)
                    self._position_is_fresh = True
        except serial.SerialException as ex:
            self.__logger.error(f"Ublox reading error:", ex)

    def _read_from_gps(self):
        """Returns GPS coordinates of the current position"""

        while True:
            try:
                read_line = self._serial.readline()
                if isinstance(read_line, bytes):
                    data = str(read_line)
                    if "GNGGA" in data and ",,," not in data:
                        data = data.split(",")
                        lati, longi = self._D2M2(data[2], data[3], data[4], data[5])
                        point_quality = data[6]
                        if -90 <= lati <= 90 and -180 <= longi <= 180:
                            # return [lati, longi, point_quality]  # , float(data[11])  # alti
                            return navigation.GPSPoint(lati, longi, point_quality, float(data[1]), time.time())
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                continue

    def _D2M2(self, Lat:str, NS:str, Lon:str, EW:str):
        """Traduce NMEA format ddmmss to ddmmmm"""

        Latdd = float(Lat[:2])
        Latmmmmm = float(Lat[2:])
        Latddmmmm = Latdd + (Latmmmmm / 60.0)
        if NS == 'S':
            Latddmmmm = -Latddmmmm

        Londd = float(Lon[:3])
        Lonmmmmm = float(Lon[3:])
        Londdmmmm = Londd + (Lonmmmmm / 60.0)
        if EW == 'W':
            Londdmmmm = -Londdmmmm
        return round(Latddmmmm, 7), round(Londdmmmm, 7)

    def _USBNMEA_OUT(self):
        """Start sending NMEA out on USB port at 19200 baud"""

        Matrame = "B5 62 06 00 14 00 03 00 00 00 00 00 00 00 00 00 00 00 23 00 03 00 00 00 00 00 43 AE"
        self._serial.write(bytearray.fromhex(Matrame))

    # Start a Hot restart
    def _hot_reset(self):
        Mythread = "B5 62 06 04 04 00 00 00 02 00 10 68"
        self._serial.write(bytearray.fromhex(Mythread))

    def _get_new_connection(self, old_conn: Optional[serial.Serial] = None):
        if old_conn is not None and old_conn.is_open:
            old_conn.close()
        new_conn = serial.Serial(port=self._ser_port, baudrate=self._ser_baudrate)
        return new_conn


class GPSUbloxAdapterWithoutThread:
    """Provides access to the robot's on-board GPS navigator (UBLOX card)"""

    def __init__(self, ser_port: str, ser_baudrate: int, last_pos_count: int):
        self._serial = serial.Serial(port=ser_port, baudrate=ser_baudrate)
        self.__logger = LoggerFactory.create(self.__class__.__name__)

    def __enter__(self):
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any):
        self.close()

    def __del__(self):
        self.close()

    def close(self):
        if self._serial.is_open:
            self._serial.close()

    def disconnect(self):
        """Obsolete method, stills here for backward compatibility, use close() instead"""

        self.close()

    def get_fresh_position(self):
        """Waits for new fresh position from gps and returns it, blocking until new position received.
        Returns copy of stored position (returned value can be safely changed with no worrying about obj reference
        features)"""

        return self._read_from_gps()

    def get_last_position(self):
        """Waits until at least one position is stored, returns last saved position copy at the moment of call
        (reference type safe)"""

        return self._read_from_gps()

    def get_last_positions_list(self): # type: ignore
        """Waits until at least one position is stored, returns list of last saved positions copies at the moment of
        call (reference type safe)"""

        raise NotImplementedError(f"Test without list")

    def _read_from_gps(self) -> List[Union[float, str]]:
        """Returns GPS coordinates of the current position"""

        while True:
            try:
                read_line = self._serial.readline()
                if isinstance(read_line, bytes):
                    data = str(read_line)
                    if "GNGGA" in data and ",,," not in data:
                        data = data.split(",")
                        lati, longi = self._D2M2(data[2], data[3], data[4], data[5])
                        point_quality = data[6]
                        return [lati, longi, point_quality]  # , float(data[11])  # alti
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                continue

    def _D2M2(self, Lat: str, NS: str, Lon: str, EW: str):
        """Traduce NMEA format ddmmss to ddmmmm"""

        Latdd = float(Lat[:2])
        Latmmmmm = float(Lat[2:])
        Latddmmmm = Latdd + (Latmmmmm / 60.0)
        if NS == 'S':
            Latddmmmm = -Latddmmmm

        Londd = float(Lon[:3])
        Lonmmmmm = float(Lon[3:])
        Londdmmmm = Londd + (Lonmmmmm / 60.0)
        if EW == 'W':
            Londdmmmm = -Londdmmmm
        return round(Latddmmmm, 7), round(Londdmmmm, 7)

    def _USBNMEA_OUT(self):
        """Start sending NMEA out on USB port at 19200 baud"""

        Matrame = "B5 62 06 00 14 00 03 00 00 00 00 00 00 00 00 00 00 00 23 00 03 00 00 00 00 00 43 AE"
        self._serial.write(bytearray.fromhex(Matrame))

    # Start a Hot restart
    def _hot_reset(self):
        Mythread = "B5 62 06 04 04 00 00 00 02 00 10 68"
        self._serial.write(bytearray.fromhex(Mythread))


class ClientMVI:
    """Provides access to the robot's on-board MVI (Machine Vision Interface)
    Attributes:
        OVERHEAD_DETECTION: MVIPipelineDescriptor
        TARGET_FINDER_DETECTION: MVIPipelineDescriptor
        RECONNECT_ATTEMPTS: int
        RECONNECT_DELAY_SEC: float
    """
    OVERHEAD_DETECTION = MVIPipelineDescriptor.OVERHEAD_DETECTION
    TARGET_FINDER_DETECTION = MVIPipelineDescriptor.TARGET_FINDER_DETECTION
    RECONNECT_ATTEMPTS = 3
    RECONNECT_DELAY_SEC = 1.0

    def __init__(self,
                 host: str,
                 port: int):
        """Initializes the ClientMVI instance with the specified host and port for MVI communication.
        Arguments:
            host: str - The hostname or IP address of the MVI server.
            port: int - The port number of the MVI server.
        """
        self.__logger = LoggerFactory.create(self.__class__.__name__)
        self.__host = host
        self.__port = port
        self.__sync_locker = threading.RLock()
        self.__released = False
        self.__current_MVI_pipeline_desciptor = None
        self.__current_MVI_state = None
        self.__id_name_map: dict[MVIPipelineDescriptor,list[str]] = dict()

        self.__client = self.__create_client()
        self.__connect_client()
        
        self.__logger.info(f"Switching to overhead detection pipeline.")
        self.switch_active_pipeline(self.OVERHEAD_DETECTION)
        
    def __enter__(self):
        """Enters the context manager for the ClientMVI instance, allowing it to be used with a 'with' statement."""
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any):
        """Exits the context manager for the ClientMVI instance, ensuring proper cleanup."""
        self.__release()

    def __release(self):
        """Releases resources associated with the ClientMVI instance, including disconnecting from the MVI server."""
        with self.__sync_locker:
            if self.__released:
                return
            self.__released = True
            self.__disconnect_client_silently()

    def __create_client(self):
        """Creates and configures a new Client instance for MVI communication."""
        client = Client(transport=config.MVI_TRANSPORT_PROTOCOL, logger_level="WARNING")
        client.register_message_type(MVICustomResultType.DETECTION_RESULT, DetectionResult) # type: ignore
        # client.register_message_type(MVICustomResultType.NAMES_RESULT, DetectionResult)
        return client

    def __connect_client(self):
        """Connects the Client instance to the MVI server using the specified host and port."""
        self.__logger.info(f"Connecting to MVI server at {self.__host}:{self.__port}.")
        self.__client.connect(self.__host, self.__port) # type: ignore

    def __disconnect_client_silently(self):
        """Disconnects the Client instance from the MVI server, handling any exceptions that may occur during disconnection."""
        try:
            self.__client.disconnect()
        except Exception as ex:
            self.__logger.warning(f"Error while disconnecting MVI client: {ex}")

    def __ensure_open(self):
        """Ensures that the ClientMVI instance is open and not released.
        Raises:
            RuntimeError: If the ClientMVI instance has already been released.
        """
        if self.__released:
            raise RuntimeError("MVI client is already released")

    def __ensure_response(self, res: Optional[ResultMessage]):
        """Ensures that the response from the MVI server is valid.
        Arguments:
            res: The response object from the MVI server.
        Raises:
            ConnectionError: If the response is None or does not have a 'message' attribute.
        """
        if not hasattr(res, "message"):
            raise ConnectionError("MVI did not return a valid response")
        return res

    def __restore_session_state(self):
        """Restores the session state of the MVI client after a reconnection. If there was a previously active pipeline descriptor or state, it switches back to that pipeline and sets the state accordingly.
        Raises:
            RuntimeError: If the MVI operation to switch pipelines or set state fails.
        """
        if self.__current_MVI_pipeline_desciptor is not None:
            self.switch_active_pipeline(self.__current_MVI_pipeline_desciptor)
        if self.__current_MVI_state is not None:
            self.__set_state_on_mvi(self.__current_MVI_state)

    def __reconnect(self):
        """Reconnects the MVI client to the server, attempting to restore the previous session state.
        Raises:
            RuntimeError: If the MVI client fails to reconnect after the specified number of attempts.
        """
        self.__logger.warning("MVI connection lost, trying to reconnect.")
        self.__disconnect_client_silently()
        self.__client = self.__create_client()
        self.__connect_client()
        self.__restore_session_state()
        self.__logger.info("MVI client reconnected.")

    def __call_mvi(self, call_type: CallType, payload: Dict[str, Any], result_type: Type[ResultType]) -> ResultType:
        """Calls the MVI server with the specified call type and payload, handling reconnection attempts if necessary.
        Arguments:
            call_type: CallType - The type of MVI call (e.g., GET, SET).
            payload: dict - The payload to send with the MVI call.
        Returns:
            ResultDTO - The result of the MVI call.
        Raises:
            RuntimeError: If the MVI call fails after the specified number of reconnection attempts.
        """
        
        _ = result_type # to avoid unused variable warning, as result_type is not used in this method but is part of the signature for type checking
        
        with self.__sync_locker:
            self.__ensure_open()
            last_error = None
            for attempt in range(self.RECONNECT_ATTEMPTS + 1):
                try:
                    res = self.__client.call(call_type, payload) # type: ignore
                    self.__check_result(res)
                    return cast(ResultType, res)
                except Exception as ex:
                    last_error = ex
                    

                if attempt >= self.RECONNECT_ATTEMPTS:
                    break

                self.__logger.warning(
                    f"MVI call failed ({last_error}). "
                    f"Reconnect attempt {attempt + 1}/{self.RECONNECT_ATTEMPTS}."
                )
                try:
                    self.__reconnect()
                except Exception as reconnect_ex:
                    last_error = reconnect_ex
                    self.__logger.error(f"MVI reconnect failed: {reconnect_ex}")
                    time.sleep(self.RECONNECT_DELAY_SEC)

            msg = f"MVI call failed after reconnect attempts: {last_error}"
            self.__logger.error(msg)
            raise RuntimeError(msg) from last_error
        
    def __check_result(self, res: Optional[ResultMessage]):
        """Checks the result of an MVI operation and raises a RuntimeError if the operation was not successful.
        Arguments:
            res: ResultDTO - The result of an MVI operation.
        Raises:
            RuntimeError: If the MVI operation was not successful, with details about the error code and message.
        """
        self.__ensure_response(res)        
        message: Dict[str, Any] = res.message # type: ignore
        if message["code"] != 0:
            msg = f"MVI error code: {message['code']}, message: {message['message']}"
            self.__logger.error(msg)
            raise RuntimeError(msg)
        
    def __check_result_dto_with_payload(self, result: ResultDTO) -> str:
        self.__logger.debug(f"Result : {result}, type: {type(result)}")
        if result.get("payload") is None: 
            raise RuntimeError(f"Expected 'payload' in result, got {result}")
        if not isinstance(result["payload"], str):
            raise RuntimeError(f"Expected 'payload' to be a string, got {type(result['payload'])}")
        return result["payload"]

    def __set_state_on_mvi(self, new_state: MVIState) -> None:
        """Sets the state of the MVI server to the specified new state.
        Arguments:
            new_state: MVIState - The new state to set on the MVI server.
        Raises:
            RuntimeError: If the MVI operation to set the state was not successful.
        """
        result = self.__call_mvi(
            CallType.SET,
            {
                "property": MVIProperty.STATE.value,
                "value": new_state.name
            },
            ResultDTO
        )
        self.__logger.info(f"Set MVI state to result: {result}")
        self.__current_MVI_state = new_state
        
    def __get_id_name_map(self) -> List[str]:
        """Retrieves the mapping of active pipeline IDs to their corresponding names from the MVI server.
        Returns:
            list[str]: A list of names corresponding to objects can be detected by pipeline.
        Raises:
            RuntimeError: If the MVI operation to retrieve the ID-name map was not successful.
        """
        result = self.__call_mvi(
            CallType.GET,
            {
                "property": MVIProperty.ID_NAME_MAP_OF_ACTIVE_PIPELINE.value
            },
            ResultDTO
        )
        
        result_dto_payload = self.__check_result_dto_with_payload(result)
        
        return json.loads(result_dto_payload)
        
    def get_name_map(self, pipeline: MVIPipelineDescriptor) -> List[str]:
        """Retrieves the mapping of active pipeline IDs to their corresponding names for the specified pipeline descriptor.
        Arguments:
            pipeline: MVIPipelineDescriptor - The pipeline descriptor for which to retrieve the ID-name map.
        """
        if pipeline not in self.__id_name_map:
            self.__id_name_map[pipeline] = self.__get_id_name_map()
        return self.__id_name_map[pipeline]
        
    def get_last_detections(self) -> DetectionResultDTO:
        """Retrieves the latest detection results from the MVI server.
        Returns:
            DetectionResultDTO: The latest detection results from the MVI server.
        Raises:
            RuntimeError: If the MVI operation to retrieve the latest detections was not successful.
        """
        result = self.__call_mvi(
            CallType.GET,
            {
                "property": MVIProperty.LATEST_DETECTIONS.value,
                "result_type": MVICustomResultType.DETECTION_RESULT.value,
            },
            DetectionResultDTO
        )
        return result

    def parse_detected_boxes(self, detection_result: DetectionResultDTO) -> List[DetectedPlantBox]:
        """Parse detection results into DetectedPlantBox instances."""

        pipeline_descriptor = self.__current_MVI_pipeline_desciptor

        if pipeline_descriptor is None:
            raise RuntimeError(
                "Cannot parse detected boxes: no current MVI pipeline descriptor is set."
            )

        id_name_map = self.__id_name_map[pipeline_descriptor]

        plants_boxes: List[DetectedPlantBox] = []

        for detection in detection_result["detections"]:
            plants_boxes.append(
                DetectedPlantBox.from_mvi_result(
                    detection,
                    id_name_map,
                )
            )

        return plants_boxes
    

    def parse_plants_positions(self, detection_result: DetectionResultDTO) -> List[Tuple[float, float]]:
        """Return the detected plant positions as (x, y) tuples."""

        smoothie_positions: List[Tuple[float, float]] = []

        for detection in detection_result["detections"]:
            if "keypoint" not in detection:
                raise RuntimeError(
                    "Expected 'keypoint' in detection, got {}".format(
                        detection
                    )
                )
                
            keypoint = cast(KeypointDTO, detection["keypoint"])

            smoothie_positions.append(
                (
                    float(keypoint["x"]),
                    float(keypoint["y"]),
                )
            )

        return smoothie_positions
    
    def violette_is_stopped(self) -> bool:
        """Checks if the MVI is in passive detection mode, indicating that it is stopped.
        Returns:
            bool: True if the MVI is in passive detection mode, False otherwise.
        Raises:
            RuntimeError: If the MVI operation to check the state was not successful.
        """
        result = self.__call_mvi(
            CallType.GET,
            {
                "property": MVIProperty.STATE.value,
            },
            ResultDTO
        )
        
        result_dto_payload = self.__check_result_dto_with_payload(result)
        
        return MVIState(json.loads(result_dto_payload)) == MVIState.PASSIVE_DETECTION
    
    def run_active_detection_on_MVI(self) -> None:
        """Sets the MVI to active detection mode, allowing it to actively detect objects and stop the robot.
        Raises:
            RuntimeError: If the MVI operation to set the state was not successful.
        """
        self.__set_state_on_mvi(MVIState.ACTIVE_DETECTION)
        
    def run_passive_detection_on_MVI(self) -> None:
        """Sets the MVI to passive detection mode, indicating that it is stopped and not actively detecting objects.
        Raises:
            RuntimeError: If the MVI operation to set the state was not successful.
        """
        self.__set_state_on_mvi(MVIState.PASSIVE_DETECTION)
        
    def switch_active_pipeline(self, new_pipeline: MVIPipelineDescriptor) -> None:
        """Switches the active MVI pipeline to the specified new pipeline descriptor.
        Arguments:
            new_pipeline: MVIPipelineDescriptor - The new pipeline descriptor to switch to.
        Raises:
            RuntimeError: If the MVI operation to switch the active pipeline was not successful.
        """
        self.__call_mvi(
            CallType.SET,
            {
                "property": MVIProperty.ACTIVE_PIPELINE,
                "value": new_pipeline.without_destroying_value,
            },
            ResultDTO
        )
        self.__current_MVI_pipeline_desciptor = new_pipeline
        
        if new_pipeline not in self.__id_name_map:
            self.__id_name_map[new_pipeline] = self.__get_id_name_map()
