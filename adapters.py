import connectors
import multiprocessing
import time
import navigation
import utility
from config import config
import cv2 as cv
import math
import queue
import threading
import serial
import pyvesc
import re
#import RPi.GPIO as GPIO
from serial import SerialException


class SmoothieAdapter:
    RESPONSE_OK = "ok\r\n"
    RESPONSE_ALARM_LOCK = "error:Alarm lock\n"
    RESPONSE_HALT = "!!\r\n"
    RESPONSE_IGNORED = "ok - ignored\n"
    RESPONSE_HOMING_FAILED = "ERROR: Homing cycle failed - check the max_travel settings"
    RESPONSE_AFTER_M999 = "WARNING: After HALT you should HOME as position is currently unknown"

    def __init__(self, smoothie_host, calibration_at_init=True):
        if type(smoothie_host) is not str:
            raise TypeError(f"[{self.__class__.__name__}] -> invalid smoothie_host type: should be str, received " + type(smoothie_host).__name__)

        if config.SMOOTHIE_BACKEND == 1:
            self.__smc = connectors.SmoothieV11TelnetConnector(smoothie_host)
        elif config.SMOOTHIE_BACKEND == 2:
            self.__smc = connectors.SmoothieV11SerialConnector(smoothie_host, config.SMOOTHIE_BAUDRATE)
        else:
            raise ValueError(f"[{self.__class__.__name__}] -> wrong config.SMOOTHIE_BACKEND value: " + str(smoothie_host))

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
                msg = f"[{self.__class__.__name__}] -> Attempt {i + 1} of switching smoothie to relative is failed, smoothie response:\n{res}"
                print(msg)
            else:
                if(res == self.RESPONSE_OK):
                    print(f"[{self.__class__.__name__}] -> The Smoothie switched to relative mode without detecting any bugs. The response was: {res}")
                else:
                    print(f"[{self.__class__.__name__}] -> A bug was detected during the Smoothie's switch to relative mode, but it was handled by the bug fix. The response was: {res}")
                break
        else:
            msg = f"[{self.__class__.__name__}] -> All attempts of switching smoothie to relative were failed! Last smoothie's response:\n{res}"
            print(msg)
            raise Exception(msg)
        #> Code patché rapidement pour la démo

        if config.SEEDER_QUANTITY > 0:
            self.seeder_close()
            res = self.seeder_close()
            if SmoothieAdapter.check_res_smoothie(res):
                msg = f"[{self.__class__.__name__}] -> Couldn't lock seeder during smoothie adapter initialization! Smoothie response: {res}"
                print(msg)

        if calibration_at_init:
            # TODO: temporary crutch - vesc is moving Z upward before smoothie loads, so we need to lower the cork a bit down
            res = self.custom_move_for(Z_F=config.Z_F_EXTRACTION_DOWN, Z=5)
            self.wait_for_all_actions_done()
            if SmoothieAdapter.check_res_smoothie(res):
                print("Couldn't move cork down for Z5! Calibration errors on Z axis are possible!")

            res = self.ext_calibrate_cork()
            if SmoothieAdapter.check_res_smoothie(res):
                print("Initial cork calibration was failed, smoothie response:\n", res)  # TODO: what if so??
                raise Exception("Initial cork calibration was failed!")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.__smc.disconnect()

    def disconnect(self):
        self.__smc.disconnect()
        
    @staticmethod
    def check_res_smoothie(res):
        return (("!" in res) or ("error" in res) or ("ERROR" in res) or ("WARNING" in res) or ("ignored" in res))

    @property
    def is_disconnect(self):
        return self._smc.is_open

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
            return self.__smc.read_some() + self.__smc.read_some() if self.__smc is connectors.SmoothieV11TelnetConnector else self.__smc.read_some()

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

    def checkendstop(self, axe):
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

    def set_current_coordinates(self, X=None, Y=None, Z=None, A=None, B=None, C=None):
        with self.__sync_locker:
            if self.__check_arg_types([type(None)], X, Y, Z, A, B, C):
                raise TypeError(f"[{self.__class__.__name__}] -> at least one axis shouldn't be None")
            if not self.__check_arg_types([float, int, type(None)], X, Y, Z, A, B, C):
                raise TypeError(f"[{self.__class__.__name__}] -> incorrect axis current value(s) type(s)")

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

    def get_smoothie_current_coordinates(self, convert_to_mms=True):
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
            for coord in response:
                coordinates[coord[0]] = float(coord[2:])
                if convert_to_mms:
                    coordinates[coord[0]] = self.smoothie_to_mm(coordinates[coord[0]], coord[0])
            return coordinates

    @classmethod
    def compare_coordinates(cls, coordinates_a, coordinates_b, precision=1e-10):
        if type(coordinates_a) != dict or type(coordinates_b) != dict:
            raise AttributeError(f"[{cls.__name__}] -> coordinates should be stored in dict")
        if len(coordinates_a) != len(coordinates_b):
            raise AttributeError(f"[{cls.__name__}] -> coordinates dicts should have similar items count")

        for key in coordinates_a:
            if abs(coordinates_a[key] - coordinates_b[key]) > precision:
                return False
        return True

    def custom_move_for(self, *,
                        X_F=None,
                        Y_F=None,
                        Z_F=None,
                        A_F=None,
                        B_F=None,
                        C_F=None,
                        X=None,
                        Y=None,
                        Z=None,
                        A=None,
                        B=None,
                        C=None):
        """Movement by some value(s)

        Minimal force is applied if multiple values are given
        """

        with self.__sync_locker:
            # check given forces
            if self.__check_arg_types([type(None)], X_F, Y_F, Z_F, A_F, B_F, C_F):
                raise TypeError(f"[{self.__class__.__name__}] -> at least one given force value shouldn't be a None")
            if not self.__check_arg_types([float, int, type(None)], X_F, Y_F, Z_F, A_F, B_F, C_F):
                raise TypeError(f"[{self.__class__.__name__}] -> incorrect force value(s) type(s)")

            # check given axes
            if self.__check_arg_types([type(None)], X, Y, Z, A, B, C):
                raise TypeError(f"[{self.__class__.__name__}] -> at least one given axis value shouldn't be a None")
            if not self.__check_arg_types([float, int, type(None)], X, Y, Z, A, B, C):
                raise TypeError(f"[{self.__class__.__name__}] -> incorrect axis value(s) type(s)")

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
                       X_F=None,
                       Y_F=None,
                       Z_F=None,
                       A_F=None,
                       B_F=None,
                       C_F=None,
                       X=None,
                       Y=None,
                       Z=None,
                       A=None,
                       B=None,
                       C=None):
        """Movement to the specified position"""

        with self.__sync_locker:
            # check given forces
            if self.__check_arg_types([type(None)], X_F, Y_F, Z_F, A_F, B_F, C_F):
                raise TypeError(f"[{self.__class__.__name__}] -> at least one given force value shouldn't be a None")
            if not self.__check_arg_types([float, int, type(None)], X_F, Y_F, Z_F, A_F, B_F, C_F):
                raise TypeError(f"[{self.__class__.__name__}] -> incorrect force value(s) type(s)")

            # check given axes
            if self.__check_arg_types([type(None)], X, Y, Z, A, B, C):
                raise TypeError(f"[{self.__class__.__name__}] -> at least one given axis value shouldn't be a None")
            if not self.__check_arg_types([float, int, type(None)], X, Y, Z, A, B, C):
                raise TypeError(f"[{self.__class__.__name__}] -> incorrect axis value(s) type(s)")

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
                                    X_F=None,
                                    Y_F=None,
                                    X=None,
                                    Y=None):
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
                        err_msg = f"[{self.__class__.__name__}] -> Couldn't do separate X movement:\n" + res
                        return err_msg
                    # Y movement
                    res = self.custom_move_for(Y_F=Y_F, Y=Y)
                    if SmoothieAdapter.check_res_smoothie(res):
                        err_msg = f"[{self.__class__.__name__}] -> Couldn't do separate Y movement:\n" + res
                        return err_msg
                    return res
            return self.custom_move_for(X_F=X_F, Y_F=Y_F, X=X, Y=Y)

    def custom_separate_xy_move_to(self, *,
                                   X_F=None,
                                   Y_F=None,
                                   X=None,
                                   Y=None):
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
                        err_msg = f"[{self.__class__.__name__}] -> Couldn't do separate X movement:\n" + res
                        return err_msg
                    # Y movement
                    res = self.custom_move_to(Y_F=Y_F, Y=Y)
                    if SmoothieAdapter.check_res_smoothie(res):
                        err_msg = f"[{self.__class__.__name__}] -> Couldn't do separate Y movement:\n" + res
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

    def ext_calibrate_cork(self):

        if not set(config.CALIBRATION_ORDER).issubset(set(["X", "Y", "Z", "A", "B", "C"])):
            raise ValueError(f"[{self.__class__.__name__}] -> unsupported axis label or wrong type")

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

    def ext_cork_up(self):
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
                    msg = f"[{self.__class__.__name__}] -> Homing failed during cork up, retry with Z{i} down before up."
                    print(msg)
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
                    if self.RESPONSE_HOMING_FAILED in response:
                        continue
                    else:
                        break
            else:
                return response

        else:
            raise RuntimeError(
                f"[{self.__class__.__name__}] -> picking up corkscrew with stoppers usage requires Z axis calibration permission in config"
            )

    @classmethod
    def mm_to_smoothie(cls, mm_axis_val, axis_label: str):
        """Converts given mms value to smoothie value applying (multiplying) coefficient corresponding to given axis
        label

        Example: config coefficient = 0.5, given mms value = 100, returned smoothie value = 50
        """

        if axis_label not in ["X", "Y", "Z", "A", "B", "C"]:
            raise ValueError(f"[{cls.__name__}] -> unsupported axis label or wrong type")
        if not SmoothieAdapter.__check_arg_types([int, float], mm_axis_val):
            raise TypeError(f"[{cls.__name__}] -> axis_value should be float or int")

        if mm_axis_val == 0:
            return mm_axis_val

        if axis_label == "X":
            return mm_axis_val * config.X_COEFFICIENT_TO_MM
        if axis_label == "Y":
            return mm_axis_val * config.Y_COEFFICIENT_TO_MM
        if axis_label == "Z":
            return mm_axis_val * config.Z_COEFFICIENT_TO_MM
        if axis_label == "A":
            return mm_axis_val * config.A_COEFFICIENT_TO_MM
        if axis_label == "B":
            return mm_axis_val * config.B_COEFFICIENT_TO_MM
        if axis_label == "C":
            return mm_axis_val * config.C_COEFFICIENT_TO_MM

    @classmethod
    def smoothie_to_mm(cls, sm_axis_val, axis_label: str):
        """Converts given smoothie value to mms value applying (dividing) coefficient corresponding to given axis
        label

        Example: coefficient = 0.5, given smoothie value = 50, returned mms value = 100
        """

        if axis_label not in ["X", "Y", "Z", "A", "B", "C"]:
            raise ValueError(f"[{cls.__name__}] -> unsupported axis label or wrong type")
        if not SmoothieAdapter.__check_arg_types([int, float], sm_axis_val):
            raise TypeError(f"[{cls.__name__}] -> axis_value should be float or int")

        if sm_axis_val == 0:
            return sm_axis_val

        if axis_label == "X":
            if config.X_COEFFICIENT_TO_MM == 0:
                raise ValueError(f"[{cls.__name__}] -> config.X_COEFFICIENT_TO_MM can't be a zero")
            return sm_axis_val / config.X_COEFFICIENT_TO_MM

        if axis_label == "Y":
            if config.Y_COEFFICIENT_TO_MM == 0:
                raise ValueError(f"[{cls.__name__}] -> config.Y_COEFFICIENT_TO_MM can't be a zero")
            return sm_axis_val / config.Y_COEFFICIENT_TO_MM

        if axis_label == "Z":
            if config.Z_COEFFICIENT_TO_MM == 0:
                raise ValueError(f"[{cls.__name__}] -> config.Z_COEFFICIENT_TO_MM can't be a zero")
            return sm_axis_val / config.Z_COEFFICIENT_TO_MM

        if axis_label == "A":
            if config.A_COEFFICIENT_TO_MM == 0:
                raise ValueError(f"[{cls.__name__}] -> config.A_COEFFICIENT_TO_MM can't be a zero")
            return sm_axis_val / config.A_COEFFICIENT_TO_MM

        if axis_label == "B":
            if config.B_COEFFICIENT_TO_MM == 0:
                raise ValueError(f"[{cls.__name__}] -> config.B_COEFFICIENT_TO_MM can't be a zero")
            return sm_axis_val / config.B_COEFFICIENT_TO_MM

        if axis_label == "C":
            if config.C_COEFFICIENT_TO_MM == 0:
                raise ValueError(f"[{cls.__name__}] -> config.C_COEFFICIENT_TO_MM can't be a zero")
            return sm_axis_val / config.C_COEFFICIENT_TO_MM

    @classmethod
    def __check_arg_types(cls, types: list, *args):
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
    def __validate_axis(cls, cur_axis_val, mov_axis_val, key_label, key_min, key_max, key_min_label, key_max_label):
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
    def __validate_force(cls, value, key_label, key_min, key_max, key_min_label, key_max_label):
        """Checks if given force can be applied. Returns None if value is ok, info/error message otherwise.
        """

        if value > key_max:
            return f"[{cls.__name__}] -> Value {value} for {key_label} goes beyond max acceptable range of {key_max_label} = {key_max}"
        if value < key_min:
            return f"[{cls.__name__}] -> Value {value} for {key_label} goes beyond min acceptable range of {key_min_label} = {key_min}"
        return None

    def __calibrate_axis(self,
                         axis_cur: multiprocessing.Value,
                         axis_label,
                         sm_axis_min,
                         sm_axis_max,
                         axis_calibration_to_max):
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


class PiCameraAdapter:

    def __init__(self):
        from picamera.array import PiRGBArray
        from picamera import PiCamera
        self._camera = PiCamera()
        self._camera.resolution = (config.CAMERA_W, config.CAMERA_H)
        self._camera.framerate = config.CAMERA_FRAMERATE
        self._raw_capture = PiRGBArray(self._camera, size=(config.CAMERA_W, config.CAMERA_H))
        self._gen = self._camera.capture_continuous(self._raw_capture, format="rgb")
        time.sleep(2)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()

    def release(self):
        self._camera.close()

    def get_image(self):
        image = cv.cvtColor(next(self._gen).array, cv.COLOR_RGB2BGR)
        self._raw_capture.truncate(0)
        return image


'''
# test
class CameraAdapterIMX219_170_BS1:
    """Buffer size is set to 1 frame, getting 2 frames per call, return last"""

    def __init__(self,
                 capture_width=config.CAMERA_W,
                 capture_height=config.CAMERA_H,
                 display_width=config.CAMERA_W,
                 display_height=config.CAMERA_H,
                 framerate=config.CAMERA_FRAMERATE,
                 flip_method=config.CAMERA_FLIP_METHOD):

        gst_config = (
                "nvarguscamerasrc ! "
                "video/x-raw(memory:NVMM), "
                "width=(int)%d, height=(int)%d, "
                "format=(string)NV12, framerate=(fraction)%d/1 ! "
                "nvvidconv flip-method=%d ! "
                "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
                "videoconvert ! "
                "video/x-raw, format=(string)BGR ! appsink"
                % (
                    capture_width,
                    capture_height,
                    framerate,
                    flip_method,
                    display_width,
                    display_height
                )
        )
        self._cap = cv.VideoCapture(gst_config, cv.CAP_GSTREAMER)
        self._cap = cv.VideoCapture(cv.CAP_PROP_BUFFERSIZE, 1)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()

    def release(self):
        self._cap.release()

    def get_image(self):
        if self._cap.isOpened():
            for i in range(self._cap.get(cv.CAP_PROP_BUFFERSIZE) + 1):
                ret, image = self._cap.read()
            # rotate for 90 degrees and crop black zones
            return cv.rotate(image, 2)[config.CROP_H_FROM:config.CROP_H_TO, config.CROP_W_FROM:config.CROP_W_TO]
        else:
            raise RuntimeError("Unable to open camera")
'''


# old with no shutter, gain and rest camera control
class CameraAdapterIMX219_170_Auto:

    def __init__(self,
                 crop_w_from,
                 crop_w_to,
                 crop_h_from,
                 crop_h_to,
                 cv_rotate_code,
                 ispdigitalgainrange_from,
                 ispdigitalgainrange_to,
                 gainrange_from,
                 gainrange_to,
                 exposuretimerange_from,
                 exposuretimerange_to,
                 aelock,
                 capture_width,
                 capture_height,
                 display_width,
                 display_height,
                 framerate,
                 nvidia_flip_method):

        self._crop_w_from = crop_w_from
        self._crop_w_to = crop_w_to
        self._crop_h_from = crop_h_from
        self._crop_h_to = crop_h_to
        self._cv_rotate_code = cv_rotate_code
        aelock = "aelock=true " if aelock else ""

        gst_config = (
                "nvarguscamerasrc ! "
                "video/x-raw(memory:NVMM), "
                "width=(int)%d, height=(int)%d, "
                "format=(string)NV12, framerate=(fraction)%d/1 ! "
                "nvvidconv flip-method=%d ! "
                "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
                "videoconvert ! "
                "video/x-raw, format=(string)BGR ! appsink"
                % (
                    capture_width,
                    capture_height,
                    framerate,
                    nvidia_flip_method,
                    display_width,
                    display_height
                )
        )

        if config.APPLY_THREAD_BUFF_CLEANING:
            self._cap = VideoCaptureNoBuffer(gst_config, cv.CAP_GSTREAMER)
        else:
            self._cap = cv.VideoCapture(gst_config, cv.CAP_GSTREAMER)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()

    def release(self):
        self._cap.release()

    def get_image(self):
        if self._cap.isOpened():
            if config.BUFF_CLEANING_DELAY > 0:
                time.sleep(config.BUFF_CLEANING_DELAY)

            if config.APPLY_THREAD_BUFF_CLEANING:
                image = self._cap.read()
            else:
                ret, image = self._cap.read()

            if config.CV_APPLY_ROTATION:
                image = cv.rotate(image, self._cv_rotate_code)

            # crop black zones
            #if config.APPLY_IMAGE_CROPPING:
            #    image = image[self._crop_h_from:self._crop_h_to, self._crop_w_from:self._crop_w_to]
            return image
        else:
            raise RuntimeError("Unable to open camera")


class CameraAdapterIMX219_170:

    def __init__(self,
                 crop_w_from,
                 crop_w_to,
                 crop_h_from,
                 crop_h_to,
                 cv_rotate_code,
                 ispdigitalgainrange_from,
                 ispdigitalgainrange_to,
                 gainrange_from,
                 gainrange_to,
                 exposuretimerange_from,
                 exposuretimerange_to,
                 aelock,
                 capture_width,
                 capture_height,
                 display_width,
                 display_height,
                 framerate,
                 nvidia_flip_method):

        self._crop_w_from = crop_w_from
        self._crop_w_to = crop_w_to
        self._crop_h_from = crop_h_from
        self._crop_h_to = crop_h_to
        self._cv_rotate_code = cv_rotate_code
        aelock = "aelock=true " if aelock else ""
        # ispdigitalgainrange="14.72 14.72" gainrange="14.72 14.72" exposuretimerange="55000 55000" aelock=true
        if config.APPLY_IMAGE_CROPPING:
            gst_config = (
                    "nvarguscamerasrc "
                    "ispdigitalgainrange=\"%.2f %.2f\" "
                    "gainrange=\"%.2f %.2f\" "
                    "exposuretimerange=\"%d %d\" "
                    "%s"
                    "! "
                    "video/x-raw(memory:NVMM), "
                    "width=(int)%d, height=(int)%d, "
                    "format=(string)NV12, framerate=(fraction)%d/1 ! "
                    "nvvidconv top=%d bottom=%d left=%d right=%d flip-method=%d ! "
                    "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
                    "videoconvert ! "
                    "video/x-raw, format=(string)BGR ! appsink"
                    % (
                        ispdigitalgainrange_from,
                        ispdigitalgainrange_to,
                        gainrange_from,
                        gainrange_to,
                        exposuretimerange_from,
                        exposuretimerange_to,
                        aelock,
                        capture_width,
                        capture_height,
                        framerate,
                        crop_h_from,
                        crop_h_to,
                        crop_w_from,
                        crop_w_to,
                        nvidia_flip_method,
                        crop_w_to-crop_w_from,
                        crop_h_to-crop_h_from
                    )
            )
        else:
            gst_config = (
                    "nvarguscamerasrc "
                    "ispdigitalgainrange=\"%.2f %.2f\" "
                    "gainrange=\"%.2f %.2f\" "
                    "exposuretimerange=\"%d %d\" "
                    "%s"
                    "! "
                    "video/x-raw(memory:NVMM), "
                    "width=(int)%d, height=(int)%d, "
                    "format=(string)NV12, framerate=(fraction)%d/1 ! "
                    "nvvidconv flip-method=%d ! "
                    "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
                    "videoconvert ! "
                    "video/x-raw, format=(string)BGR ! appsink"
                    % (
                        ispdigitalgainrange_from,
                        ispdigitalgainrange_to,
                        gainrange_from,
                        gainrange_to,
                        exposuretimerange_from,
                        exposuretimerange_to,
                        aelock,
                        capture_width,
                        capture_height,
                        framerate,
                        nvidia_flip_method,
                        display_width,
                        display_height
                    )
            )

        if config.APPLY_THREAD_BUFF_CLEANING:
            self._cap = VideoCaptureNoBuffer(gst_config, cv.CAP_GSTREAMER)
        else:
            self._cap = cv.VideoCapture(gst_config, cv.CAP_GSTREAMER)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()

    def release(self):
        self._cap.release()

    def get_image(self):
        if self._cap.isOpened():
            if config.BUFF_CLEANING_DELAY > 0:
                time.sleep(config.BUFF_CLEANING_DELAY)

            if config.APPLY_THREAD_BUFF_CLEANING:
                image = self._cap.read()
            else:
                ret, image = self._cap.read()

            if config.CV_APPLY_ROTATION:
                image = cv.rotate(image, self._cv_rotate_code)

            # crop black zones
            #if config.APPLY_IMAGE_CROPPING:
            #    image = image[self._crop_h_from:self._crop_h_to, self._crop_w_from:self._crop_w_to]
            # image = cv.imread('test.jpg') #fake image for debug
            return image
        else:
            raise RuntimeError(f"[{self.__class__.__name__}] -> Unable to open camera")


class VideoCaptureNoBuffer:
    """Minimalistic layer for cv2's VideoCapture with buffer cleaning thread ()"""

    def __init__(self, *args):
        self._cap = cv.VideoCapture(*args)
        self._queue = queue.Queue()
        self._thread = threading.Thread(target=self._reader)
        self._thread.daemon = True
        self._thread.start()

    def release(self):
        self._cap.release()

    def isOpened(self):
        return self._cap.isOpened()

    # read frames as soon as they are available, keeping only most recent one
    def _reader(self):
        while True:
            ret, frame = self._cap.read()
            if not ret:
                break
            if not self._queue.empty():
                try:
                    self._queue.get_nowait()  # discard previous (unprocessed) frame
                except queue.Empty:
                    pass
            self._queue.put(frame)

    def read(self):
        return self._queue.get()


class CompassOldAdapter:
    """Provides to the robot's on-board compass (some old card, the first one, not sure about model)"""

    def __init__(self):
        import smbus
        self._register_a = 0  # Address of Configuration register A
        self._register_b = 0x01  # Address of configuration register B
        self._register_mode = 0x02  # Address of mode register
        self._x_axis_h = 0x03  # Address of X-axis MSB data register
        self._z_axis_h = 0x05  # Address of Z-axis MSB data register
        self._y_axis_h = 0x07  # Address of Y-axis MSB data register
        self._bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards

        # write to Configuration Register A
        self._bus.write_byte_data(config.COMPASS_DEVICE_ADDRESS, self._register_a, 0x70)
        # Write to Configuration Register B for gain
        self._bus.write_byte_data(config.COMPASS_DEVICE_ADDRESS, self._register_b, 0xa0)
        # Write to mode Register for selecting mode
        self._bus.write_byte_data(config.COMPASS_DEVICE_ADDRESS, self._register_mode, 0)

    def _read_raw_data(self, address):
        """Reads raw data from compass"""

        # Read raw 16-bit value
        high = self._bus.read_byte_data(config.COMPASS_DEVICE_ADDRESS, address)
        low = self._bus.read_byte_data(config.COMPASS_DEVICE_ADDRESS, address + 1)

        # concatenate higher and lower value
        value = ((high << 8) | low)

        # get signed value from module
        return value - 65536 if value > 32768 else value

    def get_heading_angle(self):
        """Returns current heading angle in degrees"""

        x = self._read_raw_data(self._x_axis_h)
        y = self._read_raw_data(self._y_axis_h)
        heading = math.atan2(y, x) + config.COMPASS_DECLINATION

        # Due to declination check for > 360 degree
        if heading > 2 * math.pi:
            heading -= 2 * math.pi
        # check for sign
        if heading < 0:
            heading += 2 * math.pi

        # convert into angle
        return int(heading * 180 / math.pi)


class CompassBNO055Adapter:
    """Provides access to the robot's on-board compass"""

    def __init__(self):
        import adafruit_bno055
        from busio import I2C
        import board

        self._i2c = I2C(board.SCL, board.SDA)
        self._sensor = adafruit_bno055.BNO055(self._i2c)
        # turn on "compass mode"
        self._sensor.mode = adafruit_bno055.COMPASS_MODE
        # sensor.mode = adafruit_bno055.M4G_MODE

    def get_euler_angle(self):
        return self._sensor.euler


class VescAdapter:
    """Provides navigation engines (forward/backward) control using vesc"""

    def __init__(self, rpm, moving_time, alive_freq, check_freq, ser_port, ser_baudrate):
        self.start_cycle_time = time.time()

        self._ser = serial.Serial(port=ser_port, baudrate=ser_baudrate)

        self._rpm = rpm
        self._moving_time = moving_time
        self._alive_freq = alive_freq
        self._check_freq = check_freq
        self._start_time = self._next_alive_time = None
        self._allow_movement = False
        self._keep_thread_alive = True
        self._last_stop_time = 0

        self._ser.flushInput()
        self._ser.flushOutput()
        self._movement_ctrl_th = threading.Thread(target=self._movement_ctrl_th_tf, daemon=True)
        self._movement_ctrl_th.start()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    @property
    def rpm(self):
        return self._rpm

    def disconnect(self):
        self._ser.write(pyvesc.encode(pyvesc.SetRPM(0)))
        self._keep_thread_alive = False
        self._ser.close()

    def _movement_ctrl_th_tf(self):
        """Target function of movement control thread. Responsive for navigation engines work (forward/backward)"""

        try:
            while self._keep_thread_alive:
                if self._allow_movement:
                    if time.time() - self._start_time > self._moving_time:
                        self._ser.write(pyvesc.encode(pyvesc.SetRPM(0)))
                        self._last_stop_time = time.time()
                        self._allow_movement = False
                        continue

                    if time.time() > self._next_alive_time:
                        self._next_alive_time = time.time() + self._alive_freq
                        self._ser.write(pyvesc.encode(pyvesc.SetAlive))
                time.sleep(self._check_freq)
        except serial.SerialException as ex:
            print(ex)

    def start_moving(self):
        self._start_time = self._next_alive_time = time.time()
        self._ser.write(pyvesc.encode(pyvesc.SetRPM(self._rpm)))
        self._allow_movement = True

    def stop_moving(self):
        self._allow_movement = False
        self._last_stop_time = time.time()
        self._ser.write(pyvesc.encode(pyvesc.SetRPM(0)))

    def stop_current(self):
        self._ser.write(pyvesc.encode(pyvesc.SetCurrent(0)))

    def wait_for_stop(self):
        while self._allow_movement:
            time.sleep(self._check_freq)

    def apply_rpm(self, rpm):
        if self._rpm != rpm:  # TODO: bug to fix: if rpm was set by set_rpm - it won't be applied on vesc
            self._rpm = rpm
            self._ser.write(pyvesc.encode(pyvesc.SetRPM(self._rpm)))

    def set_rpm(self, rpm):
        self._rpm = rpm

    def set_moving_time(self, moving_time):
        self._moving_time = moving_time

    def set_alive_freq(self, alive_freq):
        self._alive_freq = alive_freq

    def set_check_freq(self, check_freq):
        self._check_freq = check_freq

    def is_movement_allowed(self):
        return self._allow_movement

    def get_last_stop_time(self):
        return self._last_stop_time

    def get_last_start_time(self):
        return self._start_time

    def get_last_moving_time(self):
        """Returns last moving time if VESCs are not working at the moment;
        returns current working time if VESCs are working at the moment.
        """
        if self._start_time is None:
            return 0
        elif self._allow_movement or not self._last_stop_time:
            return time.time() - self._start_time
        else:
            return self._last_stop_time - self._start_time

    def get_sensors_data(self, report_field_names):
        self._ser.write(pyvesc.encode_request(pyvesc.GetValues))
        in_buf = b''
        while self._ser.in_waiting > 0:
            in_buf += self._ser.read(self._ser.in_waiting)

        if len(in_buf) == 0:
            return None
        response, consumed = pyvesc.decode(in_buf)
        if consumed == 0:
            return None

        if isinstance(response, pyvesc.GetValues):
            report_row = {}
            for field_name in report_field_names:
                report_row[field_name] = getattr(response, field_name)
            return report_row
        return None


class VescAdapterV3:
    """Provides multiple vesc control

    To add more vesc engines:
    1) Add unique vesc ID key for users as class static member below
    2) Add keys to config.py to make it possible to enable-disable your new vesc and set it's settings easily
    3) Add new vesc engine initialization code to 'INIT ALL ALLOWED VESCS HERE' section (use code there as example)
    """

    # unique vesc ID keys for users (it's a keys for vesc can IDs, not IDs themselves)
    PROPULSION_KEY = 0
    EXTRACTION_KEY = 1

    def __init__(self, ser_port, ser_baudrate, alive_freq, check_freq, stopper_check_freq):
        gpio_is_initialized = False
        self.__stopper_check_freq = stopper_check_freq
        self.__alive_freq = alive_freq
        self.__check_freq = check_freq
        self.__next_alive_time = time.time()

        self.__can_ids = dict()
        self.__rpm = dict()
        self.__time_to_move = dict()
        self.__start_time = dict()
        self.__is_moving = dict()
        self.__last_stop_time = dict()
        self.__stopper_signals = dict()
        self.__gpio_stoppers_pins = dict()

        self.__ser = serial.Serial(port=ser_port, baudrate=ser_baudrate)
        self.__ser.flushInput()
        self.__ser.flushOutput()

        # INIT ALL ALLOWED VESCS HERE
        # init PROPULSION vesc (currently it's parent vesc so it has no checkings for ID and has parent's ID=None)
        if config.VESC_ALLOW_PROPULSION:
            if config.VESC_PROPULSION_AUTODETECT_CAN_ID:
                raise NotImplementedError("can id detection is not confirmed to work fine")
            else:
                prop_can_id = config.VESC_PROPULSION_CAN_ID
            self.__can_ids[self.PROPULSION_KEY] = prop_can_id  # parent vesc has ID=None
            self.__rpm[self.PROPULSION_KEY] = 0
            self.__time_to_move[self.PROPULSION_KEY] = 0
            self.__start_time[self.PROPULSION_KEY] = None
            self.__is_moving[self.PROPULSION_KEY] = False
            self.__last_stop_time[self.PROPULSION_KEY] = None
            self.__stopper_signals[self.PROPULSION_KEY] = config.VESC_PROPULSION_STOP_SIGNAL
            self.__gpio_stoppers_pins[self.PROPULSION_KEY] = config.VESC_PROPULSION_STOPPER_PIN
            if self.__gpio_stoppers_pins[self.PROPULSION_KEY] is not None:
                if not gpio_is_initialized:
                    raise NotImplementedError("Gpio disabled due to non-compatible library issue and non-use")
                    #GPIO.setmode(GPIO.BOARD)
                    #gpio_is_initialized = True
                #GPIO.setup(self.__gpio_stoppers_pins[self.PROPULSION_KEY], GPIO.IN)

        # init EXTRACTION vesc
        if config.VESC_ALLOW_EXTRACTION:
            if config.VESC_EXTRACTION_AUTODETECT_CAN_ID:
                raise NotImplementedError("can id detection is not confirmed to work fine")
                # ext_can_id = self.get_unregistered_can_id()
            else:
                ext_can_id = config.VESC_EXTRACTION_CAN_ID
            if ext_can_id is not None:
                self.__can_ids[self.EXTRACTION_KEY] = ext_can_id
                self.__rpm[self.EXTRACTION_KEY] = 0
                self.__time_to_move[self.EXTRACTION_KEY] = 0
                self.__start_time[self.EXTRACTION_KEY] = None
                self.__is_moving[self.EXTRACTION_KEY] = False
                self.__last_stop_time[self.EXTRACTION_KEY] = None
                self.__stopper_signals[self.EXTRACTION_KEY] = config.VESC_EXTRACTION_STOP_SIGNAL
                self.__gpio_stoppers_pins[self.EXTRACTION_KEY] = config.VESC_EXTRACTION_STOPPER_PIN
                if self.__gpio_stoppers_pins[self.EXTRACTION_KEY] is not None:
                    if not gpio_is_initialized:
                        raise NotImplementedError("Gpio disabled due to non-compatible library issue and non-use")
                        #GPIO.setmode(GPIO.BOARD)
                        #gpio_is_initialized = True
                    #GPIO.setup(self.__gpio_stoppers_pins[self.EXTRACTION_KEY], GPIO.IN)
            else:
                # TODO what robot should do if initialization was failed?
                print("extraction vesc initialization fail: couldn't determine extraction vesc ID")
        # init any new vescs (add vesc init code here)
        # ...

        self.__keep_thread_alive = True
        self._movement_ctrl_th = threading.Thread(target=self._movement_ctrl_th_tf, daemon=True)
        self._movement_ctrl_th.start()

        #Incremental rpm control
        self.__current_rpm = dict()
        self.__future_rpm = dict()

        for engine_key in config.INCREMENTAL_ENGINE_KEY:
            self.__current_rpm[engine_key] = 0
            self.__future_rpm[engine_key] = 0

        self.__keep_thread_rpm_alive = True
        self._rpm_ctrl_th = threading.Thread(target=self.__rpm_ctrl_th_tf, daemon=True)
        self._rpm_ctrl_th.start()

        # DO ALL ALLOWED CALIBRATIONS HERE
        # propulsion vasc calibration
        if config.VESC_PROPULSION_CALIBRATE_AT_INIT:
            self.set_rpm(config.VESC_PROPULSION_CALIBRATION_RPM, self.PROPULSION_KEY)
            self.set_time_to_move(config.VESC_PROPULSION_CALIBRATION_MAX_TIME, self.PROPULSION_KEY)
            self.start_moving(self.PROPULSION_KEY)
            res = self.wait_for_stopper_hit(self.PROPULSION_KEY, config.VESC_PROPULSION_CALIBRATION_MAX_TIME)
            self.stop_moving(self.PROPULSION_KEY)
            if not res:
                # TODO what robot should do if calibration was failed (there was no stopper hit)?
                print("Stopped vesc PROPULSION engine calibration due timeout (stopper signal wasn't received!)")

        # extraction vesc calibration
        if config.VESC_EXTRACTION_CALIBRATE_AT_INIT:
            # Z-5 fix (move cork little down to stop touching stopper)
            self.set_rpm(config.VESC_EXTRACTION_CALIBRATION_Z5_FIX_RPM, self.EXTRACTION_KEY)
            self.set_time_to_move(config.VESC_EXTRACTION_CALIBRATION_Z5_FIX_TIME, self.EXTRACTION_KEY)
            self.start_moving(self.EXTRACTION_KEY)
            self.wait_for_stop(self.EXTRACTION_KEY)

            # calibration
            self.set_rpm(config.VESC_EXTRACTION_CALIBRATION_RPM, self.EXTRACTION_KEY)
            self.set_time_to_move(config.VESC_EXTRACTION_CALIBRATION_MAX_TIME, self.EXTRACTION_KEY)
            self.start_moving(self.EXTRACTION_KEY)
            res = self.wait_for_stopper_hit(self.EXTRACTION_KEY, config.VESC_EXTRACTION_CALIBRATION_MAX_TIME)
            self.stop_moving(self.EXTRACTION_KEY)
            if not res:
                # TODO what robot should do if calibration was failed (there was no stopper hit)?
                print("Stopped vesc EXTRACTION engine calibration due timeout (stopper signal wasn't received!)")
        # do any new calibrations (add vesc calibration code here)
        # ...

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def close(self):
        self.__keep_thread_alive = False
        cleanup_gpio = False

        for engine_key in self.__can_ids:
            self.stop_moving(engine_key)
            #if self.__gpio_stoppers_pins[engine_key] is not None:
                #GPIO.cleanup(self.__gpio_stoppers_pins[engine_key])
                #if not cleanup_gpio:
                    #cleanup_gpio = True

        #if cleanup_gpio:
            #GPIO.cleanup()

        self._movement_ctrl_th.join(1)
        self.__ser.close()

    def get_unregistered_can_id(self):
        for can_id in range(0, 253):
            data = self.__get_firmware_version(['version_major', 'version_minor'], can_id)
            if data is not None and can_id not in self.__can_ids.values():
                return can_id
        return None

    def __get_firmware_version(self, report_field_names, can_id):
        self.__ser.write(pyvesc.encode_request(pyvesc.GetFirmwareVersion(can_id=can_id)))
        in_buf = b''
        while self.__ser.in_waiting > 0:
            in_buf += self.__ser.read(self.__ser.in_waiting)

        if len(in_buf) == 0:
            return None
        response, consumed = pyvesc.decode(in_buf)
        if consumed == 0:
            return None

        if isinstance(response, pyvesc.GetFirmwareVersion):
            report_row = {}
            for field_name in report_field_names:
                report_row[field_name] = getattr(response, field_name)
            return report_row
        return None

    def _movement_ctrl_th_tf(self):
        """Target function of movement control thread (only inner usage).

        Implements keeping multiple vesc engines alive and stopping them by a timers if they were set.
        """

        try:
            while self.__keep_thread_alive:
                # check each active engine stop timer
                for engine_key in self.__can_ids:
                    if self.__is_moving[engine_key] and time.time() - self.__start_time[engine_key] > self.__time_to_move[engine_key]:
                        self.__write_rpm_vesc(engine_key, 0)
                        #self.__ser.write(pyvesc.encode(pyvesc.SetRPM(0, can_id=self.__can_ids[engine_key])))
                        self.__last_stop_time[engine_key] = time.time()
                        self.__is_moving[engine_key] = False
                # send alive to each active
                if time.time() > self.__next_alive_time:
                    self.__next_alive_time = time.time() + 1 / self.__alive_freq
                    for engine_key in self.__is_moving:
                        if self.__is_moving[engine_key]:
                            self.__ser.write(pyvesc.encode(pyvesc.SetAlive(can_id=self.__can_ids[engine_key])))
                # wait for next checking tick
                time.sleep(1 / self.__check_freq)
        except serial.SerialException as ex:
            print(ex)

    def __write_rpm_vesc(self, engine_key, rpm):
        if engine_key in self.__current_rpm.keys():
            self.__apply_rpm_slowly(engine_key, rpm)
        else:
            self.__ser.write(pyvesc.encode(pyvesc.SetRPM(rpm, can_id=self.__can_ids[engine_key])))

    def __apply_rpm_slowly(self, engine_key, rpm):
        self.__future_rpm[engine_key] = rpm

    def __rpm_ctrl_th_tf(self):
        """Target function of rpm control thread (only inner usage)."""
        try:
            while self.__keep_thread_rpm_alive:
                for engine_key, current_rpm in self.__current_rpm.items():
                    future_rpm = self.__future_rpm[engine_key]
                    if future_rpm != current_rpm:
                        if abs(future_rpm-current_rpm) > config.STEP_INCREMENTAL_RPM:
                            new_rpm = config.STEP_INCREMENTAL_RPM
                            if future_rpm-current_rpm < 0:
                                new_rpm = -new_rpm
                        else:
                            new_rpm = future_rpm-current_rpm
                        self.__current_rpm[engine_key] += new_rpm
                        self.__ser.write(pyvesc.encode(pyvesc.SetRPM(self.__current_rpm[engine_key], can_id=self.__can_ids[engine_key])))
                time.sleep(1 / config.FREQUENCY_INCREMENTAL_RPM)
        except serial.SerialException as ex:
            print(ex)

    def start_moving(self, engine_key):
        self.__start_time[engine_key] = time.time()
        #self.__ser.write(pyvesc.encode(pyvesc.SetRPM(self.__rpm[engine_key], can_id=self.__can_ids[engine_key])))
        self.__write_rpm_vesc(engine_key, self.__rpm[engine_key])
        self.__is_moving[engine_key] = True

    def stop_moving(self, engine_key):
        if self.__is_moving[engine_key]:
            self.__last_stop_time[engine_key] = time.time()
        self.__is_moving[engine_key] = False
        #self.__ser.write(pyvesc.encode(pyvesc.SetRPM(0, can_id=self.__can_ids[engine_key])))
        self.__write_rpm_vesc(engine_key, 0)

    def wait_for_stop(self, engine_key, timeout=None):
        """Blocks caller thread until specified engine is end his work or timeout time is out (if timeout was set).

        Returns True if engine has ended his work, returns False if timeout waiting time is out.
        """

        end_t = time.time() + timeout if timeout is not None else float("inf")
        while self.__is_moving[engine_key]:
            if timeout is not None and time.time() > end_t:
                return False
            time.sleep(1 / self.__check_freq)
        return True

    def wait_for_stop_any(self, timeout=None):
        raise NotImplementedError("this feature is not implemented yet")

    # TODO add flag "stop engine at stopper hit"
    def wait_for_stopper_hit(self, engine_key, timeout=None, stop_engine_if_timeout=True):
        """Blocks caller thread until specified engine stopper hit or timeout time is out (if timeout was set).

        Returns True if stopper was hit, returns False if timeout waiting time is out
        or engine was stopped by it's own work timer.
        """

        if self.__gpio_stoppers_pins[engine_key] is None:
            self.stop_moving(engine_key)
            raise RuntimeError("stopper usage is not allowed in config \
                (engine movement is terminated to prevent occasional damage cause)")

        end_t = time.time() + timeout if timeout is not None else float("inf")
        while self.__is_moving[engine_key]:
            #if GPIO.input(self.__gpio_stoppers_pins[engine_key]) == self.__stopper_signals[engine_key]:
                #return True
            if time.time() > end_t:
                if stop_engine_if_timeout:
                    self.stop_moving(engine_key)
                return False
            time.sleep(1 / self.__stopper_check_freq)
        return False

    def wait_for_stopper_hit_any(self):
        raise NotImplementedError("this feature is not implemented yet")

    def apply_rpm(self, rpm, engine_key):
        self.__rpm[engine_key] = rpm
        self.__write_rpm_vesc(engine_key, self.__rpm[engine_key])
        #self.__ser.write(pyvesc.encode(pyvesc.SetRPM(self.__rpm[engine_key], can_id=self.__can_ids[engine_key])))

    def set_rpm(self, rpm, engine_key):
        self.__rpm[engine_key] = rpm

    def set_time_to_move(self, time_to_move, engine_key):
        self.__time_to_move[engine_key] = time_to_move

    def set_alive_freq(self, alive_freq):
        self.__alive_freq = alive_freq

    def set_check_freq(self, check_freq):
        self.__check_freq = check_freq

    def is_moving(self, engine_key):
        return self.__is_moving[engine_key]

    def get_last_stop_time(self, engine_key):
        return self.__last_stop_time[engine_key]

    def get_last_start_time(self, engine_key):
        return self.__start_time[engine_key]

    def get_last_movement_time(self, engine_key):
        """Returns last movement time if VESCs are not working at the moment;
        returns current working time if VESCs are working at the moment.
        """

        if self.__start_time[engine_key] is None:
            return 0
        elif self.__is_moving[engine_key] or self.__last_stop_time[engine_key] is None:
            return time.time() - self.__start_time[engine_key]
        else:
            return self.__last_stop_time[engine_key] - self.__start_time[engine_key]

    def get_adapter_rpm(self, engine_key):
        """Returns vesc adapter's (this instance) currently stored rpm for specified vesc engine"""

        return self.__rpm[engine_key]

    def get_sensors_data(self, report_field_names, engine_key):
        self.__ser.write(pyvesc.encode_request(pyvesc.GetValues(can_id=self.__can_ids[engine_key])))
        in_buf = b''
        while self.__ser.in_waiting > 0:
            in_buf += self.__ser.read(self.__ser.in_waiting)

        if len(in_buf) == 0:
            return None
        response, consumed = pyvesc.decode(in_buf)
        if consumed == 0:
            return None

        if isinstance(response, pyvesc.GetValues):
            report_row = {}
            for field_name in report_field_names:
                report_row[field_name] = getattr(response, field_name)
            return report_row
        return None


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

    def __init__(self, ser_port, ser_baudrate, alive_freq, check_freq, stopper_check_freq, logger_full: utility.Logger):
        self.__locker = threading.Lock()
        self.__reconnect_locker = threading.Lock()

        self.__ser_port = ser_port
        self.__ser_baudrate = ser_baudrate

        gpio_is_initialized = False
        self.__stopper_check_freq = stopper_check_freq
        self.__alive_freq = alive_freq
        self.__check_freq = check_freq
        self.__logger_full = logger_full
        self.__next_alive_time = time.time()

        self.__can_ids = dict()
        self.__current_rpm = dict()
        self.__target_rpm = dict()
        self.__use_smooth_accel = dict()
        self.__smooth_accel_next_t = dict()
        self.__use_smooth_decel = dict()
        self.__smooth_decel_next_t = dict()
        self.__time_to_move = dict()
        self.__start_time = dict()
        self.__is_moving = dict()
        self.__stop_request = dict()
        self.__last_stop_time = dict()
        self.__stopper_signals = dict()
        self.__gpio_stoppers_pins = dict()

        self.__ser = serial.Serial(port=ser_port, baudrate=ser_baudrate)
        self.__ser.flushInput()
        self.__ser.flushOutput()
        self.__ser.timeout = config.VESC_TIMEOUT_READ

        # INIT ALL ALLOWED VESCS HERE
        # init PROPULSION vesc (currently it's parent vesc so it has no checkings for ID and has parent's ID=None)
        if config.VESC_ALLOW_PROPULSION:
            if config.VESC_PROPULSION_AUTODETECT_CAN_ID:
                raise NotImplementedError(f"[{self.__class__.__name__}] -> Can id detection is not confirmed to work fine.")
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
            self.__gpio_stoppers_pins[self.PROPULSION_KEY] = config.VESC_PROPULSION_STOPPER_PIN
            if self.__gpio_stoppers_pins[self.PROPULSION_KEY] is not None:
                if not gpio_is_initialized:
                    raise NotImplementedError("Gpio disabled due to non-compatible library issue and non-use")
                    #GPIO.setmode(GPIO.BOARD)
                    #gpio_is_initialized = True
                #GPIO.setup(self.__gpio_stoppers_pins[self.PROPULSION_KEY], GPIO.IN)

        # init EXTRACTION vesc
        if config.VESC_ALLOW_EXTRACTION:
            if config.VESC_EXTRACTION_AUTODETECT_CAN_ID:
                raise NotImplementedError("can id detection is not confirmed to work fine")
                # ext_can_id = self.get_unregistered_can_id()
            else:
                ext_can_id = config.VESC_EXTRACTION_CAN_ID
            if ext_can_id is not None:
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
                self.__gpio_stoppers_pins[self.EXTRACTION_KEY] = config.VESC_EXTRACTION_STOPPER_PIN
                if self.__gpio_stoppers_pins[self.EXTRACTION_KEY] is not None:
                    if not gpio_is_initialized:
                        raise NotImplementedError("Gpio disabled due to non-compatible library issue and non-use")
                        #GPIO.setmode(GPIO.BOARD)
                        #gpio_is_initialized = True
                    #GPIO.setup(self.__gpio_stoppers_pins[self.EXTRACTION_KEY], GPIO.IN)
            else:
                # TODO what robot should do if initialization was failed?
                print(f"[{self.__class__.__name__}] -> Extraction vesc initialization fail: couldn't determine extraction vesc ID.")
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
                print("Stopped vesc PROPULSION engine calibration due timeout (stopper signal wasn't received!)")

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
                print(f"[{self.__class__.__name__}] -> Stopped vesc EXTRACTION engine calibration due timeout (stopper signal wasn't received!).")
        # do any new calibrations (add vesc calibration code here)
        # ...
        self.__last_reconnect_time = time.time() - 60

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def __del__(self):
        self.close()
        del self.__ser

    def close(self):
        if self.__ser.is_open:
            self.__keep_thread_alive = False
            cleanup_gpio = False

            for engine_key in self.__can_ids:
                self.stop_moving(engine_key)
                #if self.__gpio_stoppers_pins[engine_key] is not None:
                    #GPIO.cleanup(self.__gpio_stoppers_pins[engine_key])
                    #if not cleanup_gpio:
                        #cleanup_gpio = True

            #if cleanup_gpio:
                #GPIO.cleanup()

            self._movement_ctrl_th.join(1)
            self.__ser.close()

    def reconnect_vesc(self) :
        with self.__reconnect_locker :
            if time.time() - self.__last_reconnect_time < 60 :
                return

            print(self)
            self.__last_reconnect_time = time.time()
            self.__ser.close()

            smoothie_vesc_addr = utility.get_smoothie_vesc_addresses()
            while not "vesc" in smoothie_vesc_addr:
                msg = f"[{self.__class__.__name__}] -> Couldn't get vesc's USB address, stopping attempt to unlock with lifeline."
                print(msg)
                time.sleep(1)
                smoothie_vesc_addr = utility.get_smoothie_vesc_addresses()
                
            vesc_address = smoothie_vesc_addr["vesc"]
            msg = f"[{self.__class__.__name__}] -> Finding vesc's USB address at '{vesc_address}'."
            print(msg)
            
            could_open_port = False
            while not could_open_port :
                try : 
                    self.__ser = serial.Serial(port=vesc_address, baudrate=self.__ser_baudrate)
                    could_open_port = True
                    self.__ser.flushInput()
                    self.__ser.flushOutput()
                    self.__ser.timeout = 5
                    print(f"[{self.__class__.__name__}] -> It is reconnected!")
                except KeyboardInterrupt:
                    raise KeyboardInterrupt
                except Exception as e:
                    print(f"[{self.__class__.__name__}] -> Could not open port ({e}).")
                    time.sleep(1)

    def get_unregistered_can_id(self):
        for can_id in range(0, 253):
            data = self.__get_firmware_version(['version_major', 'version_minor'], can_id)
            if data is not None and can_id not in self.__can_ids.values():
                return can_id
        return None

    def __get_firmware_version(self, report_field_names, can_id):
        with self.__locker:
            try :
                self.__ser.write(pyvesc.encode_request(pyvesc.GetFirmwareVersion(can_id=can_id)))
                in_buf = b''
                while self.__ser.in_waiting > 0:
                    in_buf += self.__ser.read(self.__ser.in_waiting)
            except SerialException :
                self.reconnect_vesc()


        if len(in_buf) == 0:
            return None
        response, consumed = pyvesc.decode(in_buf)
        if consumed == 0:
            return None

        if isinstance(response, pyvesc.GetFirmwareVersion):
            report_row = {}
            for field_name in report_field_names:
                report_row[field_name] = getattr(response, field_name)
            return report_row
        return None

    def _movement_ctrl_th_tf(self):
        """Target function of movement control thread (only inner usage).

        Implements keeping multiple vesc engines alive and stopping them by a timers if they were set.
        """
        try:
            while self.__keep_thread_alive:
                with self.__locker:
                    # process each active engine
                    for engine_key in self.__can_ids:
                        if not self.__keep_thread_alive:
                            break
                        if not self.__is_moving[engine_key]:
                            continue
                        # engine movement timeout
                        if time.time() - self.__start_time[engine_key] >= self.__time_to_move[engine_key] or \
                                self.__stop_request[engine_key]:
                            # immediate engine stop
                            if not self.__use_smooth_decel[engine_key]:
                                try :
                                    self.__ser.write(pyvesc.encode(pyvesc.SetRPM(0, can_id=self.__can_ids[engine_key])))
                                except SerialException :
                                    self.reconnect_vesc()
                                self.__current_rpm[engine_key] = 0
                                self.__last_stop_time[engine_key] = time.time()
                                self.__is_moving[engine_key] = False
                                self.__stop_request[engine_key] = False
                            # smooth engine stop (if it's time to check)
                            elif time.time() >= self.__smooth_decel_next_t[engine_key]:
                                self.__smooth_decel_next_t[engine_key] = time.time() + config.VESC_SMOOTH_DECEL_TIME_STEP

                                # reduce speed (RPM is bigger than step so step is possible)
                                if abs(self.__current_rpm[engine_key]) > config.VESC_SMOOTH_DECEL_RPM_STEP:
                                    self.__current_rpm[engine_key] += -config.VESC_SMOOTH_DECEL_RPM_STEP \
                                        if self.__current_rpm[engine_key] > 0 else config.VESC_SMOOTH_DECEL_RPM_STEP
                                    try :
                                        self.__ser.write(pyvesc.encode(pyvesc.SetRPM(
                                            self.__current_rpm[engine_key],
                                            can_id=self.__can_ids[engine_key])))
                                    except SerialException :
                                        self.reconnect_vesc()
                                # stop engine (current RPM <= RPM step)
                                else:
                                    try :
                                        self.__ser.write(pyvesc.encode(pyvesc.SetRPM(0, can_id=self.__can_ids[engine_key])))
                                    except SerialException :
                                        self.reconnect_vesc()
                                    self.__current_rpm[engine_key] = 0
                                    self.__last_stop_time[engine_key] = time.time()
                                    self.__is_moving[engine_key] = False
                                    self.__stop_request[engine_key] = False
                        # smooth start engine if needed
                        elif self.__use_smooth_accel[engine_key] and time.time() >= self.__smooth_accel_next_t[engine_key]:
                            self.__smooth_accel_next_t[engine_key] = time.time() + config.VESC_SMOOTH_ACCEL_TIME_STEP

                            # set engine to target RPM as current-target difference is <= RPM step
                            if abs(self.__target_rpm[engine_key] - self.__current_rpm[engine_key]) <= \
                                    config.VESC_SMOOTH_ACCEL_RPM_STEP:
                                try :
                                    self.__ser.write(pyvesc.encode(pyvesc.SetRPM(
                                        self.__target_rpm[engine_key],
                                        can_id=self.__can_ids[engine_key])))
                                except SerialException :
                                    self.reconnect_vesc()
                                self.__current_rpm[engine_key] = self.__target_rpm[engine_key]
                            # increase current RPM by RPM step
                            else:
                                self.__current_rpm[engine_key] += config.VESC_SMOOTH_ACCEL_RPM_STEP \
                                    if self.__target_rpm[engine_key] > self.__current_rpm[engine_key] \
                                    else -config.VESC_SMOOTH_ACCEL_RPM_STEP
                                try :
                                    self.__ser.write(pyvesc.encode(pyvesc.SetRPM(
                                        self.__current_rpm[engine_key],
                                        can_id=self.__can_ids[engine_key])))
                                except SerialException :
                                    self.reconnect_vesc()

                    # send alive to each active
                    if time.time() > self.__next_alive_time:
                        self.__next_alive_time = time.time() + 1 / self.__alive_freq
                        for engine_key in self.__is_moving:
                            try :
                                if self.__is_moving[engine_key]:
                                    self.__ser.write(pyvesc.encode(pyvesc.SetAlive(can_id=self.__can_ids[engine_key])))
                                    
                                vesc_rpm = self.__get_rpm_sensor_data(engine_key)
                                
                                if vesc_rpm is not None:
                                    if vesc_rpm == 0 and self.__current_rpm[engine_key] != 0:
                                        self.__logger_full.write_and_flush(f"[{self.__class__.__name__}] Detect stop propulsion, send RPM again.\n")
                                        self.__ser.write(
                                            pyvesc.encode(
                                                pyvesc.SetRPM(
                                                    self.__current_rpm[engine_key],
                                                    can_id=self.__can_ids[engine_key]
                                                )
                                            )
                                        )
                                        
                            except SerialException or OSError as e :
                                if e.errno == 5 or isinstance(e, SerialException):
                                    self.reconnect_vesc()
                # wait for next checking tick
                time.sleep(1 / self.__check_freq)
        except serial.SerialException as ex:
            print(f"[{self.__class__.__name__}] -> {ex}")  # TODO should these exceptions to be ignored?
            
    def __get_rpm_sensor_data(self, engine_key):
        self.__ser.write(pyvesc.encode_request(pyvesc.GetValues(can_id=self.__can_ids[engine_key])))
        in_buf = b''
        while self.__ser.in_waiting > 0:
            try:
                in_buf += self.__ser.read(self.__ser.in_waiting)
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except Exception as e:
                self.__logger_full.write_and_flush("[Error] "+str(e)+"\n")
        if len(in_buf) != 0:
            response, consumed = pyvesc.decode(in_buf)
            if consumed != 0 and response is not None:
                return response.__dict__["rpm"]
        return None

    def start_moving(self, engine_key, smooth_acceleration: bool = False, smooth_deceleration: bool = False):
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
                    self.__ser.write(pyvesc.encode(pyvesc.SetRPM(
                        self.__target_rpm[engine_key],
                        can_id=self.__can_ids[engine_key])))
                except SerialException :
                    self.reconnect_vesc()
                self.__current_rpm[engine_key] = self.__target_rpm[engine_key]
            self.__is_moving[engine_key] = True

    def stop_moving(self, engine_key, smooth_deceleration: bool = False):        
        with self.__locker:
            self.__use_smooth_decel[engine_key] = smooth_deceleration

            if smooth_deceleration:
                self.__smooth_decel_next_t[engine_key] = 0
                self.__stop_request[engine_key] = True
            else:
                try :
                    self.__ser.write(pyvesc.encode(pyvesc.SetRPM(0, can_id=self.__can_ids[engine_key])))
                except SerialException :
                    self.reconnect_vesc()
                self.__current_rpm[engine_key] = 0
                self.__last_stop_time[engine_key] = time.time()
                self.__is_moving[engine_key] = False

    def wait_for_stop(self, engine_key, timeout=None):
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

    def wait_for_stop_any(self, timeout=None):
        raise NotImplementedError(f"[{self.__class__.__name__}] -> This feature is not implemented yet")

    def wait_for_stopper_hit(self,
                             engine_key,
                             timeout=None,
                             stop_engine_if_timeout=True,
                             stop_engine_if_stopper_hit=True):
        """Blocks caller thread until specified engine stopper hit or timeout time is out (if timeout was set).

        Returns True if stopper was hit, returns False if timeout waiting time is out
        or engine was stopped by its own work timer.
        """

        if self.__gpio_stoppers_pins[engine_key] is None:
            self.stop_moving(engine_key)
            raise RuntimeError(f"[{self.__class__.__name__}] -> Stopper usage is not allowed in config \
                (engine movement is terminated to prevent occasional damage cause)")

        end_t = time.time() + timeout if timeout is not None else float("inf")

        while True:
            with self.__locker:
                if not self.__is_moving[engine_key]:
                    return False
            #if GPIO.input(self.__gpio_stoppers_pins[engine_key]) == self.__stopper_signals[engine_key]:
                #if stop_engine_if_stopper_hit:
                    #self.stop_moving(engine_key)
                #return True
            if time.time() > end_t:
                if stop_engine_if_timeout:
                    self.stop_moving(engine_key)
                return False
            time.sleep(1 / self.__stopper_check_freq)

    def wait_for_stopper_hit_any(self):
        raise NotImplementedError(f"[{self.__class__.__name__}] -> This feature is not implemented yet")

    def set_current_rpm(self, rpm, engine_key):
        """Set as current and apply given RPM on specified by engine_key vesc engine.

        NOTICE: engine will smoothly back to its target RPM if smooth acceleration is enabled for engine_key engine.
        To apply RPM immediately, set RPM you want to apply as target RPM first by using set_target_rpm method before
        calling this.
        """

        if not isinstance(rpm, (int, float)):
            msg = f"[{self.__class__.__name__}] -> rpm must be int or float, got {type(rpm).__name__} instead"
            raise TypeError(msg)

        with self.__locker:
            try :
                self.__ser.write(pyvesc.encode(pyvesc.SetRPM(rpm, can_id=self.__can_ids[engine_key])))
            except SerialException :
                self.reconnect_vesc()
            self.__current_rpm[engine_key] = rpm

    def set_target_rpm(self, rpm, engine_key):
        """Set given RPM as target RPM for specified engine_key vesc engine.

        NOTICE: engine will speed up to this RPM smoothly if smooth acceleration is enabled for engine_key vesc engine,
        otherwise this RPM will be applied immediately during engine start. In this case high RPM values may lead to
        strong jerk during the start.
        """
        if not isinstance(rpm, (int, float)):
            msg = f"[{self.__class__.__name__}] -> rpm must be int or float, got {type(rpm).__name__} instead"
            raise TypeError(msg)

        with self.__locker:
            self.__target_rpm[engine_key] = rpm

    def set_time_to_move(self, time_to_move, engine_key):
        if not isinstance(time_to_move, (int, float)):
            msg = f"[{self.__class__.__name__}] -> time_to_move must be int or float, got {type(time_to_move).__name__} instead"
            raise TypeError(msg)
        if time_to_move < 0:
            msg = f"[{self.__class__.__name__}] -> time_to_move must be >= 0, got {str(time_to_move)} instead"
            raise ValueError(msg)

        with self.__locker:
            self.__time_to_move[engine_key] = time_to_move

    def set_alive_freq(self, alive_freq):
        if not isinstance(alive_freq, (int, float)):
            msg = f"[{self.__class__.__name__}] -> alive_freq must be int or float, got {type(alive_freq).__name__} instead"
            raise TypeError(msg)
        if alive_freq < 0:
            msg = f"[{self.__class__.__name__}] -> alive_freq must be >= 0, got {str(alive_freq)} instead"
            raise ValueError(msg)

        with self.__locker:
            self.__alive_freq = alive_freq

    def set_check_freq(self, check_freq):
        if not isinstance(check_freq, (int, float)):
            msg = f"[{self.__class__.__name__}] -> check_freq must be int or float, got {type(check_freq).__name__} instead"
            raise TypeError(msg)
        if check_freq < 0:
            msg = f"[{self.__class__.__name__}] -> check_freq must be >= 0, got {str(check_freq)} instead"
            raise ValueError(msg)

        with self.__locker:
            self.__check_freq = check_freq

    def set_smooth_acceleration(self, smooth_acceleration: bool, engine_key):
        if not isinstance(smooth_acceleration, bool):
            msg = f"[{self.__class__.__name__}] -> smooth_acceleration must be bool, got {type(smooth_acceleration).__name__} instead"
            raise TypeError(msg)

        with self.__locker:
            self.__use_smooth_accel[engine_key] = smooth_acceleration

    def set_smooth_deceleration(self, smooth_deceleration: bool, engine_key):
        if not isinstance(smooth_deceleration, bool):
            msg = f"[{self.__class__.__name__}] -> smooth_deceleration must be bool, got {type(smooth_deceleration).__name__} instead"
            raise TypeError(msg)

        with self.__locker:
            self.__use_smooth_decel[engine_key] = smooth_deceleration

    def get_smooth_acceleration(self, engine_key):
        with self.__locker:
            return self.__use_smooth_accel[engine_key]

    def get_smooth_deceleration(self, engine_key):
        with self.__locker:
            return self.__use_smooth_decel[engine_key]

    def get_last_stop_time(self, engine_key):
        with self.__locker:
            return self.__last_stop_time[engine_key]

    def get_last_start_time(self, engine_key):
        with self.__locker:
            return self.__start_time[engine_key]

    def get_last_movement_time(self, engine_key):
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

    def get_current_rpm(self, engine_key):
        """Returns specified vesc engine current RPM"""

        return self.__current_rpm[engine_key]

    def get_target_rpm(self, engine_key):
        """Returns specified vesc engine target RPM"""

        return self.__target_rpm[engine_key]

    def get_sensors_data_of_can_id(self, report_field_names, can_id):
        in_buf = b''
        with self.__locker:
            try :
                self.__ser.write(pyvesc.encode_request(pyvesc.GetValues(can_id=can_id)))
                while self.__ser.in_waiting > 0:
                    in_buf += self.__ser.read(self.__ser.in_waiting)
            except SerialException :
                self.reconnect_vesc()

        if len(in_buf) == 0:
            return None
        response, consumed = pyvesc.decode(in_buf)
        if consumed == 0:
            return None

        if isinstance(response, pyvesc.GetValues):
            report_row = {}
            for field_name in report_field_names:
                report_row[field_name] = getattr(response, field_name)
            return report_row
        return None

    def get_sensors_data(self, report_field_names, engine_key):
        in_buf = b''
        with self.__locker:
            try : 
                self.__ser.write(pyvesc.encode_request(pyvesc.GetValues(can_id=self.__can_ids[engine_key])))
                while self.__ser.in_waiting > 0:
                    in_buf += self.__ser.read(self.__ser.in_waiting)
            except SerialException :
                self.reconnect_vesc()

        if len(in_buf) == 0:
            return None
        response, consumed = pyvesc.decode(in_buf)
        if consumed == 0:
            return None

        if isinstance(response, pyvesc.GetValues):
            report_row = {}
            for field_name in report_field_names:
                report_row[field_name] = getattr(response, field_name)
            return report_row
        return None

    def is_moving(self, engine_key):
        with self.__locker:
            return self.__is_moving[engine_key]


class GPSUbloxAdapter:
    """Provides access to the robot's on-board GPS navigator (UBLOX card)"""

    def __init__(self, ser_port: str, ser_baudrate: int, last_pos_count: int):
        if not isinstance(last_pos_count, int):
            raise TypeError(f"[{self.__class__.__name__}] -> last_pos_count must be int, got {type(last_pos_count).__name__} instead")
        if last_pos_count < 1:
            raise ValueError(f"[{self.__class__.__name__}] -> last_pos_count shouldn't be less than 1, got {last_pos_count} instead")

        self._position_is_fresh = False
        self._last_pos_count = last_pos_count
        self._ser_port = ser_port
        self._ser_baudrate = ser_baudrate
        self._last_pos_container = []
        self._sync_locker = multiprocessing.RLock()

        self._serial = self._get_new_connection()

        self._keep_thread_alive = True
        self._reader_thread = threading.Thread(target=self._reader_thread_tf, daemon=True)
        self._reader_thread.start()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
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

    def get_fresh_position(self) -> list:
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

    def get_last_position(self) -> list:
        """Waits until at least one position is stored, returns last saved position copy at the moment of call
        (reference type safe)

        Returned position is in "old list" format.
        """

        while len(self._last_pos_container) < 1:
            pass
        with self._sync_locker:
            return self._last_pos_container[-1].as_old_list

    def get_last_position_non_blocking(self) -> list:
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

    def get_last_position_v2_non_blocking(self) -> navigation.GPSPoint:
        """Returns None if no positions are stored, returns last saved position copy at the moment of call
        (reference type safe)

        Returned position is an instance of navigation.GPSPoint class.
        """

        with self._sync_locker:
            # TODO currently it's not a deep copy
            return self._last_pos_container[-1] if len(self._last_pos_container) > 0 else None

    def get_last_positions_list(self):
        """Waits until at least one position is stored, returns list of last saved positions copies at the moment of
        call (reference type safe)"""

        get_last_positions_list_time = time.time()
        positions = []

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
            print(f"[{self.__class__.__name__}] -> Ublox reading error:", ex)

    def _read_from_gps(self):
        """Returns GPS coordinates of the current position"""

        while True:
            try:
                read_line = self._serial.readline()
                if isinstance(read_line, bytes):
                    data = str(read_line)
                    # if len(data) == 3:
                    #    print("None GNGGA or RTCM threads")
                    if "GNGGA" in data and ",,," not in data:
                        # bad string with no position data
                        # print(data)  # debug
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

    def _D2M2(self, Lat, NS, Lon, EW):
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

    def _get_new_connection(self, old_conn: serial.Serial = None):
        if old_conn is not None and old_conn.is_open:
            old_conn.close()
        new_conn = serial.Serial(port=self._ser_port, baudrate=self._ser_baudrate)
        # self._hot_reset()
        # self._USBNMEA_OUT()
        return new_conn


class GPSUbloxAdapterWithoutThread:
    """Provides access to the robot's on-board GPS navigator (UBLOX card)"""

    def __init__(self, ser_port: str, ser_baudrate: int, last_pos_count: int):
        self._serial = serial.Serial(port=ser_port, baudrate=ser_baudrate)
        # self._hot_reset()
        # self._USBNMEA_OUT()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
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

    def get_last_positions_list(self):
        """Waits until at least one position is stored, returns list of last saved positions copies at the moment of
        call (reference type safe)"""

        raise NotImplementedError(f"[{self.__class__.__name__}] -> Test without list")

    def _read_from_gps(self):
        """Returns GPS coordinates of the current position"""

        while True:
            try:
                read_line = self._serial.readline()
                if isinstance(read_line, bytes):
                    data = str(read_line)
                    # if len(data) == 3:
                    #    print("None GNGGA or RTCM threads")
                    if "GNGGA" in data and ",,," not in data:
                        # bad string with no position data
                        # print(data)  # debug
                        data = data.split(",")
                        lati, longi = self._D2M2(data[2], data[3], data[4], data[5])
                        point_quality = data[6]
                        return [lati, longi, point_quality]  # , float(data[11])  # alti
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                continue

    def _D2M2(self, Lat, NS, Lon, EW):
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
