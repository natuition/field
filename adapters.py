import connectors
import multiprocessing
import time
from config import config
import cv2 as cv
import math
import queue
import threading
import serial
import pyvesc
import re


class SmoothieAdapter:
    RESPONSE_OK = "ok\r\n"
    RESPONSE_ALARM_LOCK = "error:Alarm lock\n"
    RESPONSE_HALT = "!!\r\n"
    RESPONSE_WTF = "ok - ignored\n"

    def __init__(self, smoothie_host):
        if type(smoothie_host) is not str:
            raise TypeError("invalid smoothie_host type: should be str, received " + type(smoothie_host).__name__)

        if config.SMOOTHIE_BACKEND == 1:
            self._smc = connectors.SmoothieV11TelnetConnector(smoothie_host)
        elif config.SMOOTHIE_BACKEND == 2:
            self._smc = connectors.SmoothieV11SerialConnector(smoothie_host, config.SMOOTHIE_BAUDRATE)
        else:
            raise ValueError("wrong config.SMOOTHIE_BACKEND value: " + str(smoothie_host))

        self._sync_locker = multiprocessing.RLock()
        self._x_cur = multiprocessing.Value("d", 0)
        self._y_cur = multiprocessing.Value("d", 0)
        self._z_cur = multiprocessing.Value("d", 0)
        self._a_cur = multiprocessing.Value("d", 0)
        self._b_cur = multiprocessing.Value("d", 0)
        self._c_cur = multiprocessing.Value("d", 0)

        res = self.switch_to_relative()
        if res == self.RESPONSE_WTF or "ignored" in res:
            res = self.switch_to_relative()
        if res != self.RESPONSE_OK:
            # TODO: what if so?
            print("Switching smoothie to relative was failed! Smoothie's response:\n", res)

        # TODO: temporary crutch - vesc is moving Z upward before smoothie loads, so we need to lower the cork a bit down
        res = self.custom_move_for(config.Z_F_EXTRACTION_DOWN, Z=5)
        self.wait_for_all_actions_done()
        if res != self.RESPONSE_OK:
            print("Couldn't move cork down for Z-5! Calibration errors on Z axis are possible!")

        res = self.ext_calibrate_cork()
        """
        if res != self.RESPONSE_OK:
            print("Cork calibration:", res)  # TODO: what if so??
        """

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._smc.disconnect()

    def disconnect(self):
        self._smc.disconnect()

    def get_connector(self):
        """Only for debug!"""

        return self._smc

    def wait_for_all_actions_done(self):
        with self._sync_locker:
            self._smc.write("M400")
            # "ok\r\n"
            return self._smc.read_some()

    def halt(self):
        with self._sync_locker:
            self._smc.write("M112")
            # "ok Emergency Stop Requested - reset or M999 required to exit HALT state\r\n"
            return self._smc.read_some() + self._smc.read_some() if self._smc is connectors.SmoothieV11TelnetConnector else self._smc.read_some()

    def reset(self):
        with self._sync_locker:
            self._smc.write("reset")
            return self._smc.read_some()

    def freewheels(self):
        with self._sync_locker:
            self._smc.write("M18")
            return self._smc.read_some()

    def checkendstop(self, axe):
        with self._sync_locker:
            self._smc.write("M119")
            response = self._smc.read_some()
            matches = re.findall(f"(?:(?:{axe}_min)|(?:{axe}_max)):(.)", response)
            if matches:
                return matches[0]
            return 1 #refaire demande

    def switch_to_relative(self):
        with self._sync_locker:
            self._smc.write("G91")
            # "ok\r\n"
            return self._smc.read_some()

    def set_current_coordinates(self, X=None, Y=None, Z=None, A=None, B=None, C=None):
        with self._sync_locker:
            if self._check_arg_types([type(None)], X, Y, Z, A, B, C):
                raise TypeError("at least one axis shouldn't be None")
            if not self._check_arg_types([float, int, type(None)], X, Y, Z, A, B, C):
                raise TypeError("incorrect axis current value(s) type(s)")

            g_code = "G92"

            if X is not None:
                g_code += " X" + str(X * config.XY_COEFFICIENT_TO_MM)
            if Y is not None:
                g_code += " Y" + str(Y * config.XY_COEFFICIENT_TO_MM)
            if Z is not None:
                g_code += " Z" + str(Z)
            if A is not None:
                g_code += " A" + str(A)
            if B is not None:
                g_code += " B" + str(B)
            if C is not None:
                g_code += " C" + str(C)

            self._smc.write(g_code)
            response = self._smc.read_some()

            if response == self.RESPONSE_OK:
                if X is not None:
                    self._x_cur.value = X
                if Y is not None:
                    self._y_cur.value = Y
                if Z is not None:
                    self._z_cur.value = Z
                if A is not None:
                    self._a_cur.value = A
                if B is not None:
                    self._b_cur.value = B
                if C is not None:
                    self._c_cur.value = C
            return response

    def get_adapter_current_coordinates(self):
        with self._sync_locker:
            return {
                "X": self._x_cur.value,
                "Y": self._y_cur.value,
                "Z": self._z_cur.value,
                "A": self._a_cur.value,
                "B": self._b_cur.value
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

        with self._sync_locker:
            self._smc.write("M114.2")
            response, coordinates = (self._smc.read_some() + self._smc.read_some() if type(self._smc) is connectors.SmoothieV11TelnetConnector else self._smc.read_some())[:-2].split(" ")[2:], {}
            for coord in response:
                coordinates[coord[0]] = float(coord[2:])
                if convert_to_mms and coord[0] in ["X", "Y"]:
                    coordinates[coord[0]] /= config.XY_COEFFICIENT_TO_MM
            return coordinates

    def compare_coordinates(self, coordinates_a, coordinates_b, precision=1e-10):
        if type(coordinates_a) != dict or type(coordinates_b) != dict:
            raise AttributeError("coordinates should be stored in dict")
        if len(coordinates_a) != len(coordinates_b):
            raise AttributeError("coordinates dicts should have similar items count")

        for key in coordinates_a:
            if abs(coordinates_a[key] - coordinates_b[key]) > precision:
                return False
        return True

    def custom_move_for(self, F: int, X=None, Y=None, Z=None, A=None, B=None, C=None):
        """Movement by some value(s)"""

        with self._sync_locker:
            if self._check_arg_types([type(None)], X, Y, Z, A, B, C):
                raise TypeError("at least one axis shouldn't be None")
            if not self._check_arg_types([float, int, type(None)], X, Y, Z, A, B, C):
                raise TypeError("incorrect axis coordinates value(s) type(s)")
            if not self._check_arg_types([int], F):
                raise TypeError("incorrect force F value type")

            g_code = "G0"

            if X is not None:
                error_msg = self.validate_value(0, F, "F", config.XY_F_MIN, config.XY_F_MAX, "XY_F_MIN", "XY_F_MAX")
                if error_msg:
                    return error_msg
                error_msg = self.validate_value(self._x_cur.value * config.XY_COEFFICIENT_TO_MM,
                                                X * config.XY_COEFFICIENT_TO_MM, "X", config.X_MIN, config.X_MAX,
                                                "X_MIN", "X_MAX")
                if error_msg:
                    return error_msg
                g_code += " X" + str(X * config.XY_COEFFICIENT_TO_MM)

            if Y is not None:
                error_msg = self.validate_value(0, F, "F", config.XY_F_MIN, config.XY_F_MAX, "XY_F_MIN", "XY_F_MAX")
                if error_msg:
                    return error_msg
                error_msg = self.validate_value(self._y_cur.value * config.XY_COEFFICIENT_TO_MM,
                                                Y * config.XY_COEFFICIENT_TO_MM, "Y", config.Y_MIN, config.Y_MAX,
                                                "Y_MIN", "Y_MAX")
                if error_msg:
                    return error_msg
                g_code += " Y" + str(Y * config.XY_COEFFICIENT_TO_MM)

            if Z is not None:
                error_msg = self.validate_value(0, F, "F", config.Z_F_MIN, config.Z_F_MAX, "Z_F_MIN", "Z_F_MAX")
                if error_msg:
                    return error_msg
                error_msg = self.validate_value(self._z_cur.value, Z, "Z", config.Z_MIN, config.Z_MAX, "Z_MIN", "Z_MAX")
                if error_msg:
                    return error_msg
                g_code += " Z" + str(Z)

            if A is not None:
                error_msg = self.validate_value(0, F, "F", config.A_F_MIN, config.A_F_MAX, "A_F_MIN", "A_F_MAX")
                if error_msg:
                    return error_msg
                error_msg = self.validate_value(self._a_cur.value, A, "A", config.A_MIN, config.A_MAX, "A_MIN", "A_MAX")
                if error_msg:
                    return error_msg
                g_code += " A" + str(A)

            if B is not None:
                error_msg = self.validate_value(0, F, "F", config.B_F_MIN, config.B_F_MAX, "B_F_MIN", "B_F_MAX")
                if error_msg:
                    return error_msg
                error_msg = self.validate_value(self._b_cur.value, B, "B", config.B_MIN, config.B_MAX, "B_MIN", "B_MAX")
                if error_msg:
                    return error_msg
                g_code += " B" + str(B)

            if C is not None:
                error_msg = self.validate_value(0, F, "F", config.C_F_MIN, config.C_F_MAX, "C_F_MIN", "C_F_MAX")
                if error_msg:
                    return error_msg
                error_msg = self.validate_value(self._c_cur.value, C, "C", config.C_MIN, config.C_MAX, "C_MIN", "C_MAX")
                if error_msg:
                    return error_msg
                g_code += " C" + str(C)

            g_code += " F" + str(F)

            self._smc.write(g_code)
            response = self._smc.read_some()

            if response == self.RESPONSE_OK:
                if X is not None:
                    self._x_cur.value += X
                if Y is not None:
                    self._y_cur.value += Y
                if Z is not None:
                    self._z_cur.value += Z
                if A is not None:
                    self._a_cur.value += A
                if B is not None:
                    self._b_cur.value += B
                if C is not None:
                    self._c_cur.value += C
            return response

    def custom_move_to(self, F: int, X=None, Y=None, Z=None, A=None, B=None, C=None):
        """Movement to the specified position"""

        with self._sync_locker:
            if self._check_arg_types([type(None)], X, Y, Z, A, B, C):
                raise TypeError("at least one axis shouldn't be None")
            if not self._check_arg_types([float, int, type(None)], X, Y, Z, A, B, C):
                raise TypeError("incorrect axis coordinates value(s) type(s)")
            if not self._check_arg_types([int], F):
                raise TypeError("incorrect force F value type")

            g_code = "G0"

            if X is not None:
                error_msg = self.validate_value(0, F, "F", config.XY_F_MIN, config.XY_F_MAX, "XY_F_MIN", "XY_F_MAX")
                if error_msg:
                    return error_msg
                error_msg = self.validate_value(0, X * config.XY_COEFFICIENT_TO_MM, "X", config.X_MIN, config.X_MAX,
                                                "X_MIN", "X_MAX")
                if error_msg:
                    return error_msg
                smc_x = X - self._x_cur.value
                g_code += " X" + str(smc_x * config.XY_COEFFICIENT_TO_MM)

            if Y is not None:
                error_msg = self.validate_value(0, F, "F", config.XY_F_MIN, config.XY_F_MAX, "XY_F_MIN", "XY_F_MAX")
                if error_msg:
                    return error_msg
                error_msg = self.validate_value(0, Y * config.XY_COEFFICIENT_TO_MM, "Y", config.Y_MIN, config.Y_MAX,
                                                "Y_MIN", "Y_MAX")
                if error_msg:
                    return error_msg
                smc_y = Y - self._y_cur.value
                g_code += " Y" + str(smc_y * config.XY_COEFFICIENT_TO_MM)

            if Z is not None:
                error_msg = self.validate_value(0, F, "F", config.Z_F_MIN, config.Z_F_MAX, "Z_F_MIN", "Z_F_MAX")
                if error_msg:
                    return error_msg
                error_msg = self.validate_value(0, Z, "Z", config.Z_MIN, config.Z_MAX, "Z_MIN", "Z_MAX")
                if error_msg:
                    return error_msg
                smc_z = Z - self._z_cur.value
                g_code += " Z" + str(smc_z)

            if A is not None:
                error_msg = self.validate_value(0, F, "F", config.A_F_MIN, config.A_F_MAX, "A_F_MIN", "A_F_MAX")
                if error_msg:
                    return error_msg
                error_msg = self.validate_value(0, A, "A", config.A_MIN, config.A_MAX, "A_MIN", "A_MAX")
                if error_msg:
                    return error_msg
                smc_a = A - self._a_cur.value
                g_code += " A" + str(smc_a)

            if B is not None:
                error_msg = self.validate_value(0, F, "F", config.B_F_MIN, config.B_F_MAX, "B_F_MIN", "B_F_MAX")
                if error_msg:
                    return error_msg
                error_msg = self.validate_value(0, B, "B", config.B_MIN, config.B_MAX, "B_MIN", "B_MAX")
                if error_msg:
                    return error_msg
                smc_b = B - self._b_cur.value
                g_code += " B" + str(smc_b)

            if C is not None:
                error_msg = self.validate_value(0, F, "F", config.C_F_MIN, config.C_F_MAX, "C_F_MIN", "C_F_MAX")
                if error_msg:
                    return error_msg
                error_msg = self.validate_value(0, C, "C", config.C_MIN, config.C_MAX, "C_MIN", "C_MAX")
                if error_msg:
                    return error_msg
                smc_c = C - self._c_cur.value
                g_code += " C" + str(smc_c)

            g_code += " F" + str(F)

            self._smc.write(g_code)
            response = self._smc.read_some()

            if response == self.RESPONSE_OK:
                if X is not None:
                    self._x_cur.value += smc_x
                if Y is not None:
                    self._y_cur.value += smc_y
                if Z is not None:
                    self._z_cur.value += smc_z
                if A is not None:
                    self._a_cur.value += smc_a
                if B is not None:
                    self._b_cur.value += smc_b
                if C is not None:
                    self._c_cur.value += smc_c
            return response

    def nav_move_forward(self, distance, F: int):
        with self._sync_locker:
            if not self._check_arg_types([float, int], distance):
                raise TypeError("incorrect axis coordinates value type")
            if not self._check_arg_types([int], F):
                raise TypeError("incorrect force F value type")

            error_msg = self.validate_value(0, F, "F", config.B_F_MIN, config.B_F_MAX, "B_F_MIN", "B_F_MAX")
            if error_msg:
                return error_msg

            self._smc.write("G0 B{0} F{1}".format(distance, F))
            response = self._smc.read_some()
            if response == self.RESPONSE_OK:
                self._b_cur.value += distance
            return response

    def nav_move_backward(self, distance, F: int):

        raise NotImplementedError("This option is not available yet.")

    def nav_turn_wheels_for(self, value, F: int):
        with self._sync_locker:
            if not self._check_arg_types([float, int], value):
                raise TypeError("incorrect axis coordinates value type")
            if not self._check_arg_types([int], F):
                raise TypeError("incorrect force F value type")

            error_msg = self.validate_value(self._a_cur.value, value, "A", config.A_MIN, config.A_MAX, "A_MIN", "A_MAX")
            if error_msg:
                return error_msg
            error_msg = self.validate_value(0, F, "F", config.A_F_MIN, config.A_F_MAX, "A_F_MIN", "A_F_MAX")
            if error_msg:
                return error_msg

            self._smc.write("G0 A{0} F{1}".format(value, F))
            response = self._smc.read_some()
            if response == self.RESPONSE_OK:
                self._a_cur.value += value
                with open(config.LAST_ANGLE_WHEELS_FILE, "w") as angle_file:
                    angle_file.write(str(self._a_cur.value))
            return response

    def nav_turn_wheels_to(self, destination, F: int):
        with self._sync_locker:
            if not self._check_arg_types([float, int], destination):
                raise TypeError("incorrect axis coordinates value type")
            if not self._check_arg_types([int], F):
                raise TypeError("incorrect force F value type")

            error_msg = self.validate_value(0, destination, "A", config.A_MIN, config.A_MAX, "A_MIN", "A_MAX")
            if error_msg:
                return error_msg
            error_msg = self.validate_value(0, F, "F", config.A_F_MIN, config.A_F_MAX, "A_F_MIN", "A_F_MAX")
            if error_msg:
                return error_msg

            smc_a = destination - self._a_cur.value
            self._smc.write("G0 A{0} F{1}".format(smc_a, F))
            response = self._smc.read_some()
            if response == self.RESPONSE_OK:
                self._a_cur.value += smc_a
                with open(config.LAST_ANGLE_WHEELS_FILE, "w") as angle_file:
                    angle_file.write(str(self._a_cur.value))
            return response
            
    def nav_turn_wheels_left_max(self, F: int):
        return self.nav_turn_wheels_to(config.A_MIN, F)

    def nav_turn_wheels_right_max(self, F: int):
        return self.nav_turn_wheels_to(config.A_MAX, F)

    def nav_align_wheels_center(self, F: int):
        return self.nav_turn_wheels_to(config.NAV_TURN_WHEELS_CENTER, F)

    def nav_calibrate_wheels(self):
        """
        Calibrates nav. wheels and sets their current position to adapter and smoothie.
        NOT TESTED YET!
        """

        with self._sync_locker:
            res = self.custom_move_for(F=config.A_F_MAX, A=config.A_MAX)
            self.wait_for_all_actions_done()
            if res != self.RESPONSE_OK:
                return res

            res = self.custom_move_for(F=config.A_F_MAX, A=-(abs(config.A_MIN) + abs(config.A_MAX)))
            self.wait_for_all_actions_done()
            if res != self.RESPONSE_OK:
                return res

            return self.set_current_coordinates(A=config.A_MIN)

    def ext_do_extraction(self, F: int):
        raise NotImplemented("This code is need update.")

        """
        with self._sync_locker:
            if not self._check_arg_types([int], F):
                raise TypeError("incorrect force F value type")

            error_msg = self.validate_value(0, F, "F", config.Z_F_MIN, config.Z_F_MAX, "Z_F_MIN", "Z_F_MAX")
            if error_msg:
                return error_msg

            for dist in [str(-config.EXTRACTION_Z), str(config.EXTRACTION_Z)]:
                g_code = "G0 Z" + dist + " F" + str(config.Z_F_MAX)
                self._smc.write(g_code)
                response = self._smc.read_some()
                if response == self.RESPONSE_OK:
                    self._z_cur.value += config.EXTRACTION_Z
                else:
                    return response
            return response
        """

    def ext_align_cork_center(self, F: int):
        with self._sync_locker:
            if not self._check_arg_types([int], F):
                raise TypeError("incorrect force F value type")

            error_msg = self.validate_value(0, F, "F", config.XY_F_MIN, config.XY_F_MAX, "XY_F_MIN", "XY_F_MAX")
            if error_msg:
                return error_msg

            # calc cork center coords and xy movement values for smoothie g-code
            center_x, center_y = config.X_MAX / 2, config.Y_MAX / 2
            smc_x, smc_y = center_x - self._x_cur.value * config.XY_COEFFICIENT_TO_MM, center_y - self._y_cur.value * config.XY_COEFFICIENT_TO_MM
            g_code = "G0 X" + str(smc_x) + " Y" + str(smc_y) + " F" + str(F)

            self._smc.write(g_code)
            response = self._smc.read_some()
            if response == self.RESPONSE_OK:
                self._x_cur.value += smc_x / config.XY_COEFFICIENT_TO_MM
                self._y_cur.value += smc_y / config.XY_COEFFICIENT_TO_MM
            return response

    def ext_calibrate_cork(self):
        # TODO: what to do if calibration response is none?

        # Z axis calibration
        if config.USE_Z_AXIS_CALIBRATION:
            res = self._calibrate_axis(self._z_cur, "Z", config.Z_MIN, config.Z_MAX, config.Z_AXIS_CALIBRATION_TO_MAX)
            if res != self.RESPONSE_OK:
                raise RuntimeError("Couldn't pick up corkscrew, smoothie response:\n" + res)

        # X axis calibration
        if config.USE_X_AXIS_CALIBRATION:
            res = self._calibrate_axis(self._x_cur, "X", config.X_MIN, config.X_MAX, config.X_AXIS_CALIBRATION_TO_MAX)
            if res != self.RESPONSE_OK:
                raise RuntimeError("Couldn't calibrate X axis, smoothie response:\n" + res)

        # Y axis calibration
        if config.USE_Y_AXIS_CALIBRATION:
            res = self._calibrate_axis(self._y_cur, "Y", config.Y_MIN, config.Y_MAX, config.Y_AXIS_CALIBRATION_TO_MAX)
            if res != self.RESPONSE_OK:
                raise RuntimeError("Couldn't calibrate Y axis, smoothie response:\n" + res)

    def ext_cork_up(self):
        # cork up is done by Z axis calibration
        if config.USE_Z_AXIS_CALIBRATION:
            # TODO: stub (G28 isn't reading F value from smoothie config, it uses last received F)
            self._smc.write("G0 Z-0.1 F" + str(config.Z_F_EXTRACTION_UP))
            response = self._smc.read_some()
            if response != self.RESPONSE_OK:
                return response

            return self._calibrate_axis(self._z_cur, "Z", config.Z_MIN, config.Z_MAX, config.Z_AXIS_CALIBRATION_TO_MAX)
        else:
            raise RuntimeError("picking up corkscrew with stoppers usage requires Z axis calibration permission in config")

    def _calibrate_axis(self, axis_cur: multiprocessing.Value, axis_label, axis_min, axis_max, axis_calibration_to_max):
        # TODO: need to implement outer axix_cur var if removing multiprocessing.Value in future

        with self._sync_locker:
            # TODO: stub (G28 isn't reading F value from smoothie config, it uses last received F)
            if axis_label == "Z":
                self._smc.write("G0 Z-0.1 F" + str(config.Z_F_EXTRACTION_UP))
                response = self._smc.read_some()
                if response != self.RESPONSE_OK:
                    return response

            if axis_calibration_to_max:
                self._smc.write("G28 {0}{1}".format(axis_label, config.CALIBRATION_DISTANCE))
                response = self._smc.read_some()
                if response == self.RESPONSE_OK:
                    sm_val = axis_max
                    if axis_label in ["X", "Y"] and axis_max != 0:
                        axis_max /= config.XY_COEFFICIENT_TO_MM
                    axis_cur.value = axis_max
                else:
                    return response
            else:
                self._smc.write("G28 {0}{1}".format(axis_label, -config.CALIBRATION_DISTANCE))
                response = self._smc.read_some()
                if response == self.RESPONSE_OK:
                    sm_val = axis_min
                    if axis_label in ["X", "Y"] and axis_min != 0:
                        axis_min /= config.XY_COEFFICIENT_TO_MM
                    axis_cur.value = axis_min
                else:
                    return response

            # set fresh current coordinates on smoothie too
            self._smc.write("G92 {0}{1}".format(axis_label, sm_val))
            return self._smc.read_some()

    def _check_arg_types(self, types: list, *args):
        if len(args) < 1:
            raise TypeError("item(s) to check is missed")
        if type(types) is not list:
            raise TypeError("expected list of types, received " + str(type(types)))
        if len(types) < 1:
            raise ValueError("list of types should contain at least one item")

        for arg in args:
            if type(arg) not in types:
                return False
        return True

    @staticmethod
    def validate_value(cur_value, value, key_label, key_min, key_max, key_min_label, key_max_label):
        """Returns None if value is ok, error message otherwise.
        Receives smoothie values (may be not in mms)
        For force F current_value must be 0"""

        if cur_value + value > key_max:
            return "Value {0} for {1} goes beyond max acceptable range of {3} = {2}, as current value is {4}" \
                .format(value, key_label, key_max, key_max_label, cur_value)
        if cur_value + value < key_min:
            return "Value {0} for {1} goes beyond min acceptable range of {3} = {2}, as current value is {4}" \
                .format(value, key_label, key_min, key_min_label, cur_value)
        return None


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
            if config.APPLY_IMAGE_CROPPING:
                image = image[self._crop_h_from:self._crop_h_to, self._crop_w_from:self._crop_w_to]
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
            if config.APPLY_IMAGE_CROPPING:
                image = image[self._crop_h_from:self._crop_h_to, self._crop_w_from:self._crop_w_to]
            #image = cv.imread('test.jpg') #fake image for debug
            return image
        else:
            raise RuntimeError("Unable to open camera")


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


class GPSUbloxAdapter:
    """Provides access to the robot's on-board GPS navigator (UBLOX card)"""

    def __init__(self, ser_port, ser_baudrate, last_pos_count):
        if last_pos_count < 1:
            raise ValueError("last_pos_count shouldn't be less than 1")

        self._position_is_fresh = False
        self._last_pos_count = last_pos_count
        self._last_pos_container = []
        self._sync_locker = multiprocessing.RLock()

        self._serial = serial.Serial(port=ser_port, baudrate=ser_baudrate)
        #self._hot_reset()
        self._USBNMEA_OUT()

        self._keep_thread_alive = True
        self._reader_thread = threading.Thread(target=self._reader_thread_tf, daemon=True)
        self._reader_thread.start()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._keep_thread_alive = False
        self._serial.close()

    def disconnect(self):
        self._keep_thread_alive = False
        self._serial.close()

    def get_fresh_position(self):
        """Waits for new fresh position from gps and returns it, blocking until new position received.
        Returns copy of stored position (returned value can be safely changed with no worrying about obj reference
        features)"""

        while len(self._last_pos_container) < 1:
            pass
        with self._sync_locker:
            self._position_is_fresh = False
        while True:
            with self._sync_locker:
                if not self._position_is_fresh:
                    continue
            return self.get_last_position()

    def get_last_position(self):
        """Waits until at least one position is stored, returns last saved position copy at the moment of call
        (reference type safe)"""

        while len(self._last_pos_container) < 1:
            pass
        with self._sync_locker:
            position = self._last_pos_container[-1].copy()  # var may be need for context manager
            return position

    def get_last_positions_list(self):
        """Waits until at least one position is stored, returns list of last saved positions copies at the moment of
        call (reference type safe)"""

        positions = []
        while len(self._last_pos_container) < 1:
            pass
        with self._sync_locker:
            for position in self._last_pos_container:
                positions.append(position.copy())
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
            print("Ublox reading error:", ex)

    def _read_from_gps(self):
        """Returns GPS coordinates of the current position"""

        while True:
            data = str(self._serial.readline())
            # if len(data) == 3:
            #    print("None GNGGA or RTCM threads")
            if "GNGGA" in data and ",,," not in data:
                # bad string with no position data
                # print(data)  # debug
                data = data.split(",")
                try:
                    lati, longi = self._D2M2(data[2], data[3], data[4], data[5])
                except ValueError:
                    continue
                point_quality = data[6]
                return [lati, longi, point_quality]  # , float(data[11])  # alti

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

    #Start a Hot restart
    def _hot_reset(self):
        Mythread = "B5 62 06 04 04 00 00 00 02 00 10 68"
        self._serial.write(bytearray.fromhex(Mythread))
