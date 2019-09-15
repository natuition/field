#!/usr/bin/env python

import connectors
import multiprocessing
import time
from config import config
import cv2 as cv
import math


class SmoothieAdapter:

    def __init__(self, smoothie_host):
        self._sync_locker = multiprocessing.RLock()
        self._smc = connectors.SmoothieConnector(smoothie_host)
        self._x_cur = multiprocessing.Value("i", 0)
        self._y_cur = multiprocessing.Value("i", 0)
        self._z_cur = multiprocessing.Value("i", 0)
        self._a_cur = multiprocessing.Value("i", 0)
        self._b_cur = multiprocessing.Value("i", 0)
        self._c_cur = multiprocessing.Value("i", 0)
        self.switch_to_relative()
        self.ext_calibrate_cork()

    def get_connector(self):
        return self._smc

    def wait_for_all_actions_done(self):
        with self._sync_locker:
            self._smc.write("M400")
            self._smc.read_until("ok\r\n")

    def halt(self):
        with self._sync_locker:
            self._smc.write("M112")
            self._smc.read_until("ed to exit HALT state\r\n")

    def reset(self):
        with self._sync_locker:
            self._smc.write("M999")
            self._smc.read_until(" currently unknown\nok\n")

    def switch_to_relative(self):
        with self._sync_locker:
            self._smc.write("G91")
            self._smc.read_until("ok\r\n")

    def set_current_coordinates(self, X=None, Y=None, Z=None, A=None, B=None, C=None):
        if self._check_arg_types(type(None), X, Y, Z, A, B, C):
            raise TypeError("at least one axis shouldn't be None")
        if not self._check_arg_types([int, type(None)], X, Y, Z, A, B, C):
            raise TypeError("incorrect axis current value(s) type(s)")

        with self._sync_locker:
            g_code = "G92"
            if X is not None:
                g_code += " X" + str(X)
            if Y is not None:
                g_code += " Y" + str(Y)
            if Z is not None:
                g_code += " Z" + str(Z)
            if A is not None:
                g_code += " A" + str(A)
            if B is not None:
                g_code += " B" + str(B)
            if C is not None:
                g_code += " C" + str(C)

            self._smc.write(g_code)
            self._smc.read_until("ok\r\n")

            if X is not None:
                with self._x_cur.get_lock():
                    self._x_cur.value = X
            if Y is not None:
                with self._y_cur.get_lock():
                    self._y_cur.value = Y
            if Z is not None:
                with self._z_cur.get_lock():
                    self._z_cur.value = Z
            if A is not None:
                with self._a_cur.get_lock():
                    self._a_cur.value = A
            if B is not None:
                with self._b_cur.get_lock():
                    self._b_cur.value = B
            if C is not None:
                with self._c_cur.get_lock():
                    self._c_cur.value = C

    def get_adapter_current_coordinates(self):
        with self._sync_locker:
            return {
                "X": self._x_cur.value,
                "Y": self._y_cur.value,
                "Z": self._z_cur.value,
                "A": self._a_cur.value,
                "B": self._b_cur.value,
                "C": self._c_cur.value,
            }

    def get_smoothie_current_coordinates(self):
        with self._sync_locker:
            self._smc.write("M114.2")
            res = self._smc.read_some()  # unite 2 msg into 1

            """
            Answers:
            M114:   b'ok C: X:2.0240 Y:0.0000 Z:0.0000\r\n'
            M114.1  b'ok WCS: X:2.0250 Y:0.0000 Z:0.0000\r\n'
            M114.2  b'ok MCS: X:2.0250 Y:0.0000 Z:0.0000 A:0.0000 B:0.0000\r\n'
            M114.3  b'ok APOS: X:2.0250 Y:0.0000 Z:0.0000 A:0.0000 B:0.0000\r\n'
            M114.4  b'ok MP: X:2.0240 Y:0.0000 Z:0.0000 A:0.0000 B:0.0000\r\n'
            """

        raise NotImplementedError("This option is not available yet.")

    def sync_check_current_coordinates(self):
        # True if Value = smoothieware val, False otherwise

        raise NotImplementedError("This option is not available yet.")

    def custom_move_for(self, F: int, X=None, Y=None, Z=None, A=None, B=None, C=None):
        """Movement by some value(s)"""

        if self._check_arg_types(type(None), X, Y, Z, A, B, C):
            raise TypeError("at least one axis shouldn't be None")
        if not self._check_arg_types([int, type(None)], X, Y, Z, A, B, C):
            raise TypeError("incorrect axis coordinates value(s) type(s)")
        if not self._check_arg_types([int], F):
            raise TypeError("incorrect force F value type")

        with self._sync_locker:
            g_code = "G0"

            if X is not None:
                error_msg = self.validate_value(0, F, "F", config.XY_F_MIN, config.XY_F_MAX, "XY_F_MIN", "XY_F_MAX")
                if error_msg:
                    return error_msg
                error_msg = self.validate_value(self._x_cur.value, X, "X", config.X_MIN, config.X_MAX, "X_MIN", "X_MAX")
                if error_msg:
                    return error_msg
                g_code += " X" + str(X)

            if Y is not None:
                error_msg = self.validate_value(0, F, "F", config.XY_F_MIN, config.XY_F_MAX, "XY_F_MIN", "XY_F_MAX")
                if error_msg:
                    return error_msg
                error_msg = self.validate_value(self._y_cur.value, Y, "Y", config.Y_MIN, config.Y_MAX, "Y_MIN", "Y_MAX")
                if error_msg:
                    return error_msg
                g_code += " Y" + str(Y)

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

            self._smc.write(g_code)
            self._smc.read_until("ok\r\n")

            if X is not None:
                with self._x_cur.get_lock():
                    self._x_cur.value += X
            if Y is not None:
                with self._y_cur.get_lock():
                    self._y_cur.value += Y
            if Z is not None:
                with self._z_cur.get_lock():
                    self._z_cur.value += Z
            if A is not None:
                with self._a_cur.get_lock():
                    self._a_cur.value += A
            if B is not None:
                with self._b_cur.get_lock():
                    self._b_cur.value += B
            if C is not None:
                with self._c_cur.get_lock():
                    self._c_cur.value += C
        return None

    def custom_move_to(self, F: int, X=None, Y=None, Z=None, A=None, B=None, C=None):
        """Movement to the specified position"""

        if self._check_arg_types(type(None), X, Y, Z, A, B, C):
            raise TypeError("at least one axis shouldn't be None")
        if not self._check_arg_types([int, type(None)], X, Y, Z, A, B, C):
            raise TypeError("incorrect axis coordinates value(s) type(s)")
        if not self._check_arg_types([int], F):
            raise TypeError("incorrect force F value type")

        with self._sync_locker:
            g_code = "G0"

            if X is not None:
                error_msg = self.validate_value(0, F, "F", config.XY_F_MIN, config.XY_F_MAX, "XY_F_MIN", "XY_F_MAX")
                if error_msg:
                    return error_msg
                error_msg = self.validate_value(0, X, "X", config.X_MIN, config.X_MAX, "X_MIN", "X_MAX")
                if error_msg:
                    return error_msg
                smc_x = X - self._x_cur.value
                g_code += " X" + str(smc_x)

            if Y is not None:
                error_msg = self.validate_value(0, F, "F", config.XY_F_MIN, config.XY_F_MAX, "XY_F_MIN", "XY_F_MAX")
                if error_msg:
                    return error_msg
                error_msg = self.validate_value(0, Y, "Y", config.Y_MIN, config.Y_MAX, "Y_MIN", "Y_MAX")
                if error_msg:
                    return error_msg
                smc_y = Y - self._y_cur.value
                g_code += " Y" + str(smc_y)

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

            self._smc.write(g_code)
            self._smc.read_until("ok\r\n")

            if X is not None:
                with self._x_cur.get_lock():
                    self._x_cur.value += smc_x
            if Y is not None:
                with self._y_cur.get_lock():
                    self._y_cur.value += smc_y
            if Z is not None:
                with self._z_cur.get_lock():
                    self._z_cur.value += smc_z
            if A is not None:
                with self._a_cur.get_lock():
                    self._a_cur.value += smc_a
            if B is not None:
                with self._b_cur.get_lock():
                    self._b_cur.value += smc_b
            if C is not None:
                with self._c_cur.get_lock():
                    self._c_cur.value += smc_z
        return None

    def nav_move_forward(self, distance: int, F: int):
        with self._b_cur.get_lock():
            self._smc.write("G0 B{0} F{1}".format(distance, F))
            self._b_cur.value += distance
            self._smc.read_until("ok\r\n")

    def nav_move_backward(self, distance: int, F: int):

        raise NotImplementedError("This option is not available yet.")

    def nav_turn_wheels_for(self, value: int, F: int):
        with self._a_cur.get_lock():
            error_msg = self.validate_value(self._a_cur.value, value, "A", config.A_MIN, config.A_MAX, "A_MIN", "A_MAX")
            if error_msg:
                return error_msg
            error_msg = self.validate_value(0, F, "F", config.A_F_MIN, config.A_F_MAX, "A_F_MIN", "A_F_MAX")
            if error_msg:
                return error_msg

            self._smc.write("G0 A{0} F{1}".format(value, F))
            self._a_cur.value += value
            self._smc.read_until("ok\r\n")

    def nav_turn_wheels_to(self, destination: int, F: int):
        with self._a_cur.get_lock():
            error_msg = self.validate_value(0, destination, "A", config.A_MIN, config.A_MAX, "A_MIN", "A_MAX")
            if error_msg:
                return error_msg
            error_msg = self.validate_value(0, F, "F", config.A_F_MIN, config.A_F_MAX, "A_F_MIN", "A_F_MAX")
            if error_msg:
                return error_msg

            smc_a = destination - self._a_cur.value
            self._smc.write("G0 A{0} F{1}".format(smc_a, F))
            self._a_cur.value += smc_a
            self._smc.read_until("ok\r\n")

    def nav_turn_wheels_left_max(self, F: int):
        self.nav_turn_wheels_to(config.A_MIN, config.A_F_MIN)

    def nav_turn_wheels_right_max(self, F: int):
        self.nav_turn_wheels_to(config.A_MAX, config.A_F_MAX)

    def nav_align_wheels_center(self, F: int):
        self.nav_turn_wheels_to(config.NAV_TURN_WHEELS_CENTER, config.A_F_MAX)

    def ext_do_extraction(self, F: int):
        with self._z_cur.get_lock():
            for dist in [str(-config.EXTRACTION_Z), str(config.EXTRACTION_Z)]:
                g_code = "G0 Z" + dist + " F" + str(config.Z_F_MAX)
                self._smc.write(g_code)
                self._z_cur.value += config.EXTRACTION_Z
                self._smc.read_until("ok\r\n")

    def ext_align_cork_center(self, F: int):
        with self._x_cur.get_lock():
            with self._y_cur.get_lock():
                # calc cork center coords and xy movement values for smoothie g-code
                center_x, center_y = int(config.X_MAX / 2), int(config.Y_MAX / 2)
                smc_x, smc_y = center_x - self._x_cur.value, center_y - self._y_cur.value
                g_code = "G0 X" + str(smc_x) + " Y" + str(smc_y) + " F" + str(config.XY_F_MAX)
                self._smc.write(g_code)
                self._x_cur.value += smc_x
                self._y_cur.value += smc_y
                self._smc.read_until("ok\r\n")

    def ext_calibrate_cork(self):
        # X axis calibration
        if config.USE_X_AXIS_CALIBRATION:
            self._calibrate_axis(self._x_cur, "X", config.X_MIN, config.X_MAX, config.X_AXIS_CALIBRATION_TO_MAX)

        # Y axis calibration
        if config.USE_Y_AXIS_CALIBRATION:
            self._calibrate_axis(self._y_cur, "Y", config.Y_MIN, config.Y_MAX, config.Y_AXIS_CALIBRATION_TO_MAX)

        # Z axis calibration
        if config.USE_Z_AXIS_CALIBRATION:
            self._calibrate_axis(self._z_cur, "Z", config.Z_MIN, config.Z_MAX, config.Z_AXIS_CALIBRATION_TO_MAX)

    def _calibrate_axis(self, axis_cur: multiprocessing.Value, axis_label, axis_min, axis_max, axis_calibration_to_max):
        with axis_cur.get_lock():
            if axis_calibration_to_max:
                self._smc.write("G28 {0}{1}".format(axis_label, config.CALIBRATION_DISTANCE))
                self._smc.read_until("ok\r\n")
                axis_cur.value = axis_max - config.AFTER_CALIBRATION_AXIS_OFFSET
            else:
                self._smc.write("G28 {0}{1}".format(axis_label, -config.CALIBRATION_DISTANCE))
                self._smc.read_until("ok\r\n")
                axis_cur.value = axis_min + config.AFTER_CALIBRATION_AXIS_OFFSET

            # set fresh current coordinates on smoothie too
            self._smc.write("G92 {0}{1}".format(axis_label, axis_cur.value))
            self._smc.read_until("ok\r\n")

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

        For force F current_value must be 0"""

        if cur_value + value > key_max:
            return "Value {0} for {1} goes beyond max acceptable range of {3} = {2}, " \
                       .format(value, key_label, key_max, key_max_label)
        if cur_value + value < key_min:
            return "Value {0} for {1} goes beyond min acceptable range of {3} = {2}, " \
                       .format(value, key_label, key_min, key_min_label)
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

    def get_image(self):
        image = cv.cvtColor(next(self._gen).array, cv.COLOR_RGB2BGR)
        self._raw_capture.truncate(0)
        return image


class CompassAdapter:

    def __init__(self):
        import smbus
        self._register_a = 0  # Address of Configuration register A
        self._register_b = 0x01  # Address of configuration register B
        self._register_mode = 0x02  # Address of mode register
        self._x_axis_h = 0x03  # Address of X-axis MSB data register
        self._z_axis_h = 0x05  # Address of Z-axis MSB data register
        self._y_axis_h = 0x07  # Address of Y-axis MSB data register
        self._declination = -0.00669  # define declination angle of location where measurement going to be done
        self._bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards

        #write to Configuration Register A
        self._bus.write_byte_data(config.COMPASS_DEVICE_ADDRESS, self._register_a, 0x70)
        #Write to Configuration Register B for gain
        self._bus.write_byte_data(config.COMPASS_DEVICE_ADDRESS, self._register_b, 0xa0)
        #Write to mode Register for selecting mode
        self._bus.write_byte_data(config.COMPASS_DEVICE_ADDRESS, self._register_mode, 0)

    def _read_raw_data(self, address):
        # Read raw 16-bit value
        high = self._bus.read_byte_data(config.COMPASS_DEVICE_ADDRESS, address)
        low = self._bus.read_byte_data(config.COMPASS_DEVICE_ADDRESS, address + 1)

        # concatenate higher and lower value
        value = ((high << 8) | low)

        # get signed value from module
        return value - 65536 if value > 32768 else value

    def get_heading_angle(self):
        x = self._read_raw_data(self._x_axis_h)
        y = self._read_raw_data(self._y_axis_h)
        heading = math.atan2(y, x) + self._declination

        # Due to declination check for > 360 degree
        if heading > 2 * math.pi:
            heading -= 2 * math.pi
        # check for sign
        if heading < 0:
            heading += 2 * math.pi

        # convert into angle
        return int(heading * 180 / math.pi)
