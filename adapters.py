#!/usr/bin/env python

import connectors
import multiprocessing
import time
from config import config
import cv2 as cv

if config.USE_PI_CAMERA:
    from picamera.array import PiRGBArray
    from picamera import PiCamera


class SmoothieAdapter:

    def __init__(self, smoothie_host):
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
        self._smc.write("M400")
        self._smc.read_until("ok\r\n")

    def halt(self):
        self._smc.write("M112")
        self._smc.read_until("ed to exit HALT state\r\n")

    def reset(self):
        self._smc.write("M999")
        self._smc.read_until(" currently unknown\nok\n")

    def switch_to_relative(self):
        self._smc.write("G91")
        self._smc.read_until("ok\r\n")

    def set_current_coordinates(self, X=None, Y=None, Z=None, A=None, B=None, C=None):
        raise NotImplementedError("This option is not available yet.")

    def get_current_coordinates(self):
        raise NotImplementedError("This option is not available yet.")

    def check_current_coordinates(self):
        # True if Value = smoothieware val, False otherwise
        raise NotImplementedError("This option is not available yet.")

    def custom_move_for(self, X=None, Y=None, Z=None, A=None, B=None, C=None):
        """Movement by some value(s)"""
        raise NotImplementedError("This option is not available yet.")

    def custom_move_to(self, X=None, Y=None, Z=None, A=None, B=None, C=None):
        """Movement to the specified position"""
        raise NotImplementedError("This option is not available yet.")

    def nav_move_forward(self, distance: int, F: int):
        with self._b_cur.getlock():
            self._smc.write("G0 B{0} F{1}".format(distance, F))
            self._b_cur.value += distance
            self._smc.read_until("ok\r\n")

    def nav_move_backward(self, distance: int, F: int):
        raise NotImplementedError("This option is not available yet.")

    def nav_turn_wheels_for(self, value: int, F: int):
        with self._a_cur.getlock():
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
        with self._a_cur.getlock():
            error_msg = self.validate_value(0, destination, "A", config.A_MIN, config.A_MAX, "A_MIN", "A_MAX")
            if error_msg:
                return error_msg
            error_msg = self.validate_value(0, F, "F", config.A_F_MIN, config.A_F_MAX, "A_F_MIN", "A_F_MAX")
            if error_msg:
                return error_msg

            smc_a = self._calc_coords_diff(self._a_cur.value, destination)
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
        with self._x_cur.getlock():
            with self._y_cur.getlock():
                # calc cork center coords and xy movement values for smoothie g-code
                center_x, center_y = int(config.X_MAX / 2), int(config.Y_MAX / 2)
                smc_x, smc_y = self._calc_coords_diff(self._x_cur.value, center_x),\
                               self._calc_coords_diff(self._y_cur.value, center_y)
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
            self._calibrate_axis(self._y_cur, "Y", config.Y_MIN, config.Y_MAX, config.X_AXIS_CALIBRATION_TO_MAX)

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

    def _check_if_all_nones(self, *args):
        for arg in args:
            if arg is not None:
                return False
        return True

    @staticmethod
    def validate_value(cur_value, value, key_label, key_min, key_max, key_min_label, key_max_label):
        """For F current_value must be 0"""

        if cur_value + value > key_max:
            return "Value {0} for {1} goes beyond max acceptable range of {3} = {2}, " \
                       .format(value, key_label, key_max, key_max_label)
        if cur_value + value < key_min:
            return "Value {0} for {1} goes beyond min acceptable range of {3} = {2}, " \
                       .format(value, key_label, key_min, key_min_label)
        return None

    def _calc_coords_diff(self, current: int, destination: int):
        """Returns pos. or neg. value for motion"""
        return -abs(current - destination) if current > destination else abs(current - destination)


class PiCameraAdapter:

    def __init__(self):
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
