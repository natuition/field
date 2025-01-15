import os
import subprocess
import signal
import cv2
import numpy as np
import fileinput
import pwd
import grp
import time
import sys
sys.path.append('../')
from config import config
from extraction import ExtractionManagerV3
import utility
import adapters
import importlib
import json


class CameraCalibration:

    def __init__(self):
        self.max_res_scene_center_y = None
        self.max_res_scene_center_x = None
        self.scene_center_x = None
        self.scene_center_y = None
        self.crop_w_from = config.CROP_W_FROM
        self.crop_w_to = config.CROP_W_TO
        self.crop_h_from = config.CROP_H_FROM
        self.crop_h_to = config.CROP_H_TO
        self.target_x = None
        self.target_y = None
        self.target_x_mm = None
        self.target_y_mm = None

    def focus_adjustment_step(self):
        os.system("sudo systemctl restart nvargus-daemon")
        self.__cam = self.__startLiveCam()

    def focus_adjustment_step_validate(self):
        os.killpg(os.getpgid(self.__cam.pid), signal.SIGINT)
        self.__cam.wait()
        os.system("sudo systemctl restart nvargus-daemon")

    def step_crop_picture(self):
        image_saver = utility.ImageSaver()
        with adapters.CameraAdapterIMX219_170(
                0,
                config.DEPLOYMENT_CAMERA_MAX_W,
                0,
                config.DEPLOYMENT_CAMERA_MAX_H,
                config.CV_ROTATE_CODE,
                config.ISP_DIGITAL_GAIN_RANGE_FROM,
                config.ISP_DIGITAL_GAIN_RANGE_TO,
                config.GAIN_RANGE_FROM,
                config.GAIN_RANGE_TO,
                config.EXPOSURE_TIME_RANGE_FROM / 5,
                config.EXPOSURE_TIME_RANGE_TO / 5,
                config.AE_LOCK,
                config.DEPLOYMENT_CAMERA_MAX_W,
                config.DEPLOYMENT_CAMERA_MAX_H,
                config.DEPLOYMENT_CAMERA_MAX_W,
                config.DEPLOYMENT_CAMERA_MAX_H,
                config.DEPLOYMENT_CAMERA_MIN_FRAMERATE,
                config.CAMERA_FLIP_METHOD) as camera:
            time.sleep(config.DELAY_BEFORE_2ND_SCAN)
            frame = camera.get_image()
            img_origine = frame.copy()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame = cv2.GaussianBlur(frame, (21, 21), cv2.BORDER_DEFAULT)
            all_circles = cv2.HoughCircles(
                frame, cv2.HOUGH_GRADIENT, 0.9, 2500, param1=30, param2=10, minRadius=1200, maxRadius=1300)
            all_circles_rounded = np.uint16(np.around(all_circles))
            print('I have found ' +
                  str(all_circles_rounded.shape[1]) + ' circles')
            if len(all_circles_rounded) == 1:
                self.max_res_scene_center_x = all_circles_rounded[0][0][0]
                self.max_res_scene_center_y = all_circles_rounded[0][0][1]
                self.set_crop_values()
                # circle_rad = all_circles_rounded[0][0][2]
            else:
                print("Warning we found multiple circles.")
            for i in all_circles_rounded[0, :]:
                cv2.circle(img_origine, (i[0], i[1]), i[2], (204, 0, 102), 3)

            image_saver.save_image(
                img_origine, "./", specific_name="scene_center")

    def offset_calibration_step_detect(self, image_location=os.getcwd()):
        finalMsg = (None, "")
        image_saver = utility.ImageSaver()
        with adapters.CameraAdapterIMX219_170(self.crop_w_from, self.crop_w_to, self.crop_h_from,
                                              self.crop_h_to, config.CV_ROTATE_CODE,
                                              config.ISP_DIGITAL_GAIN_RANGE_FROM,
                                              config.ISP_DIGITAL_GAIN_RANGE_TO,
                                              config.GAIN_RANGE_FROM, config.GAIN_RANGE_TO,
                                              config.EXPOSURE_TIME_RANGE_FROM / 5, config.EXPOSURE_TIME_RANGE_TO / 5,
                                              config.AE_LOCK, config.CAMERA_W, config.CAMERA_H, config.CAMERA_W,
                                              config.CAMERA_H, config.CAMERA_FRAMERATE,
                                              config.CAMERA_FLIP_METHOD) as camera:
            time.sleep(config.DELAY_BEFORE_2ND_SCAN)
            frame = camera.get_image()

        img_origine = frame.copy()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.GaussianBlur(frame, (21, 21), cv2.BORDER_DEFAULT)

        #all_circles = cv2.HoughCircles(frame, cv2.HOUGH_GRADIENT, 0.9, 2500, param1=30, param2=10, minRadius=25, maxRadius=50)
        all_circles = cv2.HoughCircles(frame, cv2.HOUGH_GRADIENT, 1, 10000, param1=30, param2=5, minRadius=25, maxRadius=40)
        all_circles_rounded = np.uint16(np.around(all_circles))
        print('I have found ' + str(all_circles_rounded.shape[1]) + ' circles')
        if len(all_circles_rounded) == 1:
            self.target_x = float(all_circles_rounded[0][0][0])
            self.target_y = float(all_circles_rounded[0][0][1])
            self.target_radius = all_circles_rounded[0][0][2]
        else:
            print("Warning we found multiple circles.")
        for i in all_circles_rounded[0, :]:
            cv2.circle(img_origine, (i[0], i[1]), i[2], (102, 0, 204), 2)
            cv2.circle(img_origine, (i[0], i[1]), 2, (102, 0, 204), 2)
            cv2.line(img_origine, (i[0], i[1]-i[2]), (i[0], i[1]+i[2]), (102, 0, 204), 1)
            cv2.line(img_origine, (i[0]-i[2], i[1]), (i[0]+i[2], i[1]), (102, 0, 204), 1)
            cv2.ellipse(img_origine, (config.SCENE_CENTER_X, config.SCENE_CENTER_Y), (
                config.UNDISTORTED_ZONE_RADIUS, config.UNDISTORTED_ZONE_RADIUS), 0, 270, 360, (204, 0, 102), 3)
            cv2.line(img_origine, (config.SCENE_CENTER_X, config.SCENE_CENTER_Y), (config.SCENE_CENTER_X +
                     config.UNDISTORTED_ZONE_RADIUS, config.SCENE_CENTER_Y), (204, 0, 102), 3)
            cv2.line(img_origine, (config.SCENE_CENTER_X, config.SCENE_CENTER_Y), (config.SCENE_CENTER_X,
                     config.SCENE_CENTER_Y - config.UNDISTORTED_ZONE_RADIUS), (204, 0, 102), 3)
        if ExtractionManagerV3.is_point_in_circle(self.target_x, self.target_y, config.SCENE_CENTER_X, config.SCENE_CENTER_Y, config.UNDISTORTED_ZONE_RADIUS) and self.target_x >= config.SCENE_CENTER_X and self.target_y <= config.SCENE_CENTER_Y:
            finalMsg = (True, "Target is in undistorted zone :")
        else:
            finalMsg = (False, "Target is not in undistorted zone :")

        image_saver.save_image(
            img_origine, image_location + "/", specific_name="target_detection")

        print(f"Save here : {os.getcwd()}")

        return finalMsg

    def set_targets(self, dir):
        with open(dir + "/target_coords.json") as f:
            targets = json.load(f)
        self.target_x = targets["x"]
        self.target_y = targets["y"]

    def go_to_coins(self, smoothie):
        x = float(abs((self.target_x - config.SCENE_CENTER_X) /
                      config.ONE_MM_IN_PX))
        y = float(abs((self.target_y - config.SCENE_CENTER_Y) /
                      config.ONE_MM_IN_PX))
        res = smoothie.custom_separate_xy_move_to(X_F=config.X_F_MAX,
                                                  Y_F=config.Y_F_MAX,
                                                  X=smoothie.smoothie_to_mm(
                                                      x, "X"),
                                                  Y=smoothie.smoothie_to_mm(y, "Y"))
        if res != smoothie.RESPONSE_OK:
            msg = "INIT: Failed to move camera, smoothie response:\n" + res
            print(msg)
            exit(1)
        smoothie.wait_for_all_actions_done()

        coords = smoothie.get_adapter_current_coordinates()
        self.target_x_mm = coords["X"]
        self.target_y_mm = coords["Y"]

    def offset_calibration_step_move(self, smoothie):
        if self.target_x and self.target_y:
            if ExtractionManagerV3.is_point_in_circle(self.target_x, self.target_y, config.SCENE_CENTER_X, config.SCENE_CENTER_Y, config.UNDISTORTED_ZONE_RADIUS):
                x = float(abs((self.target_x - config.SCENE_CENTER_X) /
                          config.ONE_MM_IN_PX)) + config.CORK_TO_CAMERA_DISTANCE_X
                y = float(abs((self.target_y - config.SCENE_CENTER_Y) /
                          config.ONE_MM_IN_PX)) + config.CORK_TO_CAMERA_DISTANCE_Y
                res = smoothie.custom_separate_xy_move_to(X_F=config.X_F_MAX,
                                                          Y_F=config.Y_F_MAX,
                                                          X=smoothie.smoothie_to_mm(
                                                              x, "X"),
                                                          Y=smoothie.smoothie_to_mm(y, "Y"))
                if res != smoothie.RESPONSE_OK:
                    msg = "INIT: Failed to move camera, smoothie response:\n" + res
                    print(msg)
                    exit(1)
                smoothie.wait_for_all_actions_done()

    def offset_calibration_step_move_with_distance(self, smoothie, cork_to_camera_distance_x, cork_to_camera_distance_y):
        if self.target_x and self.target_y:
            if ExtractionManagerV3.is_point_in_circle(self.target_x, self.target_y, config.SCENE_CENTER_X, config.SCENE_CENTER_Y, config.UNDISTORTED_ZONE_RADIUS):
                x = float(abs((self.target_x - config.SCENE_CENTER_X) /
                          config.ONE_MM_IN_PX)) + cork_to_camera_distance_x
                y = float(abs((self.target_y - config.SCENE_CENTER_Y) /
                          config.ONE_MM_IN_PX)) + cork_to_camera_distance_y
                res = smoothie.custom_separate_xy_move_to(X_F=config.X_F_MAX,
                                                          Y_F=config.Y_F_MAX,
                                                          X=smoothie.smoothie_to_mm(
                                                              x, "X"),
                                                          Y=smoothie.smoothie_to_mm(y, "Y"))
                if res != smoothie.RESPONSE_OK:
                    msg = "INIT: Failed to move camera, smoothie response:\n" + res
                    print(msg)
                    exit(1)
                smoothie.wait_for_all_actions_done()

    def offset_calibration_step_move_with_distance_from_coins(self, smoothie, cork_to_camera_distance_x, cork_to_camera_distance_y):
        if self.target_x_mm and self.target_y_mm:
            if ExtractionManagerV3.is_point_in_circle(self.target_x, self.target_y, config.SCENE_CENTER_X, config.SCENE_CENTER_Y, config.UNDISTORTED_ZONE_RADIUS):
                x = self.target_x_mm + cork_to_camera_distance_x
                y = self.target_y_mm + cork_to_camera_distance_y
                res = smoothie.custom_separate_xy_move_to(X_F=config.X_F_MAX,
                                                          Y_F=config.Y_F_MAX,
                                                          X=smoothie.smoothie_to_mm(
                                                              x, "X"),
                                                          Y=smoothie.smoothie_to_mm(y, "Y"))
                if res != smoothie.RESPONSE_OK:
                    msg = "INIT: Failed to move camera, smoothie response:\n" + res
                    print(msg)
                    exit(1)
                smoothie.wait_for_all_actions_done()

    def __startLiveCam(self):
        camSP = subprocess.Popen("python3 serveurCamLive.py False", stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                 stdin=subprocess.PIPE, cwd=os.getcwd().split("/deployement")[0], shell=True, preexec_fn=os.setsid)
        return camSP

    @staticmethod
    def __changeConfigValue(path: str, value):
        with fileinput.FileInput("../config/config.py", inplace=True, backup='.bak') as file:
            found_key = False

            for line in file:
                # skip comments
                if line.startswith("#"):
                    print(line, end='')
                    continue

                # if key name is strictly equal
                elements = line.split("=")
                if len(elements) > 0 and elements[0].strip() == path:
                    print(path + " = " + str(value), end='\n')
                    found_key = True
                else:
                    print(line, end='')

            # if key is absent - add it to the end of the file
            if not found_key:
                print(path + " = " + str(value), end='\n')

        uid = pwd.getpwnam("violette").pw_uid
        gid = grp.getgrnam("violette").gr_gid
        os.chown("../config/config.py", uid, gid)

    def set_crop_values(self):
        """Version with multiple resolutions support"""

        # convert scene center from calibration max res to working res
        if config.DEPLOYMENT_CAMERA_MAX_W > config.CAMERA_W:
            self.scene_center_x = int(
                self.max_res_scene_center_x - (config.DEPLOYMENT_CAMERA_MAX_W - config.CAMERA_W) / 2)
        elif config.DEPLOYMENT_CAMERA_MAX_W == config.CAMERA_W:
            self.scene_center_x = self.max_res_scene_center_x
        else:
            msg = "Working camera resolution 'config.CAMERA_W' is bigger than max camera resolution " \
                  "'config.DEPLOYMENT_CAMERA_MAX_W'"
            print(msg)
            raise ValueError(msg)
        if config.DEPLOYMENT_CAMERA_MAX_H > config.CAMERA_H:
            self.scene_center_y = int(
                self.max_res_scene_center_y - (config.DEPLOYMENT_CAMERA_MAX_H - config.CAMERA_H) / 2)
        elif config.DEPLOYMENT_CAMERA_MAX_H == config.CAMERA_H:
            self.scene_center_y = self.max_res_scene_center_y
        else:
            msg = "Working camera resolution 'config.CAMERA_H' is bigger than max camera resolution " \
                  "'config.DEPLOYMENT_CAMERA_MAX_H'"
            print(msg)
            raise ValueError(msg)

        # define cropping settings
        if config.APPLY_IMAGE_CROPPING:
            self.crop_w_from = self.scene_center_x - config.DEPLOYMENT_CROP_GRAB_LEFT_PX
            self.crop_w_to = self.scene_center_x + config.DEPLOYMENT_CROP_GRAB_RIGHT_PX
            self.crop_h_from = self.scene_center_y - config.DEPLOYMENT_CROP_GRAB_TOP_PX
            self.crop_h_to = self.scene_center_y + config.DEPLOYMENT_CROP_GRAB_BOT_PX
        else:
            self.crop_w_from = 0
            self.crop_w_to = config.CAMERA_W
            self.crop_h_from = 0
            self.crop_h_to = config.CAMERA_H

        # make sure crop values are in range of image size
        if config.APPLY_IMAGE_CROPPING:
            if self.crop_h_from < 0:
                self.crop_h_from = 0
            if self.crop_h_to > config.CAMERA_H:
                self.crop_h_to = config.CAMERA_H
            if self.crop_w_from < 0:
                self.crop_w_from = 0
            if self.crop_w_to > config.CAMERA_W:
                self.crop_w_to = config.CAMERA_W

        # convert scene center from working camera res to cropped res
        if config.APPLY_IMAGE_CROPPING:
            if self.crop_w_from > 0:
                self.scene_center_x -= self.crop_w_from
            if self.crop_h_from > 0:
                self.scene_center_y -= self.crop_h_from

        # re-compute working zone for new image sizes
        abs_poly_points = []
        for point in config.WORKING_ZONE_POLY_POINTS_REL:
            abs_poly_points.append([
                self.scene_center_x + point[0],
                self.scene_center_y + point[1]
            ])

        CameraCalibration.__changeConfigValue(
            "SCENE_CENTER_X", self.scene_center_x)
        CameraCalibration.__changeConfigValue(
            "SCENE_CENTER_Y", self.scene_center_y)
        CameraCalibration.__changeConfigValue(
            "WORKING_ZONE_POLY_POINTS", abs_poly_points)
        CameraCalibration.__changeConfigValue("CROP_W_FROM", self.crop_w_from)
        CameraCalibration.__changeConfigValue("CROP_W_TO", self.crop_w_to)
        CameraCalibration.__changeConfigValue("CROP_H_FROM", self.crop_h_from)
        CameraCalibration.__changeConfigValue("CROP_H_TO", self.crop_h_to)
        importlib.reload(config)


def main():
    os.system("sudo systemctl restart nvargus-daemon")
    cameraCalibration: CameraCalibration = CameraCalibration()
    res = cameraCalibration.offset_calibration_step_detect()
    with open(os.getcwd() + "/target_coords.json", 'w') as f:
        json.dump({"x": cameraCalibration.target_x,
                "y": cameraCalibration.target_y}, f)
    print(json.dumps({"result":res}))


if __name__ == "__main__":
    main()
