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
import configDeployment
import utility
import adapters

class CameraCalibration:

    def __init__(self):
        self.crop_w_from ,self.crop_w_to, self.crop_h_from, self.crop_h_to =  config.CROP_W_FROM, config.CROP_W_TO, config.CROP_H_FROM, config.CROP_H_TO

    def focus_adjustment_step(self):
        os.system("sudo systemctl restart nvargus-daemon")
        self.__cam = self.__startLiveCam()

    def focus_adjustment_step_validate(self):
        os.killpg(os.getpgid(self.__cam.pid), signal.SIGINT)
        self.__cam.wait()
        os.system("sudo systemctl restart nvargus-daemon")

    def step_crop_picture(self):
        image_saver = utility.ImageSaver()
        CAMERA_W = 3264
        CAMERA_H = 2464
        CAMERA_FRAMERATE = 16
        with    adapters.CameraAdapterIMX219_170(  0, CAMERA_W, 0, CAMERA_H,
                                                    config.CV_ROTATE_CODE,
                                                    config.ISP_DIGITAL_GAIN_RANGE_FROM,
                                                    config.ISP_DIGITAL_GAIN_RANGE_TO,
                                                    config.GAIN_RANGE_FROM, config.GAIN_RANGE_TO,
                                                    config.EXPOSURE_TIME_RANGE_FROM/5, config.EXPOSURE_TIME_RANGE_TO/5,
                                                    config.AE_LOCK, 
                                                    CAMERA_W, CAMERA_H, CAMERA_W, CAMERA_H, CAMERA_FRAMERATE,
                                                    config.CAMERA_FLIP_METHOD) as camera:
            time.sleep(config.DELAY_BEFORE_2ND_SCAN)
            frame = camera.get_image()
            img_origine = frame.copy()
            frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            frame = cv2.GaussianBlur(frame, (21,21), cv2.BORDER_DEFAULT)
            all_circles = cv2.HoughCircles(frame,cv2.HOUGH_GRADIENT,0.9, 2500, param1 = 30, param2 = 10, minRadius = 1200, maxRadius = 1300)
            all_circles_rounded = np.uint16(np.around(all_circles))
            print('I have found ' + str(all_circles_rounded.shape[1]) + ' circles')
            if len(all_circles_rounded) == 1:
                self.scene_center_x = all_circles_rounded[0][0][0]
                self.scene_center_y = all_circles_rounded[0][0][1]
                self.set_crop_values()
                #circle_rad = all_circles_rounded[0][0][2]
            else:
                print("Warning we found multiple circles.")
            for i in all_circles_rounded[0, :]:
                cv2.circle(img_origine, (i[0],i[1]),i[2],(255,0,255),3)

            image_saver.save_image(img_origine, "./", specific_name="scene_center")

    def offset_calibration_step_detect(self):
        image_saver = utility.ImageSaver()
        with adapters.CameraAdapterIMX219_170(  self.crop_w_from ,self.crop_w_to, self.crop_h_from, 
                                                self.crop_h_to, config.CV_ROTATE_CODE,
                                                config.ISP_DIGITAL_GAIN_RANGE_FROM,
                                                config.ISP_DIGITAL_GAIN_RANGE_TO,
                                                config.GAIN_RANGE_FROM, config.GAIN_RANGE_TO,
                                                config.EXPOSURE_TIME_RANGE_FROM, config.EXPOSURE_TIME_RANGE_TO,
                                                config.AE_LOCK, config.CAMERA_W, config.CAMERA_H, config.CAMERA_W,
                                                config.CAMERA_H, config.CAMERA_FRAMERATE,
                                                config.CAMERA_FLIP_METHOD) as camera, \
             adapters.SmoothieAdapter(self.__get_smoothie_vesc_addresses()) as smoothie:

            time.sleep(config.DELAY_BEFORE_2ND_SCAN)
            
            frame = camera.get_image()
            img_origine = frame.copy()
            frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            frame = cv2.GaussianBlur(frame, (21,21), cv2.BORDER_DEFAULT)

            all_circles = cv2.HoughCircles(frame,cv2.HOUGH_GRADIENT,0.9, 2500, param1 = 30, param2 = 10, minRadius = 50, maxRadius = 70)
            all_circles_rounded = np.uint16(np.around(all_circles))
            print('I have found ' + str(all_circles_rounded.shape[1]) + ' circles')
            if len(all_circles_rounded) == 1:
                self.target_x = float(all_circles_rounded[0][0][0])
                self.target_y = float(all_circles_rounded[0][0][1])
                self.target_radius = all_circles_rounded[0][0][2]
            else:
                print("Warning we found multiple circles.")
            for i in all_circles_rounded[0, :]:
                cv2.circle(img_origine, (i[0],i[1]),i[2],(255,0,255),3)
                cv2.circle(img_origine, (i[0],i[1]),2,(255,0,255),3)

            if ExtractionManagerV3.is_point_in_circle(self.target_x, self.target_y, config.SCENE_CENTER_X, config.SCENE_CENTER_Y, config.UNDISTORTED_ZONE_RADIUS):
                finalMsg = "Target is in undistorted zone go on top :"
            else:
                finalMsg = "Target isn't in undistorted zone :"

            image_saver.save_image(img_origine, "./", specific_name="target_detection")

            return finalMsg
    
    def offset_calibration_step_move():
        with adapters.SmoothieAdapter(self.__get_smoothie_vesc_addresses()) as smoothie:
            if ExtractionManagerV3.is_point_in_circle(self.target_x, self.target_y, config.SCENE_CENTER_X, config.SCENE_CENTER_Y, config.UNDISTORTED_ZONE_RADIUS):
                x = float(abs((self.target_x-config.SCENE_CENTER_X)/config.ONE_MM_IN_PX)) + config.CORK_TO_CAMERA_DISTANCE_X
                y = float(abs((self.target_y-config.SCENE_CENTER_Y)/config.ONE_MM_IN_PX)) + config.CORK_TO_CAMERA_DISTANCE_Y
                res = smoothie.custom_separate_xy_move_to(  X_F=config.X_F_MAX,
                                                            Y_F=config.Y_F_MAX,
                                                            X=smoothie.smoothie_to_mm(x, "X"),
                                                            Y=smoothie.smoothie_to_mm(y, "Y"))
                if res != smoothie.RESPONSE_OK:
                    msg = "INIT: Failed to move camera, smoothie response:\n" + res
                    print(msg)
                    exit(1)
                smoothie.wait_for_all_actions_done()

    def __startLiveCam(self):
        camSP = subprocess.Popen("python3 serveurCamLive.py False", stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.PIPE, cwd=os.getcwd().split("/deployement")[0], shell=True, preexec_fn=os.setsid)
        return camSP

    def __get_smoothie_vesc_addresses(self):
        smoothie_vesc_addr = utility.get_smoothie_vesc_addresses()
        if config.SMOOTHIE_BACKEND == 1:
            smoothie_address = config.SMOOTHIE_HOST
        else:
            if "smoothie" in smoothie_vesc_addr:
                smoothie_address = smoothie_vesc_addr["smoothie"]
            else:
                msg = "Couldn't get smoothie's USB address!"
                print(msg)
                exit(1)
        return smoothie_address

    @staticmethod
    def __changeConfigValue(path: str, value):
        with fileinput.FileInput("../config/config.py", inplace=True, backup='.bak') as file:
            for line in file:
                if path in line:
                    print(path + " = " + str(value), end='\n')
                else:
                    print(line, end='')
        uid = pwd.getpwnam("violette").pw_uid
        gid = grp.getgrnam("violette").gr_gid
        os.chown("../config/config.py", uid, gid)

    def set_crop_values(self):
        rectX = self.scene_center_x - configDeployment.CAMERA_DISPLAY_W/2 - configDeployment.OFFSET_SCENE_CENTER_CENTER_CROP_VALUE_W
        rectY = self.scene_center_y - configDeployment.CAMERA_DISPLAY_H/2 - configDeployment.OFFSET_SCENE_CENTER_CENTER_CROP_VALUE_H
        self.crop_w_from = int(rectX)
        self.crop_w_to = int(self.crop_w_from + configDeployment.CAMERA_DISPLAY_W)
        self.crop_h_from = int(rectY)
        self.crop_h_to = int(self.crop_h_from + configDeployment.CAMERA_DISPLAY_H)
        CameraCalibration.__changeConfigValue("CROP_W_FROM",self.crop_w_from)
        CameraCalibration.__changeConfigValue("CROP_W_TO",self.crop_w_to)
        CameraCalibration.__changeConfigValue("CROP_H_FROM",self.crop_h_from)
        CameraCalibration.__changeConfigValue("CROP_H_TO",self.crop_h_to)


def main():
    cameraCalibration: CameraCalibration = CameraCalibration()
    cameraCalibration.focus_adjustment_step()
    test_continue = input("Press enter to continue to the next step, type anything to exit.")
    if test_continue != "":
        return
    cameraCalibration.focus_adjustment_step_validate()
    cameraCalibration.step_crop_picture()
    test_continue = input("Press enter to continue to the next step, type anything to exit.")
    if test_continue != "":
        return
    cameraCalibration.offset_calibration_step()

if __name__ == "__main__":
    main()