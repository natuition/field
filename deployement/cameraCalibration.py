import os
import subprocess
import signal
import cv2
import numpy as np

import sys
sys.path.append('../')
from config import config
import utility
import adapters

class CameraCalibration:

    def __init__(self):
        pass

    def step1(self):
        os.system("sudo systemctl restart nvargus-daemon")
        self.__cam = self.__startLiveCam()

    def step1_validate(self):
        self.__cam.send_signal(signal.SIGINT)
        self.__cam.send_signal(signal.SIGINT)
        self.__cam.wait()
        os.system("sudo systemctl restart nvargus-daemon")

    def step2(self):
        """with adapters.SmoothieAdapter(self.__get_smoothie_vesc_addresses()) as smoothie:
            res = smoothie.custom_separate_xy_move_to(  X_F=config.X_F_MAX,
                                                        Y_F=config.Y_F_MAX,
                                                        X=smoothie.smoothie_to_mm((config.X_MAX - config.X_MIN) / 2, "X"),
                                                        Y=smoothie.smoothie_to_mm(config.Y_MIN, "Y"))
            if res != smoothie.RESPONSE_OK:
                msg = "INIT: Failed to move camera to Y min, smoothie response:\n" + res
                print(msg)
                exit(1)
            smoothie.wait_for_all_actions_done()"""
        image_saver = utility.ImageSaver()
        CAMERA_W = 3280
        CAMERA_H = 2464
        with adapters.CameraAdapterIMX219_170(  0, CAMERA_W, 0, CAMERA_H,
                                                config.CV_ROTATE_CODE,
                                                config.ISP_DIGITAL_GAIN_RANGE_FROM,
                                                config.ISP_DIGITAL_GAIN_RANGE_TO,
                                                config.GAIN_RANGE_FROM, config.GAIN_RANGE_TO,
                                                config.EXPOSURE_TIME_RANGE_FROM, config.EXPOSURE_TIME_RANGE_TO,
                                                config.AE_LOCK, 
                                                CAMERA_W, CAMERA_H, CAMERA_W, CAMERA_H,
                                                config.CAMERA_FRAMERATE,
                                                config.CAMERA_FLIP_METHOD) as camera:
            frame = camera.get_image()
            img_origine = frame.copy()
            frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            all_circles = cv2.HoughCircles(frame,cv2.HOUGH_GRADIENT,0.9, 2500, param1 = 30, param2 = 10, minRadius = 1200, maxRadius = 1300)
            all_circles_rounded = np.uint16(np.around(all_circles))
            print('I have found ' + str(all_circles_rounded.shape[1]) + ' circles')
            if len(all_circles_rounded) == 1:
                scene_center_x = all_circles_rounded[0][0][0]
                scene_center_y = all_circles_rounded[0][0][1]
                circle_rad = all_circles_rounded[0][0][2]
            else:
                print("Warning we found multiple circles.")
            for i in all_circles_rounded[0, :]:
                cv2.circle(img_origine, (i[0],i[1]),i[2],(50,200,200),5)

            image_saver.save_image(img_origine, "./", label="scene_center")

    def __startLiveCam(self):
        camSP = subprocess.Popen(["python3","serveurCamLive.py"], cwd=os.getcwd().split("/uiWebRobot")[0])
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


if __name__ == "__main__":
    cameraCalibration: CameraCalibration = CameraCalibration()
    cameraCalibration.step2()