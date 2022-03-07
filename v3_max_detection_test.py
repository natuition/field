from config import config
import adapters
import cv2 as cv
import time
import utility
import detection
import pwd
import grp
import os

OUTPUT_DIR = "./test_max_detect"

def test_making(camera, detector, image_saver):
    uid = pwd.getpwnam("violette").pw_uid
    gid = grp.getgrnam("violette").gr_gid
    try:
        max_detect = 0
        while True:
            frame = camera.get_image()
            plants_boxes = detector.detect(frame, True)
            if len(plants_boxes) > max_detect:
                frameFinal = detection.draw_boxes(frame, plants_boxes)
                max_detect = len(plants_boxes)
                image_name = f"max_detect_{max_detect}"
                image_saver.save_image(frameFinal, OUTPUT_DIR+utility.get_path_slash(), specific_name=image_name)
                os.chown(OUTPUT_DIR+utility.get_path_slash()+image_name+".jpg", uid, gid)
                print(f"Max detection : {max_detect}.")

    except KeyboardInterrupt:
        print("Exiting...")
        return


def main():

    print("Loading...")
    utility.create_directories(OUTPUT_DIR)
    image_saver = utility.ImageSaver()
    detector = detection.YoloTRTDetector(config.PERIPHERY_MODEL_PATH, config.PERIPHERY_CONFIDENCE_THRESHOLD, config.PERIPHERY_NMS_THRESHOLD)
    with adapters.CameraAdapterIMX219_170(config.CROP_W_FROM, config.CROP_W_TO, config.CROP_H_FROM,
                                          config.CROP_H_TO, config.CV_ROTATE_CODE,
                                          config.ISP_DIGITAL_GAIN_RANGE_FROM, config.ISP_DIGITAL_GAIN_RANGE_TO,
                                          config.GAIN_RANGE_FROM, config.GAIN_RANGE_TO,
                                          config.EXPOSURE_TIME_RANGE_FROM, config.EXPOSURE_TIME_RANGE_TO,
                                          config.AE_LOCK, config.CAMERA_W, config.CAMERA_H, config.CAMERA_W,
                                          config.CAMERA_H, config.CAMERA_FRAMERATE, config.CAMERA_FLIP_METHOD) \
            as camera:

        time.sleep(2)

        print("Loading complete.")

        mode = input("Type y to start max detection test (y/n), other for exit: ")
        if mode.lower() == "y":
            test_making(camera, detector, image_saver)
        else:
            print("Exiting...")


if __name__ == "__main__":
    main()
