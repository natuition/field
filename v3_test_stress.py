"""
Script for energy stress test.
Provides some combination of parallel hardware usage.
Modes:
1 = X
2 = X, Y
3 = X, Y, A
4 = X, Y, A, vesc forward
5 = X, Y, A, vesc forward, camera reading
6 = X, Y, A, vesc forward, camera reading, detection
7 = X, Y, A, vesc forward, camera reading, detection, gps
"""

from config import config
import threading
import adapters
import detection
import cv2 as cv
import traceback
import time

# smoothie movements values
# need to set such values that will make all axes to stop at the roughly same time
# should be positives
X = 10
Y = 10
A = 10
F = 2000  # used for all axes

# don't change this
KEEP_WORKING = True


def __th_vesc():
    print("Loading vesc...")
    with adapters.VescAdapter(config.VESC_RPM_SLOW, float("inf"), config.VESC_ALIVE_FREQ,
                              config.VESC_CHECK_FREQ, config.VESC_PORT, config.VESC_BAUDRATE) as vesc_engine:
        print("Starting vesc...")
        vesc_engine.start_moving()
        while KEEP_WORKING:
            time.sleep(0.3)
        vesc_engine.stop_moving()
    print("Vesc is stopped correctly.")


def __th_camera():
    print("Loading camera...")
    with adapters.CameraAdapterIMX219_170(config.CROP_W_FROM, config.CROP_W_TO, config.CROP_H_FROM,
                                          config.CROP_H_TO, config.CV_ROTATE_CODE,
                                          config.ISP_DIGITAL_GAIN_RANGE_FROM, config.ISP_DIGITAL_GAIN_RANGE_TO,
                                          config.GAIN_RANGE_FROM, config.GAIN_RANGE_TO,
                                          config.EXPOSURE_TIME_RANGE_FROM, config.EXPOSURE_TIME_RANGE_TO,
                                          config.AE_LOCK, config.CAMERA_W, config.CAMERA_H, config.CAMERA_W,
                                          config.CAMERA_H, config.CAMERA_FRAMERATE,
                                          config.CAMERA_FLIP_METHOD) as camera:
        print("Starting reading from camera...")
        while KEEP_WORKING:
            frame = camera.get_image()
    print("Camera is stopped correctly.")


def __th_detection():
    print("Loading image...")
    img = cv.imread("1.jpg")

    print("Loading periphery detector...")
    periphery_detector = detection.YoloOpenCVDetection(config.PERIPHERY_CLASSES_FILE, config.PERIPHERY_CONFIG_FILE,
                                                       config.PERIPHERY_WEIGHTS_FILE, config.PERIPHERY_INPUT_SIZE,
                                                       config.PERIPHERY_CONFIDENCE_THRESHOLD,
                                                       config.PERIPHERY_NMS_THRESHOLD, config.PERIPHERY_DNN_BACKEND,
                                                       config.PERIPHERY_DNN_TARGET)
    print("Loading precise detector...")
    precise_detector = detection.YoloOpenCVDetection(config.PRECISE_CLASSES_FILE, config.PRECISE_CONFIG_FILE,
                                                     config.PRECISE_WEIGHTS_FILE, config.PRECISE_INPUT_SIZE,
                                                     config.PRECISE_CONFIDENCE_THRESHOLD,
                                                     config.PRECISE_NMS_THRESHOLD, config.PRECISE_DNN_BACKEND,
                                                     config.PRECISE_DNN_TARGET)
    print("Starting detections...")
    while KEEP_WORKING:
        per_boxes = periphery_detector.detect(img)
        pre_boxes = precise_detector.detect(img)
    print("Detectors are stopped correctly.")


def __th_gps():
    print("Loading gps...")
    with adapters.GPSUbloxAdapter(config.GPS_PORT, config.GPS_BAUDRATE, config.GPS_POSITIONS_TO_KEEP) as gps:
        print("Starting reading from gps...")
        show_ok_msg = True
        while KEEP_WORKING:
            cur_pos = gps.get_last_position()
            if show_ok_msg:
                print("Point from GPS is received, seems like GPS is ok, keep reading.")
                show_ok_msg = False
    print("Gps is stopped correctly.")


def main():
    mode = int(input("""
        1 = X
        2 = X, Y
        3 = X, Y, A
        4 = X, Y, A, vesc forward
        5 = X, Y, A, vesc forward, camera reading
        6 = X, Y, A, vesc forward, camera reading, detection
        7 = X, Y, A, vesc forward, camera reading, detection, gps
        """))

    # check settings and mode
    if mode not in range(1, 8):
        print("Wrong mode. Exiting.")
        exit(1)
    if X < 0 or Y < 0 or A < 0 or F < 0:
        print("Bad value detected in settings (at the script beginning). Exiting.")
        exit(1)

    try:
        print("Loading smoothie...")
        with adapters.SmoothieAdapter(config.SMOOTHIE_HOST) as smoothie:
            # start threads if needed
            threads = []
            if mode > 3:
                print("Loading threads...")
                threads.append(threading.Thread(target=__th_vesc, daemon=False))

                if mode > 4:
                    threads.append(threading.Thread(target=__th_camera, daemon=False))
                if mode > 5:
                    threads.append(threading.Thread(target=__th_detection, daemon=False))
                if mode > 6:
                    threads.append(threading.Thread(target=__th_gps, daemon=False))

                print("Starting threads...")
                for thread in threads:
                    thread.start()

            # start smoothie
            print("Starting smoothie movement...")
            res = smoothie.ext_align_cork_center(F=config.XY_F_MAX)
            smoothie.wait_for_all_actions_done()
            if res != smoothie.RESPONSE_OK:
                print("Smoothie cork center aligning was failed:\n", res)

            movement_positive = True
            while True:
                if movement_positive:
                    movement_positive = False
                    if mode == 1:
                        res = smoothie.custom_move_for(F=F, X=X)
                    if mode == 2:
                        res = smoothie.custom_move_for(F=F, X=X, Y=Y)
                    if mode > 2:
                        res = smoothie.custom_move_for(F=F, X=X, Y=Y, A=A)
                else:
                    movement_positive = True
                    if mode == 1:
                        res = smoothie.custom_move_for(F=F, X=-X)
                    if mode == 2:
                        res = smoothie.custom_move_for(F=F, X=-X, Y=-Y)
                    if mode > 2:
                        res = smoothie.custom_move_for(F=F, X=-X, Y=-Y, A=-A)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    print("Smoothie movement failed:\n", res)
    except KeyboardInterrupt:
        print("Stop request received.")
    except:
        print("Unexpected exception occurred:")
        print(traceback.format_exc())
    finally:
        print("Stopping everything, please wait for correct shutdown...")

        global KEEP_WORKING
        KEEP_WORKING = False

        print("Halting smoothie...")
        smoothie.halt()
        smoothie.reset()

        print("Waiting for threads...")
        for thread in threads:
            thread.join()

        print("Done.")


if __name__ == '__main__':
    main()
