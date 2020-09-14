"""
This script is for manual detector's config selection.
Loops over CONFIDENCE and NMS_TRESHOLD ranges, applies them to YOLO network and saves results for further inspection.
"""

import detection
from config import config
import os
import cv2 as cv
import glob
import platform
import numpy as np

# YOLO SETTINGS
# divided by 100. I.E. 25 means 0.25
CONFIDENCE_FROM = 20
CONFIDENCE_TO = 21
CONFIDENCE_STEP = 1
NMS_THRESHOLD_FROM = 40
NMS_THRESHOLD_TO = 41
NMS_THRESHOLD_STEP = 1
USE_PERIPHERY_NN = True
USE_PRECISE_NN = True

# PATHS SETTINGS
IMAGES_INPUT_DIR = "mds_input/"
IMAGES_OUTPUT_DIR = "mds_output/"

# OTHER SETTINGS
DRAW_WORK_ZONE = True
DRAW_UNDISTORTED_ZONE = True


def get_slash():
    return "\\" if platform.system() == "Windows" else "/"


def create_directories(*args):
    """Creates directories, receives any args count, each arg is separate dir"""

    for path in args:
        if not os.path.exists(path):
            try:
                os.mkdir(path)
            except OSError:
                print("Creation of the directory %s failed" % path)
            else:
                print("Successfully created the directory %s " % path)
        else:
            print("Directory %s is already exists" % path)


def draw_zone_circle(image, circle_center_x, circle_center_y, circle_radius):
    """Draws received circle on image. Used for drawing undistorted zone edges on photo"""

    return cv.circle(image, (circle_center_x, circle_center_y), circle_radius, (0, 0, 255), thickness=3)


def draw_zone_poly(image, np_poly_points):
    """Draws received polygon on image. Used for drawing working zone edges on photo"""

    return cv.polylines(image, [np_poly_points], isClosed=True, color=(0, 0, 255), thickness=5)


def detect_all_in_directory(detector: detection.YoloOpenCVDetection, poly_zone_points_cv, input_dir, output_dir):
    detected_cnt = 0
    paths_to_images = glob.glob(input_dir + "*.jpg")
    counter = 1
    for image_path in paths_to_images:
        print("Processing", counter, "of", len(paths_to_images), "images")
        counter += 1
        detected_cnt += detect_single(detector, poly_zone_points_cv, image_path, output_dir)
    return detected_cnt


def detect_single(detector: detection.YoloOpenCVDetection, poly_zone_points_cv, input_full_path, output_dir):
    img = cv.imread(input_full_path)
    boxes = detector.detect(img)

    if DRAW_UNDISTORTED_ZONE:
        img = draw_zone_circle(img, config.SCENE_CENTER_X, config.SCENE_CENTER_Y, config.UNDISTORTED_ZONE_RADIUS)

    if DRAW_WORK_ZONE:
        img = draw_zone_poly(img, poly_zone_points_cv)

    detection.draw_boxes(img, boxes)
    file_name = input_full_path.split(get_slash())[-1]
    cv.imwrite(output_dir + str(len(boxes)) + " " + file_name, img)
    return len(boxes)


def main():
    create_directories(IMAGES_OUTPUT_DIR)

    working_zone_points_cv = np.array(config.WORKING_ZONE_POLY_POINTS, np.int32).reshape((-1, 1, 2))

    # loop over settings ranges
    for confidence in range(CONFIDENCE_FROM, CONFIDENCE_TO, CONFIDENCE_STEP):
        confidence = round(confidence / 100, 2)
        for nms_threshold in range(NMS_THRESHOLD_FROM, NMS_THRESHOLD_TO, NMS_THRESHOLD_STEP):
            nms_threshold = round(nms_threshold / 100, 2)

            # create detector with current settings
            if USE_PERIPHERY_NN:
                periphery_det = detection.YoloOpenCVDetection(config.PERIPHERY_CLASSES_FILE,
                                                              config.PERIPHERY_CONFIG_FILE,
                                                              config.PERIPHERY_WEIGHTS_FILE,
                                                              config.PERIPHERY_INPUT_SIZE,
                                                              confidence,
                                                              nms_threshold,
                                                              config.PERIPHERY_DNN_BACKEND,
                                                              config.PERIPHERY_DNN_TARGET)

                # define cur path and create dir
                current_dir = IMAGES_OUTPUT_DIR + "periphery " + str(confidence) + " " + str(
                    nms_threshold) + get_slash()
                create_directories(current_dir)

                # detect and save images
                det_cnt = detect_all_in_directory(periphery_det, working_zone_points_cv, IMAGES_INPUT_DIR, current_dir)

                os.rename(current_dir,
                          current_dir[:-len(get_slash())] + " (detected " + str(det_cnt) + ")" + get_slash())

            if USE_PRECISE_NN:
                precise_det = detection.YoloOpenCVDetection(config.PRECISE_CLASSES_FILE,
                                                            config.PRECISE_CONFIG_FILE,
                                                            config.PRECISE_WEIGHTS_FILE,
                                                            config.PRECISE_INPUT_SIZE,
                                                            confidence,
                                                            nms_threshold,
                                                            config.PRECISE_DNN_BACKEND,
                                                            config.PRECISE_DNN_TARGET)

                # define cur path and create dir
                current_dir = IMAGES_OUTPUT_DIR + "precise " + str(confidence) + " " + str(nms_threshold) + get_slash()
                create_directories(current_dir)

                # detect and save images
                det_cnt = detect_all_in_directory(precise_det, working_zone_points_cv, IMAGES_INPUT_DIR, current_dir)

                os.rename(current_dir,
                          current_dir[:-len(get_slash())] + " (detected " + str(det_cnt) + ")" + get_slash())


if __name__ == '__main__':
    main()
