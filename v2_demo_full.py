from config import config
import detection
import adapters
import time
import math
import os
import cv2 as cv
import logging
import glob
import datetime
import numpy as np
from matplotlib.patches import Polygon

# for drawing
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection

# paths
# LOG_DIR = "log/" + str(str(datetime.datetime.now()).split(".")[:-1])[2:-2].replace(":", "-") + "/"
LOG_DIR = "log/"
LOG_FILE = "v2_demo_full.log"

# distance between camera and cork, mm
CORK_CAMERA_DISTANCE = 57

# circle zones
WORKING_ZONE_RADIUS = 700
UNDISTORTED_ZONE_RADIUS = 240

# square zones
WORKING_ZONE_X_MIN = 325
WORKING_ZONE_X_MAX = 2057
WORKING_ZONE_Y_MIN = 1040
WORKING_ZONE_Y_MAX = 1400

# polygon working zone
# these points are raw pixel coordinates of polygon which defines robot physical working area
WORKING_ZONE_POLY_POINTS = [[387, 618], [504, 553], [602, 506], [708, 469], [842, 434], [1021, 407], [1228, 410],
                            [1435, 443], [1587, 492], [1726, 558], [1867, 637], [1881, 675], [1919, 795], [1942, 926],
                            [1954, 1055], [1953, 1176], [1551, 1187], [1145, 1190], [724, 1190], [454, 1188],
                            [286, 1188], [283, 1082], [296, 979], [318, 874], [351, 753]]

# logging settings
logging.basicConfig(format='%(asctime)s > %(module)s.%(funcName)s %(levelname)s: %(message)s (line %(lineno)d)',
                    datefmt='%I:%M:%S %p',
                    filename=LOG_DIR + LOG_FILE,
                    filemode='w',
                    level=logging.DEBUG)

# these control points are used to calculate approximate coordinates for camera movement so that the plant is in the
# undistorted zone
# x_px, y_px, x_mm, y_mm, point № (to find necessary point in jpg schematic)
IMAGE_CONTROL_POINTS_MAP = [
    [1148, 1042, 0, 20, 0],
    [996, 1042, -20, 20, 1],
    [851, 1046, -40, 20, 2],
    [725, 1054, -60, 20, 3],
    [617, 1061, -80, 20, 4],
    [529, 1069, -100, 20, 5],
    [459, 1077, -120, 20, 6],
    [400, 1085, -140, 20, 7],
    [357, 1091, -160, 20, 8],
    [321, 1097, -180, 20, 9],
    [1146, 897, 0, 40, 10],
    [999, 899, -20, 40, 11],
    [859, 905, -40, 40, 12],
    [734, 919, -60, 40, 13],
    [629, 934, -80, 40, 14],
    [540, 950, -100, 40, 15],
    [470, 964, -120, 40, 16],
    [409, 980, -140, 40, 17],
    [363, 994, -160, 40, 18],
    [329, 1005, -180, 40, 19],
    [1146, 767, 0, 60, 20],
    [1006, 770, -20, 60, 21],
    [872, 778, -40, 60, 22],
    [750, 796, -60, 60, 23],
    [647, 817, -80, 60, 24],
    [558, 839, -100, 60, 25],
    [487, 861, -120, 60, 26],
    [426, 880, -140, 60, 27],
    [379, 901, -160, 60, 28],
    [343, 919, -180, 60, 29],
    [1146, 658, 0, 80, 30],
    [1014, 661, -20, 80, 31],
    [887, 671, -40, 80, 32],
    [770, 691, -60, 80, 33],
    [669, 714, -80, 80, 34],
    [583, 739, -100, 80, 35],
    [511, 765, -120, 80, 36],
    [448, 788, -140, 80, 37],
    [397, 815, -160, 80, 38],
    [361, 837, -180, 80, 39],
    [1145, 567, 0, 100, 40],
    [1022, 571, -20, 100, 41],
    [904, 581, -40, 100, 42],
    [791, 601, -60, 100, 43],
    [694, 626, -80, 100, 44],
    [608, 653, -100, 100, 45],
    [538, 681, -120, 100, 46],
    [473, 707, -140, 100, 47],
    [421, 734, -160, 100, 48],
    [381, 760, -180, 100, 49],
    [1144, 496, 0, 120, 50],
    [1030, 499, -20, 120, 51],
    [919, 509, -40, 120, 52],
    [814, 528, -60, 120, 53],
    [720, 551, -80, 120, 54],
    [637, 577, -100, 120, 55],
    [566, 607, -120, 120, 56],
    [500, 634, -140, 120, 57],
    [441, 667, -160, 120, 58],
    [403, 693, -180, 120, 59],
    [1144, 439, 0, 140, 60],
    [1038, 441, -20, 140, 61],
    [935, 449, -40, 140, 62],
    [835, 468, -60, 140, 63],
    [746, 489, -80, 140, 64],
    [664, 517, -100, 140, 65],
    [593, 546, -120, 140, 66],
    [530, 572, -140, 140, 67],
    [472, 604, -160, 140, 68],
    [421, 632, -180, 140, 69],
    [1298, 1044, 20, 20, 70],
    [1432, 1048, 40, 20, 71],
    [1550, 1054, 60, 20, 72],
    [1647, 1061, 80, 20, 73],
    [1726, 1078, 100, 20, 74],
    [1793, 1075, 120, 20, 75],
    [1847, 1082, 140, 20, 76],
    [1886, 1087, 160, 20, 77],
    [1921, 1091, 180, 20, 78],
    [1292, 901, 20, 40, 79],
    [1423, 909, 40, 40, 80],
    [1540, 925, 60, 40, 81],
    [1636, 929, 80, 40, 82],
    [1717, 955, 100, 40, 83],
    [1786, 969, 120, 40, 84],
    [1839, 831, 140, 40, 85],
    [1879, 955, 160, 40, 86],
    [1914, 1005, 180, 40, 87],
    [1284, 775, 20, 60, 88],
    [1411, 788, 40, 60, 89],
    [1525, 806, 60, 60, 90],
    [1620, 828, 80, 60, 91],
    [1700, 849, 100, 60, 92],
    [1770, 868, 120, 60, 93],
    [1826, 886, 140, 60, 94],
    [1868, 907, 160, 60, 95],
    [1903, 924, 180, 60, 96],
    [1275, 677, 20, 80, 97],
    [1397, 682, 40, 80, 98],
    [1506, 703, 60, 80, 99],
    [1600, 727, 80, 80, 100],
    [1681, 752, 100, 80, 101],
    [1750, 776, 120, 80, 102],
    [1808, 799, 140, 80, 103],
    [1851, 824, 160, 80, 104],
    [1887, 845, 180, 80, 105],
    [1265, 788, 20, 100, 106],
    [1380, 593, 40, 100, 107],
    [1486, 615, 60, 100, 108],
    [1577, 640, 80, 100, 109],
    [1655, 668, 100, 100, 110],
    [1727, 694, 120, 100, 111],
    [1788, 719, 140, 100, 112],
    [1830, 747, 160, 100, 113],
    [1868, 773, 180, 100, 114],
    [1287, 505, 20, 120, 115],
    [1365, 520, 40, 120, 116],
    [1453, 541, 60, 120, 117],
    [1552, 567, 80, 120, 118],
    [1630, 595, 100, 120, 119],
    [1700, 621, 120, 120, 120],
    [1761, 647, 140, 120, 121],
    [1809, 679, 160, 120, 122],
    [1843, 705, 180, 120, 123],
    [1249, 445, 20, 140, 124],
    [1350, 460, 40, 140, 125],
    [1444, 480, 60, 140, 126],
    [1528, 506, 80, 140, 127],
    [1605, 532, 100, 140, 128],
    [1674, 557, 120, 140, 129],
    [1735, 584, 140, 140, 130],
    [1784, 713, 160, 140, 131],
    [1821, 635, 180, 140, 132]
]


def get_closest_control_point(plant_px_x, plant_px_y, points_map):
    index, min_distance = None, float("inf")
    for i in range(len(points_map)):
        cur_distance = math.sqrt((plant_px_x - points_map[i][0]) ** 2 + (plant_px_y - points_map[i][1]) ** 2)
        if cur_distance < min_distance:
            min_distance, index = cur_distance, i
    return points_map[index]


def adapt_y_axis_value(value, axis_max_value):
    """This function converts the value for the Y axis"""

    """The coordinate grid used in OpenCV differs from Matplotlib: if you represent two-dimensional coordinate axes,
    then OpenCV counts (0) along the y axis "from the top", and Matplotlib starts from the "bottom", so before using 
    the coordinates along the y axis, you need to change the system accordingly Matplotlib measurements"""

    return axis_max_value - value


def clear_log_dir():
    images = glob.glob(LOG_DIR + "*.jpg")
    for file_path in images:
        os.remove(file_path)


def px_to_smoothie_value(target_px, center_px, one_mm_in_px):
    """Converts px into mm-s"""

    # returns wrong sign for x because of different 0 position between smoothie and image
    return (target_px - center_px) / one_mm_in_px


def sort_plant_boxes_dist(boxes: list, current_px_x, current_px_y):
    """Returns sorted list making closest to center plants first"""

    return sorted(boxes, key=lambda box: box.get_distance_from(current_px_x, current_px_y))


def min_plant_box_dist(boxes: list, current_px_x, current_px_y):
    """Returns closest to current coordinates plant box"""

    return min(boxes, key=lambda box: box.get_distance_from(current_px_x, current_px_y))


def sort_plant_boxes_conf(boxes: list):
    """Returns list sorted by descending plant confidence"""

    return sorted(boxes, key=lambda box: box.get_confidence(), reverse=True)


def is_point_in_rect(point_x, point_y, left, top, right, bottom):
    """Returns True if (x,y) point in the rectangle or on it's border, else False"""

    return point_x >= left and point_x <= right and point_y >= top and point_y <= bottom


def is_point_in_poly(point_x, point_y, polygon: Polygon):
    return polygon.contains_point([point_x, point_y])


def is_point_in_circle(point_x, point_y, circle_center_x, circle_center_y, circle_radius):
    """Returns True if (x,y) point in the circle or on it's border, False otherwise"""

    return math.sqrt((point_x - circle_center_x) ** 2 + (point_y - circle_center_y) ** 2) <= circle_radius


def draw_zones_circle(image, center_x, center_y, undist_zone_radius, work_zone_radius):
    cv.circle(image, (center_x, center_y), undist_zone_radius, (0, 0, 255), thickness=3)
    cv.circle(image, (center_x, center_y), work_zone_radius, (0, 0, 255), thickness=3)
    return image


def draw_zones(image, circle_center_x, circle_center_y, circle_radius, np_poly_points):
    image = cv.circle(image, (circle_center_x, circle_center_y), circle_radius, (0, 0, 255), thickness=3)
    return cv.polylines(image, [np_poly_points], isClosed=True, color=(0, 0, 255), thickness=5)


def draw_zone_poly(image, np_poly_points):
    return cv.polylines(image, [np_poly_points], isClosed=True, color=(0, 0, 255), thickness=5)


def main():
    time.sleep(5)
    log_counter = 1
    if not os.path.exists(LOG_DIR):
        try:
            os.mkdir(LOG_DIR)
        except OSError:
            print("Creation of the directory %s failed" % LOG_DIR)
            logging.error("Creation of the directory %s failed" % LOG_DIR)
        else:
            print("Successfully created the directory %s " % LOG_DIR)
            logging.info("Successfully created the directory %s " % LOG_DIR)

    # working zone pre-calculations
    # these points list is changed for usage in matplotlib (it has differences in the coords system)
    # working_zone_points_plt = list(
    #    map(lambda item: [item[0], config.CROP_H_TO - config.CROP_H_FROM - item[1]], WORKING_ZONE_POLY_POINTS))
    working_zone_polygon = Polygon(WORKING_ZONE_POLY_POINTS)

    # these points array is used for drawing zone using OpenCV
    working_zone_points_cv = np.array(WORKING_ZONE_POLY_POINTS, np.int32).reshape((-1, 1, 2))

    # remove old images from log dir
    print("Removing .jpg images from log directory")
    logging.debug("Removing .jpg images from log directory")
    clear_log_dir()

    # create smoothieboard adapter (API for access and control smoothieboard)
    while True:
        try:
            smoothie = adapters.SmoothieAdapter(config.SMOOTHIE_HOST)
            print("Successfully connected to smoothie")
            logging.info("Successfully connected to smoothie")
            break
        except OSError as error:
            logging.warning(repr(error))
            print(repr(error))

    detector = detection.YoloOpenCVDetection()

    with adapters.CameraAdapterIMX219_170() as camera:
        print("Warming up the camera")
        logging.debug("Warming up the camera")
        time.sleep(5)

        # main loop, detection and motion
        while True:
            logging.debug("Starting detection and motion main loop iteration")
            # go to scan position
            # smoothie.ext_align_cork_center(config.XY_F_MAX)
            smoothie.custom_move_to(config.XY_F_MAX, X=config.X_MAX / 2, Y=config.Y_MIN)
            smoothie.wait_for_all_actions_done()

            image = camera.get_image()
            img_y_c, img_x_c = int(image.shape[0] / 2), int(image.shape[1] / 2)
            plant_boxes = detector.detect(image)
            plant_boxes = sort_plant_boxes_dist(plant_boxes, config.X_MAX / 2, config.Y_MIN)

            # check if no plants detected
            if len(plant_boxes) < 1:
                print("No plants detected on view scan (img " + str(log_counter) + "), moving forward")
                logging.info("No plants detected on view scan (img " + str(log_counter) + "), moving forward")
                log_img = image.copy()
                log_img = draw_zones(log_img, img_x_c, img_y_c, UNDISTORTED_ZONE_RADIUS, working_zone_points_cv)
                if config.SAVE_DEBUG_IMAGES:
                    cv.imwrite(LOG_DIR + str(log_counter) + " overview scan (see no plants).jpg", log_img)
                log_counter += 1

                # move forward for 30 sm
                res = smoothie.custom_move_for(1000, B=5.43)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    print("Couldn't move forward (for 30 sm), smoothie error occurred:", res)
                    logging.critical("Couldn't move forward (for 30 sm), smoothie error occurred: " + res)
                    # exit(1)
                continue
            else:
                print("Found " + str(len(plant_boxes)) + " plants on view scan (img " + str(log_counter) + ")")
                logging.info("Found " + str(len(plant_boxes)) + " plants on view scan (img " + str(log_counter) + ")")
                log_img = image.copy()
                log_img = draw_zones(log_img, img_x_c, img_y_c, UNDISTORTED_ZONE_RADIUS, working_zone_points_cv)
                log_img = detection.draw_boxes(log_img, plant_boxes)
                if config.SAVE_DEBUG_IMAGES:
                    cv.imwrite(LOG_DIR + str(log_counter) + " overview scan (see " + str(len(plant_boxes)) + " plants).jpg",
                               log_img)
                log_counter += 1

            # loop over all detected plants
            for box in plant_boxes:
                logging.debug("Starting loop over plants list iteration")

                # smoothie.ext_align_cork_center(config.XY_F_MAX)  # camera in real center
                smoothie.custom_move_to(config.XY_F_MAX, X=config.X_MAX / 2, Y=config.Y_MIN)
                smoothie.wait_for_all_actions_done()

                box_x, box_y = box.get_center_points()
                """
                print("Processing plant on x=" + str(box_x) + " y=" + str(box_y))
                logging.info("Processing plant on x=" + str(box_x) + " y=" + str(box_y))
                """

                # if plant is in working zone and can be reached by cork
                if is_point_in_poly(box_x, box_y, working_zone_polygon):
                    print("Plant is in working zone")
                    logging.info("Plant is in working zone")

                    while True:
                        print("Starting extraction loop")
                        logging.info("Starting extraction loop")

                        box_x, box_y = box.get_center_points()

                        print("Processing plant in x=" + str(box_x) + " y=" + str(box_y))
                        logging.info("Processing plant in x=" + str(box_x) + " y=" + str(box_y))

                        # if inside undistorted zone
                        if is_point_in_circle(box_x, box_y, img_x_c, img_y_c, UNDISTORTED_ZONE_RADIUS):
                            print("Plant is in undistorted zone")
                            logging.info("Plant is in undistorted zone")

                            # calculate values to move camera over a plant
                            sm_x = px_to_smoothie_value(box_x, img_x_c, config.ONE_MM_IN_PX)
                            sm_y = -px_to_smoothie_value(box_y, img_y_c, config.ONE_MM_IN_PX)
                            # swap camera and cork for extraction immediately
                            sm_y += CORK_CAMERA_DISTANCE

                            print("Calculated smoothie moving coordinates X=" + str(sm_x) + " Y=" + str(sm_y))
                            logging.debug("Calculated smoothie moving coordinates X=" + str(sm_x) + " Y=" + str(sm_y))

                            ad_cur_coord = smoothie.get_adapter_current_coordinates()
                            print("Adapter coordinates: " + str(ad_cur_coord))
                            logging.info("Adapter coordinates: " + str(ad_cur_coord))

                            sm_cur_coord = smoothie.get_smoothie_current_coordinates()
                            print("Smoothie coordinates: " + str(sm_cur_coord))
                            logging.info("Smoothie coordinates: " + str(sm_cur_coord))

                            # move camera over a plant
                            print("Moving camera to the plant")
                            logging.info("Moving camera to the plant")

                            res = smoothie.custom_move_for(config.XY_F_MAX, X=sm_x, Y=sm_y)
                            smoothie.wait_for_all_actions_done()
                            if res != smoothie.RESPONSE_OK:
                                print("Couldn't move camera over plant, smoothie error occurred:", res)
                                logging.critical(
                                    "Couldn't move camera over plant, smoothie error occurred: " + str(res))
                                # exit(1)

                            # temp debug 1
                            log_img = camera.get_image()
                            if config.SAVE_DEBUG_IMAGES:
                                cv.imwrite(LOG_DIR + str(log_counter) + " extracting (cork in upper position).jpg",
                                           log_img)
                            log_counter += 1

                            # extraction, cork down
                            print("Extracting plant (cork down)")
                            logging.info("Extracting plant (cork down)")

                            res = smoothie.custom_move_for(config.Z_F_MAX, Z=-30)
                            smoothie.wait_for_all_actions_done()
                            if res != smoothie.RESPONSE_OK:
                                print("Couldn't move the extractor down, smoothie error occurred:", res)
                                logging.critical(
                                    "Couldn't move the extractor down, smoothie error occurred:" + str(res))
                                # exit(1)

                            # temp debug 2
                            log_img = camera.get_image()
                            if config.SAVE_DEBUG_IMAGES:
                                cv.imwrite(LOG_DIR + str(log_counter) + " extracting (cork in lower position).jpg",
                                           log_img)
                            log_counter += 1

                            # extraction, cork up
                            print("Extracting plant (cork up)")
                            logging.info("Extracting plant (cork up)")

                            res = smoothie.ext_cork_up()
                            smoothie.wait_for_all_actions_done()
                            if res != smoothie.RESPONSE_OK:
                                print("Couldn't move the extractor up, smoothie error occurred:", res)
                                logging.critical("Couldn't move the extractor up, smoothie error occurred:" + str(res))
                                # exit(1)
                            break

                        # if outside undistorted zone but in working zone
                        else:
                            print("Plant is outside undistorted zone, moving to")
                            logging.info("Plant is outside undistorted zone, moving to")

                            # calculate values for move camera closer to a plant
                            control_point = get_closest_control_point(box_x, box_y, IMAGE_CONTROL_POINTS_MAP)
                            sm_x = control_point[2]
                            sm_y = control_point[3]

                            debug_text = "Moving to px x=" + str(control_point[0]) + " y=" + str(control_point[1]) + \
                                         " (control point #" + str(control_point[4]) + ")"
                            print(debug_text)
                            logging.info(debug_text)

                            # move camera closer to a plant
                            res = smoothie.custom_move_for(config.XY_F_MAX, X=sm_x, Y=sm_y)
                            smoothie.wait_for_all_actions_done()
                            if res != smoothie.RESPONSE_OK:
                                print("Couldn't move to plant, smoothie error occurred:", res)
                                logging.critical("Couldn't move to plant, smoothie error occurred: " + str(res))
                                # exit(1)

                            # make new photo and re-detect plants
                            image = camera.get_image()
                            temp_plant_boxes = detector.detect(image)

                            # check if no plants detected
                            if len(temp_plant_boxes) < 1:
                                print("No plants detected (plant was in working zone before), trying to move on\
                                    next item")
                                logging.info("No plants detected (plant was in working zone before), trying to move on\
                                    next item")
                                log_img = image.copy()
                                log_img = draw_zones(log_img, img_x_c, img_y_c, UNDISTORTED_ZONE_RADIUS,
                                                     working_zone_points_cv)
                                if config.SAVE_DEBUG_IMAGES:
                                    cv.imwrite(LOG_DIR + str(log_counter) + " in working zone branch - see no plants.jpg",
                                               log_img)
                                log_counter += 1
                                break

                            # log
                            log_img = image.copy()
                            log_img = draw_zones(log_img, img_x_c, img_y_c, UNDISTORTED_ZONE_RADIUS,
                                                 working_zone_points_cv)
                            log_img = detection.draw_boxes(log_img, temp_plant_boxes)
                            if config.SAVE_DEBUG_IMAGES:
                                cv.imwrite(LOG_DIR + str(log_counter) + " in working zone branch - all plants.jpg", log_img)
                            log_counter += 1

                            # get closest box (exactly update current box from main list coordinates after moving closer)
                            box = min_plant_box_dist(temp_plant_boxes, img_x_c, img_y_c)

                            # log
                            log_img = image.copy()
                            log_img = draw_zones(log_img, img_x_c, img_y_c, UNDISTORTED_ZONE_RADIUS,
                                                 working_zone_points_cv)
                            log_img = detection.draw_box(log_img, box)
                            if config.SAVE_DEBUG_IMAGES:
                                cv.imwrite(LOG_DIR + str(log_counter) + " in working zone branch - closest plant.jpg", log_img)
                            log_counter += 1

                # if not in working zone
                else:
                    print("Skipped", str(box), "(not in working area)")
                    logging.info("Skipped " + str(box) + " (not in working area)")

            # move forward for 30 sm
            res = smoothie.custom_move_for(1000, B=5.43)
            smoothie.wait_for_all_actions_done()
            if res != smoothie.RESPONSE_OK:
                print("Couldn't move forward (for 30 sm), smoothie error occurred:", res)
                logging.critical("Couldn't move forward (for 30 sm), smoothie error occurred: " + str(res))
                # exit(1)


def tools_test():
    test_boxes = [
        detection.DetectedPlantBox(5, 5, 60, 60, ["Test_1"], 0, 0.89),
        detection.DetectedPlantBox(200, 200, 250, 250, ["Test_2"], 0, 0.74),
        detection.DetectedPlantBox(400, 400, 450, 450, ["Test_3"], 0, 0.67)
    ]

    # test px to mm converter
    """
    px_x = px_to_smoohie_value(460, 500, config.ONE_MM_IN_PX)
    px_y = px_to_smoohie_value(450, 500, config.ONE_MM_IN_PX)
    print("Px values x, y: ", px_x, px_y)
    """

    # test sort box by dist
    print()
    cur_x = 1000
    cur_y = 1000

    res_min = min_plant_box_dist(test_boxes, cur_x, cur_y)
    print(res_min.get_name(), res_min.get_confidence())

    res = sort_plant_boxes_dist(test_boxes, cur_x, cur_y)

    for plant_box in res:
        print(plant_box.get_name(), plant_box.get_confidence())

    print(res == res_min)

    # test sort box by confidence
    """
    print()
    res = sort_plant_boxes_conf(test_boxes)

    for plant_box in res:
        print(plant_box.get_name(), plant_box.get_confidence())
    """


if __name__ == "__main__":
    main()
