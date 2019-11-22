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

# paths
LOG_DIR = "log/"
LOG_FILE = str(str(datetime.datetime.now()).split(".")[:-1])[2:-2] + ".log"

# circle zones
WORKING_ZONE_RADIUS = 700
UNDISTORTED_ZONE_RADIUS = 240

# square zones
WORKING_ZONE_X_MIN = 325
WORKING_ZONE_X_MAX = 2057
WORKING_ZONE_Y_MIN = 1040
WORKING_ZONE_Y_MAX = 1400

logging.basicConfig(format='%(asctime)s > %(module)s.%(funcName)s %(levelname)s: %(message)s (line %(lineno)d)',
                    datefmt='%I:%M:%S %p',
                    filename=LOG_DIR + LOG_FILE,
                    filemode='w',
                    level=logging.DEBUG)


def clear_log_dir():
    images = glob.glob(LOG_DIR + "*.jpg")
    for file_path in images:
        os.remove(file_path)


def px_to_smoohie_value(target_px, center_px, one_mm_in_px):
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


def is_point_in_circle(point_x, point_y, circle_center_x, circle_center_y, circle_radius):
    """Returns True if (x,y) point in the circle or on it's border, False otherwise"""

    return math.sqrt((point_x - circle_center_x) ** 2 + (point_y - circle_center_y) ** 2) <= circle_radius


def draw_zones_circle(image, center_x, center_y, undist_zone_radius, work_zone_radius):
    cv.circle(image, (center_x, center_y), undist_zone_radius, (0, 0, 255), thickness=3)
    cv.circle(image, (center_x, center_y), work_zone_radius, (0, 0, 255), thickness=3)
    return image


def draw_zones(image, left, top, right, bottom, circle_center_x, circle_center_y, circle_radius):
    cv.circle(image, (circle_center_x, circle_center_y), circle_radius, (0, 0, 255), thickness=3)
    cv.rectangle(image, (left, top), (right, bottom), (0, 0, 255), thickness=3)
    return image


def main():
    log_counter = 1
    if not os.path.exists(LOG_DIR):
        try:
            os.mkdir(LOG_DIR)
        except OSError:
            print("Creation of the directory %s failed" % LOG_DIR)
        else:
            print("Successfully created the directory %s " % LOG_DIR)

    # remove old images from log dir
    print("Removing .jpg images from log directory")
    logging.debug("Removing .jpg images from log directory")
    clear_log_dir()

    smoothie = adapters.SmoothieAdapter(config.SMOOTHIE_HOST)
    detector = detection.YoloOpenCVDetection()

    with adapters.CameraAdapterIMX219_170() as camera:
        print("Warming up the camera")
        logging.debug("Warming up the camera")
        time.sleep(5)

        # main loop, detection and motion
        while True:
            logging.debug("Starting detection and motion main loop iteration")
            smoothie.ext_align_cork_center(config.XY_F_MAX)
            smoothie.wait_for_all_actions_done()

            image = camera.get_image()
            img_y_c, img_x_c = int(image.shape[0] / 2), int(image.shape[1] / 2)
            plant_boxes = detector.detect(image)
            plant_boxes = sort_plant_boxes_dist(plant_boxes, config.CORK_CENTER_X, config.CORK_CENTER_Y)

            # check if no plants detected
            if len(plant_boxes) < 1:
                print("No plants detected on view scan (img " + str(log_counter) + "), moving forward")
                logging.info("No plants detected on view scan (img " + str(log_counter) + "), moving forward")
                log_img = image.copy()
                log_img = draw_zones(log_img, WORKING_ZONE_X_MIN, WORKING_ZONE_Y_MIN, WORKING_ZONE_X_MAX,
                                   WORKING_ZONE_Y_MAX,
                                   img_x_c, img_y_c, UNDISTORTED_ZONE_RADIUS)
                cv.imwrite(LOG_DIR + str(log_counter) + " overview scan (see no plants).jpg", log_img)
                log_counter += 1

                # move forward for 30 sm
                res = smoothie.custom_move_for(1000, B=-16.3)
                smoothie.wait_for_all_actions_done()
                if res != smoothie.RESPONSE_OK:
                    print("Couldn't move forward (for 30 sm), smoothie error occurred:", res)
                    logging.critical("Couldn't move forward (for 30 sm), smoothie error occurred: " + res)
                    exit(1)
                continue
            else:
                print("Found " + str(len(plant_boxes)) + " plants on view scan (img " + str(log_counter) + ")")
                logging.info("Found " + str(len(plant_boxes)) + " plants on view scan (img " + str(log_counter) + ")")
                log_img = image.copy()
                log_img = draw_zones(log_img, WORKING_ZONE_X_MIN, WORKING_ZONE_Y_MIN, WORKING_ZONE_X_MAX,
                                     WORKING_ZONE_Y_MAX, img_x_c, img_y_c, UNDISTORTED_ZONE_RADIUS)
                log_img = detection.draw_boxes(log_img, plant_boxes)
                cv.imwrite(LOG_DIR + str(log_counter) + " overview scan (see " + str(len(plant_boxes)) + " plants).jpg",
                           log_img)
                log_counter += 1

            # loop over all detected plants
            for box in plant_boxes:
                logging.debug("Starting loop over plants list iteration")

                smoothie.ext_align_cork_center(config.XY_F_MAX)  # camera in real center
                smoothie.wait_for_all_actions_done()
                box_x, box_y = box.get_center_points()

                print("Processing plant on x=" + str(box_x) + " y=" + str(box_y))
                logging.info("Processing plant on x=" + str(box_x) + " y=" + str(box_y))

                # if inside the working zone
                if is_point_in_rect(box_x, box_y, WORKING_ZONE_X_MIN, WORKING_ZONE_Y_MIN, WORKING_ZONE_X_MAX,
                                    WORKING_ZONE_Y_MAX):

                    print("Plant is in working zone")
                    logging.info("Plant is in working zone")

                    while True:
                        print("Starting extraction loop")
                        logging.info("Starting extraction loop")

                        box_x, box_y = box.get_center_points()

                        print("Processing plant on x=" + str(box_x) + " y=" + str(box_y))
                        logging.info("Processing plant on x=" + str(box_x) + " y=" + str(box_y))

                        # if inside undistorted zone
                        if is_point_in_circle(box_x, box_y, img_x_c, img_y_c, UNDISTORTED_ZONE_RADIUS):
                            print("Plant is in undistorted zone")
                            logging.info("Plant is in undistorted zone")

                            # calculate values to move camera over a plant
                            sm_x = px_to_smoohie_value(box_x, img_x_c, config.ONE_MM_IN_PX)
                            sm_y = -px_to_smoohie_value(box_y, img_y_c, config.ONE_MM_IN_PX)

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
                                exit(1)

                            # move cork to the camera position
                            print("Moving cork to the camera position (y+57)")
                            logging.info("Moving cork to the camera position (y+57)")

                            res = smoothie.custom_move_for(config.XY_F_MAX, Y=57)
                            smoothie.wait_for_all_actions_done()
                            if res != smoothie.RESPONSE_OK:
                                print("Couldn't move cork over plant, smoothie error occurred:", res)
                                logging.critical("Couldn't move cork over plant, smoothie error occurred: " + str(res))
                                exit(1)

                            # extraction, cork down
                            print("Extracting plant (cork down)")
                            logging.info("Extracting plant (cork down)")

                            res = smoothie.custom_move_for(config.Z_F_MAX, Z=-30)
                            smoothie.wait_for_all_actions_done()
                            if res != smoothie.RESPONSE_OK:
                                print("Couldn't move the extractor down, smoothie error occurred:", res)
                                logging.critical(
                                    "Couldn't move the extractor down, smoothie error occurred:" + str(res))
                                exit(1)

                            # extraction, cork up
                            print("Extracting plant (cork up)")
                            logging.info("Extracting plant (cork up)")

                            res = smoothie.ext_cork_up()
                            smoothie.wait_for_all_actions_done()
                            if res != smoothie.RESPONSE_OK:
                                print("Couldn't move the extractor up, smoothie error occurred:", res)
                                logging.critical("Couldn't move the extractor up, smoothie error occurred:" + str(res))
                                exit(1)
                            break

                        # if outside undistorted zone but in working zone
                        else:
                            print("Plant is outside undistorted zone, moving to")
                            logging.info("Plant is outside undistorted zone, moving to")

                            # calculate values for move camera closer to a plant
                            sm_x = px_to_smoohie_value(box_x, img_x_c, config.ONE_MM_IN_PX)
                            sm_y = -px_to_smoohie_value(box_y, img_y_c, config.ONE_MM_IN_PX)
                            # move for a half distance
                            sm_x = int(sm_x / 2)
                            sm_y = int(sm_y / 2)

                            print("Moving for half distance x=" + str(sm_x) + " y=" + str(sm_y))
                            logging.info("Moving for half distance x=" + str(sm_x) + " y=" + str(sm_y))

                            # move camera closer to a plant
                            res = smoothie.custom_move_for(config.XY_F_MAX, X=sm_x, Y=sm_y)
                            smoothie.wait_for_all_actions_done()
                            if res != smoothie.RESPONSE_OK:
                                print("Couldn't move to plant, smoothie error occurred:", res)
                                logging.critical("Couldn't move to plant, smoothie error occurred: " + str(res))
                                exit(1)

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
                                log_img = draw_zones(log_img, WORKING_ZONE_X_MIN, WORKING_ZONE_Y_MIN,
                                                      WORKING_ZONE_X_MAX,
                                                      WORKING_ZONE_Y_MAX, img_x_c, img_y_c, UNDISTORTED_ZONE_RADIUS)
                                cv.imwrite(LOG_DIR + str(log_counter) + " in working zone branch - see no plants.jpg",
                                           log_img)
                                log_counter += 1
                                break

                            # log
                            log_img = image.copy()
                            log_img = draw_zones(log_img, WORKING_ZONE_X_MIN, WORKING_ZONE_Y_MIN, WORKING_ZONE_X_MAX,
                                                 WORKING_ZONE_Y_MAX, img_x_c, img_y_c, UNDISTORTED_ZONE_RADIUS)
                            log_img = detection.draw_boxes(log_img, temp_plant_boxes)
                            cv.imwrite(LOG_DIR + str(log_counter) + " in working zone branch - all plants.jpg", log_img)
                            log_counter += 1

                            # get closest box (exactly update current box from main list coordinates after moving closer)
                            box = min_plant_box_dist(temp_plant_boxes, img_x_c, img_y_c)

                            # log
                            log_img = image.copy()
                            log_img = draw_zones(log_img, WORKING_ZONE_X_MIN, WORKING_ZONE_Y_MIN, WORKING_ZONE_X_MAX,
                                                 WORKING_ZONE_Y_MAX, img_x_c, img_y_c, UNDISTORTED_ZONE_RADIUS)
                            log_img = detection.draw_box(log_img, box)
                            cv.imwrite(LOG_DIR + str(log_counter) + " in working zone branch - closest plant.jpg", log_img)
                            log_counter += 1

                # if not in working zone
                else:
                    print("skipped", str(box), "(not in working area)")
                    logging.info("skipped " + str(box) + " (not in working area)")

            # move forward for 30 sm
            res = smoothie.custom_move_for(1000, B=-5.43)
            smoothie.wait_for_all_actions_done()
            if res != smoothie.RESPONSE_OK:
                print("Couldn't move forward (for 30 sm), smoothie error occurred:", res)
                logging.critical("Couldn't move forward (for 30 sm), smoothie error occurred: " + str(res))
                exit(1)


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
