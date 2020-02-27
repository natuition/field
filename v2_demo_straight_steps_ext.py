import adapters
import detection
import cv2 as cv
from config import config
import time
import datetime
from matplotlib.patches import Polygon
import math
import os

# settings
STEP_DISTANCE_MM = 30  # depends on robot's working zone physical dimensions, don't change that or some of plants will be passed
STEP_TRAVEL_FORCE = 1100
ONE_MM_IN_SMOOTHIE = -0.0181  # smoothie command = dist_mm * this (ONLY FOR B AXIS!!!)
CORK_CAMERA_DISTANCE = 57  # distance between camera and cork on the robot, mm
IMAGES_OUTPUT_DIR = "debug_output/"
SAVE_IMAGES = True

# DON'T TOUCH THIS
WORKING_ZONE_POLY_POINTS = [[387, 618], [504, 553], [602, 506], [708, 469], [842, 434], [1021, 407], [1228, 410], [1435, 443], [1587, 492], [1726, 558], [1867, 637], [1881, 675], [1919, 795], [1942, 926], [1954, 1055], [1953, 1176], [1551, 1187], [1145, 1190], [724, 1190], [454, 1188], [286, 1188], [283, 1082], [296, 979], [318, 874], [351, 753]]
IMAGE_CONTROL_POINTS_MAP = [[1148, 1042, 0, 20, 0], [996, 1042, -20, 20, 1], [851, 1046, -40, 20, 2], [725, 1054, -60, 20, 3], [617, 1061, -80, 20, 4], [529, 1069, -100, 20, 5], [459, 1077, -120, 20, 6], [400, 1085, -140, 20, 7], [357, 1091, -160, 20, 8], [321, 1097, -180, 20, 9], [1146, 897, 0, 40, 10], [999, 899, -20, 40, 11], [859, 905, -40, 40, 12], [734, 919, -60, 40, 13], [629, 934, -80, 40, 14], [540, 950, -100, 40, 15], [470, 964, -120, 40, 16], [409, 980, -140, 40, 17], [363, 994, -160, 40, 18], [329, 1005, -180, 40, 19], [1146, 767, 0, 60, 20], [1006, 770, -20, 60, 21], [872, 778, -40, 60, 22], [750, 796, -60, 60, 23], [647, 817, -80, 60, 24], [558, 839, -100, 60, 25], [487, 861, -120, 60, 26], [426, 880, -140, 60, 27], [379, 901, -160, 60, 28], [343, 919, -180, 60, 29], [1146, 658, 0, 80, 30], [1014, 661, -20, 80, 31], [887, 671, -40, 80, 32], [770, 691, -60, 80, 33], [669, 714, -80, 80, 34], [583, 739, -100, 80, 35], [511, 765, -120, 80, 36], [448, 788, -140, 80, 37], [397, 815, -160, 80, 38], [361, 837, -180, 80, 39], [1145, 567, 0, 100, 40], [1022, 571, -20, 100, 41], [904, 581, -40, 100, 42], [791, 601, -60, 100, 43], [694, 626, -80, 100, 44], [608, 653, -100, 100, 45], [538, 681, -120, 100, 46], [473, 707, -140, 100, 47], [421, 734, -160, 100, 48], [381, 760, -180, 100, 49], [1144, 496, 0, 120, 50], [1030, 499, -20, 120, 51], [919, 509, -40, 120, 52], [814, 528, -60, 120, 53], [720, 551, -80, 120, 54], [637, 577, -100, 120, 55], [566, 607, -120, 120, 56], [500, 634, -140, 120, 57], [441, 667, -160, 120, 58], [403, 693, -180, 120, 59], [1144, 439, 0, 140, 60], [1038, 441, -20, 140, 61], [935, 449, -40, 140, 62], [835, 468, -60, 140, 63], [746, 489, -80, 140, 64], [664, 517, -100, 140, 65], [593, 546, -120, 140, 66], [530, 572, -140, 140, 67], [472, 604, -160, 140, 68], [421, 632, -180, 140, 69], [1298, 1044, 20, 20, 70], [1432, 1048, 40, 20, 71], [1550, 1054, 60, 20, 72], [1647, 1061, 80, 20, 73], [1726, 1078, 100, 20, 74], [1793, 1075, 120, 20, 75], [1847, 1082, 140, 20, 76], [1886, 1087, 160, 20, 77], [1921, 1091, 180, 20, 78], [1292, 901, 20, 40, 79], [1423, 909, 40, 40, 80], [1540, 925, 60, 40, 81], [1636, 929, 80, 40, 82], [1717, 955, 100, 40, 83], [1786, 969, 120, 40, 84], [1839, 831, 140, 40, 85], [1879, 955, 160, 40, 86], [1914, 1005, 180, 40, 87], [1284, 775, 20, 60, 88], [1411, 788, 40, 60, 89], [1525, 806, 60, 60, 90], [1620, 828, 80, 60, 91], [1700, 849, 100, 60, 92], [1770, 868, 120, 60, 93], [1826, 886, 140, 60, 94], [1868, 907, 160, 60, 95], [1903, 924, 180, 60, 96], [1275, 677, 20, 80, 97], [1397, 682, 40, 80, 98], [1506, 703, 60, 80, 99], [1600, 727, 80, 80, 100], [1681, 752, 100, 80, 101], [1750, 776, 120, 80, 102], [1808, 799, 140, 80, 103], [1851, 824, 160, 80, 104], [1887, 845, 180, 80, 105], [1265, 788, 20, 100, 106], [1380, 593, 40, 100, 107], [1486, 615, 60, 100, 108], [1577, 640, 80, 100, 109], [1655, 668, 100, 100, 110], [1727, 694, 120, 100, 111], [1788, 719, 140, 100, 112], [1830, 747, 160, 100, 113], [1868, 773, 180, 100, 114], [1287, 505, 20, 120, 115], [1365, 520, 40, 120, 116], [1453, 541, 60, 120, 117], [1552, 567, 80, 120, 118], [1630, 595, 100, 120, 119], [1700, 621, 120, 120, 120], [1761, 647, 140, 120, 121], [1809, 679, 160, 120, 122], [1843, 705, 180, 120, 123], [1249, 445, 20, 140, 124], [1350, 460, 40, 140, 125], [1444, 480, 60, 140, 126], [1528, 506, 80, 140, 127], [1605, 532, 100, 140, 128], [1674, 557, 120, 140, 129], [1735, 584, 140, 140, 130], [1784, 713, 160, 140, 131], [1821, 635, 180, 140, 132]]
UNDISTORTED_ZONE_RADIUS = 240


def create_directories(*args):
    """Create two photos output directories: with detected plants, and without them"""

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


def create_smoothie_connection(smoothie_ip):
    """Create smoothieboard adapter instance (API for access and smoothieboard control)"""

    while True:
        try:
            smoothie = adapters.SmoothieAdapter(smoothie_ip)
            log_msg = "Successfully connected to smoothie"
            print(log_msg)
            return smoothie
        except OSError as error:
            print(repr(error))


def move_forward(force, sm_distance, smoothie: adapters.SmoothieAdapter):
    """Move forward for a specified distance with specified force"""

    res = smoothie.custom_move_for(force, B=sm_distance)
    if res != smoothie.RESPONSE_OK:
        log_msg = "Couldn't move forward, smoothie error occurred: " + str(res)
        print(log_msg)
        exit(1)


def get_current_time():
    """Returns formatted string with current time (YYYY-MM-DD HH-MM-SS)"""

    date = str(datetime.datetime.now())
    return date[:date.rindex(".")].replace(":", "-")


def save_image(path_to_save, image, counter, session_label, sep=" "):
    """Assembles image file name and saves received image under this name to specified directory"""

    session_label = session_label + sep if session_label != "" else ""
    cv.imwrite(path_to_save + session_label + get_current_time() + sep + str(counter) + ".jpg", image)


def is_point_in_poly(point_x, point_y, polygon: Polygon):
    """Returns True if received polygon object contains received point, False otherwise"""

    return polygon.contains_point([point_x, point_y])


def is_point_in_circle(point_x, point_y, circle_center_x, circle_center_y, circle_radius):
    """Returns True if (x,y) point in the circle or on it's border, False otherwise"""

    return math.sqrt((point_x - circle_center_x) ** 2 + (point_y - circle_center_y) ** 2) <= circle_radius


def px_to_smoothie_value(target_px, center_px, one_mm_in_px):
    """Converts the distance given in pixels to the distance in millimeters"""

    # returns wrong sign for x because of different 0 position between smoothie and image
    return (target_px - center_px) / one_mm_in_px


def get_closest_control_point(plant_px_x, plant_px_y, points_map):
    """Returns image control point, which is closest to the given plant center point"""

    index, min_distance = None, float("inf")
    for i in range(len(points_map)):
        cur_distance = math.sqrt((plant_px_x - points_map[i][0]) ** 2 + (plant_px_y - points_map[i][1]) ** 2)
        if cur_distance < min_distance:
            min_distance, index = cur_distance, i
    return points_map[index]


def min_plant_box_dist(boxes: list, current_px_x, current_px_y):
    """Returns plant box, which is closest to the given point coordinates"""

    return min(boxes, key=lambda box: box.get_distance_from(current_px_x, current_px_y))


def extract_all_plants(smoothie: adapters.SmoothieAdapter, camera: adapters.CameraAdapterIMX219_170,
                       detector: detection.YoloOpenCVDetection, working_zone_polygon: Polygon, image, plant_boxes: list):
    """Extract all plants found in current position"""

    img_y_c, img_x_c = int(image.shape[0] / 2), int(image.shape[1] / 2)

    # loop over all detected plants
    for box in plant_boxes:
        # go to the view position
        smoothie.custom_move_to(config.XY_F_MAX, X=config.X_MAX / 2, Y=config.Y_MIN)
        smoothie.wait_for_all_actions_done()

        box_x, box_y = box.get_center_points()

        # if plant is in working zone (can be reached by cork)
        if is_point_in_poly(box_x, box_y, working_zone_polygon):
            # extraction loop
            while True:
                box_x, box_y = box.get_center_points()

                # if plant inside undistorted zone
                if is_point_in_circle(box_x, box_y, img_x_c, img_y_c, UNDISTORTED_ZONE_RADIUS):
                    print("Plant is in undistorted zone")
                    # calculate values to move camera over a plant
                    sm_x = px_to_smoothie_value(box_x, img_x_c, config.ONE_MM_IN_PX)
                    sm_y = -px_to_smoothie_value(box_y, img_y_c, config.ONE_MM_IN_PX)
                    # swap camera and cork for extraction immediately
                    sm_y += CORK_CAMERA_DISTANCE

                    # move cork over a plant
                    res = smoothie.custom_move_for(config.XY_F_MAX, X=sm_x, Y=sm_y)
                    smoothie.wait_for_all_actions_done()
                    if res != smoothie.RESPONSE_OK:
                        print("Couldn't move cork over plant, smoothie error occurred:", res)
                        break

                    # extraction, cork down
                    res = smoothie.custom_move_for(F=1700, Z=-30)
                    smoothie.wait_for_all_actions_done()
                    if res != smoothie.RESPONSE_OK:
                        print("Couldn't move the extractor down, smoothie error occurred:", res)
                        break

                    # extraction, cork up
                    res = smoothie.ext_cork_up()
                    smoothie.wait_for_all_actions_done()
                    if res != smoothie.RESPONSE_OK:
                        print("Couldn't move the extractor up, smoothie error occurred:", res)
                        exit(1)
                    break

                # if outside undistorted zone but in working zone
                else:
                    # calculate values for move camera closer to a plant
                    control_point = get_closest_control_point(box_x, box_y, IMAGE_CONTROL_POINTS_MAP)
                    sm_x = control_point[2]
                    sm_y = control_point[3]

                    # move camera closer to a plant
                    res = smoothie.custom_move_for(config.XY_F_MAX, X=sm_x, Y=sm_y)
                    smoothie.wait_for_all_actions_done()
                    if res != smoothie.RESPONSE_OK:
                        print("Couldn't move to plant, smoothie error occurred:", res)
                        break

                    # make new photo and re-detect plants
                    image = camera.get_image()
                    temp_plant_boxes = detector.detect(image)

                    # check case if no plants detected
                    if len(temp_plant_boxes) == 0:
                        print("No plants detected (plant was in working zone before), trying to move on next item")
                        break

                    # get closest box (update current box from main list coordinates after moving closer)
                    box = min_plant_box_dist(temp_plant_boxes, img_x_c, img_y_c)

        # if not in working zone
        else:
            print("Skipped", str(box), "(not in working area)")


def main():
    create_directories(IMAGES_OUTPUT_DIR)
    working_zone_polygon = Polygon(WORKING_ZONE_POLY_POINTS)
    counter = 1

    print("Connecting to smoothie")
    smoothie = create_smoothie_connection(config.SMOOTHIE_HOST)

    print("Loading neural network")
    detector = detection.YoloOpenCVDetection()

    print("Loading camera")
    with adapters.CameraAdapterIMX219_170() as camera:
        time.sleep(2)

        print("Moving camera to the view position")
        # go to the view position (y min, x max / 2)
        smoothie.custom_move_to(config.XY_F_MAX, X=config.X_MAX / 2, Y=config.Y_MIN)
        smoothie.wait_for_all_actions_done()

        print("Starting main loop")
        # main control loop
        while True:
            frame = camera.get_image()
            plant_boxes = detector.detect(frame)

            # save photo
            if SAVE_IMAGES:
                frame = detection.draw_boxes(frame, plant_boxes)
                save_image(IMAGES_OUTPUT_DIR, frame, counter, "View")
                counter += 1

            if len(plant_boxes) == 0:
                print("No plants detected")
            else:
                print("Detected", len(plant_boxes), "plants")
                extract_all_plants(smoothie, camera, detector, working_zone_polygon, frame, plant_boxes)
                print("Moving camera to the view position")
                # go to the view position (y min, x max / 2)
                smoothie.custom_move_to(config.XY_F_MAX, X=config.X_MAX / 2, Y=config.Y_MIN)
                smoothie.wait_for_all_actions_done()

            print("Moving step forward")
            move_forward(STEP_TRAVEL_FORCE, STEP_DISTANCE_MM * ONE_MM_IN_SMOOTHIE, smoothie)  # F1100, B-5.2 = 30 cm with max speed (B-104 F1900 for min speed 30 cm)
            smoothie.wait_for_all_actions_done()


if __name__ == "__main__":
    main()
