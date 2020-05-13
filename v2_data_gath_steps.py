import adapters
import detection
import cv2 as cv
from config import config
import time
import datetime
import os
from matplotlib.patches import Polygon
import math
import serial
import pyvesc

# paths
WITH_PLANTS_DIR = "with plants/"
WITHOUT_PLANTS_DIR = "without plants/"

# settings
ONE_SMOOTHIE_IN_MM = 55.248618
ONE_MM_IN_SMOOTHIE = 0.0181  # smoothie command = mm dist * this
VESC_SER_PORT = "/dev/ttyACM0"
VESC_SER_BAUD_RATE = 115200
VESC_SER_TIMEOUT = 0
VESC_RPM = -5000

# DON'T TOUCH THIS
WORKING_ZONE_POLY_POINTS = [[387, 618], [504, 553], [602, 506], [708, 469], [842, 434], [1021, 407], [1228, 410], [1435, 443], [1587, 492], [1726, 558], [1867, 637], [1881, 675], [1919, 795], [1942, 926], [1954, 1055], [1953, 1176], [1551, 1187], [1145, 1190], [724, 1190], [454, 1188], [286, 1188], [283, 1082], [296, 979], [318, 874], [351, 753]]
IMAGE_CONTROL_POINTS_MAP = [[1148, 1042, 0, 20, 0], [996, 1042, -20, 20, 1], [851, 1046, -40, 20, 2], [725, 1054, -60, 20, 3], [617, 1061, -80, 20, 4], [529, 1069, -100, 20, 5], [459, 1077, -120, 20, 6], [400, 1085, -140, 20, 7], [357, 1091, -160, 20, 8], [321, 1097, -180, 20, 9], [1146, 897, 0, 40, 10], [999, 899, -20, 40, 11], [859, 905, -40, 40, 12], [734, 919, -60, 40, 13], [629, 934, -80, 40, 14], [540, 950, -100, 40, 15], [470, 964, -120, 40, 16], [409, 980, -140, 40, 17], [363, 994, -160, 40, 18], [329, 1005, -180, 40, 19], [1146, 767, 0, 60, 20], [1006, 770, -20, 60, 21], [872, 778, -40, 60, 22], [750, 796, -60, 60, 23], [647, 817, -80, 60, 24], [558, 839, -100, 60, 25], [487, 861, -120, 60, 26], [426, 880, -140, 60, 27], [379, 901, -160, 60, 28], [343, 919, -180, 60, 29], [1146, 658, 0, 80, 30], [1014, 661, -20, 80, 31], [887, 671, -40, 80, 32], [770, 691, -60, 80, 33], [669, 714, -80, 80, 34], [583, 739, -100, 80, 35], [511, 765, -120, 80, 36], [448, 788, -140, 80, 37], [397, 815, -160, 80, 38], [361, 837, -180, 80, 39], [1145, 567, 0, 100, 40], [1022, 571, -20, 100, 41], [904, 581, -40, 100, 42], [791, 601, -60, 100, 43], [694, 626, -80, 100, 44], [608, 653, -100, 100, 45], [538, 681, -120, 100, 46], [473, 707, -140, 100, 47], [421, 734, -160, 100, 48], [381, 760, -180, 100, 49], [1144, 496, 0, 120, 50], [1030, 499, -20, 120, 51], [919, 509, -40, 120, 52], [814, 528, -60, 120, 53], [720, 551, -80, 120, 54], [637, 577, -100, 120, 55], [566, 607, -120, 120, 56], [500, 634, -140, 120, 57], [441, 667, -160, 120, 58], [403, 693, -180, 120, 59], [1144, 439, 0, 140, 60], [1038, 441, -20, 140, 61], [935, 449, -40, 140, 62], [835, 468, -60, 140, 63], [746, 489, -80, 140, 64], [664, 517, -100, 140, 65], [593, 546, -120, 140, 66], [530, 572, -140, 140, 67], [472, 604, -160, 140, 68], [421, 632, -180, 140, 69], [1298, 1044, 20, 20, 70], [1432, 1048, 40, 20, 71], [1550, 1054, 60, 20, 72], [1647, 1061, 80, 20, 73], [1726, 1078, 100, 20, 74], [1793, 1075, 120, 20, 75], [1847, 1082, 140, 20, 76], [1886, 1087, 160, 20, 77], [1921, 1091, 180, 20, 78], [1292, 901, 20, 40, 79], [1423, 909, 40, 40, 80], [1540, 925, 60, 40, 81], [1636, 929, 80, 40, 82], [1717, 955, 100, 40, 83], [1786, 969, 120, 40, 84], [1839, 831, 140, 40, 85], [1879, 955, 160, 40, 86], [1914, 1005, 180, 40, 87], [1284, 775, 20, 60, 88], [1411, 788, 40, 60, 89], [1525, 806, 60, 60, 90], [1620, 828, 80, 60, 91], [1700, 849, 100, 60, 92], [1770, 868, 120, 60, 93], [1826, 886, 140, 60, 94], [1868, 907, 160, 60, 95], [1903, 924, 180, 60, 96], [1275, 677, 20, 80, 97], [1397, 682, 40, 80, 98], [1506, 703, 60, 80, 99], [1600, 727, 80, 80, 100], [1681, 752, 100, 80, 101], [1750, 776, 120, 80, 102], [1808, 799, 140, 80, 103], [1851, 824, 160, 80, 104], [1887, 845, 180, 80, 105], [1265, 788, 20, 100, 106], [1380, 593, 40, 100, 107], [1486, 615, 60, 100, 108], [1577, 640, 80, 100, 109], [1655, 668, 100, 100, 110], [1727, 694, 120, 100, 111], [1788, 719, 140, 100, 112], [1830, 747, 160, 100, 113], [1868, 773, 180, 100, 114], [1287, 505, 20, 120, 115], [1365, 520, 40, 120, 116], [1453, 541, 60, 120, 117], [1552, 567, 80, 120, 118], [1630, 595, 100, 120, 119], [1700, 621, 120, 120, 120], [1761, 647, 140, 120, 121], [1809, 679, 160, 120, 122], [1843, 705, 180, 120, 123], [1249, 445, 20, 140, 124], [1350, 460, 40, 140, 125], [1444, 480, 60, 140, 126], [1528, 506, 80, 140, 127], [1605, 532, 100, 140, 128], [1674, 557, 120, 140, 129], [1735, 584, 140, 140, 130], [1784, 713, 160, 140, 131], [1821, 635, 180, 140, 132]]
UNDISTORTED_ZONE_RADIUS = 240

# distance between camera and cork, mm
CORK_CAMERA_DISTANCE = 57


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


def input_session_info():
    """Ask user for info about total travel distance, travel step, and label added to images names at this session"""

    travel_distance = float(input("Travel distance (meters): "))
    travel_distance = int(travel_distance * 1000)  # convert to mm
    travel_step = float(input("Travel step (centimeters): "))
    travel_step = int(travel_step * 10)  # convert to mm
    session_label = input("Label for that session (will be added to each image name): ")
    return travel_distance, travel_step, session_label


def move_forward_smoothie(force, distance, smoothie: adapters.SmoothieAdapter):
    """Move forward for a specified distance with specified force using smoothieboard"""

    res = smoothie.custom_move_for(force, B=distance)
    smoothie.wait_for_all_actions_done()
    if res != smoothie.RESPONSE_OK:
        log_msg = "Couldn't move forward, smoothie error occurred: " + str(res)
        print(log_msg)
        exit(1)


def move_forward_vesc(rpm, move_secs, alive_freq, check_freq, ser):
    """Move forward with specified motor RPM for specified time using vesc.

    alive_freq determines "keep working" signal sending frequency
    check_freq determines frequency of checking the need to stop"""

    stop_time = next_alive_time = time.time()

    # start moving with defined RPM
    ser.write(pyvesc.encode(pyvesc.SetRPM(rpm)))

    while True:
        # let vesc know that we're not dead and keep moving
        if time.time() > next_alive_time:
            next_alive_time = time.time() + alive_freq
            ser.write(pyvesc.encode(pyvesc.SetAlive))

        # stop moving
        if time.time() - stop_time > move_secs:
            ser.write(pyvesc.encode(pyvesc.SetRPM(0)))
            break

        time.sleep(check_freq)


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


def gather_data(smoothie: adapters.SmoothieAdapter, camera: adapters.CameraAdapterIMX219_170,
                detector: detection.YoloOpenCVDetection, counter, session_label, working_zone_polygon):
    """Gathers photos from robot's on-board camera (no position changes for robot's body).
    Saves single image if no plants were detected on view scan to separate directory.
    Saves images with few positions for each detected plant if any plants detected, and saves all that images to the
    separate directory."""

    # go to the view position (y min, x max / 2)
    smoothie.custom_move_to(config.XY_F_MAX, X=config.X_MAX / 2, Y=config.Y_MIN)
    smoothie.wait_for_all_actions_done()

    image = camera.get_image()
    img_y_c, img_x_c = int(image.shape[0] / 2), int(image.shape[1] / 2)
    plant_boxes = detector.detect(image)

    # save image once if no plants detected, get more each plant positions images otherwise
    if len(plant_boxes) == 0:
        save_image(WITHOUT_PLANTS_DIR, image, counter, session_label)
        counter += 1
    # if any plants detected on the image
    else:
        save_image(WITH_PLANTS_DIR, image, counter, session_label)
        counter += 1

        # loop over all detected plants
        for box in plant_boxes:
            # go to the view position (y min, x max / 2)
            smoothie.custom_move_to(config.XY_F_MAX, X=config.X_MAX / 2, Y=config.Y_MIN)
            smoothie.wait_for_all_actions_done()
            box_x, box_y = box.get_center_points()

            # if plant is in working zone and can be reached by cork
            if is_point_in_poly(box_x, box_y, working_zone_polygon):
                while True:
                    box_x, box_y = box.get_center_points()

                    # if inside undistorted zone
                    if is_point_in_circle(box_x, box_y, img_x_c, img_y_c, UNDISTORTED_ZONE_RADIUS):
                        # get image right over plant
                        image = camera.get_image()
                        save_image(WITH_PLANTS_DIR, image, counter, session_label)
                        counter += 1

                        sm_x = px_to_smoothie_value(box_x, img_x_c, config.ONE_MM_IN_PX)
                        sm_y = -px_to_smoothie_value(box_y, img_y_c, config.ONE_MM_IN_PX)

                        # try to get images over a plant and for 8 sides (left right top bot and diagonals)
                        # for x_shift, y_shift in [[sm_x, sm_y],[-20,0],[0,20],[20,0],[20,0],[0,-20],[0,-20],[-20,0],[-20,0]]:  # do 8 additional photos around plant
                        for x_shift, y_shift in [[sm_x, sm_y]]:  # only one photo right over plant
                            response = smoothie.custom_move_for(config.XY_F_MAX, X=x_shift, Y=y_shift)
                            smoothie.wait_for_all_actions_done()
                            # skip this photo if couldn't change camera position
                            # skipping will affect that plant's other photos positions
                            if response != smoothie.RESPONSE_OK:
                                continue
                            image = camera.get_image()
                            save_image(WITH_PLANTS_DIR, image, counter, session_label)
                            counter += 1
                        break

                    # if outside undistorted zone but in working zone
                    else:
                        control_point = get_closest_control_point(box_x, box_y, IMAGE_CONTROL_POINTS_MAP)
                        sm_x = control_point[2]
                        sm_y = control_point[3]

                        # move camera closer to a plant
                        response = smoothie.custom_move_for(config.XY_F_MAX, X=sm_x, Y=sm_y)
                        smoothie.wait_for_all_actions_done()
                        if response != smoothie.RESPONSE_OK:
                            print("Something gone wrong with control point #" + str(control_point[4]),
                                  "and smoothie error occurred when I tried to go closer to a plant:", response)
                            break

                        # make a new photo and re-detect plants
                        temp_plant_boxes = detector.detect(camera.get_image())
                        # check if no plants detected
                        if len(temp_plant_boxes) == 0:
                            break
                        # get closest box (exactly update current box from main list coordinates after moving closer)
                        box = min_plant_box_dist(temp_plant_boxes, img_x_c, img_y_c)
    return counter


def main():
    travel_distance_mm, travel_step_mm, session_label = input_session_info()
    create_directories(WITH_PLANTS_DIR, WITHOUT_PLANTS_DIR)
    smoothie = create_smoothie_connection(config.SMOOTHIE_HOST)
    detector = detection.YoloOpenCVDetection()
    working_zone_polygon = Polygon(WORKING_ZONE_POLY_POINTS)

    with adapters.CameraAdapterIMX219_170() as camera:
        time.sleep(2)
        counter = 1

        with serial.Serial(VESC_SER_PORT, baudrate=VESC_SER_BAUD_RATE, timeout=VESC_SER_TIMEOUT) as ser:
            print("Data gathering started.")

            for _ in range(0, travel_distance_mm, travel_step_mm):
                counter = gather_data(smoothie, camera, detector, counter, session_label, working_zone_polygon)
                # move_forward_smoothie(1100, -5.2, smoothie)  # F1100, B-5.2 = 30 cm with max speed (B-104 F1900 for min speed 30 cm)  # for smoothie moving forward control
                move_forward_vesc(VESC_RPM, 2, 0.5, 0.01, ser)

            # gather data in the final position
            counter = gather_data(smoothie, camera, detector, counter, session_label, working_zone_polygon)

            print("Data gathering done.", str(counter - 1), "images collected at this session.")


if __name__ == "__main__":
    main()
