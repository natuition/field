"""Spiral movement, detection and extraction over given by ABCD points area"""

import os
import adapters
import navigation
from config import config
import time
import datetime
import utility
import traceback
import SensorProcessing
import socketForRTK
from socketForRTK.Client import Client
import csv
import detection
from matplotlib.patches import Polygon
import math
import cv2 as cv
import numpy as np


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


def load_coordinates(file_path):
    positions_list = []
    with open(file_path) as file:
        for line in file:
            if line != "":
                positions_list.append(list(map(float, line.split(" "))))
    return positions_list


def save_gps_coordinates(points: list, file_name):
    with open(file_name, "w") as file:
        for point in points:
            str_point = str(point[0]) + " " + str(point[1]) + "\n"
            file.write(str_point)


def get_current_time():
    """Returns formatted string with current time (YYYY-MM-DD HH-MM-SS)"""

    date = str(datetime.datetime.now())
    return date[:date.rindex(".")].replace(":", "-")


def ask_for_ab_points(gps: adapters.GPSUbloxAdapter):
    """Ask user for moving vector AB points"""

    input("Press enter to save point B")
    point_b = gps.get_fresh_position()
    print("Point B saved.")
    input("Press enter to save point A")
    point_a = gps.get_fresh_position()
    print("Point A saved.")
    return [point_a, point_b]


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


def any_plant_in_working_zone(plant_boxes: list, working_zone_polygon: Polygon):
    """
    Returns True if at least one plant box center is in given polygon, False otherwise.
    :param plant_boxes:
    :param working_zone_polygon:
    :return:
    """

    if len(plant_boxes) == 0:
        return False
    for box in plant_boxes:
        box_x, box_y = box.get_center_points()
        if is_point_in_poly(box_x, box_y, working_zone_polygon):
            return True
    return False


def draw_zone_circle(image, circle_center_x, circle_center_y, circle_radius):
    """Draws received circle on image. Used for drawing undistorted zone edges on photo"""

    return cv.circle(image, (circle_center_x, circle_center_y), circle_radius, (0, 0, 255), thickness=3)


def draw_zone_poly(image, np_poly_points):
    """Draws received polygon on image. Used for drawing working zone edges on photo"""

    return cv.polylines(image, [np_poly_points], isClosed=True, color=(0, 0, 255), thickness=5)


def save_image(path_to_save, image, counter, session_label, sep=" "):
    """
    Assembles image file name and saves received image under this name to specified directory.
    Counter and session label may be passed if was set to None.
    """

    session_label = session_label + sep if session_label else ""
    counter = sep + str(counter) if counter or counter == 0 else ""
    cv.imwrite(path_to_save + session_label + get_current_time() + counter + ".jpg", image)


def extract_all_plants(smoothie: adapters.SmoothieAdapter, camera: adapters.CameraAdapterIMX219_170,
                       precise_det: detection.YoloOpenCVDetection, working_zone_polygon: Polygon, frame,
                       plant_boxes: list, undistorted_zone_radius, working_zone_points_cv, img_output_dir):
    """Extract all plants found in current position"""

    img_y_c, img_x_c = int(frame.shape[0] / 2), int(frame.shape[1] / 2)

    # debug image saving
    if config.SAVE_DEBUG_IMAGES:
        frame = draw_zone_circle(frame, img_x_c, img_y_c, undistorted_zone_radius)
        frame = draw_zone_poly(frame, working_zone_points_cv)
        frame = detection.draw_boxes(frame, plant_boxes)
        save_image(img_output_dir, frame, None, None)

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
                if is_point_in_circle(box_x, box_y, img_x_c, img_y_c, config.UNDISTORTED_ZONE_RADIUS):
                    print("Plant is in undistorted zone")
                    # calculate values to move camera over a plant
                    sm_x = px_to_smoothie_value(box_x, img_x_c, config.ONE_MM_IN_PX)
                    sm_y = -px_to_smoothie_value(box_y, img_y_c, config.ONE_MM_IN_PX)
                    # swap camera and cork for extraction immediately
                    sm_x += config.CORK_TO_CAMERA_DISTANCE_X
                    sm_y += config.CORK_TO_CAMERA_DISTANCE_Y

                    # move cork over a plant
                    res = smoothie.custom_move_for(config.XY_F_MAX, X=sm_x, Y=sm_y)
                    smoothie.wait_for_all_actions_done()
                    if res != smoothie.RESPONSE_OK:
                        print("Couldn't move cork over plant, smoothie error occurred:", res)
                        break

                    # extraction, cork down
                    res = smoothie.custom_move_for(F=1700, Z=-35)  # TODO: calculation -Z depending on box size
                    smoothie.wait_for_all_actions_done()
                    if res != smoothie.RESPONSE_OK:
                        print("Couldn't move the extractor down, smoothie error occurred:", res)
                        break

                    # extraction, cork up
                    res = smoothie.ext_cork_up()
                    smoothie.wait_for_all_actions_done()
                    if res != smoothie.RESPONSE_OK:
                        msg = "Couldn't move the extractor up, smoothie error occurred: " + res + \
                              "emergency exit as I don't want break corkscrew."
                        print(msg)
                        exit(1)

                    # Daisy additional corners extraction
                    if box.get_name() == "Daisy":  # TODO: need to create flexible extraction method choosing (maybe dict of functions)
                        box_x_half, box_y_half = box.get_sizes()
                        box_x_half, box_y_half = int(box_x_half / 2 / config.ONE_MM_IN_PX), \
                                                 int(box_y_half / 2 / config.ONE_MM_IN_PX)

                        for x_shift, y_shift in [[-box_x_half, box_y_half], [0, -box_y_half * 2], [box_x_half * 2, 0],
                                                 [0, box_y_half * 2]]:
                            # move to the corner
                            response = smoothie.custom_move_for(config.XY_F_MAX, X=x_shift, Y=y_shift)
                            smoothie.wait_for_all_actions_done()
                            if response != smoothie.RESPONSE_OK:
                                msg = "Aborting movement to the corner (couldn't reach): " + response
                                print(msg)
                                break

                            # extraction, cork down
                            res = smoothie.custom_move_for(F=1700, Z=-42)  # TODO: calculation -Z depending on box size
                            smoothie.wait_for_all_actions_done()
                            if res != smoothie.RESPONSE_OK:
                                msg = "Couldn't move the extractor down, smoothie error occurred: " + res
                                print(msg)
                                break

                            # extraction, cork up
                            res = smoothie.ext_cork_up()
                            smoothie.wait_for_all_actions_done()
                            if res != smoothie.RESPONSE_OK:
                                msg = "Couldn't move the extractor up, smoothie error occurred: " + res + \
                                      "emergency exit as I don't want break corkscrew."
                                print(msg)
                                exit(1)
                    break

                # if outside undistorted zone but in working zone
                else:
                    # calculate values for move camera closer to a plant
                    control_point = get_closest_control_point(box_x, box_y, config.IMAGE_CONTROL_POINTS_MAP)
                    sm_x, sm_y = control_point[2], control_point[3]

                    # move camera closer to a plant
                    res = smoothie.custom_move_for(config.XY_F_MAX, X=sm_x, Y=sm_y)
                    smoothie.wait_for_all_actions_done()
                    if res != smoothie.RESPONSE_OK:
                        print("Couldn't move to plant, smoothie error occurred:", res)
                        break

                    # make new photo and re-detect plants
                    frame = camera.get_image()
                    temp_plant_boxes = precise_det.detect(frame)

                    # check case if no plants detected
                    if len(temp_plant_boxes) == 0:
                        print("No plants detected (plant was in working zone before), trying to move on next item")
                        break

                    # debug image saving
                    if config.SAVE_DEBUG_IMAGES:
                        frame = draw_zone_circle(frame, img_x_c, img_y_c, undistorted_zone_radius)
                        frame = draw_zone_poly(frame, working_zone_points_cv)
                        frame = detection.draw_boxes(frame, temp_plant_boxes)
                        save_image(img_output_dir, frame, None, None)

                    # get closest box (update current box from main list coordinates after moving closer)
                    box = min_plant_box_dist(temp_plant_boxes, img_x_c, img_y_c)

        # if not in working zone
        else:
            print("Skipped", str(box), "(not in working area)")


def move_to_point_and_extract(coords_from_to: list, gps: adapters.GPSUbloxAdapter, vesc_engine: adapters.VescAdapter,
                              smoothie: adapters.SmoothieAdapter, camera: adapters.CameraAdapterIMX219_170,
                              periphery_det: detection.YoloOpenCVDetection, precise_det: detection.YoloOpenCVDetection,
                              client, logger_full: utility.Logger, logger_table: utility.Logger, report_field_names,
                              used_points_history: list, nav: navigation.GPSComputing, working_zone_polygon,
                              undistorted_zone_radius, working_zone_points_cv, img_output_dir):
    """
    Moves to the given target point and extracts all weeds on the way.
    :param coords_from_to:
    :param gps:
    :param vesc_engine:
    :param smoothie:
    :param camera:
    :param periphery_det:
    :param precise_det:
    :param client:
    :param logger_full:
    :param logger_table:
    :param report_field_names:
    :param used_points_history:
    :param nav:
    :param working_zone_polygon:
    :return:
    """

    raw_angles_history = []
    stop_helping_point = nav.get_coordinate(coords_from_to[1], coords_from_to[0], 90, 1000)
    prev_maneuver_time = time.time()
    prev_point = gps.get_last_position()
    vesc_engine.apply_rpm(int(config.VESC_RPM / 2))
    vesc_engine.start_moving()

    # main navigation control loop
    while True:
        frame = camera.get_image()
        plants_boxes = periphery_det.detect(frame)

        # stop and extract all plants if there any in working zone
        if len(plants_boxes) > 0 and any_plant_in_working_zone(plants_boxes, working_zone_polygon):
            vesc_engine.stop_moving()
            extract_all_plants(smoothie, camera, precise_det, working_zone_polygon, frame, plants_boxes,
                               undistorted_zone_radius, working_zone_points_cv, img_output_dir)
            vesc_engine.start_moving()

        # navigation control
        cur_pos = gps.get_last_position()

        if str(cur_pos) == str(prev_point):
            # msg = "Got the same position, added to history, calculations skipped. Am I stuck?"
            # print(msg)
            # logger_full.write(msg + "\n")
            continue

        used_points_history.append(cur_pos.copy())

        if not client.sendData("{};{}".format(cur_pos[0], cur_pos[1])):
            msg = "[Client] Connection closed !"
            print(msg)
            logger_full.write(msg + "\n")

        distance = nav.get_distance(cur_pos, coords_from_to[1])

        msg = "Distance to B: " + str(distance)
        # print(msg)
        logger_full.write(msg + "\n")

        # check if arrived
        _, side = nav.get_deviation(coords_from_to[1], stop_helping_point, cur_pos)
        # if distance <= config.COURSE_DESTINATION_DIFF:  # old way
        if side != 1:  # TODO: maybe should use both side and distance checking methods at once
            vesc_engine.stop_moving()
            # msg = "Arrived (allowed destination distance difference " + str(config.COURSE_DESTINATION_DIFF) + " mm)"
            msg = "Arrived to " + str(coords_from_to[1])
            # print(msg)
            logger_full.write(msg + "\n")
            break

        # reduce speed if near the target point
        if config.USE_SPEED_LIMIT:
            distance_from_start = nav.get_distance(coords_from_to[0], cur_pos)
            if distance < config.DECREASE_SPEED_TRESHOLD or distance_from_start < config.DECREASE_SPEED_TRESHOLD:
                vesc_engine.apply_rpm(int(config.VESC_RPM / 2))
            else:
                vesc_engine.apply_rpm(config.VESC_RPM)

        # do maneuvers not more often than specified value
        cur_time = time.time()
        if cur_time - prev_maneuver_time < config.MANEUVERS_FREQUENCY:
            continue
        prev_maneuver_time = cur_time

        msg = "Timestamp: " + str(cur_time)
        # print(msg)
        logger_full.write(msg + "\n")

        msg = "Prev: " + str(prev_point) + " Cur: " + str(cur_pos) + " A: " + str(coords_from_to[0]) \
              + " B: " + str(coords_from_to[1])
        # print(msg)
        logger_full.write(msg + "\n")

        raw_angle = nav.get_angle(prev_point, cur_pos, cur_pos, coords_from_to[1])

        # sum(e)
        if len(raw_angles_history) >= config.WINDOW:
            raw_angles_history.pop(0)
        raw_angles_history.append(raw_angle)

        sum_angles = sum(raw_angles_history)
        if sum_angles > config.SUM_ANGLES_HISTORY_MAX:
            msg = "Sum angles " + str(sum_angles) + " is bigger than max allowed value " + \
                  str(config.SUM_ANGLES_HISTORY_MAX) + ", setting to " + str(config.SUM_ANGLES_HISTORY_MAX)
            # print(msg)
            logger_full.write(msg + "\n")
            sum_angles = config.SUM_ANGLES_HISTORY_MAX
        elif sum_angles < -config.SUM_ANGLES_HISTORY_MAX:
            msg = "Sum angles " + str(sum_angles) + " is less than min allowed value " + \
                  str(-config.SUM_ANGLES_HISTORY_MAX) + ", setting to " + str(-config.SUM_ANGLES_HISTORY_MAX)
            # print(msg)
            logger_full.write(msg + "\n")
            sum_angles = -config.SUM_ANGLES_HISTORY_MAX

        angle_kp_ki = raw_angle * config.KP + sum_angles * config.KI
        target_angle_sm = angle_kp_ki * -config.A_ONE_DEGREE_IN_SMOOTHIE  # smoothie -Value == left, Value == right
        ad_wheels_pos = smoothie.get_adapter_current_coordinates()["A"]
        sm_wheels_pos = smoothie.get_smoothie_current_coordinates()["A"]

        # compute order angle (smoothie can't turn for huge values immediately also as cancel movement,
        # so we need to do nav. actions in steps)
        order_angle_sm = target_angle_sm - ad_wheels_pos

        # check for out of update frequency and smoothie execution speed range (for nav wheels)
        if order_angle_sm > config.MANEUVERS_FREQUENCY * config.A_DEGREES_PER_SECOND * \
                config.A_ONE_DEGREE_IN_SMOOTHIE:
            msg = "Order angle changed from " + str(order_angle_sm) + " to " + str(
                config.MANEUVERS_FREQUENCY * config.A_DEGREES_PER_SECOND +
                config.A_ONE_DEGREE_IN_SMOOTHIE) + " due to exceeding degrees per tick allowed range."
            # print(msg)
            logger_full.write(msg + "\n")
            order_angle_sm = config.MANEUVERS_FREQUENCY * config.A_DEGREES_PER_SECOND * \
                             config.A_ONE_DEGREE_IN_SMOOTHIE
        elif order_angle_sm < -(config.MANEUVERS_FREQUENCY * config.A_DEGREES_PER_SECOND *
                                config.A_ONE_DEGREE_IN_SMOOTHIE):
            msg = "Order angle changed from " + str(order_angle_sm) + " to " + str(-(
                    config.MANEUVERS_FREQUENCY * config.A_DEGREES_PER_SECOND *
                    config.A_ONE_DEGREE_IN_SMOOTHIE)) + " due to exceeding degrees per tick allowed range."
            # print(msg)
            logger_full.write(msg + "\n")
            order_angle_sm = -(config.MANEUVERS_FREQUENCY * config.A_DEGREES_PER_SECOND *
                               config.A_ONE_DEGREE_IN_SMOOTHIE)

        # convert to global smoothie coordinates
        order_angle_sm += ad_wheels_pos

        # checking for out of smoothie supported range
        if order_angle_sm > config.A_MAX:
            msg = "Global order angle changed from " + str(order_angle_sm) + " to config.A_MAX = " + \
                  str(config.A_MAX) + " due to exceeding smoothie allowed values range."
            # print(msg)
            logger_full.write(msg + "\n")
            order_angle_sm = config.A_MAX
        elif order_angle_sm < config.A_MIN:
            msg = "Global order angle changed from " + str(order_angle_sm) + " to config.A_MIN = " + \
                  str(config.A_MIN) + " due to exceeding smoothie allowed values range."
            # print(msg)
            logger_full.write(msg + "\n")
            order_angle_sm = config.A_MIN

        raw_angle = round(raw_angle, 2)
        angle_kp_ki = round(angle_kp_ki, 2)
        order_angle_sm = round(order_angle_sm, 2)
        sum_angles = round(sum_angles, 2)
        distance = round(distance, 2)
        ad_wheels_pos = round(ad_wheels_pos, 2)
        sm_wheels_pos = round(sm_wheels_pos, 2)
        gps_quality = cur_pos[2]

        msg = str(gps_quality).ljust(5) + str(raw_angle).ljust(8) + str(angle_kp_ki).ljust(8) + str(order_angle_sm).ljust(8) + str(sum_angles).ljust(8) + str(distance).ljust(13) + str(ad_wheels_pos).ljust(8) + str(sm_wheels_pos).ljust(9)
        print(msg)
        logger_full.write(msg + "\n")

        # load sensors data to csv
        s = ","
        msg = str(gps_quality) + s + str(raw_angle) + s + str(angle_kp_ki) + s + str(order_angle_sm) + s + \
              str(sum_angles) + s + str(distance) + s + str(ad_wheels_pos) + s + str(sm_wheels_pos)
        vesc_data = vesc_engine.pick_sensors_data(report_field_names)
        if vesc_data is not None:
            msg += s
            for key in vesc_data:
                msg += str(vesc_data[key]) + s
            msg = msg[:-1]
        logger_table.write(msg + "\n")

        prev_point = cur_pos
        response = smoothie.nav_turn_wheels_to(order_angle_sm, config.A_F_MAX)

        if response != smoothie.RESPONSE_OK:
            msg = "Smoothie response is not ok: " + response + "\n"
            print(msg)
            logger_full.write(msg + "\n\n")

        # vesc_engine.pick_sensors_data(report_writer, report_field_names)


def compute_x1_x2_points(point_a: list, point_b: list, nav: navigation.GPSComputing, logger: utility.Logger):
    """
    Computes p. x1 with config distance from p. A and p. x2 with the same distance from p. B. Distance is loaded from
     config file. Returns None if AB <= that distance (as there's no place for robot maneuvers).

    :param point_a:
    :param point_b:
    :param nav:
    :param logger:
    :return:
    """

    cur_vec_dist = nav.get_distance(point_a, point_b)

    # check if moving vector is too small for maneuvers
    if config.MANEUVER_START_DISTANCE * 2 >= cur_vec_dist:
        msg = "No place for maneuvers; config start maneuver distance is (that will be multiplied by 2): " + \
              str(config.MANEUVER_START_DISTANCE) + " current moving vector distance is: " + str(cur_vec_dist) + \
            " Given points are: " + str(point_a) + " " + str(point_b)
        # print(msg)
        logger.write(msg + "\n")
        return None, None

    point_x1 = nav.get_point_on_vector(point_a, point_b, config.MANEUVER_START_DISTANCE)
    point_x2 = nav.get_point_on_vector(point_a, point_b, cur_vec_dist - config.MANEUVER_START_DISTANCE)
    return point_x1, point_x2


def compute_x2_spiral(point_a: list, point_b: list, nav: navigation.GPSComputing, logger: utility.Logger):
    """
    Computes p. x2 with distance + spiral interval distance from p. B. Distances are loaded from
     config file. Returns None if AB <= distance * 2 + interval (as there's no place for robot maneuvers).
    :param point_a:
    :param point_b:
    :param nav:
    :param logger:
    :return:
    """

    cur_vec_dist = nav.get_distance(point_a, point_b)

    # check if moving vector is too small for maneuvers
    if config.MANEUVER_START_DISTANCE * 2 + config.SPIRAL_SIDES_INTERVAL >= cur_vec_dist:
        msg = "No place for maneuvers; Config maneuver distance is (that will be multiplied by 2): " + \
              str(config.MANEUVER_START_DISTANCE) + " Config spiral interval: " + str(config.SPIRAL_SIDES_INTERVAL) + \
              " Current moving vector distance is: " + str(cur_vec_dist) + " Given points are: " + str(point_a) + \
              " " + str(point_b)
        # print(msg)
        logger.write(msg + "\n")
        return None
    return nav.get_point_on_vector(point_a, point_b, cur_vec_dist - config.MANEUVER_START_DISTANCE -
                                   config.SPIRAL_SIDES_INTERVAL)


def compute_x1_x2_int_points(point_a: list, point_b: list, nav: navigation.GPSComputing, logger: utility.Logger):
    """

    :param point_a:
    :param point_b:
    :param nav:
    :param logger:
    :return:
    """

    cur_vec_dist = nav.get_distance(point_a, point_b)

    # check if moving vector is too small for maneuvers
    if config.SPIRAL_SIDES_INTERVAL * 2 >= cur_vec_dist:
        msg = "No place for maneuvers; Config spiral interval (that will be multiplied by 2): " + \
              str(config.SPIRAL_SIDES_INTERVAL) + " Current moving vector distance is: " + str(cur_vec_dist) + \
              " Given points are: " + str(point_a) + " " + str(point_b)
        # print(msg)
        logger.write(msg + "\n")
        return None

    point_x1_int = nav.get_point_on_vector(point_a, point_b, config.SPIRAL_SIDES_INTERVAL)
    point_x2_int = nav.get_point_on_vector(point_a, point_b, cur_vec_dist - config.SPIRAL_SIDES_INTERVAL)
    return point_x1_int, point_x2_int


def add_points_to_path(path: list, *args):
    for point in args:
        if point is None:
            return False
        path.append(point)
    return True


def check_points_for_nones(*args):
    for point in args:
        if point is None:
            return False
    return True


def build_path(abcd_points: list, nav: navigation.GPSComputing, logger: utility.Logger):

    path = []
    a, b, c, d = abcd_points[0], abcd_points[1], abcd_points[2], abcd_points[3]

    # get moving points A1 - ... - D2 spiral
    a1, a2 = compute_x1_x2_points(a, b, nav, logger)
    b1, b2 = compute_x1_x2_points(b, c, nav, logger)
    c1, c2 = compute_x1_x2_points(c, d, nav, logger)
    d1, d2 = compute_x1_x2_points(d, a, nav, logger)
    d2_spiral = compute_x2_spiral(d, a, nav, logger)

    # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
    if not add_points_to_path(path, a, a1, a2, b1, b2, c1, c2, d1, d2_spiral):
        return path

    # get A'B'C'D' (prepare next ABCD points)
    b1_int, b2_int = compute_x1_x2_int_points(b, c, nav, logger)
    d1_int, d2_int = compute_x1_x2_int_points(d, a, nav, logger)

    if not check_points_for_nones(b1_int, b2_int, d1_int, d2_int):
        return path

    a_new, b_new = compute_x1_x2_int_points(d2_int, b1_int, nav, logger)
    c_new, d_new = compute_x1_x2_int_points(b2_int, d1_int, nav, logger)

    if not check_points_for_nones(a_new, b_new, c_new, d_new):
        return path

    a, b, c, d, d2_int_prev = a_new, b_new, c_new, d_new, d2_int

    # keep reducing sides for spiral
    while True:
        # get A'B'C'D' (prepare next ABCD points)
        b1_int, b2_int = compute_x1_x2_int_points(b, c, nav, logger)
        d1_int, d2_int = compute_x1_x2_int_points(d, a, nav, logger)

        if not check_points_for_nones(b1_int, b2_int, d1_int, d2_int):
            return path

        a_new, b_new = compute_x1_x2_int_points(d2_int, b1_int, nav, logger)
        c_new, d_new = compute_x1_x2_int_points(b2_int, d1_int, nav, logger)

        if not check_points_for_nones(a_new, b_new, c_new, d_new):
            return path

        # get moving points A1 - ... - D2 spiral
        a1, a2 = compute_x1_x2_points(d2_int_prev, b, nav, logger)
        b1, b2 = compute_x1_x2_points(b, c, nav, logger)
        c1, c2 = compute_x1_x2_points(c, d, nav, logger)
        d1, d2 = compute_x1_x2_points(d, a, nav, logger)
        d2_spiral = compute_x2_spiral(d, a, nav, logger)

        # check if there's a point(s) which shouldn't be used as there's no place for robot maneuvers
        if not add_points_to_path(path, a1, a2, b1, b2, c1, c2, d1, d2_spiral):
            return path

        a, b, c, d, d2_int_prev = a_new, b_new, c_new, d_new, d2_int


def main():
    create_directories(config.DEBUG_IMAGES_PATH)

    working_zone_polygon = Polygon(config.WORKING_ZONE_POLY_POINTS)
    working_zone_points_cv = np.array(config.WORKING_ZONE_POLY_POINTS, np.int32).reshape((-1, 1, 2))
    nav = navigation.GPSComputing()
    used_points_history = []
    logger_full = utility.Logger("full log " + get_current_time() + ".txt")
    logger_table = utility.Logger("table log " + get_current_time() + ".csv")

    # sensors picking
    report_field_names = ['elapsed_time', 'temp_fet_filtered', 'temp_motor_filtered', 'avg_motor_current',
                          'avg_input_current', 'rpm', 'input_voltage']
    report_file = open('report.csv', 'w', newline='')
    report_writer = csv.DictWriter(report_file, fieldnames=report_field_names)
    report_writer.writeheader()

    # QGIS and sensor data transmitting
    path = os.path.abspath(os.getcwd())
    sensor_processor = SensorProcessing.SensorProcessing(path, 0)
    sensor_processor.startServer()
    client = socketForRTK.Client.Client(4000)
    time.sleep(1)
    if not client.connectionToServer():
        msg = "Connection refused for Server RTK."
        print(msg)
        logger_full.write(msg + "\n")
    sensor_processor.startSession()

    try:
        msg = "Initializing..."
        print(msg)
        logger_full.write(msg + "\n")

        # load field coordinates
        field_gps_coords = load_coordinates(config.INPUT_GPS_FIELD_FILE)  # [A, B, C, D]
        if len(field_gps_coords) != 4:
            msg = "Expected 4 gps points in " + config.INPUT_GPS_FIELD_FILE + ", got " + str(len(field_gps_coords))
            print(msg)
            logger_full.write(msg + "\n")
            exit(1)

        # generate path points
        path_points = build_path(field_gps_coords, nav, logger_full)
        if len(path_points) > 0:
            save_gps_coordinates(path_points, "generated_path " + get_current_time() + ".txt")
        else:
            msg = "List of path points is empty, saving canceled."
            print(msg)
            logger_full.write(msg + "\n")
        if len(path_points) < 2:
            msg = "Expected at least 2 points in path, got " + str(len(path_points)) + \
                  " instead (1st point is starting point)."
            print(msg)
            logger_full.write(msg + "\n")
            exit(1)

        # load and connect to everything
        print("Loading camera...")
        camera = adapters.CameraAdapterIMX219_170(config.CROP_W_FROM, config.CROP_W_TO, config.CROP_H_FROM,
                                                  config.CROP_H_TO, config.CV_ROTATE_CODE,
                                                  config.ISP_DIGITAL_GAIN_RANGE_FROM, config.ISP_DIGITAL_GAIN_RANGE_TO,
                                                  config.GAIN_RANGE_FROM, config.GAIN_RANGE_TO,
                                                  config.EXPOSURE_TIME_RANGE_FROM, config.EXPOSURE_TIME_RANGE_TO,
                                                  config.AE_LOCK, config.CAMERA_W, config.CAMERA_H, config.CAMERA_H,
                                                  config.CAMERA_W, config.CAMERA_FRAMERATE, config.CAMERA_FLIP_METHOD)
        print("Loading periphery detector...")
        periphery_detector = detection.YoloOpenCVDetection(config.PERIPHERY_CLASSES_FILE, config.PERIPHERY_CONFIG_FILE,
                                                           config.PERIPHERY_WEIGHTS_FILE, config.PERIPHERY_INPUT_SIZE,
                                                           config.PERIPHERY_CONFIDENCE_THRESHOLD,
                                                           config.PERIPHERY_NMS_THRESHOLD)
        print("Loading precise detector...")
        precise_detector = detection.YoloOpenCVDetection(config.PRECISE_CLASSES_FILE, config.PRECISE_CONFIG_FILE,
                                                         config.PRECISE_WEIGHTS_FILE, config.PRECISE_INPUT_SIZE,
                                                         config.PRECISE_CONFIDENCE_THRESHOLD,
                                                         config.PRECISE_NMS_THRESHOLD)
        print("Loading smoothie...")
        smoothie = adapters.SmoothieAdapter(config.SMOOTHIE_HOST)
        print("Loading vesc...")
        vesc_engine = adapters.VescAdapter(int(config.VESC_RPM / 2), config.VESC_MOVING_TIME, config.VESC_ALIVE_FREQ,
                                           config.VESC_CHECK_FREQ, config.VESC_PORT, config.VESC_BAUDRATE)
        print("Loading gps...")
        gps = adapters.GPSUbloxAdapter(config.GPS_PORT, config.GPS_BAUDRATE, config.GPS_POSITIONS_TO_KEEP)

        # set smoothie's A axis to 0 (nav turn wheels)
        response = smoothie.set_current_coordinates(A=0)
        if response != smoothie.RESPONSE_OK:
            msg = "Failed to set A=0 on smoothie (turning wheels init position), response message:\n" + response
            print(msg)
            logger_full.write(msg + "\n")

        # ask permission to start moving
        msg = "Initializing done. Press enter to start moving."
        input(msg)
        logger_full.write(msg + "\n")

        msg = 'GpsQ|Raw ang|Res ang|Ord ang|Sum ang|Distance    |Adapter|Smoothie|'
        print(msg)
        logger_full.write(msg + "\n")
        msg = 'GpsQ,Raw ang,Res ang,Ord ang,Sum ang,Distance,Adapter,Smoothie,'
        for field_name in report_field_names:
            msg += field_name + ","
        msg = msg[:-1]
        logger_table.write(msg + "\n")

        # path points visiting loop
        for i in range(1, len(path_points)):
            from_to = [path_points[i - 1], path_points[i]]
            from_to_dist = nav.get_distance(from_to[0], from_to[1])

            msg = "Current movement vector: " + str(from_to) + " Vector size: " + str(from_to_dist)
            # print(msg)
            logger_full.write(msg + "\n")

            move_to_point_and_extract(from_to, gps, vesc_engine, smoothie, camera, periphery_detector, precise_detector,
                                      client, logger_full, logger_table, report_field_names, used_points_history, nav,
                                      working_zone_polygon, config.UNDISTORTED_ZONE_RADIUS, working_zone_points_cv,
                                      config.DEBUG_IMAGES_PATH)

        msg = "Done!"
        print(msg)
        logger_full.write(msg + "\n")
    except KeyboardInterrupt:
        msg = "Stopped by a keyboard interrupt (Ctrl + C)"
        print(msg)
        logger_full.write(msg + "\n")
    except Exception:
        msg = "Exception occurred:\n" + traceback.format_exc()
        print(msg)
        logger_full.write(msg + "\n")
    finally:
        # save log data
        if len(used_points_history) > 0:
            save_gps_coordinates(used_points_history, "used_gps_history " + get_current_time() + ".txt")
        else:
            msg = "used_gps_history list has 0 elements!"
            print(msg)
            logger_full.write(msg + "\n")

        adapter_points_history = gps.get_last_positions_list()
        if len(adapter_points_history) > 0:
            save_gps_coordinates(adapter_points_history, "adapter_gps_history " + get_current_time() + ".txt")
        else:
            msg = "adapter_gps_history list has 0 elements!"
            print(msg)
            logger_full.write(msg + "\n")

        # close log and hardware connections
        logger_full.close()
        logger_table.close()
        report_file.close()  # TODO: not used anymore, remove this
        camera.release()
        smoothie.disconnect()
        vesc_engine.disconnect()
        gps.disconnect()

        # close transmitting connections
        sensor_processor.endSession()
        client.closeConnection()
        sensor_processor.stopServer()


if __name__ == '__main__':
    main()
