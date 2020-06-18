"""Spiral movement over given by ABCD points area"""

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


def move_to_point(coords_from_to: list, used_points_history: list, gps: adapters.GPSUbloxAdapter,
                  vesc_engine: adapters.VescAdapter, smoothie: adapters.SmoothieAdapter, logger_full: utility.Logger,
                  client, nav: navigation.GPSComputing, raw_angles_history: list, report_writer, report_field_names,
                  logger_table: utility.Logger):
    """
    Moves to given point.

    :param coords_from_to:
    :param used_points_history:
    :param gps:
    :param vesc_engine:
    :param smoothie:
    :param logger_full:
    :param client:
    :param nav:
    :param raw_angles_history:
    :return:
    """

    raw_angles_history = []
    stop_helping_point = nav.get_coordinate(coords_from_to[1], coords_from_to[0], 90, 1000)
    prev_maneuver_time = time.time()
    prev_point = gps.get_fresh_position()  # TODO: maybe it's ok to get last position instead of waiting for fresh
    vesc_engine.apply_rpm(int(config.VESC_RPM / 2))
    vesc_engine.start_moving()

    # main navigation control loop
    while True:
        cur_pos = gps.get_fresh_position()
        used_points_history.append(cur_pos.copy())

        if not client.sendData("{};{}".format(cur_pos[0], cur_pos[1])):
            msg = "[Client] Connection closed !"
            print(msg)
            logger_full.write(msg + "\n")

        if str(cur_pos) == str(prev_point):
            msg = "Got the same position, added to history, calculations skipped. Am I stuck?"
            print(msg)
            logger_full.write(msg + "\n")
            continue

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
    nav = navigation.GPSComputing()
    used_points_history = []
    raw_angles_history = []
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

        smoothie = adapters.SmoothieAdapter(config.SMOOTHIE_HOST)
        vesc_engine = adapters.VescAdapter(int(config.VESC_RPM / 2), config.VESC_MOVING_TIME, config.VESC_ALIVE_FREQ,
                                           config.VESC_CHECK_FREQ, config.VESC_PORT, config.VESC_BAUDRATE)
        gps = adapters.GPSUbloxAdapter(config.GPS_PORT, config.GPS_BAUDRATE, config.GPS_POSITIONS_TO_KEEP)

        # set smoothie's A axis to 0 (nav turn wheels)
        response = smoothie.set_current_coordinates(A=0)
        if response != smoothie.RESPONSE_OK:
            msg = "Failed to set A=0 on smoothie (turning wheels init position), response message:\n" + response
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

            move_to_point(from_to, used_points_history, gps, vesc_engine, smoothie, logger_full, client, nav,
                          raw_angles_history, report_writer, report_field_names, logger_table)

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
        report_file.close()
        smoothie.disconnect()
        vesc_engine.disconnect()
        gps.disconnect()

        # close transmitting connections
        sensor_processor.endSession()
        client.closeConnection()
        sensor_processor.stopServer()


if __name__ == '__main__':
    main()
