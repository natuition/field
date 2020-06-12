"""Creates two gps points AB and saves them into the file (specified in config), then moves from B to A point."""

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
                  vesc_engine: adapters.VescAdapter, smoothie: adapters.SmoothieAdapter, logger: utility.Logger,
                  client, nav: navigation.GPSComputing, raw_angles_history: list):
    """
    Moves to given point.

    :param coords_from_to:
    :param used_points_history:
    :param gps:
    :param vesc_engine:
    :param smoothie:
    :param logger:
    :param client:
    :param nav:
    :return:
    :param raw_angles_history:
    """

    # raw_angles_history = []
    stop_helping_point = nav.get_coordinate(coords_from_to[1], coords_from_to[0], 90, 1000)
    prev_maneuver_time = time.time()
    prev_point = gps.get_fresh_position()  # TODO: maybe it's ok to get last position instead of waiting for fresh
    vesc_engine.start_moving()

    # main navigation control loop
    while True:
        cur_pos = gps.get_fresh_position()
        used_points_history.append(cur_pos.copy())

        if not client.sendData("{};{}".format(cur_pos[0], cur_pos[1])):
            msg = "[Client] Connection closed !"
            print(msg)
            logger.write(msg + "\n")

        if str(cur_pos) == str(prev_point):
            msg = "Got the same position, added to history, calculations skipped"
            print(msg)
            logger.write(msg + "\n")
            continue

        distance = nav.get_distance(cur_pos, coords_from_to[1])

        msg = "Distance to B: " + str(distance)
        print(msg)
        logger.write(msg + "\n")

        # check if arrived
        _, side = nav.get_deviation(coords_from_to[1], stop_helping_point, cur_pos)
        # if distance <= config.COURSE_DESTINATION_DIFF:  # old way
        if side != 1:  # TODO: maybe should use both side and distance checking methods at once
            vesc_engine.stop_moving()
            # msg = "Arrived (allowed destination distance difference " + str(config.COURSE_DESTINATION_DIFF) + " mm)"
            msg = "Arrived."
            print(msg)
            logger.write(msg + "\n")
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
        print(msg)
        logger.write(msg + "\n")

        msg = "Prev: " + str(prev_point) + " Cur: " + str(cur_pos) + " A: " + str(coords_from_to[0]) \
              + " B: " + str(coords_from_to[1])
        print(msg)
        logger.write(msg + "\n")

        raw_angle = nav.get_angle(prev_point, cur_pos, cur_pos, coords_from_to[1])

        # sum(e)
        if len(raw_angles_history) >= config.WINDOW:
            raw_angles_history.pop(0)
        raw_angles_history.append(raw_angle)

        sum_angles = sum(raw_angles_history)
        if sum_angles > config.SUM_ANGLES_HISTORY_MAX:
            msg = "Sum angles " + str(sum_angles) + " is bigger than max allowed value " + \
                  str(config.SUM_ANGLES_HISTORY_MAX) + ", setting to " + str(config.SUM_ANGLES_HISTORY_MAX)
            print(msg)
            logger.write(msg)
            sum_angles = config.SUM_ANGLES_HISTORY_MAX
        elif sum_angles < -config.SUM_ANGLES_HISTORY_MAX:
            msg = "Sum angles " + str(sum_angles) + " is less than min allowed value " + \
                  str(-config.SUM_ANGLES_HISTORY_MAX) + ", setting to " + str(-config.SUM_ANGLES_HISTORY_MAX)
            print(msg)
            logger.write(msg)
            sum_angles = -config.SUM_ANGLES_HISTORY_MAX

        angle_kp_ki = raw_angle * config.KP + sum_angles * config.KI

        target_angle_sm = angle_kp_ki * -config.A_ONE_DEGREE_IN_SMOOTHIE  # smoothie -Value == left, Value == right
        ad_wheels_pos = smoothie.get_adapter_current_coordinates()["A"]
        sm_wheels_pos = smoothie.get_smoothie_current_coordinates()["A"]

        # compute order angle (smoothie can't turn for huge values immediately also as cancel movement,
        # so we need to do nav. actions in steps)
        order_angle_sm = target_angle_sm - ad_wheels_pos

        # check for out of update frequency and smoothie execution speed (for nav wheels)
        if order_angle_sm > config.MANEUVERS_FREQUENCY * config.A_DEGREES_PER_SECOND * \
                config.A_ONE_DEGREE_IN_SMOOTHIE:
            msg = "Order angle changed from " + str(order_angle_sm) + " to " + str(
                config.MANEUVERS_FREQUENCY * config.A_DEGREES_PER_SECOND +
                config.A_ONE_DEGREE_IN_SMOOTHIE) + " due to exceeding degrees per tick allowed range."
            print(msg)
            logger.write(msg + "\n")
            order_angle_sm = config.MANEUVERS_FREQUENCY * config.A_DEGREES_PER_SECOND * \
                             config.A_ONE_DEGREE_IN_SMOOTHIE
        elif order_angle_sm < -(config.MANEUVERS_FREQUENCY * config.A_DEGREES_PER_SECOND *
                                config.A_ONE_DEGREE_IN_SMOOTHIE):
            msg = "Order angle changed from " + str(order_angle_sm) + " to " + str(-(
                    config.MANEUVERS_FREQUENCY * config.A_DEGREES_PER_SECOND *
                    config.A_ONE_DEGREE_IN_SMOOTHIE)) + " due to exceeding degrees per tick allowed range."
            print(msg)
            logger.write(msg + "\n")
            order_angle_sm = -(config.MANEUVERS_FREQUENCY * config.A_DEGREES_PER_SECOND *
                               config.A_ONE_DEGREE_IN_SMOOTHIE)

        # convert to global coordinates
        order_angle_sm += ad_wheels_pos

        # checking for out of smoothie supported range
        if order_angle_sm > config.A_MAX:
            msg = "Global order angle changed from " + str(order_angle_sm) + " to config.A_MAX = " + \
                  str(config.A_MAX) + " due to exceeding smoothie allowed values range."
            print(msg)
            logger.write(msg + "\n")
            order_angle_sm = config.A_MAX
        elif order_angle_sm < config.A_MIN:
            msg = "Global order angle changed from " + str(order_angle_sm) + " to config.A_MIN = " + \
                  str(config.A_MIN) + " due to exceeding smoothie allowed values range."
            print(msg)
            logger.write(msg + "\n")
            order_angle_sm = config.A_MIN

        msg = "Adapter wheels pos (target): " + str(ad_wheels_pos) + " Smoothie wheels pos (current): " \
              + str(sm_wheels_pos)
        print(msg)
        logger.write(msg + "\n")
        msg = "KI: " + str(config.KI) + " Sum angles: " + str(sum_angles) + " Sum angles history: " + \
              str(raw_angles_history)
        print(msg)
        logger.write(msg + "\n")
        msg = "KP: " + str(config.KP) + " Raw angle: " + str(raw_angle) + " Angle * KP + sum(angles) * KI: " + \
              str(angle_kp_ki) + " Smoothie target angle: " + str(target_angle_sm) + \
              " Smoothie absolute order angle: " + str(order_angle_sm)
        print(msg)
        logger.write(msg + "\n")

        prev_point = cur_pos
        response = smoothie.nav_turn_wheels_to(order_angle_sm, config.A_F_MAX)

        msg = "Smoothie response: " + response
        print(msg)
        logger.write(msg)

        # next tick indent
        print()
        logger.write("\n")


def main():
    nav = navigation.GPSComputing()
    used_points_history = []
    raw_angles_history = []
    logger = utility.Logger("console " + get_current_time() + ".txt")

    # QGIS and sensor data transmitting
    path = os.path.abspath(os.getcwd())
    sensor_processor = SensorProcessing.SensorProcessing(path, 0)
    sensor_processor.startServer()
    client = socketForRTK.Client.Client(4000)
    time.sleep(1)
    if not client.connectionToServer():
        msg = "Connection refused for Server RTK."
        print(msg)
        logger.write(msg + "\n")
    sensor_processor.startSession()

    try:
        msg = "Initializing..."
        print(msg)
        logger.write(msg + "\n")

        smoothie = adapters.SmoothieAdapter(config.SMOOTHIE_HOST)
        vesc_engine = adapters.VescAdapter(config.VESC_RPM, config.VESC_MOVING_TIME, config.VESC_ALIVE_FREQ,
                                           config.VESC_CHECK_FREQ, config.VESC_PORT, config.VESC_BAUDRATE)
        gps = adapters.GPSUbloxAdapter(config.GPS_PORT, config.GPS_BAUDRATE, config.GPS_POSITIONS_TO_KEEP)

        # set smoothie's A axis to 0 (nav turn wheels)
        response = smoothie.set_current_coordinates(A=0)
        if response != smoothie.RESPONSE_OK:
            msg = "Failed to set A=0 on smoothie (turning wheels init position), response message:\n" + response
            print(msg)
            logger.write(msg + "\n")
        msg = "Initializing done."
        print(msg)
        logger.write(msg + "\n")

        # pick from gps
        field_gps_coords = load_coordinates(config.INPUT_GPS_FIELD_FILE)  # [A, B, C, D]
        if len(field_gps_coords) != 4:
            msg = "Expected 4 gps points in " + config.INPUT_GPS_FIELD_FILE + ", got " + str(len(field_gps_coords))
            print(msg)
            logger.write(msg)
        field_gps_coords.append(field_gps_coords[0].copy())  # this makes robot move back to starting point (point A)
        field_gps_coords.append(field_gps_coords[1].copy())

        # start moving forward
        msg = "Press enter to start moving"
        input(msg)
        logger.write(msg + "\n")

        # perimeter bypass loop
        for i in range(1, len(field_gps_coords) - 1):
            # current vector is marked as XY where X2 and Y1 is corner maneuver points
            # example: current side is AB which marked as XY, then point X2 is on AB, Y1 is on BC and
            # X2 and Y1 near ^ABC angle (XYZ). ABCD are "global" points which corresponding field.txt file
            # while XYZ displays currently processed points

            # XY vector
            current_vector = [field_gps_coords[i-1], field_gps_coords[i]]
            cur_vec_dist = nav.get_distance(current_vector[0], current_vector[1])

            msg = "Current movement vector: " + str(current_vector) + " Vector size: " + str(cur_vec_dist)
            print(msg)
            logger.write(msg)

            # check if moving vector is too small for maneuvers
            if config.MANEUVER_START_DISTANCE >= cur_vec_dist:
                msg = "No place for maneuvers; config distance before maneuver is " + \
                      str(config.MANEUVER_START_DISTANCE) + " current moving vector distance is " + str(cur_vec_dist)
                print(msg)
                logger.write(msg + "\n")
                break
            # compute X2 point
            else:
                dist_to_x2 = cur_vec_dist - config.MANEUVER_START_DISTANCE
                point_x2 = nav.get_point_on_vector(current_vector[0], current_vector[1], dist_to_x2)

                msg = "Point X2: " + str(point_x2) + "[X X2] size: " + str(dist_to_x2)
                print(msg)
                logger.write(msg)

            # move to X2
            from_to = [current_vector[0], point_x2]
            move_to_point(from_to, used_points_history, gps, vesc_engine, smoothie, logger, client, nav,
                          raw_angles_history)

            # TO DO: BUG - we checking YZ to be less than maneuver dist but robot will be in Y1, not in Y, so it's not
            # correct. Maybe config.MANEUVER_START_DISTANCE should be multiplied by 2 when checking that vector size is
            # big enough

            # go from X2 to Y1 point (see description in comments above)
            # YZ vector
            current_vector = [field_gps_coords[i], field_gps_coords[i+1]]
            cur_vec_dist = nav.get_distance(current_vector[0], current_vector[1])

            msg = "Next movement vector: " + str(current_vector) + " Vector size: " + str(cur_vec_dist)
            print(msg)
            logger.write(msg)

            # check if moving vector is too small for maneuvers
            if config.MANEUVER_START_DISTANCE >= cur_vec_dist:
                msg = "No place for maneuvers; config distance before maneuver is " + \
                      str(config.MANEUVER_START_DISTANCE) + " next moving vector distance is " + str(cur_vec_dist)
                print(msg)
                logger.write(msg + "\n")
                break
            # compute Y1 point to move (current vector is marked as XY where X2 and Y1 is corner maneuver points)
            # example: next side is BC which marked as XY, then point Y1 is on BC (YZ)
            else:
                dist_to_y1 = config.MANEUVER_START_DISTANCE
                point_y1 = nav.get_point_on_vector(current_vector[0], current_vector[1], dist_to_y1)

                msg = "Point Y1: " + str(point_y1) + "[Y Y1] size: " + str(dist_to_y1)
                print(msg)
                logger.write(msg)

            # move to Y1
            from_to = [current_vector[0], point_y1]
            move_to_point(from_to, used_points_history, gps, vesc_engine, smoothie, logger, client, nav,
                          raw_angles_history)

        msg = "Done!"
        print(msg)
        logger.write(msg + "\n")
    except KeyboardInterrupt:
        msg = "Stopped by a keyboard interrupt (Ctrl + C)"
        print(msg)
        logger.write(msg + "\n")
    except Exception:
        msg = "Exception occurred:\n" + traceback.format_exc()
        print(msg)
        logger.write(msg)
    finally:
        # save log data
        if len(used_points_history) > 0:
            save_gps_coordinates(used_points_history, "used_gps_history " + get_current_time() + ".txt")
        else:
            msg = "used_gps_history list has 0 elements!"
            print(msg)
            logger.write(msg)

        adapter_points_history = gps.get_last_positions_list()
        if len(adapter_points_history) > 0:
            save_gps_coordinates(adapter_points_history, "adapter_gps_history " + get_current_time() + ".txt")
        else:
            msg = "adapter_gps_history list has 0 elements!"
            print(msg)
            logger.write(msg)

        # close log and hardware connections
        logger.close()
        smoothie.disconnect()
        vesc_engine.disconnect()
        gps.disconnect()

        # close transmitting connections
        sensor_processor.endSession()
        client.closeConnection()
        sensor_processor.stopServer()


if __name__ == '__main__':
    main()
