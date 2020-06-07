"""Creates two gps points AB and saves them into the file (specified in config), then moves from B to A point."""

import os
import adapters
import navigation
from config import config
import time
import datetime
import utility
import traceback
# """
from SensorProcessing import SensorProcessing
from socketForRTK.Client import Client
# """


# old way
NORTH_POINT = [90.0000, 0.0000]
SOUTH_POINT = [-90.0000, 0.0000]


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


def main():
    used_points_history = []
    adapter_points_history = []
    logger = utility.Logger("console " + get_current_time() + ".txt")

    # """
    path = os.path.abspath(os.getcwd())
    sP = SensorProcessing(path, 0)
    sP.startServer()

    client = Client(4000)
    time.sleep(1)
    if not client.connectionToServer():
        msg = "Connection refused for Server RTK."
        print(msg)
        logger.write(msg + "\n")

    sP.startSession()
    # """

    try:
        nav = navigation.GPSComputing()

        msg = "Initializing..."
        print(msg)
        logger.write(msg + "\n")

        with adapters.SmoothieAdapter(config.SMOOTHIE_HOST) as smoothie:
            with adapters.VescAdapter(config.VESC_RPM, config.VESC_MOVING_TIME, config.VESC_ALIVE_FREQ,
                                      config.VESC_CHECK_FREQ, config.VESC_PORT, config.VESC_BAUDRATE) as vesc_engine:
                with adapters.GPSUbloxAdapter(config.GPS_PORT, config.GPS_BAUDRATE,
                                              config.GPS_POSITIONS_TO_KEEP) as gps:

                    # set smoothie's A axis to 0 (nav turn wheels)
                    response = smoothie.set_current_coordinates(A=0)
                    if response != smoothie.RESPONSE_OK:
                        msg = "Failed to set A=0 on smoothie (turning wheels init position), response message:\n" + response
                        print(msg)
                        logger.write(msg + "\n")
                    msg = "Initializing done."
                    print(msg)
                    logger.write(msg + "\n")

                    # get route (field) and save it
                    msg = "Enter 1 to create and save field.txt points file, 2 to load existing file: "
                    command = input(msg)
                    msg += command
                    logger.write(msg + "\n")

                    # pick from gps
                    if command == "1":
                        msg = "Press enter to save point B"
                        input(msg)
                        logger.write(msg)
                        point_b = gps.get_fresh_position()
                        msg = "Point B saved."
                        print(msg)
                        logger.write(msg)

                        # ask info for moving to next point and move there
                        rpm = 11000
                        msg = "RPM = " + str(rpm) + "; set moving time (seconds; will start moving immediately): "
                        moving_time = input(msg)
                        msg += moving_time
                        moving_time = float(moving_time)

                        vesc_engine.set_rpm(11000)
                        vesc_engine.set_moving_time(moving_time)
                        vesc_engine.start_moving()
                        vesc_engine.wait_for_stop()
                        vesc_engine.set_moving_time(config.VESC_MOVING_TIME)
                        vesc_engine.set_rpm(config.VESC_RPM)

                        msg = "Press enter to save point A"
                        input(msg)
                        logger.write(msg)
                        point_a = gps.get_fresh_position()
                        msg = "Point A saved."
                        print(msg)
                        logger.write(msg)

                        field_gps_coords = [point_a, point_b]
                        save_gps_coordinates(field_gps_coords, "field " + get_current_time() + ".txt")
                    # load from file
                    elif command == "2":
                        field_gps_coords = load_coordinates(config.INPUT_GPS_FIELD_FILE)
                    else:
                        msg = "Wrong command, exiting."
                        print(msg)
                        logger.write(msg + "\n")
                        exit(1)

                    # start moving forward
                    msg = "Press enter to start moving"
                    input(msg)
                    logger.write(msg + "\n")
                    prev_maneuver_time = time.time()
                    prev_point = gps.get_fresh_position()
                    vesc_engine.start_moving()

                    # main navigation control loop
                    while True:
                        cur_pos = gps.get_fresh_position()
                        used_points_history.append(cur_pos.copy())

                        # """
                        if not client.sendData("{};{}".format(cur_pos.copy()[0], cur_pos.copy()[1])):
                            msg = "[Client] Connection closed !"
                            print(msg)
                            logger.write(msg + "\n")
                        # """

                        if str(cur_pos) == str(prev_point):
                            msg = "Got the same position, added to history, calculations skipped"
                            print(msg)
                            logger.write(msg + "\n")
                            continue

                        # check if arrived
                        # TO DO: it won't work if deviation > course destination diff, as robot will be far away on some
                        # side and will never get too close to the path ending point
                        distance = nav.get_distance(cur_pos, field_gps_coords[1])

                        msg = "Distance to B: " + distance
                        print(msg)
                        logger.write(msg + "\n")

                        if distance <= config.COURSE_DESTINATION_DIFF:
                            vesc_engine.stop_moving()
                            msg = "Arrived (allowed destination distance difference " + config.COURSE_DESTINATION_DIFF\
                                  + " mm)"
                            print(msg)
                            logger.write(msg + "\n")
                            break

                        # do maneuvers not more often than specified value
                        cur_time = time.time()
                        if cur_time - prev_maneuver_time < config.MANEUVERS_FREQUENCY:
                            continue
                        prev_maneuver_time = cur_time

                        # check for course deviation. if deviation is bigger than a threshold
                        """
                        # old
                        ras = nav.get_corner(SOUTH_POINT, NORTH_POINT, cur_pos, field_gps_coords[1])
                        rar = nav.get_corner(SOUTH_POINT, NORTH_POINT, prev_point, cur_pos)
                        angle = PID * ras - rar
                        """

                        msg = "Timestamp: " + str(cur_time)
                        print(msg)
                        logger.write(msg + "\n")

                        msg = "Prev: " + prev_point + " Cur: " + cur_pos + " A: " + str(field_gps_coords[0]) + " B: " +\
                              str(field_gps_coords[1])
                        print(msg)
                        logger.write(msg + "\n")

                        # TO DO: or vice versa, depends on computing function
                        angle = nav.get_angle(prev_point, cur_pos, cur_pos, field_gps_coords[1])
                        angle_pid = angle * config.PID
                        wheels_angle_sm = angle_pid * config.A_ONE_DEGREE_IN_SMOOTHIE  # smoothie -V == left, V == right
                        ad_wheels_pos = smoothie.get_adapter_current_coordinates()["A"]
                        sm_wheels_pos = smoothie.get_smoothie_current_coordinates()["A"]

                        # TO DO: not working yet as moving TO abs 4, need work in relative with checks
                        if wheels_angle_sm > config.A_MAX:
                            msg = "Wheels turn value changed from " + wheels_angle_sm + " to config.A_MAX = " +\
                                  config.A_MAX
                            print(msg)
                            logger.write(msg + "\n")
                            wheels_angle_sm = config.A_MAX
                        elif wheels_angle_sm < config.A_MIN:
                            msg = "Wheels turn value changed from " + wheels_angle_sm + " to config.A_MIN = " +\
                                  config.A_MIN
                            print(msg)
                            logger.write(msg + "\n")
                            wheels_angle_sm = config.A_MIN

                        """
                        # old
                        print("A:", field_gps_coords[0], "B:", field_gps_coords[1], "Prev:", prev_point, "Cur:",
                              cur_pos)
                        print("PID:", PID, "RAS:", ras, "RAR:", rar, "Computed degrees:", angle, "Sending to smoothie:", wheels_angle_sm)
                        """

                        msg = "Adapter wheels pos (target): " + ad_wheels_pos + " Smoothie wheels pos (current) " +\
                              sm_wheels_pos
                        print(msg)
                        logger.write(msg + "\n")
                        msg = "Angle: " + angle + " Angle * PID: " + angle_pid + " Sending B value to smoothie: " +\
                              wheels_angle_sm
                        print(msg)
                        logger.write(msg + "\n")

                        # next block indent
                        print()
                        logger.write("\n")

                        prev_point = cur_pos
                        response = smoothie.nav_turn_wheels_to(wheels_angle_sm, config.A_F_MAX)

                        msg = "Smoothie response: " + response
                        print(msg)
                        logger.write(msg)
                    adapter_points_history = gps.get_last_positions_list()
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
        if len(used_points_history) > 0:
            save_gps_coordinates(used_points_history, "used_gps_history " + get_current_time() + ".txt")
        else:
            msg = "used_gps_history list has 0 elements!"
            print(msg)
            logger.write(msg)
        if len(adapter_points_history) > 0:
            save_gps_coordinates(adapter_points_history, "adapter_gps_history " + get_current_time() + ".txt")
        else:
            msg = "adapter_gps_history list has 0 elements!"
            print(msg)
            logger.write(msg)
        logger.close()
        # """
        sP.endSession()
        client.closeConnection()
        sP.stopServer()
        # """


if __name__ == '__main__':
    main()
