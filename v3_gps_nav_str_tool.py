"""Creates two gps points AB and saves them into the file (specified in config), then moves from B to A point."""


import adapters
import navigation
from config import config
import time
import datetime


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


def ask_for_ab_points(gps: adapters.GPSUblockAdapter):
    """Ask user for moving vector AB points"""

    # TO DO: remove time.sleep when blocking gps.get_fresh_position function will be done
    sleep_time = 1
    # wait for gps
    while gps.get_stored_pos_count() == 0:
        time.sleep(0.05)

    input("Press enter to save point A")
    time.sleep(sleep_time)
    point_a = gps.get_last_position()
    print("Point A saved.")
    input("Press enter to save point B")
    time.sleep(sleep_time)
    point_b = gps.get_last_position()
    print("Point B saved.")
    return [point_b, point_a]


def main():
    points_history = []
    prev_point = None
    try:
        nav = navigation.GPSComputing()

        print("Initializing...", end=" ")
        with adapters.SmoothieAdapter(config.SMOOTHIE_HOST) as smoothie:
            with adapters.VescAdapter(config.VESC_RPM, config.VESC_MOVING_TIME, config.VESC_ALIVE_FREQ,
                                      config.VESC_CHECK_FREQ, config.VESC_PORT, config.VESC_BAUDRATE) as vesc_engine:
                with adapters.GPSUblockAdapter(config.GPS_PORT, config.GPS_BAUDRATE,
                                               config.GPS_POSITIONS_TO_KEEP) as gps:
                    print("done.")
                    field_gps_coords = ask_for_ab_points(gps)
                    save_gps_coordinates(field_gps_coords, "field " + get_current_time() + ".txt")
                    input("Press enter to start moving")
                    # start moving forward
                    vesc_engine.start_moving()

                    # main navigation control loop
                    # TO DO: this loop is working much faster than gps, need to evaluate sleep time or waiting for new P
                    while True:
                        time.sleep(0.2)
                        cur_pos = gps.get_last_position()
                        # if str(cur_pos) == prev_point:
                        #    continue
                        # prev_point = str(cur_pos)
                        points_history.append(cur_pos.copy())
                        # check if arrived
                        # TO DO: it won't work if deviation > course destination diff, as robot will be far away on some
                        # side and will never get too close to the path ending point
                        distance = nav.get_distance(cur_pos, field_gps_coords[1])

                        print("Distance to B:", distance)

                        if distance <= config.COURSE_DESTINATION_DIFF:
                            vesc_engine.stop_moving()
                            print("Arrived (allowed destination distance difference", config.COURSE_DESTINATION_DIFF,
                                  "mm)")
                            break

                        # check for course deviation. if deviation is bigger than a threshold
                        deviation, side = nav.get_deviation(field_gps_coords[0], field_gps_coords[1], cur_pos)
                        nav_wheels_position = smoothie.get_adapter_current_coordinates()["A"]

                        print("A:", field_gps_coords[0], "B:", field_gps_coords[1], "Cur:", cur_pos)
                        print("Deviation:", deviation, "side flag:", side)

                        if deviation > config.COURSE_SIDE_DEVIATION_MAX:
                            # deviation to the left side
                            if side == -1:
                                # if not turned wheels yet
                                # TO DO: try to make different wheels turning values for different deviation
                                if not (-config.COURSE_ADJ_SMC_VAL - 0.001 < nav_wheels_position < -config.COURSE_ADJ_SMC_VAL + 0.001):
                                    # TO DO: check abs(COURSE_ADJ_SMC_VAL - nav_wheels_position) and set bigger or
                                    # lesser turning value
                                    smoothie.nav_turn_wheels_to(-config.COURSE_ADJ_SMC_VAL, config.A_F_MAX)
                            # deviation to the right side
                            elif side == 1:
                                # if not turned wheels yet
                                # TO DO: try to make different wheels turning values for different deviation
                                if not (config.COURSE_ADJ_SMC_VAL - 0.001 < nav_wheels_position < config.COURSE_ADJ_SMC_VAL + 0.001):
                                    # TO DO: check abs(COURSE_ADJ_SMC_VAL - nav_wheels_position) and set bigger or
                                    # lesser turning value
                                    smoothie.nav_turn_wheels_to(config.COURSE_ADJ_SMC_VAL, config.A_F_MAX)
                            # it's an error if deviation is big but side flag is telling us that we're on moving vector
                            else:
                                print("Abnormal state: deviation =", deviation, "at", side,
                                      "side. Emergency stop applied.")
                                exit(1)
                        # if deviation is ok (0 or less than an adj. threshold)
                        else:
                            # if not turned wheels yet
                            # TO DO: try to make different wheels turning values for different deviation
                            if not (-0.001 < nav_wheels_position < 0.001):
                                # TO DO: check abs(COURSE_ADJ_SMC_VAL - nav_wheels_position) and set bigger or
                                # lesser turning value
                                smoothie.nav_align_wheels_center(config.A_F_MAX)
        print("Done!")
    except KeyboardInterrupt:
        print("Stopped by a keyboard interrupt (Ctrl + C)")
    finally:
        save_gps_coordinates(points_history, "gps_history " + get_current_time() + ".txt")


if __name__ == '__main__':
    main()
