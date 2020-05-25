"""Creates two gps points AB and saves them into the file (specified in config), then moves from B to A point."""


import adapters
import navigation
from config import config
import time
import datetime


# Settings
PID = 1  # wheels turning degrees multiplier
MANEUVERS_DELAY_MIN = 2  # seconds

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

    # TO DO: remove time.sleep when blocking gps.get_fresh_position function will be done
    sleep_time = 1
    # wait for gps
    while gps.get_stored_pos_count() == 0:
        time.sleep(0.05)

    input("Press enter to save point B")
    time.sleep(sleep_time)
    point_b = gps.get_last_position()
    print("Point B saved.")
    input("Press enter to save point A")
    time.sleep(sleep_time)
    point_a = gps.get_last_position()
    print("Point A saved.")
    return [point_a, point_b]


def main():
    points_history = []
    try:
        nav = navigation.GPSComputing()

        print("Initializing...", end=" ")
        with adapters.SmoothieAdapter(config.SMOOTHIE_HOST) as smoothie:
            with adapters.VescAdapter(config.VESC_RPM, config.VESC_MOVING_TIME, config.VESC_ALIVE_FREQ,
                                      config.VESC_CHECK_FREQ, config.VESC_PORT, config.VESC_BAUDRATE) as vesc_engine:
                with adapters.GPSUbloxAdapter(config.GPS_PORT, config.GPS_BAUDRATE,
                                              config.GPS_POSITIONS_TO_KEEP) as gps:
                    print("done.")

                    # get route (field) and save it
                    field_gps_coords = ask_for_ab_points(gps)
                    save_gps_coordinates(field_gps_coords, "field " + get_current_time() + ".txt")

                    # start moving forward
                    input("Press enter to start moving")
                    prev_maneuver_time = time.time()
                    prev_point = gps.get_fresh_position()
                    vesc_engine.start_moving()

                    # main navigation control loop
                    while True:
                        cur_pos = gps.get_fresh_position()
                        points_history.append(cur_pos.copy())

                        if str(cur_pos) == str(prev_point):
                            print("Got the same position, added to history, but have to skip calculations")
                            continue

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

                        # do maneuvers not more often than specified value
                        cur_time = time.time()
                        if cur_time - prev_maneuver_time < MANEUVERS_DELAY_MIN:
                            continue
                        prev_maneuver_time = cur_time

                        # check for course deviation. if deviation is bigger than a threshold
                        """
                        ras = nav.get_corner(SOUTH_POINT, NORTH_POINT, cur_pos, field_gps_coords[1])
                        rar = nav.get_corner(SOUTH_POINT, NORTH_POINT, prev_point, cur_pos)
                        angle = PID * ras - rar
                        """

                        print("Prev:", prev_point, "Cur:", cur_pos, "A:", field_gps_coords[0], "B:", field_gps_coords[1])

                        angle = nav.get_angle(prev_point, cur_pos, cur_pos, field_gps_coords[1])  # or vice versa, depends on computing function
                        wheels_angle_sm = -(angle * config.A_ONE_DEGREE_IN_SMOOTHIE)  # smoothie -V = left, V = right
                        ad_wheels_pos = smoothie.get_adapter_current_coordinates()["A"]
                        sm_wheels_pos = smoothie.get_smoothie_current_coordinates()["A"]

                        """
                        print("A:", field_gps_coords[0], "B:", field_gps_coords[1], "Prev:", prev_point, "Cur:",
                              cur_pos)
                        print("PID:", PID, "RAS:", ras, "RAR:", rar, "Computed degrees:", angle, "Sending to smoothie:", wheels_angle_sm)
                        """
                        print("Adapter wheels pos (target):", ad_wheels_pos, "Smoothie wheels pos (current)",
                              sm_wheels_pos, "\n")
                        print("Angle:", angle, "Smoothie B value:", wheels_angle_sm)

                        prev_point = cur_pos
                        smoothie.nav_turn_wheels_to(wheels_angle_sm, config.A_F_MAX)
        print("Done!")
    except KeyboardInterrupt:
        print("Stopped by a keyboard interrupt (Ctrl + C)")
    finally:
        save_gps_coordinates(points_history, "gps_history " + get_current_time() + ".txt")


if __name__ == '__main__':
    main()
