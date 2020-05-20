import adapters
import navigation
from config import config
import time


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


def main():
    points_history = []
    prev_point = None
    try:
        # load gps points; for now it should have only two positions: path starting and ending point
        field_gps_coords = load_coordinates(config.INPUT_GPS_FIELD_FILE)
        nav = navigation.GPSComputing()

        print("Initializing...", end=" ")
        with adapters.SmoothieAdapter(config.SMOOTHIE_HOST) as smoothie:
            with adapters.VescAdapter(config.VESC_RPM, config.VESC_MOVING_TIME, config.VESC_ALIVE_FREQ,
                                      config.VESC_CHECK_FREQ, config.VESC_PORT, config.VESC_BAUDRATE) as vesc_engine:
                with adapters.GPSUblockAdapter(config.GPS_PORT, config.GPS_BAUDRATE,
                                               config.GPS_POSITIONS_TO_KEEP) as gps:
                    print("done.")
                    # start moving forward
                    vesc_engine.start_moving()
                    # wait until at least one gps point is ready
                    while gps.get_stored_pos_count() == 0:
                        time.sleep(0.05)

                    # main navigation control loop
                    # TO DO: this loop is working much faster than gps, need to evaluate sleep time or waiting for new P
                    while True:
                        cur_pos = gps.get_last_position()
                        if str(cur_pos) == prev_point:
                            continue
                        prev_point = str(cur_pos)
                        points_history.append(cur_pos)
                        # check if arrived
                        # TO DO: it won't work if deviation > course destination diff, as robot will be far away on some
                        # side and will never get too close to the path ending point
                        if nav.get_distance(cur_pos, field_gps_coords[1]) <= config.COURSE_DESTINATION_DIFF:
                            vesc_engine.stop_moving()
                            print("Arrived (allowed destination distance difference", config.COURSE_DESTINATION_DIFF,
                                  "mm)")
                            break

                        # check for course deviation. if deviation is bigger than a threshold
                        deviation, side = nav.get_deviation(field_gps_coords[0], field_gps_coords[1], cur_pos)
                        nav_wheels_position = smoothie.get_adapter_current_coordinates()["A"]
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
        save_gps_coordinates(points_history, config.OUTPUT_GPS_HISTORY_FILE)


if __name__ == '__main__':
    main()
