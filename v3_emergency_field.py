"""
Script for creation field.txt with 4 corner points.
Moves robot forward for a bit, reads vector, and generates points based on this vector and given sizes.
"""

from config import config
import adapters
import navigation

# settings
FIELD_SIZE = 15000  # mms
RPM = -11000
MOVING_TIME = 3  # seconds


def save_gps_coordinates(points: list, file_name):
    with open(file_name, "w") as file:
        for point in points:
            str_point = str(point[0]) + " " + str(point[1]) + "\n"
            file.write(str_point)


def main():
    nav = navigation.GPSComputing()
    with adapters.GPSUbloxAdapter(config.GPS_PORT, config.GPS_BAUDRATE, config.GPS_POSITIONS_TO_KEEP) as gps:
        with adapters.VescAdapter(RPM, MOVING_TIME, config.VESC_ALIVE_FREQ, config.VESC_CHECK_FREQ,
                                  config.VESC_PORT, config.VESC_BAUDRATE) as vesc_engine:
            print("Getting a starting point...")
            starting_point = gps.get_fresh_position()

            print("Moving forward...")
            vesc_engine.start_moving()
            vesc_engine.wait_for_stop()

            print("Getting point A...")
            A = gps.get_fresh_position()

            print("Computing rest points...")
            B = nav.get_coordinate(A, starting_point, 180, FIELD_SIZE)
            C = nav.get_coordinate(B, A, 90, FIELD_SIZE)
            D = nav.get_coordinate(C, B, 90, FIELD_SIZE)

            print("Saving field.txt file...")
            save_gps_coordinates([A, B, C, D], "field.txt")

            print("Done.")


if __name__ == '__main__':
    main()
