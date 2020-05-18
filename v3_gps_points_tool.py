import adapters
import time


# Paths settings
FIELD_GPS_POINTS_FILE = "field.txt"


# GPS settings
GPS_PORT = "/dev/ttyTHS1"
GPS_BAUDRATE = 19200
GPS_POSITIONS_TO_KEEP = 1


def save_gps_coordinates(points: list, file_name):
    with open(file_name, "w") as file:
        for point in points:
            str_point = str(point[0]) + " " + str(point[1]) + "\n"
            file.write(str_point)


def main():
    gps = adapters.GPSUblockAdapter(GPS_PORT, GPS_BAUDRATE, GPS_POSITIONS_TO_KEEP)
    while gps.get_stored_pos_count() == 0:
        pass

    input("Press enter to save point A")
    time.sleep(1)
    point_a = gps.get_last_position()
    print("Point A saved.")

    input("Press enter to save point B")
    time.sleep(1)
    point_b = gps.get_last_position()
    print("Point B saved.")

    save_gps_coordinates([point_a, point_b], FIELD_GPS_POINTS_FILE)

    print("Done!")


if __name__ == '__main__':
    main()
