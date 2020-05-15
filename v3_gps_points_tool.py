import adapters
import time


# Paths settings
FIELD_GPS_POINTS_FILE = "field.txt"


# GPS settings
GPS_PORT = "/dev/ttyTHS1"
GPS_BAUDRATE = 19200
GPS_POSITIONS_TO_KEEP = 1


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

    with open(FIELD_GPS_POINTS_FILE, "w") as file:
        file.write(str(point_a[0]) + " " + str(point_a[1]) + "\n")
        file.write(str(point_b[0]) + " " + str(point_b[1]))

    print("Done!")


if __name__ == '__main__':
    main()
