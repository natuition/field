"""Creates two gps points AB and saves them into the file (specified in config)"""


from config import config
import adapters
import time


def save_gps_coordinates(points: list, file_name):
    with open(file_name, "w") as file:
        for point in points:
            str_point = str(point[0]) + " " + str(point[1]) + "\n"
            file.write(str_point)


def main():
    with adapters.GPSUbloxAdapter(config.GPS_PORT, config.GPS_BAUDRATE, config.GPS_POSITIONS_TO_KEEP) as gps:
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

        save_gps_coordinates([point_a, point_b], config.INPUT_GPS_FIELD_FILE)

        print("Done!")


if __name__ == '__main__':
    main()
