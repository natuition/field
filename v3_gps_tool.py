"""Creates gps points and saves them into the file (file name is specified in the config)"""


from config import config
import adapters


def save_gps_coordinates(points: list, file_name):
    with open(file_name, "w") as file:
        for point in points:
            str_point = str(point[0]) + " " + str(point[1]) + "\n"
            file.write(str_point)


def main():
    with adapters.GPSUbloxAdapter(config.GPS_PORT, config.GPS_BAUDRATE, config.GPS_POSITIONS_TO_KEEP) as gps:
        i = 1
        points = []

        while True:
            cmd = input("Press enter to save point " + str(i) + ", type anything to exit.")
            if cmd != "":
                break
            point = gps.get_fresh_position()
            points.append(point)
            print("Point", i, "saved.\n")

        save_gps_coordinates(points, config.INPUT_GPS_FIELD_FILE)
        print("Done!")


if __name__ == '__main__':
    main()
