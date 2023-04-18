import time
from notification import NotificationClient
import utility

time_start = utility.get_current_time()


def load_coordinates(file_path):
    positions_list = []
    with open(file_path) as file:
        for line in file:
            if line != "":
                positions_list.append(list(map(float, line.split(" "))))
    return positions_list


try:

    with NotificationClient(time_start) as notif:
        gps_point = load_coordinates("./field.txt")
        notif.set_field(gps_point, "test_1")
        notif.set_treated_weed_types({"Daisy", "Plantain", "Plaintain narroleaf"})
        extracted_plants = dict()
        extracted_plants["Daisy"] = 1
        extracted_plants["Plantain"] = 0
        gps_point[0].append(4)
        gps_point[1].append(2)
        gps_point[2].append(4)
        gps_point[3].append(4)
        while True:
            notif.set_current_coordinate(gps_point[0])
            time.sleep(0.25)
            notif.set_current_coordinate(gps_point[1])
            extracted_plants["Plantain"] += 2
            notif.set_extracted_plants(extracted_plants)
            time.sleep(0.25)
            notif.set_current_coordinate(gps_point[2])
            time.sleep(0.25)
            notif.set_current_coordinate(gps_point[3])
            extracted_plants["Daisy"] += 1
            notif.set_extracted_plants(extracted_plants)
            time.sleep(0.25)

except KeyboardInterrupt:
    msg = "Stopped by a keyboard interrupt (Ctrl + C)"
    print(msg)
