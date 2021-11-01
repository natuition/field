"""This module provides tools for robot data and statistic collection on client (robot control script) side.
"""

import time
import datetime
import os
import pickle
import json

class DataCollector:
    """This class stores and saves collected data as txt file, or (will be added later) sends data to the local
    database.
    """

    def __init__(self, notification, load_from_file=False, file_path="", ui_msg_queue=None):
        if load_from_file:
            structure = self.__load_from_file(file_path)
            # data
            self.__detected_plants = structure["__detected_plants"]
            self.__extracted_plants = structure["__extracted_plants"]
            self.__vesc_moving_time = structure["__vesc_moving_time"]
            self.__cork_moving_time = structure["__cork_moving_time"]
            self.__previous_sessions_working_time = structure["__previous_sessions_working_time"]
        else:
            # data
            self.__detected_plants = dict()
            self.__extracted_plants = dict()
            self.__vesc_moving_time = 0
            self.__cork_moving_time = 0
            self.__previous_sessions_working_time = 0

        self.__start_time = time.time()
        self.__notification = notification
        self.__ui_msg_queue = ui_msg_queue

    def __load_from_file(self, file_path: str):
        """Using pickle module loads data structure from a binary file with a given name

        Uses structure designed for this class (structure keys), shouldn't be used somewhere else except debug purposes.
        """

        with open(file_path, "rb") as input_file:
            return pickle.load(input_file)

    def dump_to_file(self, file_path: str):
        """Using pickle module dumps stored data into a binary file with a given name

        Uses structure designed for this class (structure keys).
        """

        structure = dict()
        structure["__detected_plants"] = self.__detected_plants
        structure["__extracted_plants"] = self.__extracted_plants
        structure["__vesc_moving_time"] = self.__vesc_moving_time
        structure["__cork_moving_time"] = self.__cork_moving_time
        structure["__previous_sessions_working_time"] = \
            self.__previous_sessions_working_time + self.get_session_working_time()

        with open(file_path, "wb") as output_file:
            pickle.dump(structure, output_file)

    def __send_to_ui(self):
        if self.__ui_msg_queue is not None:
            self.__ui_msg_queue.send(json.dumps({"datacollector": [self.__detected_plants,self.__extracted_plants]}))

    def __format_time(self, seconds):
        """Returns given seconds as formatted DD:HH:MM:SS MSS str time
        """

        return str(datetime.timedelta(seconds=seconds))

    def __swap_files(self, old_file_path, new_file_path):
        """Removes old file, then renames new file to the old one's name
        """

        if os.path.exists(old_file_path):
            os.remove(old_file_path)
        os.rename(new_file_path, old_file_path)

    def add_detections_data(self, type_label: str, count: int):
        """Adds given detections data to the stored values
        """

        if type(count) is not int:
            raise TypeError("'count' type should be int, got " + type(count).__name__)
        if count <= 0:
            raise ValueError("'count' value should be greater than 0, got " + str(count))

        if type_label in self.__detected_plants:
            self.__detected_plants[type_label] += count
        else:
            self.__detected_plants[type_label] = count

        self.__send_to_ui()

    def add_extractions_data(self, type_label: str, count: int):
        """Adds given extractions data to the stored values
        """

        if type(count) is not int:
            raise TypeError("'count' type should be int, got " + type(count).__name__)
        if count <= 0:
            raise ValueError("'count' value should be greater than 0, got " + str(count))

        if type_label in self.__extracted_plants:
            self.__extracted_plants[type_label] += count
        else:
            self.__extracted_plants[type_label] = count

        if self.__notification.is_continuous_information_sending():
            self.__notification.set_extracted_plants(self.__extracted_plants)

        self.__send_to_ui()

    def add_vesc_moving_time_data(self, seconds: float):
        """Adds given vesc moving time to the stored value
        """

        if type(seconds) not in [float, int]:
            msg = f"seconds should be int or float, got {str(type(seconds))} instead"
            raise TypeError(msg)
        if seconds < 0:
            msg = f"seconds must be equal or greater than zero, got {str(seconds)}"
            raise ValueError(msg)
        self.__vesc_moving_time += seconds

    def add_cork_moving_time_data(self, seconds: float):
        """Adds given cork moving time to the stored value
        """

        if type(seconds) not in [float, int]:
            msg = f"seconds should be int or float, got {str(type(seconds))} instead"
            raise TypeError(msg)
        if seconds < 0:
            msg = f"seconds must be equal or greater than zero, got {str(seconds)}"
            raise ValueError(msg)
        self.__cork_moving_time += seconds

    def get_session_working_time(self):
        """Returns how much seconds is past after launch till this function call moment
        """

        return time.time() - self.__start_time

    def save_all_data(self, output_file_path: str):
        """Saves data as formatted txt file
        """

        new_file = output_file_path + ".new"  # temp file to avoid data loss if any error occurred during saving
        with open(new_file, "w") as file:
            file.write("Last session robot working time: " + self.__format_time(self.get_session_working_time()) + "\n")
            file.write("Total field robot working time: " + \
                       self.__format_time(self.get_session_working_time() + \
                                          self.__previous_sessions_working_time) + "\n\n")

            # detections statistics
            file.write("Total field detections\n")
            for label in self.__detected_plants:
                file.write(label + ": " + str(self.__detected_plants[label]) + "\n")
            if len(self.__detected_plants) > 0:
                file.write("\n")

            # extractions statistic
            file.write("Total field extractions\n")
            for label in self.__extracted_plants:
                file.write(label + ": " + str(self.__extracted_plants[label]) + "\n")
            if len(self.__extracted_plants) > 0:
                file.write("\n")

            # vesc and cork statistic
            file.write("Total field vesc moving time: " + self.__format_time(self.__vesc_moving_time) + "\n")
            file.write("Total field cork moving time: " + self.__format_time(self.__cork_moving_time) + "\n")

        # swap old file with the new
        self.__swap_files(output_file_path, new_file)
