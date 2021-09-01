"""
This module provides tools for robot data and statistic collection on client (robot control script) side.
"""

import time
import datetime
import os


class DataCollector:
    """
    This class stores and saves collected data as txt file, or (will be added later) sends data to the local database.
    """

    def __init__(self, notification):
        # data
        self.__detected_plants = dict()
        self.__extracted_plants = dict()
        self.__vesc_moving_time = 0
        self.__cork_moving_time = 0

        self.__start_time = time.time()
        self.__end_time = None
        self.__working_time_formatted = None
        self.__working_time_seconds = None
        self.__notification = notification

    def __record_end_time(self):
        self.__end_time = time.time()
        self.__working_time_seconds = self.__end_time - self.__start_time
        self.__working_time_formatted = str(datetime.timedelta(seconds=self.__working_time_seconds))

    def __swap_files(self, old_file_path, new_file_path):
        if os.path.exists(old_file_path):
            os.remove(old_file_path)
        os.rename(new_file_path, old_file_path)

    def add_detections_data(self, type_label: str, count: int):
        if type(count) is not int:
            raise TypeError("'count' type should be int, got " + type(count).__name__)
        if count <= 0:
            raise ValueError("'count' value should be greater than 0, got " + str(count))

        if type_label in self.__detected_plants:
            self.__detected_plants[type_label] += count
        else:
            self.__detected_plants[type_label] = count

    def save_detections_data(self, file_full_path: str):
        self.__record_end_time()

        new_file = file_full_path + ".new"  # temp file to avoid data loss if any error occurred during saving
        with open(new_file, "w") as file:
            file.write("Last session working time: " + self.__working_time_formatted + "\n")
            file.write("Last session detections\n")
            for label in self.__detected_plants:
                file.write(label + ": " + str(self.__detected_plants[label]) + "\n")

        # swap old file with the new
        self.__swap_files(file_full_path, new_file)

    def add_extractions_data(self, type_label: str, count: int):
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

    def save_extractions_data(self, file_full_path: str):
        self.__record_end_time()

        new_file = file_full_path + ".new"  # temp file to avoid data loss if any error occurred during saving
        with open(new_file, "w") as file:
            file.write("Last session working time: " + self.__working_time_formatted + "\n")
            file.write("Last session extractions\n")
            for label in self.__extracted_plants:
                file.write(label + ": " + str(self.__extracted_plants[label]) + "\n")

        # swap old file with the new
        self.__swap_files(file_full_path, new_file)

    def add_vesc_moving_time_data(self, seconds: float):
        if type(seconds) not in [float, int]:
            msg = f"seconds should be int or float, got {str(type(seconds))} instead"
            raise TypeError(msg)
        if seconds < 0:
            msg = f"seconds must be equal or greater than zero, got {str(seconds)}"
            raise ValueError(msg)
        self.__vesc_moving_time += seconds

    def save_vesc_moving_time_data(self):
        raise NotImplementedError("This function is not implemented yet")

    def add_cork_moving_time_data(self, seconds: float):
        if type(seconds) not in [float, int]:
            msg = f"seconds should be int or float, got {str(type(seconds))} instead"
            raise TypeError(msg)
        if seconds < 0:
            msg = f"seconds must be equal or greater than zero, got {str(seconds)}"
            raise ValueError(msg)
        self.__cork_moving_time += seconds

    def save_cork_moving_time_data(self):
        raise NotImplementedError("This function is not implemented yet")

    def save_all_data(self, output_file_path: str):
        self.__record_end_time()

        new_file = output_file_path + ".new"  # temp file to avoid data loss if any error occurred during saving
        with open(new_file, "w") as file:
            file.write("Last session working time: " + self.__working_time_formatted + "\n\n")

            # detections statistics
            file.write("Last session detections\n")
            for label in self.__detected_plants:
                file.write(label + ": " + str(self.__detected_plants[label]) + "\n")
            if len(self.__detected_plants) > 0:
                file.write("\n")

            # extractions statistic
            file.write("Last session extractions\n")
            for label in self.__extracted_plants:
                file.write(label + ": " + str(self.__extracted_plants[label]) + "\n")
            if len(self.__extracted_plants) > 0:
                file.write("\n")

            # vesc and cork statistic
            file.write("Vesc total moving time: " + str(datetime.timedelta(seconds=self.__vesc_moving_time)) + "\n")
            file.write("Cork total moving time: " + str(datetime.timedelta(seconds=self.__cork_moving_time)) + "\n")

        # swap old file with the new
        self.__swap_files(output_file_path, new_file)
