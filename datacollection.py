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

    def __init__(self):
        self.__detected_plants = dict()
        self.__extracted_plants = dict()
        self.__start_time = time.time()
        self.__end_time = None
        self.__working_time_formatted = None
        self.__working_time_seconds = None

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
