"""
This module provides tools for robot data and statistic collection on client (robot control script) side.
"""

import time
import datetime


class DataCollector:
    """
    This class stores and saves collected data as txt file, or (will be added later) sends data to the local database.
    """

    def __init__(self):
        self._extracted_plants = dict()
        self._start_time = time.time()
        self._end_time = None
        self._working_time_formatted = None
        self._working_time_seconds = None

    def record_end_time(self):
        self._end_time = time.time()
        self._working_time_seconds = self._end_time - self._start_time
        self._working_time_formatted = str(datetime.timedelta(seconds=self._working_time_seconds))

    def add_extractions_data(self, type_label, count):
        """
        Used for adding info about extracted plants.
        :param type_label:
        :param count:
        :return:
        """

        if type(count) is not int:
            raise TypeError("'count' type should be int, got " + type(count).__name__)
        if count <= 0:
            raise ValueError("'count' value should be greater than 0, got " + str(count))

        if type_label in self._extracted_plants:
            self._extracted_plants[type_label] += count
        else:
            self._extracted_plants[type_label] = count

    def save_current_data(self, file_full_path):
        # calculate session working time
        if self._end_time is None:
            self.record_end_time()

        # save data to a file
        with open(file_full_path, "w") as file:
            file.write("Last session working time: " + self._working_time_formatted + "\n")
            file.write("Last session extractions:\n")
            for label in self._extracted_plants:
                file.write(label + ": " + str(self._extracted_plants[label]) + "\n")
