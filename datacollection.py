"""
This module provides tools for robot data and statistic collection on client (robot control script) side.
"""

import psycopg2
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
        self.__detected_plants_by_image = dict()
        self.__extracted_plants_by_image = dict()
        self.__GPSPoint_by_image = dict()
        self.__start_time = time.time()
        self.__end_time = None
        self.__working_time_formatted = None
        self.__working_time_seconds = None
        self.__sessionID = 0
        self.__database_connection = psycopg2.connect(host="localhost", port = 5432, database="postgres", user="postgres", password="Plantule69")
        self.create_session_in_database()

    def create_session_in_database(self):
        self.__record_end_time()
        cursor = self.__database_connection.cursor()
        # INSERT INTO public.sessions(start_time, end_time) VALUES (?, ?);
        start_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.__start_time))
        end_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.__end_time))
        cursor.execute("INSERT INTO sessions (start_time, end_time) VALUES(%s, %s) RETURNING id", (start_time, end_time))
        self.__sessionID = cursor.fetchone()[0]
        cursor.execute("COMMIT")
        cursor.close()

    def update_end_time_session_in_database(self):
        self.__record_end_time()
        cursor = self.__database_connection.cursor()
        end_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.__end_time))
        cursor.execute("UPDATE sessions SET end_time=%s WHERE id=%s", (end_time,self.__sessionID))
        cursor.execute("COMMIT")
        cursor.close()

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

    #----------------------------------Database----------------------------------

    def add_detections_data_by_image(self, type_label: str, count: int, imagePath: str, GPSPoint: list):
        if type(count) is not int:
            raise TypeError("'count' type should be int, got " + type(count).__name__)
        if count <= 0:
            raise ValueError("'count' value should be greater than 0, got " + str(count))

        if imagePath not in self.__detected_plants_by_image:
            self.__detected_plants_by_image[imagePath] = dict()
            self.__GPSPoint_by_image[imagePath] = GPSPoint

        if type_label not in self.__detected_plants_by_image[imagePath]:
            self.__detected_plants_by_image[imagePath][type_label] = count
        else:
            self.__detected_plants_by_image[imagePath][type_label] += count

    def save_detections_data_in_database(self):
        self.update_end_time_session_in_database()

        cursor = self.__database_connection.cursor()

        for image in self.__detected_plants_by_image:
            cursor.execute("INSERT INTO images(file_name) VALUES (%s) RETURNING id", (image,))
            imageId = cursor.fetchone()[0]
            cursor.execute("INSERT INTO gpspoints(latitude, longitude, quality) VALUES (%s, %s, %s) RETURNING id", (self.__GPSPoint_by_image[image][0],self.__GPSPoint_by_image[image][1],self.__GPSPoint_by_image[image][2]))
            GPSPointId = cursor.fetchone()[0]
            cursor.execute("COMMIT")
            for label in self.__detected_plants_by_image[image]:
                cursor.execute("SELECT id FROM weedtypes WHERE label=%s",(label,))
                weedTypeId = cursor.fetchone()[0]
                cursor.execute("INSERT INTO weedsstatistics(image_id, gps_point_id, weed_type_id, session_id, detected_count) VALUES (%s, %s, %s, %s, %s)", (imageId,GPSPointId,weedTypeId,self.__sessionID,self.__detected_plants_by_image[image][label]))
                cursor.execute("COMMIT")

        cursor.close()

        self.__detected_plants_by_image.clear()
        self.__GPSPoint_by_image.clear()

    def add_extractions_data_by_image(self, type_label: str, count: int, imagePath: str, GPSPoint: list):
        if type(count) is not int:
            raise TypeError("'count' type should be int, got " + type(count).__name__)
        if count <= 0:
            raise ValueError("'count' value should be greater than 0, got " + str(count))

        self.__extracted_plants_by_image[imagePath] = dict()
        self.__GPSPoint_by_image[imagePath] = GPSPoint
        self.__extracted_plants_by_image[imagePath][type_label] = count

    def save_extractions_data_in_database(self):
        self.update_end_time_session_in_database()

        cursor = self.__database_connection.cursor()

        for image in self.__extracted_plants_by_image:
            cursor.execute("INSERT INTO images(file_name) VALUES (%s) RETURNING id", (image,))
            imageId = cursor.fetchone()[0]
            cursor.execute("INSERT INTO gpspoints(latitude, longitude, quality) VALUES (%s, %s, %s) RETURNING id", (self.__GPSPoint_by_image[image][0],self.__GPSPoint_by_image[image][1],self.__GPSPoint_by_image[image][2]))
            GPSPointId = cursor.fetchone()[0]
            cursor.execute("COMMIT")
            for label in self.__extracted_plants_by_image[image]:
                cursor.execute("SELECT id FROM weedtypes WHERE label=%s",(label,))
                weedTypeId = cursor.fetchone()[0]
                cursor.execute("INSERT INTO weedsstatistics(image_id, gps_point_id, weed_type_id, session_id, extracted_count) VALUES (%s, %s, %s, %s, %s)", (imageId,GPSPointId,weedTypeId,self.__sessionID,self.__extracted_plants_by_image[image][label]))
                cursor.execute("COMMIT")

        cursor.close()

        self.__extracted_plants_by_image.clear()
        self.__GPSPoint_by_image.clear()
        
    #----------------------------------Database----------------------------------

class ImageCollector:
    pass
