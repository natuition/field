"""
This module provides tools for robot data and statistic collection on client (robot control script) side.
"""

import psycopg2
import time
import datetime
import os

class PathType:
    FIELD = "field"
    PLANNED_PATH= "plannedPath"
    WORKING_PATH= "workingPath"
    ALL_TYPE = [FIELD,PLANNED_PATH,WORKING_PATH]

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
        #Datas for save in database
        self.__detected_plants_by_image = dict()
        self.__extracted_plants_by_image = dict()
        self.__vesc_statistic = list()
        self.__GPSPoint_by_image = dict()
        self.__session_id = None
        self.__path_id = dict()
        self.__paths = dict()
        for typePath in PathType.ALL_TYPE:
            self.__paths[typePath] = list()
        self.__index_point_paths = dict()
        for typePath in PathType.ALL_TYPE:
            self.__index_point_paths[typePath] = 0
        self.__database_connection = psycopg2.connect(host="localhost", port = 5432, database="postgres", user="postgres", password="Plantule69")
        self.create_session_in_database()

    def create_session_in_database(self):
        self.__record_end_time()
        cursor = self.__database_connection.cursor()
        # INSERT INTO public.sessions(start_time, end_time) VALUES (?, ?);
        start_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.__start_time))
        end_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.__end_time))
        cursor.execute("INSERT INTO sessions (start_time, end_time) VALUES(%s, %s) RETURNING id", (start_time, end_time))
        self.__session_id = cursor.fetchone()[0]
        cursor.execute("COMMIT")
        cursor.close()

    def update_end_time_session_in_database(self):
        self.__record_end_time()
        cursor = self.__database_connection.cursor()
        end_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.__end_time))
        cursor.execute("UPDATE sessions SET end_time=%s WHERE id=%s", (end_time,self.__session_id))
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

    #----------------------------------Save in local database----------------------------------

    def save_gps_points_in_database(self, cursor, GPSPoint: list):

        if len(GPSPoint) == 2:
            GPSPoint.append(1)
        cursor.execute("SELECT id FROM gpspoints WHERE latitude=%s AND longitude=%s AND quality=%s",(GPSPoint[0],GPSPoint[1],GPSPoint[2]))
        GPSPointId = cursor.fetchall()
        if not GPSPointId:
            cursor.execute("INSERT INTO gpspoints(latitude, longitude, quality) VALUES (%s, %s, %s) RETURNING id", (GPSPoint[0],GPSPoint[1],GPSPoint[2]))
            GPSPointId = cursor.fetchall()
            cursor.execute("COMMIT")

        return GPSPointId[0]


    def add_detections_data_by_image(self, type_label: str, count: int, imageName: str, GPSPoint: list):
        if type(count) is not int:
            raise TypeError("'count' type should be int, got " + type(count).__name__)
        if count <= 0:
            raise ValueError("'count' value should be greater than 0, got " + str(count))

        if imageName not in self.__detected_plants_by_image:
            self.__detected_plants_by_image[imageName] = dict()
            self.__GPSPoint_by_image[imageName] = GPSPoint

        if type_label not in self.__detected_plants_by_image[imageName]:
            self.__detected_plants_by_image[imageName][type_label.replace(" ", "")] = count
        else:
            self.__detected_plants_by_image[imageName][type_label.replace(" ", "")] += count

    def save_detections_data_in_database(self):
        self.update_end_time_session_in_database()

        cursor = self.__database_connection.cursor()

        for image in self.__detected_plants_by_image:
            cursor.execute("INSERT INTO images(file_name) VALUES (%s) RETURNING id", (image,))
            imageId = cursor.fetchall()[0]
            GPSPointId = self.save_gps_points_in_database(cursor,self.__GPSPoint_by_image[image])
            for label in self.__detected_plants_by_image[image]:
                cursor.execute("SELECT id FROM weedtypes WHERE label=%s",(label,))
                weedTypeId = cursor.fetchall()[0]
                cursor.execute("INSERT INTO weedsstatistics(image_id, gps_point_id, weed_type_id, session_id, detected_count) VALUES (%s, %s, %s, %s, %s)", (imageId,GPSPointId,weedTypeId,self.__session_id,self.__detected_plants_by_image[image][label]))
                cursor.execute("COMMIT")

        cursor.close()

        self.__detected_plants_by_image.clear()
        self.__GPSPoint_by_image.clear()

    def add_extractions_data_by_image(self, type_label: str, count: int, imageName: str, GPSPoint: list):
        if type(count) is not int:
            raise TypeError("'count' type should be int, got " + type(count).__name__)
        if count <= 0:
            raise ValueError("'count' value should be greater than 0, got " + str(count))

        self.__extracted_plants_by_image[imageName] = dict()
        self.__GPSPoint_by_image[imageName] = GPSPoint
        self.__extracted_plants_by_image[imageName][type_label.replace(" ", "")] = count

    def save_extractions_data_in_database(self):
        self.update_end_time_session_in_database()

        cursor = self.__database_connection.cursor()

        for image in self.__extracted_plants_by_image:
            cursor.execute("INSERT INTO images(file_name) VALUES (%s) RETURNING id", (image,))
            imageId = cursor.fetchall()[0]
            GPSPointId = self.save_gps_points_in_database(cursor,self.__GPSPoint_by_image[image])
            for label in self.__extracted_plants_by_image[image]:
                cursor.execute("SELECT id FROM weedtypes WHERE label=%s",(label,))
                weedTypeId = cursor.fetchall()[0]
                cursor.execute("INSERT INTO weedsstatistics(image_id, gps_point_id, weed_type_id, session_id, extracted_count) VALUES (%s, %s, %s, %s, %s)", (imageId,GPSPointId,weedTypeId,self.__session_id,self.__extracted_plants_by_image[image][label]))
                cursor.execute("COMMIT")

        cursor.close()

        self.__extracted_plants_by_image.clear()
        self.__GPSPoint_by_image.clear()

    def add_vesc_data(self,temp_fet_filtered: float, temp_motor_filtered: float, avg_motor_current: float, avg_input_current: float, rpm: float, input_voltage: float):
        for var in [temp_fet_filtered,temp_motor_filtered,avg_motor_current,avg_input_current,rpm,input_voltage]:
            if type(var) is not float:
                raise TypeError("'count' type should be float, got " + type(var).__name__)
        self.__vesc_statistic.append([input_voltage,rpm,avg_motor_current,avg_input_current,temp_motor_filtered,temp_fet_filtered])

    def save_vesc_data_in_database(self):
        self.update_end_time_session_in_database()

        cursor = self.__database_connection.cursor()
        current_time_formatted = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
        for data in self.__vesc_statistic:
            cursor.execute("INSERT INTO vescstatistic(session_id, timestamp, voltage, rpm, avg_motor, avg_vesc, temp_motor, temp_vesc) VALUES (%s, %s, %s, %s, %s, %s, %s, %s)", (self.__session_id,current_time_formatted,data[0],data[1],data[2],data[3],data[4],data[5]))
            cursor.execute("COMMIT")
        cursor.close()

        self.__vesc_statistic.clear()

    def add_path_point(self, point: list, pathType: str):
        if type(point) is not list:
            raise TypeError("'count' type should be list, got " + type(point).__name__)  
        self.__paths[pathType].append(point)
    
    def save_path_in_database(self, pathType: str):
        cursor = self.__database_connection.cursor()

        if pathType not in self.__path_id:
            cursor.execute("SELECT id FROM pathtype WHERE label=%s",(pathType,))
            pathTypeId = cursor.fetchall()[0]
            current_time_formatted = time.strftime('%Y-%m-%d_%H:%M:%S', time.localtime(time.time()))
            cursor.execute("INSERT INTO paths(label, session_id, path_type_id)VALUES (%s, %s, %s) RETURNING id",(pathType+"_"+current_time_formatted,self.__session_id,pathTypeId))
            self.__path_id[pathType] = cursor.fetchone()[0]
            cursor.execute("COMMIT")

        for point in self.__paths[pathType]:
            GPSPointId = self.save_gps_points_in_database(cursor,point)

            cursor.execute("INSERT INTO pointsofpaths(point_number, path_id, gps_point_id)VALUES (%s, %s, %s)", (self.__index_point_paths[pathType],self.__path_id[pathType],GPSPointId))
            self.__index_point_paths[pathType] += 1

        self.__paths[pathType].clear()
        
    #----------------------------------Save in local database----------------------------------

class ImageCollector:
    pass
