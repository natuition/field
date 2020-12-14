import platform
import datetime
import os
import serial.tools.list_ports
import threading
from time import sleep
import psutil
import glob
import detection
import cv2 as cv


class ImageSaver:
    """
    Implements flexible ways to save images and detected objects on them
    """

    def __init__(self, counter: int = 0):
        if type(counter) is not int:
            raise TypeError("'counter' type should be int, got " + type(counter).__name__)

        self.__counter = counter

    def get_counter(self):
        return self.__counter

    def set_counter(self, counter: int):
        if type(counter) is not int:
            raise TypeError("'counter' type should be int, got " + type(counter).__name__)

        self.__counter = counter

    def save_image(self, image, directory: str, extension: str = "jpg", sep: str = "_", specific_name=None, label=None,
                   plants_boxes=None):
        """
        Saves image in different ways.

        extension should not contain point separator: "jpg"
        specific_name should not contain file extension
        """

        # define file name (image and txt if plants_boxes are passed)
        if specific_name:
            file_name = specific_name
        else:
            file_name = get_current_time() + sep + str(self.__counter)
            self.__counter += 1
            if label:
                file_name += sep + label

        # save image
        cv.imwrite(directory + file_name + "." + extension, image)

        # save plants boxes if passed
        if type(plants_boxes) is list and len(plants_boxes) > 0:
            with open(directory + file_name + ".txt", "w") as txt_file:
                plant_box: detection.DetectedPlantBox
                for plant_box in plants_boxes:
                    txt_file.write(plant_box.get_as_yolo(return_as_text=True) + "\n")


class TrajectorySaver:
    """Provides safe gps points saving (robot's trajectory)"""

    def __init__(self, full_path):
        self.__full_path = full_path
        self.__output_file = open(full_path, "w")
        self.__last_received_point = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def close(self):
        self.__output_file.close()

    def save_point(self, point: list, save_raw=False, flush_immediately=True):
        """
        Saves given point if it is different from previous received gps point.
        """

        if not str(point) == self.__last_received_point:
            self.__last_received_point = str(point)

            str_point = str(point[0]) + " " + str(point[1]) + "\n" if save_raw else str(point) + "\n"
            self.__output_file.write(str_point)

            if flush_immediately:
                self.__output_file.flush()


class MemoryManager:
    """
    Provides tools for removing obsolete files from given directory.
    Supports:
    - manual blocking cleaning: called manually from code, blocks caller thread until completely executed
    - manual non-blocking cleaning: called manually, does single time cleaning in a separate thread, can be stopped at
      any time
    - auto-cleaning: automatically calls for cleaner each N seconds (set as parameter), can be stopped at any time
    """

    def __init__(self, path: str, files_to_keep_count: int = 600, check_frequency_seconds: int = 60,
                 memory_threshold: int = 95368):
        self.__files_to_keep_count = files_to_keep_count
        self.__path = path
        self.__check_frequency_seconds = check_frequency_seconds
        self.__memory_threshold = memory_threshold

        self.__auto_cleaner_thread = threading.Thread(target=self.__auto_cleaner_tf, daemon=True)
        self.__keep_auto_cleaner_thread_alive = False

        self.__manual_cleaner_thread = threading.Thread(target=self.__manual_cleaner_tf, daemon=True)
        self.__keep_manual_cleaner_thread_alive = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop_auto_cleaning()
        self.stop_clean_manual_non_blocking()

    def start_auto_cleaning(self):
        self.__keep_auto_cleaner_thread_alive = True
        self.__auto_cleaner_thread.start()

    def stop_auto_cleaning(self):
        self.__keep_auto_cleaner_thread_alive = False

    def start_clean_manual_non_blocking(self):
        self.__keep_manual_cleaner_thread_alive = True
        self.__manual_cleaner_thread.run()

    def stop_clean_manual_non_blocking(self):
        self.__keep_manual_cleaner_thread_alive = False

    def start_clean_manual_blocking(self):
        """This function does disk cleaning, blocking until executed"""

        files_to_delete = self.__get_files_to_delete_list()
        for file in files_to_delete:
            os.remove(file)

    def __get_files_to_delete_list(self):
        # TODO: can be optimized (seems like getmtime is called multiple times during sorting)
        full_files_list = sorted(glob.glob(self.__path + "*"), key=os.path.getmtime, reverse=True)
        return full_files_list[self.__files_to_keep_count:]

    def __manual_cleaner_tf(self):
        """Manual cleaner thread target function"""

        files_to_delete = self.__get_files_to_delete_list()
        for file in files_to_delete:
            if self.__keep_manual_cleaner_thread_alive:
                os.remove(file)
            else:
                break

    def __auto_cleaner_tf(self):
        """Auto cleaner thread target function"""

        while self.__keep_auto_cleaner_thread_alive:
            if (round((psutil.disk_usage("/").used / (2 ** 20)), 2)) > self.__memory_threshold:
                files_to_delete = self.__get_files_to_delete_list()
                for file in files_to_delete:
                    if self.__keep_auto_cleaner_thread_alive:
                        os.remove(file)
                    else:
                        return
            sleep(self.__check_frequency_seconds)


class Logger:
    """
    Writes into the file with specified name str data, flushing data on each receiving
    """

    def __init__(self, file_name, add_time=True, time_sep=" "):
        self._file = open(file_name, "w")
        self.__add_time = add_time
        self.__time_sep = time_sep

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._file.__exit__(exc_type, exc_val, exc_tb)

    def write_and_flush(self, s):
        if self.__add_time:
            s = get_current_time() + self.__time_sep + s
        self._file.write(s)
        self._file.flush()

    def write(self, s):
        if self.__add_time:
            s = get_current_time() + self.__time_sep + s
        self._file.write(s)

    def close(self):
        self._file.close()


def get_current_time():
    """Returns current time as formatted string"""

    return datetime.datetime.now().strftime("%d-%m-%Y %H-%M-%S %f")


def create_directories(*args):
    """Creates directories, receives any args count, each arg is separate dir"""

    for path in args:
        if not os.path.exists(path):
            try:
                os.mkdir(path)
            except OSError:
                print("Creation of the directory %s failed" % path)
            else:
                print("Successfully created the directory %s " % path)
        else:
            print("Directory %s is already exists" % path)


def get_path_slash():
    return "\\" if platform.system() == "Windows" else "/"


def get_smoothie_vesc_addresses():
    equipment_by_port = dict()
    for port, desc, other in sorted(serial.tools.list_ports.comports()):
        if "Smoothie" in desc:
            equipment_by_port["smoothie"] = port
        if "ChibiOS/RT" in desc:
            equipment_by_port["vesc"] = port
    return equipment_by_port
