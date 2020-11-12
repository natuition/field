import platform
import datetime
import os
import serial.tools.list_ports
import threading
from time import sleep
import psutil


class MemoryManager:
    """
    Provides tools for removing obsolete files from given directory.
    Supports:
    - manual blocking cleaning: called manually from code, blocks caller thread until completely executed
    - manual non-blocking cleaning: called manually, does single time cleaning in a separate thread, can be stopped at any time
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
        os.chdir(self.__path)
        full_files_list = sorted(os.listdir(os.getcwd()), key=os.path.getmtime, reverse=True)
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
