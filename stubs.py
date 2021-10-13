import os
import threading
import multiprocessing
import time


class GPSUbloxAdapterStub:
    def __init__(self, ser_port, ser_baudrate, last_pos_count, gps_points_file_path: str, points_reading_delay=1):
        if last_pos_count < 1:
            raise ValueError("last_pos_count shouldn't be less than 1")
        if not os.path.isfile(gps_points_file_path):
            raise FileNotFoundError("file " + gps_points_file_path + " is not found")
        if points_reading_delay < 0:
            raise ValueError("points reading delay can't be less than zero")

        self.__points_reading_delay = points_reading_delay
        self.__stub_gps_points_reader_pool = []
        # load stub gps points
        with open(gps_points_file_path, "r") as gps_points_file:
            for line in gps_points_file.readlines():
                if line.startswith("[") and line.endswith("]\n"):
                    parsed_point = line[1:-2].split(", ")
                    try:
                        gps_point = [float(parsed_point[0]),
                                     float(parsed_point[1]),
                                     parsed_point[2].replace("'", "")]
                        self.__stub_gps_points_reader_pool.append(gps_point)
                    except (IndexError, ValueError):
                        pass
        if len(self.__stub_gps_points_reader_pool) < 1:
            raise ValueError("given stub gps points file contains no proper gps points, gps pool is empty")

        self.__position_is_fresh = False
        self.__last_pos_count = last_pos_count
        self.__last_pos_container = []
        self.__sync_locker = multiprocessing.RLock()
        self.__i = 0
        self.__reading_is_paused = False

        self.__keep_thread_alive = True
        self.__reader_thread = threading.Thread(target=self.__reader_thread_tf, daemon=True)
        self.__reader_thread.start()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    def disconnect(self):
        self.__keep_thread_alive = False

    def pause_reading(self):
        self.__reading_is_paused = True

    def continue_reading(self):
        self.__reading_is_paused = False

    def get_fresh_position(self):
        while len(self.__last_pos_container) < 1:
            pass
        with self.__sync_locker:
            self.__position_is_fresh = False
        while True:
            with self.__sync_locker:
                if not self.__position_is_fresh:
                    continue
            return self.get_last_position()

    def get_last_position(self):
        while len(self.__last_pos_container) < 1:
            pass
        with self.__sync_locker:
            return self.__last_pos_container[-1].copy()

    def get_last_positions_list(self):
        positions = []
        while len(self.__last_pos_container) < 1:
            pass
        with self.__sync_locker:
            for position in self.__last_pos_container:
                positions.append(position.copy())
            return positions

    def get_stored_pos_count(self):
        with self.__sync_locker:
            return len(self.__last_pos_container)

    def __reader_thread_tf(self):
        while self.__keep_thread_alive:
            # pause/continue reading
            while self.__reading_is_paused:
                time.sleep(0.01)

            position = self.__read_from_gps()
            with self.__sync_locker:
                if len(self.__last_pos_container) == self.__last_pos_count:
                    self.__last_pos_container.pop(0)
                self.__last_pos_container.append(position)
                self.__position_is_fresh = True

    def __read_from_gps(self):
        point = self.__stub_gps_points_reader_pool[self.__i].copy()
        self.__i += 1
        if self.__i == len(self.__stub_gps_points_reader_pool):
            self.__i = 0
        time.sleep(self.__points_reading_delay)
        return point  # [lati, longi, "point_quality"]


class VESCStub:
    def __init__(self, rpm, moving_time, alive_freq, check_freq, ser_port, ser_baudrate):
        raise NotImplementedError("this stub is need to be updated")

        self._rpm = rpm
        self._moving_time = moving_time
        self._alive_freq = alive_freq
        self._check_freq = check_freq
        self._allow_movement = False
        self._keep_thread_alive = True

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    def disconnect(self):
        pass

    def start_moving(self):
        self._allow_movement = True

    def stop_moving(self):
        self._allow_movement = False

    def wait_for_stop(self):
        pass

    def apply_rpm(self, rpm):
        self._rpm = rpm

    def set_rpm(self, rpm):
        self._rpm = rpm

    def set_moving_time(self, moving_time):
        self._moving_time = moving_time

    def set_alive_freq(self, alive_freq):
        self._alive_freq = alive_freq

    def set_check_freq(self, check_freq):
        self._check_freq = check_freq

    def is_movement_allowed(self):
        return self._allow_movement

    def pick_sensors_data(self, report_field_names):
        return None


class SmoothieAdapterStub:
    RESPONSE_OK = "ok\r\n"
    RESPONSE_ALARM_LOCK = 'error:Alarm lock\n'

    def __init__(self, smoothie_host):
        raise NotImplementedError("this stub is need to be updated")

        self._x_cur = 0
        self._y_cur = 0
        self._z_cur = 0
        self._a_cur = 0
        self._b_cur = 0
        self._c_cur = 0

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    def disconnect(self):
        pass

    def wait_for_all_actions_done(self):
        pass

    def custom_move_to(self, F: int, X=None, Y=None, Z=None, A=None, B=None, C=None):
        return self.RESPONSE_OK

    def custom_move_for(self, *args):
        return self.RESPONSE_OK

    def ext_cork_up(self):
        return self.RESPONSE_OK

    def get_adapter_current_coordinates(self):
        return {
            "X": 0,
            "Y": 0,
            "Z": 0,
            "A": 0,
            "B": 0
            # "C": 0
        }

    def get_smoothie_current_coordinates(self):
        return {
            "X": 0,
            "Y": 0,
            "Z": 0,
            "A": 0,
            "B": 0
            # "C": 0
        }

    def set_current_coordinates(self, X=None, Y=None, Z=None, A=None, B=None, C=None):
        return self.RESPONSE_OK

    def nav_turn_wheels_to(self, *args):
        return self.RESPONSE_OK


class Client:
    """
        Class manage the client which sends location data.
    """

    def __init__(self, port):
        pass

    def connectionToServer(self):
        return True

    def sendData(self, data):
        return True

    def closeConnection(self):
        pass
