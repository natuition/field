class GPSStub:
    def __init__(self, ser_port, ser_baudrate, last_pos_count):
        if last_pos_count < 1:
            raise ValueError("last_pos_count shouldn't be less than 1")

        self._position_is_fresh = False
        self._last_pos_count = last_pos_count
        self._last_pos_container = []

        self._keep_thread_alive = True

        self.point_flag = True

    def get_fresh_position(self):
        point = [46.1578732, -1.1348602, 6] if self.point_flag else [46.1578733, -1.1348602, 6]
        self.point_flag = not self.point_flag
        return point

    def get_last_position(self):
        return self.get_fresh_position()

    def get_last_positions_list(self):
        return self._last_pos_container.copy()

    def get_stored_pos_count(self):
        return len(self._last_pos_container)

    def disconnect(self):
        pass


class VESCStub:
    def __init__(self, rpm, moving_time, alive_freq, check_freq, ser_port, ser_baudrate):
        self._rpm = rpm
        self._moving_time = moving_time
        self._alive_freq = alive_freq
        self._check_freq = check_freq
        self._allow_movement = False
        self._keep_thread_alive = True

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
        self._x_cur = 0
        self._y_cur = 0
        self._z_cur = 0
        self._a_cur = 0
        self._b_cur = 0
        self._c_cur = 0

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


class Client():
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
