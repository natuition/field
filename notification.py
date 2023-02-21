import threading
from time import sleep
import navigation
import requests
from contextlib import closing
from websocket import create_connection
import datetime
from pytz import timezone
import json
from config import config


class SyntheseRobot:
    OP = "Robot_OP"
    HS = "Robot_HS"
    ANTI_THEFT = "Robot_ANTI_THEFT"


class NotificationClient:

    def __init__(self, time_start):
        self.__port = 8080
        self.__ip = "172.16.3.5"
        self.__time_start = datetime.datetime.strptime(
            time_start, "%d-%m-%Y %H-%M-%S %f").replace(tzinfo=timezone('Europe/Berlin'))
        self.__keep_thread_alive = True
        self.__input_voltage = None

        self.__init_treated_plant = False
        self.__treated_plant = None

        self.__init_field = False
        self.__field_id = None
        self.__field = None
        self.__field_name = None

        self.__path_point_number = 0

        self.__alive_sending_timeout = config.ALIVE_SENDING_TIMEOUT
        self.__max_lenght_point_history = config.MAX_LENGHT_POINT_HISTORY
        self.__robot_sn = config.ROBOT_SN

        self.__coordinate_with_extracted_weed = list()
        self.__extracted_weeds = dict()
        self.__last_extracted_weeds_send = dict()
        self.__sync_locker = threading.Lock()

        self.__continuous_information_sending = config.CONTINUOUS_INFORMATION_SENDING
        self.__init_robot_on_datagathering()

        self.__report_th = threading.Thread(
            target=self.__report_th_tf, daemon=True)
        self.__report_th.start()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def stop(self):
        print("[Notification] Stopping service...")
        self.__keep_thread_alive = False
        self.__report_th.join()

    def setStatus(self, status: SyntheseRobot):
        # Todo
        pass

    def isConnected(self):
        return self.__keep_thread_alive

    def set_current_coordinate(self, current_coordinate):
        last_coordinate_with_extracted_weed = dict()
        if self.__extracted_weeds:
            extracted_weeds = dict()
            for key, value in self.__extracted_weeds.items():
                if key in self.__last_extracted_weeds_send:
                    if value - self.__last_extracted_weeds_send[key] != 0:
                        extracted_weeds[key] = value - \
                            self.__last_extracted_weeds_send[key]
                else:
                    extracted_weeds[key] = value
            self.__last_extracted_weeds_send = dict(
                self.__extracted_weeds)
            if extracted_weeds:
                last_coordinate_with_extracted_weed["extracted_weeds"] = extracted_weeds
        last_coordinate_with_extracted_weed["path_point_number"] = self.__path_point_number
        self.__path_point_number += 1
        with self.__sync_locker:
            last_coordinate_with_extracted_weed["current_coordinate"] = current_coordinate
            self.__coordinate_with_extracted_weed.append(
                last_coordinate_with_extracted_weed)
            if len(self.__coordinate_with_extracted_weed) > self.__max_lenght_point_history:
                self.__coordinate_with_extracted_weed.pop(0)

    def set_treated_plant(self, treated_plant):
        self.__treated_plant = treated_plant

    def set_field(self, field, field_name):
        self.__field = field
        self.__field_name = field_name
        self.antiTheftZone = navigation.AntiTheftZone(field)

    def set_input_voltage(self, input_voltage):
        self.__input_voltage = input_voltage

    def set_extracted_plants(self, extracted_weeds):
        self.__extracted_weeds = extracted_weeds

    def is_continuous_information_sending(self):
        return self.__continuous_information_sending

    def __report_th_tf(self):
        while not self.__init_field or not self.__init_treated_plant:
            if not self.__init_treated_plant and self.__treated_plant is not None:
                self.__send_treated_weed()
                self.__init_treated_plant = True
            if not self.__init_field and self.__field is not None and self.__field_name is not None:
                self.__send_field()
            sleep(0.5)
        # init session
        send_session = {
            "start_time": self.__time_start.isoformat(),
            "end_time": self.__time_start.isoformat(),
            "robot_serial_number": self.__robot_sn,
            "field_id": self.__field_id
        }
        response = requests.post(
            f"http://{self.__ip}:{self.__port}/api/v1/data_gathering/session", json=send_session)

        if response != 201:
            Exception(
                f"Error when sending session: {response.status_code}.")

        session_id = response.json()["id"]

        while self.__keep_thread_alive:
            try:
                with closing(create_connection(f"ws://{self.__ip}:{self.__port}/api/v1/data_gathering/ws/{self.__robot_sn}/{session_id}")) as conn:
                    print("[Notification] Connected")
                    while self.__keep_thread_alive:
                        frame = dict()
                        if self.__coordinate_with_extracted_weed:
                            coordinate_with_extracted_weed = {
                                "coordinate_with_extracted_weed": list(
                                    self.__coordinate_with_extracted_weed)
                            }
                            frame.update(coordinate_with_extracted_weed)
                            with self.__sync_locker:
                                self.__coordinate_with_extracted_weed.clear()
                            if frame:
                                conn.send(json.dumps(frame))
                            if self.__input_voltage:
                                send_vesc_statistic = {
                                    "session_id": session_id,
                                    "voltage": self.__input_voltage,
                                    "timestamp": datetime.datetime.now().isoformat()
                                }
                                response = requests.post(
                                    f"http://{self.__ip}:{self.__port}/api/v1/data_gathering/vesc_statistic", json=send_vesc_statistic)

                                if response != 201:
                                    Exception(
                                        f"Error when sending input voltage: {response.status_code}.")

                                self.__input_voltage = None
                        sleep(self.__alive_sending_timeout)
            except BrokenPipeError:
                print("[Notification] Reconnecting...")
            print("[Notification] Disconnected")

    def __init_robot_on_datagathering(self):
        response = requests.post(
            f"http://{self.__ip}:{self.__port}/api/v1/data_gathering/robot", json={"serial_number": self.__robot_sn})
        if response.status_code != 201 and response.status_code != 400:
            raise Exception("Can't save robot in database")

    def __send_treated_weed(self):
        for weed_type_name in self.__treated_plant:
            # create weed_type_name
            weed_type = {"label": weed_type_name}
            response = requests.post(
                f"http://{self.__ip}:{self.__port}/api/v1/data_gathering/weed_type", json=weed_type)
            if response != 201 and response != 200:
                Exception(
                    f"Error when sending treated weed: {response.status_code}.")

    def __send_field(self):
        # field : [A, B, C, D] ou A : [lat, long]
        # field_name : string
        # create field_name
        field = {"label": self.__field_name,
                 "robot_serial_number": self.__robot_sn}
        response = requests.post(
            f"http://{self.__ip}:{self.__port}/api/v1/data_gathering/field", json=field)

        if response != 201 and response != 200:
            Exception(f"Error when sending field: {response.status_code}.")

        self.__field_id = response.json()["id"]

        if response.status_code == 201:
            for point in self.__field:
                # create gps_point
                send_gps_point = {
                    "quality": 0, "latitude": point[0], "longitude": point[1]}
                response = requests.post(
                    f"http://{self.__ip}:{self.__port}/api/v1/data_gathering/gps_point", json=send_gps_point)

                if response.status_code == 201:
                    gps_point_id = response.json()["id"]
                    # link to field_name
                    send_field_corner = {
                        "field_id": self.__field_id, "gps_point_id": gps_point_id}
                    response = requests.post(
                        f"http://{self.__ip}:{self.__port}/api/v1/data_gathering/field_corner", json=send_field_corner)
        self.__init_field = True
