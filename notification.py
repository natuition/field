import threading
from time import sleep
import navigation
import requests
import websocket
from datetime import datetime
import json
from config import config
import socket
import time
import _thread as thread
import pytz
import enum

class RobotSynthesis(enum.Enum):
    OP = "OP"
    WORKING = "WORKING"
    HS = "HS"
    ANTI_THEFT = "ANTI_THEFT"


class RobotSynthesisServer:
    def __init__(self, fleet_tick_delay=60):
        self.__fleet_tick_delay = fleet_tick_delay
        self.__fleet_ip = config.DATAGATHERING_HOST
        self.__fleet_port = config.DATAGATHERING_PORT

        self.__host = config.ROBOT_SYNTHESIS_HOST
        self.__port = config.ROBOT_SYNTHESIS_PORT

        self.__robot_stynthesis = RobotSynthesis.OP
        self.__robot_sn = config.ROBOT_SN

        # clients connection listener
        self.__conn_listener = socket.socket()
        self.__conn_listener.bind((self.__host, self.__port))
        self.__conn_listener.listen(5)
        self.__keep_conn_listener_alive = True
        self.__conn_accept_th = threading.Thread(
            target=self.__conn_accept_tf,
            name="__conn_accept_th",
            daemon= True)

        self.__keep_robot_stynthesis_alive = True
        self.__robot_stynthesis_th = threading.Thread(
            target=self.__robot_stynthesis_tf,
            name="__robot_stynthesis_th",
            daemon= True)

        self.__conn_accept_th.start()
        self.__robot_stynthesis_th.start()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def close(self):
        self.__keep_conn_listener_alive = False
        self.__keep_robot_stynthesis_alive = False
        self.__conn_listener.close()

    def __on_new_client(self, clientsocket, addr):
        while self.__keep_conn_listener_alive: 
            try:
                msg = clientsocket.recv(1024)
                if not msg:
                    break
                self.__robot_stynthesis = RobotSynthesis[msg.decode()]
            except KeyboardInterrupt:
                print(
                    "KeyboardInterrupt (parent process should get it instead)")
        clientsocket.close()

    def __conn_accept_tf(self):
        while self.__keep_conn_listener_alive:
            try:
                client, address = self.__conn_listener.accept()
                thread.start_new_thread(
                    self.__on_new_client, (client, address))
            except KeyboardInterrupt:
                print(
                    "KeyboardInterrupt (parent process should get it instead)")
            except Exception as ex:
                if not self.__keep_conn_listener_alive:
                    print(ex)

    def __robot_stynthesis_tf(self):
        while self.__keep_robot_stynthesis_alive:
            try:
                send_robot_stynthesis = {
                    "robot_synthesis": self.__robot_stynthesis.value,
                    "robot_serial_number": self.__robot_sn
                }
                response = requests.post(
                    f"http://{self.__fleet_ip}:{self.__fleet_port}/api/v1/data_gathering/robot_status", json=[send_robot_stynthesis])
                time.sleep(self.__fleet_tick_delay)
            except KeyboardInterrupt:
                print("Connections accepting thread got a KeyboardInterrupt (parent process should get it instead)")

class RobotSynthesisClient:
    def __init__(self):
        self.__host = config.ROBOT_SYNTHESIS_HOST
        self.__port = config.ROBOT_SYNTHESIS_PORT
        self.__robot_synthesis_server_conn = None

        self.__last_robot_synthesis = RobotSynthesis.OP
        self.__last_robot_synthesis_locker = threading.Lock()

        self.__keep_robot_synthesis_server_writing_alive = True
        self.__robot_synthesis_server_writer_th = threading.Thread(
            target=self.__robot_synthesis_server_writing_tf,
            name="__robot_synthesis_server_writer_th",
            daemon=True)

        self.__robot_synthesis_server_writer_th.start()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def close(self):
        self.__keep_robot_synthesis_server_writing_alive = False
        self.__robot_synthesis_server_conn.close()

    def __reconnect(self):
        if self.__robot_synthesis_server_conn is not None:
            try:
                self.__robot_synthesis_server_conn.shutdown(socket.SHUT_RDWR)
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            finally:
                pass

            try:
                self.__robot_synthesis_server_conn.close()
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            finally:
                pass

        while self.__keep_robot_synthesis_server_writing_alive:
            try:
                self.__robot_synthesis_server_conn = socket.socket()
                error_indicator = self.__robot_synthesis_server_conn.connect_ex(
                    (self.__host, self.__port))
                if error_indicator == 0:
                    break
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            finally:
                pass

    def set_last_robot_synthesis(self, robot_synthesis: RobotSynthesis):
        with self.__last_robot_synthesis_locker:
            self.__last_robot_synthesis = robot_synthesis

    def __robot_synthesis_server_writing_tf(self):
        need_to_reconnect = False

        last_send_robot_synthesis = ""

        while self.__keep_robot_synthesis_server_writing_alive:
            if need_to_reconnect:
                self.__reconnect()
                need_to_reconnect = False

            try:
                if self.__robot_synthesis_server_conn is None:
                    need_to_reconnect = True
                    continue

                if last_send_robot_synthesis != self.__last_robot_synthesis:
                    with self.__last_robot_synthesis_locker:
                        self.__robot_synthesis_server_conn.send(self.__last_robot_synthesis.value.encode())
                    last_send_robot_synthesis = self.__last_robot_synthesis
                else:
                    time.sleep(1)
            except KeyboardInterrupt:
                print(
                    "KeyboardInterrupt (parent process should get it instead)")
                raise KeyboardInterrupt
            except (socket.error, socket.herror, socket.gaierror):
                need_to_reconnect = True
                continue
            except (ValueError, IndexError):
                continue


class NotificationClient:

    def __init__(self, time_start):
        self.__port = config.DATAGATHERING_PORT
        self.__ip = config.DATAGATHERING_HOST
        self.__time_start = datetime.strptime(
            time_start, "%d-%m-%Y %H-%M-%S %f").astimezone(pytz.timezone('Europe/Berlin'))
        self.__keep_thread_alive = True
        self.__input_voltage = None

        self.__init_treated_plant = False
        self.__treated_plant = None

        self.__init_field = False
        self.__field_id = None
        self.__field = None
        self.__field_name = None

        self.__session_id = None
        self.__ws = None

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

        self.__start_report_th = threading.Thread(
            target=self.__start_report_th_tf, daemon=True)
        self.__start_report_th.start()

        self.__robot_synthesis_client = RobotSynthesisClient() 

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def stop(self):
        print("[Notification] Stopping service...")
        self.__keep_thread_alive = False
        if self.__ws:
            self.__ws.close()
        self.__start_report_th.join()

    def setStatus(self, status: RobotSynthesis):
        self.__robot_synthesis_client.set_last_robot_synthesis(status)

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

    def __websocket_start(self):
        self.__ws = websocket.WebSocketApp(f"ws://{self.__ip}:{self.__port}/api/v1/data_gathering/ws/robot/{self.__robot_sn}/{self.__session_id}",
                                           on_open=self.__websocket_running, on_error=self.__websocket_on_error, on_close=self.__websocket_on_close)
        self.__ws.run_forever(ping_interval=5, ping_timeout=3)

    def __websocket_on_close(self, close_status_code, close_msg):
        print("[Notification] Disconnected")

    def __websocket_on_error(self, conn, error):
        conn.close()
        print(f"[Notification] Error : {error}.")
        if self.__keep_thread_alive:
            print("[Notification] Reconnecting...")
            self.__websocket_start()

    def __websocket_running(self, conn):
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
                        "session_id": self.__session_id,
                        "voltage": self.__input_voltage,
                        "timestamp": datetime.now(pytz.timezone('Europe/Berlin')).isoformat()
                    }
                    response = requests.post(
                        f"http://{self.__ip}:{self.__port}/api/v1/data_gathering/vesc_statistic", json=send_vesc_statistic)

                    if response != 201:
                        Exception(
                            f"Error when sending input voltage: {response.status_code}.")

                    self.__input_voltage = None
            sleep(self.__alive_sending_timeout)

    def __start_report_th_tf(self):
        while (not self.__init_field or not self.__init_treated_plant) and self.__keep_thread_alive:
            if not self.__init_treated_plant and self.__treated_plant is not None:
                self.__send_treated_weed()
                self.__init_treated_plant = True
            if not self.__init_field and self.__field is not None and self.__field_name is not None:
                self.__send_field()
            sleep(0.5)
        if self.__keep_thread_alive:
            print("[Notification] Init session")
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

            self.__session_id = response.json()["id"]

            if self.__keep_thread_alive:
                print("[Notification] Connecting...")
                self.__websocket_start()

    def __init_robot_on_datagathering(self):
        response = requests.post(
            f"http://{self.__ip}:{self.__port}/api/v1/data_gathering/robot", json={"serial_number": self.__robot_sn})
        if response.status_code != 201 and response.status_code != 200:
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
