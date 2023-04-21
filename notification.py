import threading
import navigation
import requests
import websocket
import datetime
import json
from config import config
import socket
import time
import _thread as thread
import pytz
import enum
import traceback


class RobotStates(enum.Enum):
    ENABLED = "ENABLED"  # power is on, but robot is not currently at work
    WORKING = "WORKING"  # at work
    OUT_OF_SERVICE = "OUT_OF_SERVICE"
    ANTI_THEFT = "ANTI_THEFT"


class RobotStateServer:
    def __init__(self, fleet_tick_delay=60):
        self.__sync_locker = threading.Lock()

        self.__fleet_tick_delay = fleet_tick_delay
        self.__fleet_ip = config.DATAGATHERING_HOST
        self.__fleet_port = config.DATAGATHERING_PORT

        self.__host = config.ROBOT_SYNTHESIS_HOST
        self.__port = config.ROBOT_SYNTHESIS_PORT

        self.__robot_state = RobotStates.ENABLED
        self.__robot_sn = config.ROBOT_SN

        # clients connection listener
        self.__conn_listener = socket.socket()
        self.__conn_listener.bind((self.__host, self.__port))
        self.__conn_listener.listen(5)
        self.__keep_conn_listener_alive = True
        self.__conn_accept_th = threading.Thread(
            target=self.__client_conn_accept_tf,
            name="__conn_accept_th",
            daemon=True)

        self.__keep_robot_state_sender_alive = True
        self.__robot_state_sender_th = threading.Thread(
            target=self.__robot_state_sender_tf,
            name="__robot_state_sender_tf",
            daemon=True)

        self.__conn_accept_th.start()
        self.__robot_state_sender_th.start()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def close(self):
        self.__keep_conn_listener_alive = False
        self.__keep_robot_state_sender_alive = False
        self.__conn_listener.close()

    def wait(self):
        self.__conn_accept_th.join()

    def __client_data_reader_tf(self, client_socket: socket.socket, addr):
        msg = "[RobotStateServer] New client, ready to receive info"
        print(msg, flush=True)

        try:
            while self.__keep_conn_listener_alive:
                try:
                    data = client_socket.recv(1024)
                    if not data:
                        break
                    self.__robot_state = RobotStates[data.decode()]
                except KeyboardInterrupt:
                    raise KeyboardInterrupt
                except:
                    print(traceback.format_exc())
        except KeyboardInterrupt:
            msg = f"[RobotStateServer] Robot state client reader thread '{threading.current_thread().name}' " \
                  f"caught KBI (parent process should get it instead)"
            print(msg, flush=True)
            raise KeyboardInterrupt
        except:
            pass
        finally:
            client_socket.close()

    def __client_conn_accept_tf(self):
        print("[RobotStateServer] Ready to receive clients.", flush=True)
        while self.__keep_conn_listener_alive:
            try:
                client, address = self.__conn_listener.accept()
                thread.start_new_thread(self.__client_data_reader_tf, (client, address))
            except KeyboardInterrupt:
                msg = f"[RobotStateServer] Robot state conn accepting thread '{threading.current_thread().name}' " \
                      f"caught KBI (parent process should get it instead)"
                print(msg, flush=True)
                raise KeyboardInterrupt
            except:
                if self.__keep_conn_listener_alive:
                    print(traceback.format_exc())

    def __robot_state_sender_tf(self):
        while self.__keep_robot_state_sender_alive:
            try:
                with self.__sync_locker:
                    robot_state_to_send = {
                        "robot_synthesis": self.__robot_state.value,
                        "robot_serial_number": self.__robot_sn
                    }
                response = requests.post(
                    f"http://{self.__fleet_ip}:{self.__fleet_port}/api/v1/data_gathering/robot_status",
                    json=[robot_state_to_send])
            except KeyboardInterrupt:
                msg = f"[RobotStateServer] Robot state sender thread '{threading.current_thread().name}' caught KBI " \
                      f"(parent process should get it instead)"
                print(msg, flush=True)
                raise KeyboardInterrupt
            except:
                msg = f"[RobotStateServer] Failed send robot status to remote data gathering server:\n" \
                      f"{traceback.format_exc()}"
                print(msg, flush=True)
            time.sleep(self.__fleet_tick_delay)


class RobotStateClient:
    def __init__(self):
        self.__state_update_freq = 1  # how often to check if state was changed

        self.__host = config.ROBOT_SYNTHESIS_HOST
        self.__port = config.ROBOT_SYNTHESIS_PORT
        self.__robot_state_server_socket: socket.socket = None

        self.__robot_state = RobotStates.ENABLED
        self.__robot_state_is_fresh = False
        self.__robot_state_locker = threading.Lock()

        self.__need_to_reconnect = True

        self.__keep_robot_state_sender_alive = True
        self.__robot_state_sender_th = threading.Thread(
            target=self.__robot_state_sender_tf,
            name="__robot_state_sender_th",
            daemon=True)

        self.__robot_state_sender_th.start()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def close(self):
        self.__keep_robot_state_sender_alive = False
        self.__robot_state_server_socket.close()

    def __reconnect(self):
        if self.__robot_state_server_socket is not None:
            try:
                self.__robot_state_server_socket.shutdown(socket.SHUT_RDWR)
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                pass

            try:
                self.__robot_state_server_socket.close()
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                pass

        while self.__keep_robot_state_sender_alive:
            try:
                self.__robot_state_server_socket = socket.socket()
                error_indicator = self.__robot_state_server_socket.connect_ex((self.__host, self.__port))
                if error_indicator == 0:
                    self.__need_to_reconnect = False
                    break
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                pass

    def set_robot_state(self, robot_state: RobotStates):
        with self.__robot_state_locker:
            self.__robot_state = robot_state
            self.__robot_state_is_fresh = True

    def __robot_state_sender_tf(self):
        while self.__keep_robot_state_sender_alive:
            time.sleep(self.__state_update_freq)

            if self.__need_to_reconnect:
                self.__reconnect()

            try:
                if self.__robot_state_server_socket is None:
                    need_to_reconnect = True
                    continue

                with self.__robot_state_locker:
                    if self.__robot_state_is_fresh:
                        state_to_send = self.__robot_state
                        self.__robot_state_is_fresh = False
                    else:
                        continue

                self.__robot_state_server_socket.send(state_to_send.value.encode())
            except KeyboardInterrupt:
                msg = f"[RobotStateClient] Robot state client sender thread '{threading.current_thread().name}' " \
                      f"caught KBI (parent process should get it instead)"
                print(msg, flush=True)
                raise KeyboardInterrupt
            except (socket.error, socket.herror, socket.gaierror):
                with self.__robot_state_locker:
                    self.__robot_state_is_fresh = True
                need_to_reconnect = True
            except (ValueError, IndexError):
                pass
            except:
                msg = f"[RobotStateClient] Robot state client sender thread '{threading.current_thread().name}' " \
                      f"unexpected error:\n{traceback.format_exc()}"
                print(msg)


class NotificationClient:
    __RES_CODE_EXISTING = 200
    __RES_CODE_CREATED = 201

    def __init__(self, time_start):
        self.__port = config.DATAGATHERING_PORT
        self.__ip = config.DATAGATHERING_HOST
        self.__time_start = datetime.datetime.strptime(
            time_start,
            "%d-%m-%Y %H-%M-%S %f").astimezone(pytz.timezone('Europe/Berlin'))

        self.__input_voltage = None
        self.__input_voltage_sync_locker = threading.Lock()

        self.__robot_sn = config.ROBOT_SN
        self.__robot_sn_is_init = False

        self.__treated_weed_types: set = None
        self.__treated_weed_types_are_init = False
        self.__treated_weed_types_sync_locker = threading.Lock()

        self.__field = None
        self.__field_id = None
        self.__field_name = None
        self.__field_is_init = False
        self.__field_sync_locker = threading.Lock()

        self.__session_id = None
        self.anti_theft_zone = None

        self.__ws = None
        self.__ws_reconnection_required = True

        self.__path_point_number = 0
        self.__point_queue_max_length = config.MAX_LENGHT_POINT_HISTORY

        self.__total_ext_weeds = dict()
        self.__total_ext_weeds_last_sent = dict()

        self.__pos_with_weeds_queue = list()
        self.__pos_with_weeds_queue_sync_locker = threading.Lock()

        self.__robot_state_client = RobotStateClient()

        self.__keep_data_sender_th_alive = True
        self.__data_sender_th = threading.Thread(target=self.__data_sender_tf, daemon=True)
        self.__data_sender_th.start()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def close(self):
        msg = "[NotificationClient] Closing..."
        print(msg, flush=True)
        self.__keep_data_sender_th_alive = False
        if self.__ws is not None:
            try:
                self.__ws.close()
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                pass
        self.__data_sender_th.join()

    def is_working(self):
        return self.__keep_data_sender_th_alive

    def is_continuous_information_sending(self):
        return config.CONTINUOUS_INFORMATION_SENDING

    def set_treated_weed_types(self, treated_weed_types: set):
        with self.__treated_weed_types_sync_locker:
            self.__treated_weed_types = treated_weed_types

    def set_field(self, field, field_name):
        with self.__field_sync_locker:
            self.__field = field
            self.__field_name = field_name
            self.anti_theft_zone = navigation.AntiTheftZone(field)

    def set_robot_state(self, robot_state: RobotStates):
        self.__robot_state_client.set_robot_state(robot_state)

    def set_current_coordinate(self, current_coordinate):
        new_pos_and_weeds_record, newly_extracted_weeds = dict(), dict()
        if self.__total_ext_weeds:
            for key, value in self.__total_ext_weeds.items():
                if key in self.__total_ext_weeds_last_sent:
                    if value - self.__total_ext_weeds_last_sent[key] != 0:
                        newly_extracted_weeds[key] = value - self.__total_ext_weeds_last_sent[key]
                else:
                    newly_extracted_weeds[key] = value

            self.__total_ext_weeds_last_sent = self.__total_ext_weeds

        if newly_extracted_weeds:
            new_pos_and_weeds_record["extracted_weeds"] = newly_extracted_weeds
        new_pos_and_weeds_record["path_point_number"] = self.__path_point_number
        self.__path_point_number += 1
        new_pos_and_weeds_record["current_coordinate"] = current_coordinate

        with self.__pos_with_weeds_queue_sync_locker:
            self.__pos_with_weeds_queue.append(new_pos_and_weeds_record)
            if len(self.__pos_with_weeds_queue) > self.__point_queue_max_length:
                self.__pos_with_weeds_queue.pop(0)

    def set_input_voltage(self, input_voltage):
        with self.__input_voltage_sync_locker:
            self.__input_voltage = input_voltage

    def set_extracted_plants(self, extracted_weeds):
        self.__total_ext_weeds = extracted_weeds

    def __reconnect_ws(self):
        """Reconnects with a web socket to the DB distant server"""

        # close prev connection
        if self.__ws is not None:
            try:
                self.__ws.close()
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                msg = f"[NotificationClient] Failed to close connection before opening a new one:\n" \
                      f"{traceback.format_exc()}"
                print(msg)

        # open new connection
        try:
            self.__ws = websocket.WebSocket()
            self.__ws.connect(
                f"ws://{self.__ip}:{self.__port}/api/v1/data_gathering/ws/robot/{self.__robot_sn}/{self.__session_id}")
            self.__ws_reconnection_required = False
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except:
            msg = f"[NotificationClient] Failed to open new connection:\n{traceback.format_exc()}"
            print(msg)

    def __data_sender_tf(self):
        self.__do_inits()

        pos_and_ext_weeds_json = dict()
        vesc_statistics_json = dict()

        while self.__keep_data_sender_th_alive:
            if self.__ws_reconnection_required:
                self.__reconnect_ws()

            # send points data
            with self.__pos_with_weeds_queue_sync_locker:
                if self.__pos_with_weeds_queue and not pos_and_ext_weeds_json:
                    pos_and_ext_weeds_json = {"coordinate_with_extracted_weed": self.__pos_with_weeds_queue.copy()}
                    self.__pos_with_weeds_queue.clear()
            if pos_and_ext_weeds_json:
                try:
                    self.__ws.send(json.dumps(pos_and_ext_weeds_json))
                    pos_and_ext_weeds_json.clear()
                except KeyboardInterrupt:
                    raise KeyboardInterrupt
                except websocket.WebSocketException:
                    self.__ws_reconnection_required = True
                except:
                    msg = f"[NotificationClient] Failed to send points to remote server:\n{traceback.format_exc()}"
                    print(msg)

            # don't send vesc data if need to stop working as it may delay class instance proper closing for a while
            if not self.__keep_data_sender_th_alive:
                break

            # send vesc data
            with self.__input_voltage_sync_locker:
                if self.__input_voltage is not None:
                    vesc_statistics_json = {
                        "session_id": self.__session_id,
                        "voltage": self.__input_voltage,
                        "timestamp": datetime.datetime.now(pytz.timezone('Europe/Berlin')).isoformat()
                    }
                    self.__input_voltage = None
            if vesc_statistics_json:
                try:
                    response = requests.post(
                        f"http://{self.__ip}:{self.__port}/api/v1/data_gathering/vesc_statistic",
                        json=vesc_statistics_json)
                    if response.status_code == self.__RES_CODE_CREATED:
                        vesc_statistics_json.clear()
                    else:
                        msg = f"[NotificationClient] Failed to send input voltage, res code: {response.status_code}"
                        print(msg)
                except KeyboardInterrupt:
                    raise KeyboardInterrupt
                except:
                    msg = f"[NotificationClient] Failed to send input voltage, unexpected exception occured:\n" \
                          f"{traceback.format_exc()}"
                    print(msg)

            time.sleep(config.ALIVE_SENDING_TIMEOUT)

    def __do_inits(self):
        # send robot sn
        while not self.__robot_sn_is_init and self.__keep_data_sender_th_alive:
            try:
                self.__send_robot_sn()
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                pass
            if not self.__robot_sn_is_init and self.__keep_data_sender_th_alive:
                time.sleep(0.5)

        # send treated weed types
        while not self.__treated_weed_types_are_init and self.__keep_data_sender_th_alive:
            try:
                self.__send_treated_weed_types()
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                pass
            if not self.__treated_weed_types_are_init and self.__keep_data_sender_th_alive:
                time.sleep(0.5)

        # send current field
        while not self.__field_is_init and self.__keep_data_sender_th_alive:
            try:
                self.__send_field()
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                pass
            if not self.__field_is_init and self.__keep_data_sender_th_alive:
                time.sleep(0.5)

        # send current session
        while self.__session_id is None and self.__keep_data_sender_th_alive:
            try:
                self.__send_session()
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                pass
            if self.__session_id is None and self.__keep_data_sender_th_alive:
                time.sleep(0.5)

    def __send_session(self):
        with self.__field_sync_locker:
            if self.__field_id is None:
                msg = f"[NotificationClient] Failed to send current session as field_id is None"
                print(msg)
                return

            session_to_send = {
                "start_time": self.__time_start.isoformat(),
                "end_time": self.__time_start.isoformat(),
                "robot_serial_number": self.__robot_sn,
                "field_id": self.__field_id
            }
            response = requests.post(
                f"http://{self.__ip}:{self.__port}/api/v1/data_gathering/session",
                json=session_to_send)

            if response.status_code != self.__RES_CODE_CREATED:
                msg = f"[NotificationClient] Failed to send current session '{session_to_send}', res code: " \
                      f"{response.status_code}"
                print(msg)
                return

            self.__session_id = response.json()["id"]

    def __send_robot_sn(self):
        response = requests.post(
            f"http://{self.__ip}:{self.__port}/api/v1/data_gathering/robot",
            json={"serial_number": self.__robot_sn})
        if response.status_code != self.__RES_CODE_EXISTING and response.status_code != self.__RES_CODE_CREATED:
            msg = f"[NotificationClient] Failed to send robot's serial, res code: {response.status_code}"
            print(msg)
            return
        self.__robot_sn_is_init = True

    def __send_treated_weed_types(self):
        with self.__treated_weed_types_sync_locker:
            if self.__treated_weed_types is None:
                msg = f"[NotificationClient] Failed to setup treated plants as treated plants were not given " \
                      f"(they are None)"
                print(msg)
                return

            # get list of weed types known by DB
            response = requests.get(f"http://{self.__ip}:{self.__port}/api/v1/data_gathering/weeds_types")
            if response.status_code != self.__RES_CODE_EXISTING:
                msg = f"[NotificationClient] Failed to get weeds list from DB, res code: {response.status_code}"
                print(msg)
                return

            # get set of weed types absent in DB
            weeds_types_to_send = self.__treated_weed_types.difference(map(lambda item: item["label"], response.json()))

            # send absent weed types to DB
            for weed_type_name in weeds_types_to_send:
                weed_type = {"label": weed_type_name}
                response = requests.post(
                    f"http://{self.__ip}:{self.__port}/api/v1/data_gathering/weed_type",
                    json=weed_type)
                if response.status_code != self.__RES_CODE_EXISTING and response.status_code != self.__RES_CODE_CREATED:
                    msg = f"[NotificationClient] Error when sending treated weed '{weed_type}', res code: " \
                          f"{response.status_code}"
                    print(msg)
                    return
            self.__treated_weed_types_are_init = True

    def __send_field(self):
        with self.__field_sync_locker:
            if self.__field_name is None:
                msg = f"[NotificationClient] Failed to send field as stored field_name is None"
                print(msg)
                return
            if self.__field is None:
                msg = f"[NotificationClient] Failed to send field as stored field is None"
                print(msg)
                return
            if len(self.__field) == 0:
                msg = f"[NotificationClient] Failed to send field as stored field contains 0 points"
                print(msg)
                return

            # create field
            # field : [A, B, C, D] ou A : [lat, long]
            # field_name : string
            field = {"label": self.__field_name,
                     "robot_serial_number": self.__robot_sn}
            response = requests.post(f"http://{self.__ip}:{self.__port}/api/v1/data_gathering/field", json=field)

            if response.status_code != self.__RES_CODE_EXISTING and response.status_code != self.__RES_CODE_CREATED:
                print(f"[NotificationClient] Failed send field '{field}', res code: '{response.status_code}'")
                return

            self.__field_id = response.json()["id"]

            if response.status_code == self.__RES_CODE_CREATED:
                for point in self.__field:
                    # create gps_point
                    gps_point_to_send = {"quality": 0, "latitude": point[0], "longitude": point[1]}
                    response = requests.post(
                        f"http://{self.__ip}:{self.__port}/api/v1/data_gathering/gps_point",
                        json=gps_point_to_send)

                    if response.status_code == self.__RES_CODE_CREATED:
                        gps_point_id = response.json()["id"]
                        # link to field_name
                        field_corner_to_send = {"field_id": self.__field_id, "gps_point_id": gps_point_id}
                        response = requests.post(
                            f"http://{self.__ip}:{self.__port}/api/v1/data_gathering/field_corner",
                            json=field_corner_to_send)
                        if response.status_code != self.__RES_CODE_CREATED:
                            msg = f"[NotificationClient] Failed to create field corner '{field_corner_to_send}', " \
                                  f"res code: '{response.status_code}'"
                            print(msg)
                    else:
                        msg = f"[NotificationClient] Failed to create gps point, res code: " \
                              f"'{response.status_code}'"
                        print(msg)
            self.__field_is_init = True
