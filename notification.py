from socket import socket, AF_INET, SOCK_STREAM, error as SocketError
import errno
import threading
from time import sleep
from config import config
import utility
import navigation

class SyntheseRobot:
    OP = "Robot_OP"
    HS = "Robot_HS"
    ANTI_THEFT = "Robot_ANTI_THEFT"

class NotificationClient:

    def __init__(self, time_start):
        self.port = 888
        self.ip = "172.16.0.9"
        self._keep_thread_alive = True
        self.connected = False
        self.timeout = config.ALIVE_SENDING_TIMEOUT
        self.status = SyntheseRobot.OP
        self.socket = socket(AF_INET, SOCK_STREAM)
        self.socket.settimeout(10)
        self.current_coordinate = []
        self.treated_plant = None
        self.field = None
        self.input_voltage = None
        self.extracted_plants = None
        self.antiTheftZone = None
        self.time_start = time_start
        self.first_send = False
        self.last_extracted_plants_send = dict()
        self._report_th = threading.Thread(target=self._report_th_tf, daemon=True)
        self._report_th.start()        

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._keep_thread_alive = False
        self._close()

    def stop(self):
        if config.CONTINUOUS_INFORMATION_SENDING:
            msg = ""
            if self.extracted_plants is not None:
                msg += f";{self.extracted_plants}"
            self.socket.send(f"STOP;{utility.get_current_time()}{msg}".encode("utf-8"))
        print("[Notification] Stopping service...")
        self._keep_thread_alive = False
        self._close()

    def setStatus(self, status: SyntheseRobot):
        self.status = status
        if status == SyntheseRobot.HS:
            self.socket.send(self.status.encode("utf-8"))

    def isConnected(self):
        return self._keep_thread_alive

    def _close(self):
        try:
            self.socket.send(f"CLOSE;{utility.get_current_time()}".encode("utf-8"))
            self.socket.close() 
        except:
            pass
        print("[Notification] Disconnected") 

    def set_current_coordinate(self, current_coordinate):
        if set(self.current_coordinate) != set(current_coordinate) and self.antiTheftZone is not None:
            current_coordinate_in_zone = self.antiTheftZone.coordianate_are_in_zone(current_coordinate)
            if current_coordinate_in_zone and self.status == SyntheseRobot.ANTI_THEFT:
                self.status = SyntheseRobot.OP
            elif not current_coordinate_in_zone and self.status == SyntheseRobot.OP:
                self.status = SyntheseRobot.ANTI_THEFT
        self.current_coordinate = current_coordinate

    def set_treated_plant(self, treated_plant):
        self.treated_plant = treated_plant

    def set_field(self, field):
        self.field = field
        self.antiTheftZone = navigation.AntiTheftZone(field)

    def set_input_voltage(self, input_voltage):
        self.input_voltage = input_voltage
    
    def set_extracted_plants(self, extracted_plants):
        self.extracted_plants = extracted_plants

    def is_continuous_information_sending(self):
        return config.CONTINUOUS_INFORMATION_SENDING
         
    def _report_th_tf(self):
        print("[Notification] Connection in progress...")
        while not self.connected and self._keep_thread_alive:
            try:
                self.socket.connect((self.ip, self.port))
                print("[Notification] Connected")
                self.connected = True
            except:
                sleep(10)

        while self._keep_thread_alive:
            try:
                if config.CONTINUOUS_INFORMATION_SENDING:

                    if self.time_start is not None and self.input_voltage is not None and self.treated_plant is not None and self.field is not None and not self.first_send:
                            self.socket.send(f"START;{self.time_start};{self.input_voltage};{self.treated_plant};{self.field}".encode("utf-8"))
                            self.first_send = True

                    if self.first_send:
                        msg = ""
                        if self.current_coordinate is not None:
                            msg += f";{self.current_coordinate}"
                        if self.extracted_plants is not None:
                            send_dict = dict()
                            for key, value in self.extracted_plants.items():
                                if key in self.last_extracted_plants_send:
                                    if value - self.last_extracted_plants_send[key] != 0:
                                        send_dict[key] = value - self.last_extracted_plants_send[key]
                                else:
                                    send_dict[key] = value
                            self.last_extracted_plants_send = dict(self.extracted_plants)
                            if send_dict:
                                msg += f";{send_dict}"

                        if self.status==SyntheseRobot.ANTI_THEFT:
                            msg = ""
                        
                        self.socket.send(f"{self.time_start};{self.status}{msg}".encode("utf-8"))
                    else:
                        self.socket.send(self.status.encode("utf-8"))

                else:
                    self.socket.send(self.status.encode("utf-8"))

                sleep(self.timeout)

            except SocketError:
                print("[Notification] Connection lost reconnecting...")
                reconnect = False
                while not reconnect and self._keep_thread_alive:
                    try:
                        self.socket.connect((self.ip, self.port))
                        reconnect = True
                        print("[Notification] Reconnected")
                    except:
                        continue
            except Exception:
                self.stop()
