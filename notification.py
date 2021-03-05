from socket import socket, AF_INET, SOCK_STREAM, error as SocketError
import errno
import threading
from time import sleep

class SyntheseRobot:
    OP = "Robot_OP"
    HS = "Robot_HS"

class NotificationClient:

    def __init__(self):
        self.port = 888
        self.ip = "172.16.0.9"
        self._keep_thread_alive = True
        self.timeout = 1
        self.status = SyntheseRobot.OP
        self.socket = socket(AF_INET, SOCK_STREAM)
        self.socket.connect((self.ip, self.port))
        self.socket.settimeout(3)
        print("[Notification] Connected")
        self._report_th = threading.Thread(target=self._report_th_tf, daemon=True)
        self._report_th.start()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._keep_thread_alive = False
        self._close()

    def stop(self):
        self._keep_thread_alive = False
        self._close()

    def setStatus(self, status: SyntheseRobot):
        self.status = status
        self.socket.send(self.status.encode("utf-8"))

    def isConnected(self):
        return self._keep_thread_alive

    def _close(self):
        try:
            self.socket.send("".encode("utf-8"))
            self.socket.close() 
            print("[Notification] Disconnected") 
        except Exception:
            print("[Notification] Disconnected") 
         
        
 
    def _report_th_tf(self):
        while self._keep_thread_alive:
            try:
                self.socket.send(self.status.encode("utf-8"))
                sleep(self.timeout)
            except SocketError as e:
                if e.errno == errno.ECONNRESET:
                    reconnect = False
                    while not reconnect:
                        try:
                            self.socket = socket(AF_INET, SOCK_STREAM)
                            rep = self.socket.connect_ex((self.ip, self.port))
                            reconnect = rep == 0
                            if reconnect:
                                print("[Notification] Reconnected")
                            sleep(1)
                        except Exception:
                            pass
                elif e.errno == errno.EPIPE:
                    self.stop()
            except Exception as e:
                self.stop()

