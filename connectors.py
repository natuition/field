import telnetlib
import serial


class SmoothieV11TelnetConnector:

    def __init__(self, host: str):
        self._host = host
        self._tn = telnetlib.Telnet(self._host)
        self._tn.read_until(b"Smoothie command shell\r\n")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def close(self):
        self._tn.write(b"exit\n")
        self._tn.read_all()
        self._tn.close()

    def disconnect(self):
        """Obsolete method, remains for backward compatibility. Use close() method instead."""

        self.close()

    def get_telnet(self):
        """Only for debug!"""

        return self._tn

    def write(self, command: str):
        if type(command) != str:
            raise TypeError("invalid command type: should be str, received " + type(command).__name__)
        if command == "":
            raise ValueError("invalid command value: should not be an empty string")

        self._tn.write(command.encode("ascii") + b"\n")

    def read_until(self, value: str, timeout=None):
        return self._tn.read_until(value.encode("ascii"), timeout).decode()

    def read_until_not(self, value: str):
        value = value.encode("ascii")
        while True:
            res = self.read_some()
            if res != value:
                return res.decode()

    def read_some(self):
        return self._tn.read_some().decode()

    def read_eager(self):
        return self._tn.read_eager().decode()


class SmoothieV11SerialConnector:
    def __init__(self, port: str, baudrate: int):
        self._port = port
        self._ser = serial.Serial(port, baudrate)
        self._ser.flushInput()
        self._ser.flushOutput()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    def disconnect(self):
        self._ser.close()

    @property
    def is_open(self):
        return self._ser.is_open

    def get_serial(self):
        """Only for debug!"""

        return self._ser

    def write(self, command: str):
        if type(command) != str:
            raise TypeError("invalid command type: should be str, received " + type(command).__name__)
        if command == "":
            raise ValueError("invalid command value: should not be an empty string")

        self._ser.write(command.encode("ascii") + b"\n")

    def read_some(self):
        return self._ser.readline().decode()
