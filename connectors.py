#!/usr/bin/env python

import telnetlib
import time


class SmoothieConnector:

    def __init__(self, host: str):
        self._host = host
        self._tn = telnetlib.Telnet(self._host)
        self._tn.read_until(b"Smoothie command shell\r\n")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    def get_telnet(self):
        return self._tn

    def disconnect(self):
        self._tn.write(b"exit\n")
        self._tn.read_all()
        self._tn.close()

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

    def read_very_eager(self):
        return self._tn.read_very_eager().decode()


def _test():
    g_code = "G0 X20 F200"
    host = "192.168.2.222"
    sc = SmoothieConnector(host)

    ###

    print("Done.")


if __name__ == "__main__":
    _test()
