import threading
import time
from serial import Serial,SerialException
import pyvesc
from queue import Queue

class VescInteractiveAdapter:
    """Provides navigation engines (forward/backward) control using vesc"""

    def __init__(self, rpm, alive_freq, check_freq, ser_port, ser_baudrate):
        self.start_cycle_time = time.time()

        self._ser = Serial(port=ser_port, baudrate=ser_baudrate)

        self._rpm = rpm
        self._alive_freq = alive_freq
        self._check_freq = check_freq
        self._queue = Queue()
        self._direction = 1
        self._start_time = self._next_alive_time = None
        self._allow_movement = False
        self._keep_thread_alive = True
        self._last_stop_time = 0

        self._ser.flushInput()
        self._ser.flushOutput()
        self._movement_ctrl_th = threading.Thread(target=self._movement_ctrl_th_tf, daemon=True)
        self._movement_ctrl_th.start()
        
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    def disconnect(self):
        self._ser.write(pyvesc.encode(pyvesc.SetRPM(0)))
        self._keep_thread_alive = False
        self._ser.close()

    def _movement_ctrl_th_tf(self):
        """Target function of movement control thread. Responsive for navigation engines work (forward/backward)"""

        try:
            while self._keep_thread_alive:
                if self._allow_movement:
                    if not self._queue.empty():
                        self._queue.get()
                        self._ser.write(pyvesc.encode(pyvesc.SetAlive))
                time.sleep(self._check_freq)
        except SerialException as ex:
            print(ex)

    def start_moving_forward(self):
        self._direction = 1
        self.start_moving()
        
    def start_moving_backward(self):
        self._direction = -1
        self.start_moving()
        
    def start_moving(self):
        self._start_time = self._next_alive_time = time.time()
        self._ser.write(pyvesc.encode(pyvesc.SetRPM(self._direction*self._rpm)))
        self._allow_movement = True
        self.alive()
    
    def alive(self):
        self._ser.write(pyvesc.encode(pyvesc.SetRPM(self._direction*self._rpm)))
        if self._queue.qsize()<2:
            self._queue.put("keepUp")

    def stop_moving(self):
        self._allow_movement = False
        self._last_stop_time = time.time()
        self._ser.write(pyvesc.encode(pyvesc.SetRPM(0)))

    def apply_rpm(self, rpm):
        if self._rpm != rpm:  # TODO: bug to fix: if rpm was set by set_rpm - it won't be applied on vesc
            self._rpm = rpm
            self._ser.write(pyvesc.encode(pyvesc.SetRPM(self._rpm)))

    def set_rpm(self, rpm):
        self._rpm = rpm

    def set_alive_freq(self, alive_freq):
        self._alive_freq = alive_freq

    def set_check_freq(self, check_freq):
        self._check_freq = check_freq

    def is_movement_allowed(self):
        return self._allow_movement

    def get_last_stop_time(self):
        return self._last_stop_time
