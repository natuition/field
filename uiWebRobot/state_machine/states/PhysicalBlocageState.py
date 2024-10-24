import sys
sys.path.append('../')

from flask_socketio import SocketIO
import signal
import posix_ipc
import threading
from datetime import datetime, timezone
import os
import json

from state_machine import State
from state_machine.states import WaitWorkingState
from state_machine.states import ErrorState
from state_machine import Events
from shared_class.robot_synthesis import RobotSynthesis

from uiWebRobot.state_machine.FrontEndObjects import AuditButtonState, ButtonState, FrontEndObjects
from uiWebRobot.state_machine import utilsFunction
from config import config
import utility

# This state corresponds when the robot is physically blocking.
class PhysicalBlocageState(State.State) :
    def __init__(self, socketio: SocketIO, logger: utility.Logger, isAudit: bool) :
        self.socketio = socketio
        self.logger = logger

        msg = f"[{self.__class__.__name__}] -> Physically blocked"
        self.logger.write_and_flush(msg + "\n")
        print(msg)        