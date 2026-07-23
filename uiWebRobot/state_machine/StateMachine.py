from flask_socketio import SocketIO
import sys

import utility
from uiWebRobot.state_machine.states import CheckState
from uiWebRobot.state_machine.states import ErrorState
from uiWebRobot.state_machine.Events import Events
from uiWebRobot.state_machine import State
from uiWebRobot.state_machine.FrontEndObjects import FrontEndObjects
from notification import RobotStateClient
from logger import LoggerFactory


class StateMachine:

    def __init__(self, socketio, robot_state_client: RobotStateClient):
        utility.create_directories("logs/")
        self.__file_logger = utility.Logger(
            "logs/" + utility.get_current_time()
        )

        self.__logger = LoggerFactory.create(self.__class__.__name__)

        self.__original_stderr = sys.stderr
        sys.stderr = ErrorLogger(
            self.__file_logger,
            self.__original_stderr,
        )

        self.socketio: SocketIO = socketio
        self.currentState: State.State = None
        self.__robot_state_client = robot_state_client

        self.change_current_state(
            CheckState(socketio, self.__file_logger)
        )

    def on_event(self, event: Events):
        msg = f"{self.currentState} received event : {event}."
        self.__file_logger.write_and_flush(msg+"\n")
        self.__logger.info(msg)

        try:
            newState = self.currentState.on_event(event)
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except Exception as e:
            self.__file_logger.write_and_flush(f"[Error on_event] <{e.__class__.__name__}> : "+str(e)+"\n")
            newState = ErrorState(self.socketio,self.__file_logger,str(e))

        if newState is None:
            msg = f"Error last state : {self.currentState}."
            self.__file_logger.write_and_flush(msg+"\n")
            newState = ErrorState(self.socketio,self.__file_logger,msg)

        if str(newState) in ["StartingState","ResumeState"]:
            self.change_current_state(newState.on_event(Events.CONFIG_IS_SET))
        else:
            self.change_current_state(newState)
    
    def on_socket_data(self, data):
        try:
            self.currentState = self.currentState.on_socket_data(data)
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except Exception as e:
            self.__file_logger.write_and_flush(f"[Error on_socket_data] <{e.__class__.__name__}> : "+str(e)+"\n")
            self.change_current_state(ErrorState(self.socketio,self.__file_logger,str(e)))
    
    def change_current_state(self, newState):
        self.currentState = newState
        self.__robot_state_client.set_robot_state(self.currentState.robot_synthesis_value)
        msg = f"New state : {self.currentState}."
        self.__file_logger.write_and_flush(msg+"\n")
        self.__logger.info(msg)

    def getStatusOfControls(self):
        frontEndObjects: (FrontEndObjects|dict) = self.currentState.getStatusOfControls()
        if isinstance(frontEndObjects,dict):
            return frontEndObjects
        else:
            return frontEndObjects.to_json()
    
    def getField(self):
        return self.currentState.getField()

    def close(self):
        sys.stderr = self.__original_stderr

class ErrorLogger:
    def __init__(self, file_logger: utility.Logger, original_stream):
        self.__file_logger = file_logger
        self.__original_stream = original_stream
        self.__buffer = ""

    def write(self, text: str) -> int:
        if not text:
            return 0

        # Affichage dans le terminal
        self.__original_stream.write(text)
        self.__original_stream.flush()

        # Enregistrement dans le fichier, sans ajouter un second saut de ligne
        self.__buffer += text

        while "\n" in self.__buffer:
            line, self.__buffer = self.__buffer.split("\n", 1)

            if line:
                self.__file_logger.write_and_flush(line + "\n")

        return len(text)

    def flush(self) -> None:
        self.__original_stream.flush()

        if self.__buffer:
            self.__file_logger.write_and_flush(self.__buffer)
            self.__buffer = ""

    def isatty(self) -> bool:
        return self.__original_stream.isatty()

    def fileno(self) -> int:
        return self.__original_stream.fileno()

    @property
    def encoding(self):
        return self.__original_stream.encoding