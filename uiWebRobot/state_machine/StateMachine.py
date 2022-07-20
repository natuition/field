import sys
sys.path.append('../')

from flask_socketio import SocketIO

import utility
from state_machine.states import CheckState
from state_machine.states import ErrorState
from state_machine.states import Events
from state_machine.states import State


class StateMachine:

    def __init__(self, socketio):
        utility.create_directories("logs/")
        self.logger = utility.Logger("logs/"+utility.get_current_time())
        sys.stderr = ErrorLogger(self.logger)
        self.socketio: SocketIO = socketio
        self.currentState: State.State = CheckState.CheckState(socketio,self.logger)
        msg = f"Current state : {self.currentState}."
        self.logger.write_and_flush(msg+"\n")
        print(msg)

    def on_event(self, event: Events.Events):
        print()
        msg = f"{self.currentState} received event : {event}."
        self.logger.write_and_flush(msg+"\n")
        print(msg)

        try:
            newState = self.currentState.on_event(event)
        except Exception as e:
            self.logger.write_and_flush("[Error] "+str(e)+"\n")
            newState = ErrorState.ErrorState(self.socketio,self.logger,str(e))

        if newState is None:
            msg = f"Error last state : {self.currentState}."
            self.logger.write_and_flush(msg+"\n")
            newState = ErrorState.ErrorState(self.socketio,self.logger,msg)

        if str(newState) in ["StartingState","ResumeState"]:
            self.currentState = newState.on_event(Events.Events.CONFIG_IS_SET)
        else:
            self.currentState = newState

        msg = f"Current state : {self.currentState}."
        self.logger.write_and_flush(msg+"\n")
        print(msg)
    
    def on_socket_data(self, data):
        self.currentState = self.currentState.on_socket_data(data)

    def getStatusOfControls(self):
        return self.currentState.getStatusOfControls()
    
    def getField(self):
        return self.currentState.getField()

class ErrorLogger:

    def __init__(self, logger: utility.Logger):
        self.logger = logger

    def write (self, s):
        print(s)
        self.logger.write_and_flush(s+"\n")